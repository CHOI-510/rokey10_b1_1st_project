import { Color, DoubleSide, Euler, Group, Mesh, MeshStandardMaterial, Vector3, type Object3D, type Material } from "three";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";

export type RobotMesh = { root: Group; joints: { object: Group; axis: Vector3 }[] };
const numbers = (text: string | null, fallback: number[]) => text ? text.trim().split(/\s+/).map(Number) : fallback;
function origin(group: Group, element: Element | null) {
  if (!element) return;
  const xyz = numbers(element.getAttribute("xyz"), [0,0,0]);
  const rpy = numbers(element.getAttribute("rpy"), [0,0,0]);
  group.position.set(xyz[0]*1000,xyz[1]*1000,xyz[2]*1000);
  group.quaternion.setFromEuler(new Euler(rpy[0],rpy[1],rpy[2],"ZYX"));
}
export function disposeRobot(root: Object3D) {
  root.traverse(object => {
    if (object instanceof Mesh) {
      object.geometry.dispose();
      (Array.isArray(object.material) ? object.material : [object.material]).forEach(material=>material.dispose());
    }
  });
}
export async function loadM0609(signal: AbortSignal): Promise<RobotMesh> {
  const response = await fetch("/robots/m0609/m0609.urdf", {signal});
  if (!response.ok) throw new Error("M0609 URDF load failed");
  const xml = new DOMParser().parseFromString(await response.text(),"application/xml");
  if (xml.querySelector("parsererror")) throw new Error("Invalid M0609 URDF");
  const root = new Group(), links = new Map<string,Group>(), motors = new Map<string,{object:Group;axis:Vector3}>();
  const loads: Promise<void>[] = [];
  for (const element of xml.querySelectorAll("robot > link")) {
    const link = new Group(); link.name = element.getAttribute("name") || ""; links.set(link.name,link);
    for (const visual of element.querySelectorAll(":scope > visual")) {
      const mesh = visual.querySelector("geometry > mesh");
      if (!mesh) continue;
      const path = mesh.getAttribute("filename") || "";
      if (!path.startsWith("/robots/m0609/")) throw new Error("Unexpected M0609 mesh path");
      const holder = new Group(); origin(holder,visual.querySelector("origin")); link.add(holder);
      const scale = numbers(mesh.getAttribute("scale"),[1,1,1]);
      holder.scale.set(scale[0]*1000,scale[1]*1000,scale[2]*1000);
      loads.push((async()=>{
        const result = await fetch(path,{signal});
        if (!result.ok) throw new Error("M0609 mesh load failed");
        // This URDF renderer uses native Z-up coordinates. Bypass the loader's
        // Y-up conversion at the metadata boundary; vertex data stay unchanged.
        // Previously we undid that conversion after parsing, causing a warning per mesh.
        const dae = (await result.text()).replace(/(<up_axis>\s*)Z_UP(\s*<\/up_axis>)/, "$1Y_UP$2");
        const parsed = new ColladaLoader().parse(dae,path.slice(0,path.lastIndexOf("/")+1));
        if (!parsed) throw new Error("Invalid M0609 Collada mesh");
        const model = parsed.scene;
        const extras: Object3D[] = [];
        model.traverse(object=>{
          if (object.type.includes("Camera") || object.type.includes("Light")) extras.push(object);
          if (object instanceof Mesh) {
            const convert = (old: Material) => {
              const color = "color" in old && old.color instanceof Color ? old.color.clone() : new Color("#e6e9ec");
              const material = new MeshStandardMaterial({color,roughness:.45,metalness:.18,side:DoubleSide});
              old.dispose(); return material;
            };
            object.material = Array.isArray(object.material) ? object.material.map(convert) : convert(object.material);
          }
        });
        extras.forEach(object=>object.removeFromParent()); holder.add(model);
      })());
    }
  }
  const children = new Set<string>();
  for (const joint of xml.querySelectorAll("robot > joint")) {
    const parentName=joint.querySelector("parent")?.getAttribute("link") || "";
    const childName=joint.querySelector("child")?.getAttribute("link") || "";
    const parent=links.get(parentName), child=links.get(childName);
    if (!parent || !child) throw new Error("Invalid M0609 joint tree");
    const attachment=new Group();origin(attachment,joint.querySelector("origin")); parent.add(attachment);
    const motor=new Group();attachment.add(motor);motor.add(child);children.add(childName);
    if (joint.getAttribute("type")==="revolute") {
      const axis=numbers(joint.querySelector("axis")?.getAttribute("xyz") || null,[0,0,1]);
      motors.set(joint.getAttribute("name") || "",{object:motor,axis:new Vector3(...axis).normalize()});
    }
  }
  for (const [name,link] of links) if (!children.has(name)) root.add(link);
  const results=await Promise.allSettled(loads);
  if (signal.aborted || results.some(result=>result.status==="rejected")) {disposeRobot(root);throw new Error("M0609 assets unavailable");}
  const joints=Array.from({length:6},(_,i)=>motors.get(`joint_${i+1}`));
  if (joints.some(joint=>!joint)) {disposeRobot(root);throw new Error("M0609 joints missing");}
  return {root,joints:joints as RobotMesh["joints"]};
}
export function setRobotJoints(robot: RobotMesh, degrees: number[]) {
  robot.joints.forEach((joint,i)=>joint.object.quaternion.setFromAxisAngle(joint.axis,degrees[i]*Math.PI/180));
  robot.root.updateMatrixWorld(true);
}
