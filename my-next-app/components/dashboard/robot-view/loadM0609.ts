import { BoxGeometry, Color, DoubleSide, Euler, Group, Mesh, MeshStandardMaterial, Vector3, type Object3D, type Material } from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";

type Motor = { object: Group; axis: Vector3 };
export type RobotMesh = { root: Group; joints: Motor[]; fingers: (Motor & { multiplier: number })[]; payload: { mesh: Mesh; left: Group; right: Group; base: Group } };
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
  const documents = await Promise.all(["/robots/m0609/m0609.urdf", "/robots/rg2/rg2.urdf"].map(async path => {
    const response=await fetch(path,{signal});
    if (!response.ok) throw new Error("Robot URDF load failed");
    const xml=new DOMParser().parseFromString(await response.text(),"application/xml");
    if (xml.querySelector("parsererror")) throw new Error("Invalid robot URDF");
    return xml;
  }));
  const elements=(selector: string)=>documents.flatMap(xml=>Array.from(xml.querySelectorAll(selector)));
  const root = new Group(), links = new Map<string,Group>(), motors = new Map<string,{object:Group;axis:Vector3}>();
  const loads: Promise<void>[] = [];
  for (const element of elements("robot > link")) {
    const link = new Group(); link.name = element.getAttribute("name") || ""; links.set(link.name,link);
    for (const visual of element.querySelectorAll(":scope > visual")) {
      const mesh = visual.querySelector("geometry > mesh");
      if (!mesh) continue;
      const path = mesh.getAttribute("filename") || "";
      if (!path.startsWith("/robots/m0609/") && !path.startsWith("/robots/rg2/")) throw new Error("Unexpected M0609 mesh path");
      const holder = new Group(); origin(holder,visual.querySelector("origin")); link.add(holder);
      const scale = numbers(mesh.getAttribute("scale"),[1,1,1]);
      holder.scale.set(scale[0]*1000,scale[1]*1000,scale[2]*1000);
      loads.push((async()=>{
        const result = await fetch(path,{signal});
        if (!result.ok) throw new Error("M0609 mesh load failed");
        if (path.endsWith(".stl")) {
          const geometry=new STLLoader().parse(await result.arrayBuffer());
          const rgba=numbers(visual.querySelector("material > color")?.getAttribute("rgba") || null,[.8,.8,.8,1]);
          holder.add(new Mesh(geometry,new MeshStandardMaterial({color:new Color(rgba[0],rgba[1],rgba[2]),roughness:.5,metalness:.15,side:DoubleSide})));
          return;
        }
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
  for (const joint of elements("robot > joint")) {
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
  const fingers=elements("robot > joint").filter(joint=>joint.getAttribute("name")?.startsWith("rg2_") && joint.getAttribute("type")==="revolute").map(joint=>({
    ...motors.get(joint.getAttribute("name")!)!,
    multiplier:Number(joint.querySelector("mimic")?.getAttribute("multiplier") ?? 1),
  }));
  const base=links.get("rg2_base_link")!,left=links.get("rg2_left_inner_finger")!,right=links.get("rg2_right_inner_finger")!;
  // Centered geometry follows the finger pads, including the RG2 mounting orientation.
  const payload=new Mesh(new BoxGeometry(24,24,24),new MeshStandardMaterial({color:"#e4a63b",roughness:.65,side:DoubleSide}));
  payload.name="held-object";payload.visible=false;base.add(payload);
  return {root,joints:joints as RobotMesh["joints"],fingers,payload:{mesh:payload,left,right,base}};
}
export function setRobotJoints(robot: RobotMesh, degrees: number[], gripperClosed?: boolean, holding=false) {
  robot.joints.forEach((joint,i)=>joint.object.quaternion.setFromAxisAngle(joint.axis,degrees[i]*Math.PI/180));
  // UI poses only; no measured finger width is currently available from the API.
  // 24 mm demo object + pad offset, derived from rg2.urdf linkage dimensions.
  // Stop fingers at the object surface instead of fully closing through the object.
  const gripAngle=Math.atan2(39.592,38.177)+Math.asin((17.178-18-24/2)/Math.hypot(38.177,39.592));
  const fingerAngle=holding ? gripAngle : gripperClosed ? .785398 : -.558505;
  robot.fingers.forEach(joint=>joint.object.quaternion.setFromAxisAngle(joint.axis,fingerAngle*joint.multiplier));
  robot.root.updateMatrixWorld(true);
  const {mesh,left,right,base}=robot.payload;
  mesh.visible=holding;
  if (holding) {
    // Inner pads face one another; their average is the visual grasp center.
    const center=left.localToWorld(new Vector3(0,18,35))
      .add(right.localToWorld(new Vector3(0,18,35))).multiplyScalar(.5);
    mesh.position.copy(base.worldToLocal(center));
    mesh.updateMatrixWorld(true);
  }
}
