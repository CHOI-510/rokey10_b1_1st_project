"use client";
// @refresh reset
// Recreate the renderer on development edits so changed hooks never reuse old GPU/SVG state.
import { SVGRenderer } from "three/addons/renderers/SVGRenderer.js";
import { simplifyForSvg } from "./svgRobotGeometry";
import { useEffect, useRef } from "react";
import { AmbientLight, DirectionalLight, Mesh, MeshStandardMaterial, OrthographicCamera, Scene, WebGLRenderer } from "three";
import { disposeRobot, loadM0609, setRobotJoints, type RobotMesh } from "./loadM0609";

type Props = { joints: number[]; clip: number[]; muted: boolean; gripperClosed?: boolean; holding: boolean; onStatus: (status: "loading" | "ready" | "error") => void };
type Frame = Pick<Props,"joints" | "clip" | "muted" | "gripperClosed" | "holding">;
type Runtime = { renderer: WebGLRenderer | SVGRenderer; scene: Scene; camera: OrthographicCamera; robot?: RobotMesh };
function draw(runtime: Runtime, frame: Frame) {
  runtime.camera.projectionMatrix.fromArray(frame.clip).transpose();
  runtime.camera.projectionMatrixInverse.copy(runtime.camera.projectionMatrix).invert();
  if (runtime.robot) {
    setRobotJoints(runtime.robot,frame.joints,frame.gripperClosed,frame.holding);
    runtime.robot.root.traverse(object=>{
      if (object instanceof Mesh) for (const material of Array.isArray(object.material)?object.material:[object.material]) {
        if (material instanceof MeshStandardMaterial) {
          const transparent=frame.muted;
          if (material.transparent!==transparent) {material.transparent=transparent;material.needsUpdate=true;}
          material.opacity=frame.muted ? .4 : 1;
        }
      }
    });
  }
  runtime.renderer.render(runtime.scene,runtime.camera);
  if (runtime.renderer instanceof SVGRenderer) runtime.renderer.domElement.style.backgroundColor="transparent";
}
export default function M0609RobotMesh({joints,clip,muted,gripperClosed,holding,onStatus}: Props) {
  const container=useRef<HTMLDivElement>(null);
  const runtime=useRef<Runtime|null>(null);
  const latest=useRef<Frame>({joints,clip,muted,gripperClosed,holding});
  useEffect(()=>{
    latest.current={joints,clip,muted,gripperClosed,holding};
    if (runtime.current) draw(runtime.current,latest.current);
  },[joints,clip,muted,gripperClosed,holding]);
  useEffect(()=>{
    const element=container.current;
    if (!element) return;
    let active=true;
    const abort=new AbortController();
    let current: Runtime;
    onStatus("loading");
    try {
      let renderer: WebGLRenderer | SVGRenderer;
      try {
        renderer=new WebGLRenderer({alpha:true,antialias:true});
        renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
        renderer.setSize(900,520,false);renderer.setClearColor(0,0);
      } catch {
        renderer=new SVGRenderer();renderer.setSize(900,520);renderer.setPrecision(2);
        renderer.domElement.style.background="transparent";
      }
      renderer.domElement.style.width="100%";renderer.domElement.style.height="100%";
      element.replaceChildren(renderer.domElement);
      const scene=new Scene(),camera=new OrthographicCamera();
      scene.add(new AmbientLight(0xffffff,.9));
      const key=new DirectionalLight(0xffffff,.7);key.position.set(700,-600,1200);scene.add(key);
      const fill=new DirectionalLight(0xc7e5f2,.4);fill.position.set(-400,500,800);scene.add(fill);
      current={renderer,scene,camera};runtime.current=current;
    } catch {
      onStatus("error");return;
    }
    const lost=(event: Event)=>{event.preventDefault();if(active)onStatus("error");};
    const restored=()=>{if(active&&current.robot){draw(current,latest.current);onStatus("ready");}};
    current.renderer.domElement.addEventListener("webglcontextlost",lost);current.renderer.domElement.addEventListener("webglcontextrestored",restored);
    void loadM0609(abort.signal).then(robot=>{
      if(!active){disposeRobot(robot.root);return;}
      if(current.renderer instanceof SVGRenderer)simplifyForSvg(robot.root);
      current.robot=robot;current.scene.add(robot.root);draw(current,latest.current);onStatus("ready");
    }).catch(()=>{if(active)onStatus("error");});
    return ()=>{
      active=false;abort.abort();runtime.current=null;
      current.renderer.domElement.removeEventListener("webglcontextlost",lost);current.renderer.domElement.removeEventListener("webglcontextrestored",restored);
      if(current.robot)disposeRobot(current.robot.root);
      if(current.renderer instanceof WebGLRenderer){current.renderer.dispose();current.renderer.forceContextLoss();}
      element.replaceChildren();
    };
  },[onStatus]);
  return <div ref={container} aria-label="Doosan M0609 · OnRobot RG2 URDF 외형" style={{width:"100%",height:"100%",display:"block",pointerEvents:"none"}}/>;
}
