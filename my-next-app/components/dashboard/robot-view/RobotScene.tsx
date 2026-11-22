"use client";
import { useRef, useState } from "react";
import M0609RobotMesh from "./M0609RobotMesh";
import { projectBasePoint, clipMatrixFromProjection } from "./projection";
import { boxFaces, faceCenter, viewDepth, STOCK_FLOOR_Z, type BoxGeometry } from "./boxGeometry";
import { armPoints, type Vec3 } from "./model";

import { SCAN, type Workspace } from "./workspace";

type Props = {
  workspace: Workspace;
  joints: number[]; counts: Record<string, number>; yaw: number; elevation: number; zoom: number;
  holding: boolean; muted: boolean; target?: string; tcp?: Vec3;
  onRotate: (yaw: number) => void;
};
export default function RobotScene({ workspace, joints, counts, yaw, elevation, zoom, holding, muted, target, tcp, onRotate }: Props) {
  const [meshStatus,setMeshStatus] = useState<"loading" | "ready" | "error">("loading");
  const drag = useRef<{ x: number; angle: number } | null>(null);
  const points = armPoints(joints);
  if (tcp) points[points.length - 1] = tcp;
  const boxes: BoxGeometry[] = workspace.zones.flatMap(zone => zone.id === "U" ? []
    : Array.from({ length: Math.min(Math.max(counts[zone.id] || 0, 0), 6) }, (_, i) => ({
      key: `${zone.id}${i}`, color: zone.color, width: Math.min(zone.id === "B" ? 24 : 20, zone.spacing * .42), height: zone.height,
      position: [zone.xyz[0], zone.xyz[1] - Math.floor(i / 3) * zone.spacing, STOCK_FLOOR_Z + (i % 3) * zone.height] as Vec3,
    })));
  const rawProject = (point: Vec3) => projectBasePoint(point, yaw, elevation, 1);
  // Fixed workspace bounds keep the camera steady as joints and inventory change.
  const bounds: Vec3[] = [[0,0,0], [0,0,800], workspace.measure, [230,-115,0], [570,265,0]];
  workspace.zones.forEach(({id,xyz:[x,y,z],height}) => {
    for (const px of [x-50,x+50]) for (const py of [y-95,y+75]) {
      bounds.push([px,py,0],[px,py,id === "U" ? z : STOCK_FLOOR_Z+height*3]);
    }
  });
  const projected = bounds.map(rawProject);
  const minX = Math.min(...projected.map(p=>p[0])), maxX = Math.max(...projected.map(p=>p[0]));
  const minY = Math.min(...projected.map(p=>p[1])), maxY = Math.max(...projected.map(p=>p[1]));
  const fit = Math.min(1, 800 / Math.max(1,maxX-minX), 350 / Math.max(1,maxY-minY));
  const project = (point: Vec3) => {
    const [x,y] = rawProject(point);
    return [450 + (x-(minX+maxX)/2)*fit*zoom, 255 + (y-(minY+maxY)/2)*fit*zoom];
  };
  const polygon = (vertices: Vec3[]) => vertices.map(p => project(p).join(",")).join(" ");
  const line = (a: Vec3, b: Vec3) => { const [x1,y1] = project(a), [x2,y2] = project(b); return { x1, y1, x2, y2 }; };
  const depth = (point: Vec3) => viewDepth(point, yaw, elevation);
  const box = (geometry: BoxGeometry) => {
    const faces = boxFaces(geometry).map((vertices,index)=>({vertices,index}))
      .sort((a,b)=>depth(faceCenter(a.vertices))-depth(faceCenter(b.vertices)));
    return <g key={geometry.key} data-stock-box={geometry.key} stroke={geometry.color} strokeWidth="1">
      {faces.map(({vertices,index}) => <polygon key={index} points={polygon(vertices)} fill={geometry.color}
        style={{filter:`brightness(${index===1?1.22:index<4?.95:.78})`}} />)}
    </g>;
  };
  const orderedBoxes = [...boxes].sort((a,b)=>depth([a.position[0],a.position[1],a.position[2]+a.height/2])
    -depth([b.position[0],b.position[1],b.position[2]+b.height/2]));
  return <svg viewBox="0 0 900 520" className="h-full min-h-[280px] w-full touch-pan-y select-none cursor-grab active:cursor-grabbing" role="img" aria-label="M0609 URDF 로봇 외형과 적재 구역의 3차원 뷰. 판별존, 미분류 투하점, 탐지 경로를 BASE 좌표로 표시. 드래그로 시점 회전."
    onPointerDown={event => { drag.current = { x: event.clientX, angle: yaw }; event.currentTarget.setPointerCapture(event.pointerId); }}
    onPointerMove={event => { if (drag.current) onRotate(drag.current.angle + (event.clientX - drag.current.x) * .35); }}
    onPointerUp={() => { drag.current = null; }} onPointerCancel={() => { drag.current = null; }}>
    <defs><linearGradient id="robot-link" x2="1" y2="1"><stop stopColor="#fff"/><stop offset="1" stopColor="#bfccd6"/></linearGradient></defs>
    <polygon points={polygon([[-180,-380,0],[920,-380,0],[920,330,0],[-180,330,0]])} fill="#f5f8fa" stroke="#dce5eb" />
    {Array.from({length:12},(_,i)=>-180+i*100).map(x=><line key={`x${x}`} {...line([x,-380,0],[x,330,0])} stroke="#e4ebef" />)}
    {Array.from({length:8},(_,i)=>-380+i*100).map(y=><line key={`y${y}`} {...line([-180,y,0],[920,y,0])} stroke="#e4ebef" />)}
    <polygon points={polygon([[230,-115,0],[570,-115,0],[570,265,0],[230,265,0]])} fill="#dbeafe" fillOpacity=".45" stroke="#7ca6cd" strokeDasharray="5 4"/>
    {SCAN.y.lanes.map((x, i) => <line key={`scanY${i}`} {...line([x,SCAN.y.start,SCAN.y.z],[x,SCAN.y.end,SCAN.y.z])} stroke="#60a5fa" strokeWidth="1" opacity=".45"/>)}
    {SCAN.x.lanes.map((y, i) => <line key={`scanX${i}`} {...line([SCAN.x.start,y,SCAN.x.z],[SCAN.x.end,y,SCAN.x.z])} stroke="#93a3b8" strokeWidth="1" opacity=".3" strokeDasharray="4 3"/>)}
    <text x={project([390,285,0])[0]} y={project([390,285,0])[1]} fill="#487295" fontSize="14" textAnchor="middle">탐지존 · Y 탐색 / X 확인</text>
    {(() => {
      const [x,y,z] = workspace.measure;
      const marker = project(workspace.measure);
      return <g><polygon points={polygon([[x-45,y-45,z],[x+45,y-45,z],[x+45,y+45,z],[x-45,y+45,z]])} fill="#dcebea" stroke="#117d75" strokeDasharray="5 4"/>
        <circle cx={marker[0]} cy={marker[1]} r="4" fill="#117d75"/>
        <text x={project([x,y-65,z])[0]} y={project([x,y-65,z])[1]} fill="#117d75" fontSize="14" textAnchor="middle">판별존</text></g>;
    })()}
    <g data-layer="zone-floors">
      {workspace.zones.map(zone=>{
        const [x,y]=zone.xyz, active=target===zone.id;
        return <polygon key={zone.id} points={polygon([[x-45,y-90,STOCK_FLOOR_Z],[x+45,y-90,STOCK_FLOOR_Z],[x+45,y+40,STOCK_FLOOR_Z],[x-45,y+40,STOCK_FLOOR_Z]])}
          fill={active?zone.color:"#fff"} fillOpacity={active ? .18 : 1} stroke={zone.color} strokeWidth={active?3:1.3}/>;
      })}
    </g>
    {([[120,0,0],[0,120,0],[0,0,120]] as Vec3[]).map((end,i)=><g key={`axis${i}`}>
      <line {...line([0,0,0],end)} stroke={["#e56464","#16a085","#598be1"][i]} strokeWidth="2"/>
      <text x={project(end)[0]+5} y={project(end)[1]} fontSize="13" fill={["#e56464","#16a085","#598be1"][i]}>{["+X","+Y","+Z"][i]}</text>
    </g>)}
    {meshStatus !== "ready" && <g opacity={muted ? .4 : 1}>
      <ellipse cx={project([0,0,0])[0]} cy={project([0,0,0])[1]+10} rx="45" ry="15" fill="#1e293b" opacity=".1"/>
      <line {...line([0,0,0],[0,0,85])} stroke="#78929a" strokeWidth="58" strokeLinecap="round"/>
      <line {...line([0,0,10],[0,0,80])} stroke="#127d75" strokeWidth="42" strokeLinecap="round"/>
      {points.slice(1).map((p,i)=>{
        const start=points[i], xy=project(p), width=i<3?28:17;
        return <g key={i}>
          <line {...line(start,p)} stroke="#738b99" strokeWidth={width+5} strokeLinecap="round"/>
          <line {...line(start,p)} stroke="url(#robot-link)" strokeWidth={width} strokeLinecap="round"/>
          <circle cx={xy[0]} cy={xy[1]} r={i<3?15:10} fill="#0f827a" stroke="#e5fffa" strokeWidth="3"/>
          <circle cx={xy[0]} cy={xy[1]} r="3" fill="#c4eee7"/>
        </g>;
      })}
      {holding && box({position:points[points.length-1],color:"#e4a63b",key:"carried",width:24,height:24})}
    </g>}
    <foreignObject x="0" y="0" width="900" height="520" pointerEvents="none">
      <M0609RobotMesh joints={joints} clip={clipMatrixFromProjection(project,depth)} muted={muted} onStatus={setMeshStatus}/>
    </foreignObject>
    {meshStatus === "ready" && <g opacity={muted ? .4 : 1}>
      <line {...line(points[points.length-2],points[points.length-1])} stroke="#667681" strokeWidth="8" strokeLinecap="round"/>
      {holding && box({position:points[points.length-1],color:"#e4a63b",key:"carried",width:24,height:24})}
    </g>}
    {/* Stock is an always-visible overlay at its actual world position. */}
    <g data-layer="stock-boxes">{orderedBoxes.map(box)}</g>
    <g data-layer="stock-outlines" fill="none" strokeWidth="1.1" opacity=".65" pointerEvents="none">
      {orderedBoxes.map(geometry=><g key={geometry.key} stroke={geometry.color}>
        {boxFaces(geometry).map((face,i)=><polygon key={i} points={polygon(face)}/>)}
      </g>)}
    </g>
    <g data-layer="zone-labels">
      {workspace.zones.map(zone=>{
        const [x,y,z]=zone.xyz;
        const zoneBoxes=boxes.filter(item=>item.key.startsWith(zone.id));
        const anchorPoints: Vec3[] = [[x-45,y-90,0],[x+45,y+40,0],...zoneBoxes.flatMap(item=>boxFaces(item).flat())];
        const labelX=project([x,y-zone.spacing/2,0])[0];
        const labelY=Math.max(...anchorPoints.map(point=>project(point)[1]))+19;
        const marker=project(zone.xyz);
        return <g key={zone.id}>
          {zone.id==="U" && <g>
            <line {...line([x,y,0],[x,y,z])} stroke={zone.color} strokeDasharray="4 4"/>
            <circle cx={marker[0]} cy={marker[1]} r="7" fill="white" stroke={zone.color} strokeWidth="2"/>
            <text x={marker[0]} y={marker[1]-14} textAnchor="middle" fill="#64748b" fontSize="12">미분류 투하 · Z {z}</text>
          </g>}
          <rect x={labelX-29} y={labelY-13} width="58" height="24" rx="12" fill={zone.color}/>
          <text x={labelX} y={labelY+4} textAnchor="middle" fill="white" fontSize="13" fontWeight="700">{zone.id} · {counts[zone.id]||0}</text>
        </g>;
      })}
    </g>
    {tcp && <g><circle cx={project(tcp)[0]} cy={project(tcp)[1]} r="8" fill="none" stroke="#f59a08" strokeWidth="2"/><text x={project(tcp)[0]+12} y={project(tcp)[1]} fill="#ae6b00" fontSize="13">TCP</text></g>}
    <text x="24" y="466" fill="#718096" fontSize="12">{meshStatus === "ready" ? "Doosan M0609 · URDF 모델" : meshStatus === "error" ? "3D 모델을 사용할 수 없어 관절 개념도를 표시합니다" : "M0609 3D 모델 불러오는 중…"}</text>
    <text x="24" y="486" fill="#8b99a7" fontSize="13">BASE · mm</text>
    <text x="24" y="507" fill="#8b99a7" fontSize="12">드래그하여 회전 · 버튼으로 확대</text>
  </svg>;
}
