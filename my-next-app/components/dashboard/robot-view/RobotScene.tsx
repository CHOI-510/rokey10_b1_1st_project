"use client";
import { useRef } from "react";
import { armPoints, type Vec3 } from "./model";

import { SCAN, type Workspace } from "./workspace";

type Props = {
  workspace: Workspace;
  joints: number[]; counts: Record<string, number>; yaw: number; elevation: number; zoom: number;
  holding: boolean; muted: boolean; target?: string; tcp?: Vec3;
  onRotate: (yaw: number) => void;
};
export default function RobotScene({ workspace, joints, counts, yaw, elevation, zoom, holding, muted, target, tcp, onRotate }: Props) {
  const drag = useRef<{ x: number; angle: number } | null>(null);
  const angle = yaw * Math.PI / 180, tilt = elevation * Math.PI / 180;
  const project = ([x, y, z]: Vec3) => {
    const rx = (x - 330) * Math.cos(angle) - y * Math.sin(angle);
    const ry = (x - 330) * Math.sin(angle) + y * Math.cos(angle);
    return [450 + rx * .65 * zoom, 340 + (ry * Math.sin(tilt) - (z - 60) * Math.cos(tilt)) * .65 * zoom];
  };
  const points = armPoints(joints);
  // Actual BASE TCP replaces the conceptual gripper endpoint when telemetry is live.
  if (tcp) points[points.length - 1] = tcp;
  const polygon = (vertices: Vec3[]) => vertices.map(p => project(p).join(",")).join(" ");
  const line = (a: Vec3, b: Vec3) => { const [x1,y1] = project(a), [x2,y2] = project(b); return { x1, y1, x2, y2 }; };
  const box = (position: Vec3, color: string, key: string, w = 28, h = 21) => {
    const [x,y,z] = position;
    const top: Vec3[] = [[x-w,y-w,z+h],[x+w,y-w,z+h],[x+w,y+w,z+h],[x-w,y+w,z+h]];
    return <g key={key} stroke={color} strokeWidth="1">
      <polygon points={polygon([[x-w,y+w,z],[x+w,y+w,z],[x+w,y+w,z+h],[x-w,y+w,z+h]])} fill={color} />
      <polygon points={polygon([[x+w,y-w,z],[x+w,y+w,z],[x+w,y+w,z+h],[x+w,y-w,z+h]])} fill={color} opacity=".7" />
      <polygon points={polygon(top)} fill={color} className="brightness-125" />
    </g>;
  };
  return <svg viewBox="0 0 900 520" className="h-full min-h-[280px] w-full touch-pan-y select-none cursor-grab active:cursor-grabbing" role="img" aria-label="M0609 로봇 관절 구조와 적재 구역의 3차원 개념도. 판별존, 미분류 투하점, 탐지 경로를 BASE 좌표로 표시. 드래그로 시점 회전."
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
    {workspace.zones.map(zone=>{
      const [x,y,z]=zone.xyz;
      const active=target===zone.id;
      const label=project([x,y+65,zone.id === "U" ? z : 0]);
      return <g key={zone.id}>
        {zone.id === "U" && <g>
          <line {...line([x,y,0],[x,y,z])} stroke={zone.color} strokeDasharray="4 4"/>
          <circle cx={project(zone.xyz)[0]} cy={project(zone.xyz)[1]} r="7" fill="#fff" stroke={zone.color} strokeWidth="2"/>
          <text x={project(zone.xyz)[0]} y={project(zone.xyz)[1]-14} textAnchor="middle" fill="#64748b" fontSize="12">미분류 투하 · Z {z}</text>
        </g>}
        <polygon points={polygon([[x-45,y-90,3],[x+45,y-90,3],[x+45,y+40,3],[x-45,y+40,3]])} fill={active?zone.color:"#fff"} fillOpacity={active ? .18 : 1} stroke={zone.color} strokeWidth={active?3:1.3} />
        {zone.id!=="U" && Array.from({length:Math.min(counts[zone.id]||0,6)},(_,i)=>box([x,y-Math.floor(i/3)*zone.spacing,z+(i%3)*zone.height],zone.color,`${zone.id}${i}`,zone.id==="B"?24:20,zone.height))}
        <rect x={label[0]-29} y={label[1]-13} width="58" height="24" rx="12" fill={zone.color}/>
        <text x={label[0]} y={label[1]+4} textAnchor="middle" fill="white" fontSize="13" fontWeight="700">{zone.id} · {counts[zone.id]||0}</text>
      </g>;
    })}
    {([[120,0,0],[0,120,0],[0,0,120]] as Vec3[]).map((end,i)=><g key={`axis${i}`}>
      <line {...line([0,0,0],end)} stroke={["#e56464","#16a085","#598be1"][i]} strokeWidth="2"/>
      <text x={project(end)[0]+5} y={project(end)[1]} fontSize="13" fill={["#e56464","#16a085","#598be1"][i]}>{["+X","+Y","+Z"][i]}</text>
    </g>)}
    <g opacity={muted ? .4 : 1}>
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
      {holding && box(points[points.length-1],"#e4a63b","carried",24,24)}
    </g>
    {tcp && <g><circle cx={project(tcp)[0]} cy={project(tcp)[1]} r="8" fill="none" stroke="#f59a08" strokeWidth="2"/><text x={project(tcp)[0]+12} y={project(tcp)[1]} fill="#ae6b00" fontSize="13">TCP</text></g>}
    <text x="24" y="486" fill="#8b99a7" fontSize="13">BASE · mm</text>
    <text x="24" y="507" fill="#8b99a7" fontSize="12">드래그하여 회전 · 버튼으로 확대</text>
  </svg>;
}
