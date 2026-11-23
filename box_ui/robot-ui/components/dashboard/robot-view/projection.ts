import type { Vec3 } from "./model";

// Display correction only. BASE coordinates, telemetry and control commands stay unchanged.
export function projectBasePoint([x, y, z]: Vec3, yaw: number, elevation: number, zoom: number): [number, number] {
  const angle = yaw * Math.PI / 180;
  const tilt = elevation * Math.PI / 180;
  const displayY = -y;
  const rx = (x - 330) * Math.cos(angle) - displayY * Math.sin(angle);
  const ry = (x - 330) * Math.sin(angle) + displayY * Math.cos(angle);
  return [450 + rx * .65 * zoom, 340 + (ry * Math.sin(tilt) - (z - 60) * Math.cos(tilt)) * .65 * zoom];
}

// Row-major orthographic clip matrix matching the SVG projection exactly.
export function clipMatrixFromProjection(project: (point: Vec3) => number[], depth: (point: Vec3) => number): number[] {
  const origin: Vec3 = [0,0,0];
  const axes: Vec3[] = [[1,0,0],[0,1,0],[0,0,1]];
  const p = project(origin), d = depth(origin);
  const slopes = axes.map(axis => { const value=project(axis); return [value[0]-p[0],value[1]-p[1],depth(axis)-d]; });
  return [
    ...slopes.map(s=>2*s[0]/900),2*p[0]/900-1,
    ...slopes.map(s=>-2*s[1]/520),1-2*p[1]/520,
    ...slopes.map(s=>-s[2]/3000),-d/3000,
    0,0,0,1,
  ];
}
