import type { Vec3 } from "./model";

// Visual pallet surface only; robot TCP/contact reference heights are separate.
export const STOCK_FLOOR_Z = 3;

export type BoxGeometry = { key: string; position: Vec3; color: string; width: number; height: number };
export function viewDepth([x, y, z]: Vec3, yaw: number, elevation: number): number {
  const angle = yaw * Math.PI / 180, tilt = elevation * Math.PI / 180;
  const rotatedY = (x - 330) * Math.sin(angle) - y * Math.cos(angle);
  return rotatedY * Math.cos(tilt) + z * Math.sin(tilt);
}
export function boxFaces({ position: [x,y,z], width: w, height: h }: BoxGeometry): Vec3[][] {
  return [
    [[x-w,y-w,z],[x+w,y-w,z],[x+w,y+w,z],[x-w,y+w,z]],
    [[x-w,y-w,z+h],[x+w,y-w,z+h],[x+w,y+w,z+h],[x-w,y+w,z+h]],
    [[x-w,y-w,z],[x+w,y-w,z],[x+w,y-w,z+h],[x-w,y-w,z+h]],
    [[x-w,y+w,z],[x+w,y+w,z],[x+w,y+w,z+h],[x-w,y+w,z+h]],
    [[x-w,y-w,z],[x-w,y+w,z],[x-w,y+w,z+h],[x-w,y-w,z+h]],
    [[x+w,y-w,z],[x+w,y+w,z],[x+w,y+w,z+h],[x+w,y-w,z+h]],
  ];
}
export function faceCenter(face: Vec3[]): Vec3 {
  return [0,1,2].map(axis => face.reduce((sum, point) => sum + point[axis], 0) / face.length) as Vec3;
}
