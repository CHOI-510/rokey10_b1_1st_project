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
