export type Vec3 = [number, number, number];
export type RobotTelemetry = {
  jointsDeg: [number, number, number, number, number, number];
  tcpMm: Vec3;
  receivedAt: number;
  gripperClosed?: boolean;
  target?: "A" | "B" | "C" | "D" | "U";
};
export const HOME = [0, 0, 90, 0, 90, 0];
// Joint origins/rpy from dsr_description2/urdf/m0609.urdf, converted to mm.
const origins: { xyz: Vec3; rpy: Vec3 }[] = [
  { xyz: [0, 0, 134.5], rpy: [0, 0, 0] },
  { xyz: [0, 6.2, 0], rpy: [0, -1.571, -1.571] },
  { xyz: [411, 0, 0], rpy: [0, 0, 1.571] },
  { xyz: [0, -368, 0], rpy: [1.571, 0, 0] },
  { xyz: [0, 0, 0], rpy: [-1.571, 0, 0] },
  { xyz: [0, -121, 0], rpy: [1.571, 0, 0] },
];
type Matrix = number[][];
const identity = (): Matrix => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
function multiply(a: Matrix, b: Matrix): Matrix {
  return a.map(row => b[0].map((_, col) => row.reduce((sum, v, k) => sum + v * b[k][col], 0)));
}
function rotate(matrix: Matrix, point: Vec3): Vec3 {
  return matrix.map(row => row.reduce((sum, v, k) => sum + v * point[k], 0)) as Vec3;
}
function rz(t: number): Matrix { return [[Math.cos(t), -Math.sin(t), 0], [Math.sin(t), Math.cos(t), 0], [0, 0, 1]]; }
function rpy([r, p, y]: Vec3): Matrix {
  const rx = [[1, 0, 0], [0, Math.cos(r), -Math.sin(r)], [0, Math.sin(r), Math.cos(r)]];
  const ry = [[Math.cos(p), 0, Math.sin(p)], [0, 1, 0], [-Math.sin(p), 0, Math.cos(p)]];
  return multiply(multiply(rz(y), ry), rx);
}
export function armPoints(joints: number[]): Vec3[] {
  let rotation = identity();
  let position: Vec3 = [0, 0, 0];
  const points: Vec3[] = [[0, 0, 0]];
  origins.forEach((origin, index) => {
    const delta = rotate(rotation, origin.xyz);
    position = position.map((v, k) => v + delta[k]) as Vec3;
    points.push(position);
    rotation = multiply(multiply(rotation, rpy(origin.rpy)), rz(joints[index] * Math.PI / 180));
  });
  const gripper = rotate(rotation, [0, 0, 90]); // Concept gripper length, not a calibrated RG2 mesh.
  points.push(position.map((v, k) => v + gripper[k]) as Vec3);
  return points;
}
export const ZONES = [
  { id: "A", color: "#12b6a5", xyz: [480, -200, 28.1], spacing: 45, height: 20 },
  { id: "B", color: "#3d82f6", xyz: [630, -200, 30.2], spacing: 45, height: 25 },
  { id: "C", color: "#f59a08", xyz: [635, -30, 27.9], spacing: 30, height: 20 },
  { id: "D", color: "#8b5cf6", xyz: [635, 115, 27.9], spacing: 30, height: 20 },
  { id: "U", color: "#94a3b8", xyz: [793, -113, 0], spacing: 30, height: 20 },
] as const;
export function demoFrame(seconds: number) {
  const poses = [HOME, [0, 25, 110, 0, 45, 0], [0, 35, 115, 0, 30, 0], [-25, 25, 95, 0, 55, 20], [-25, 40, 100, 0, 40, 20], [-25, 25, 95, 0, 55, 20], HOME];
  const labels = ["측정존 접근", "물품 집기", "상승 · B 구역 이동", "하강 · 적재", "그리퍼 개방 · 상승", "HOME 복귀"];
  const t = seconds % 18;
  const step = Math.floor(t / 3);
  const u = (t % 3) / 3;
  const ease = u * u * (3 - 2 * u);
  return { joints: poses[step].map((v, k) => v + (poses[step + 1][k] - v) * ease),
    label: labels[step], step, holding: step >= 2 && step <= 3 };
}
