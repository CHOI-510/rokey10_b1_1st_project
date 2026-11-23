import { ZONES, type Vec3 } from "./model";

// BASE coordinates in mm. Defaults: box_comm/common.py FIXED_POSES.
// Live operation overrides these from public.loc via database_work.get_fixed_poses().
export type LocationRow = {
  id: string; coords: unknown;
  inventory_calibration?: Record<string, { signature?: { xy?: unknown } }> | null;
};
function coordinateValues(value: unknown): (number | null)[] | null {
  if (typeof value === "string") {
    const text = value;
    try { value = JSON.parse(text); }
    catch { value = text.trim().replace(/^\(/, "").replace(/\)$/, "").split(","); }
  }
  if (value && typeof value === "object" && !Array.isArray(value)) {
    const object = value as Record<string, unknown>;
    value = [object.x, object.y, object.z ?? null];
  }
  if (!Array.isArray(value) || ![2,3,6].includes(value.length)) return null;
  const numbers = value.map(v => v === null || v === undefined ? null
    : typeof v === "number" || (typeof v === "string" && v.trim() !== "") ? Number(v) : NaN);
  if (numbers.some(v => v !== null && !Number.isFinite(v)) || numbers[0] === null || numbers[1] === null) return null;
  return numbers;
}
export type Workspace = {
  measure: Vec3;
  issues: string[];
  zones: { id: string; color: string; xyz: Vec3; spacing: number; height: number }[];
  sources: Record<string, "DB" | "설정 기본값" | "좌표 미설정">;
};
export function workspaceFromLocations(rows: LocationRow[]): Workspace {
  const sources: Workspace["sources"] = {};
  const issues: string[] = [];
  const positions = new Map<string, (number | null)[]>();
  for (const row of rows) {
    const name = String(row.id || "").trim();
    const id = name === "판별존" || name.toLowerCase() === "measure_zone" ? "measure_zone" : name.toUpperCase();
    let values = coordinateValues(row.coords);
    if (["A","B","C","D"].includes(id)) {
      const measuredXY = row.inventory_calibration?.["1"]?.signature?.xy;
      if (!Array.isArray(measuredXY) || measuredXY.length !== 2
        || !measuredXY.every(v => typeof v === "number" && Number.isFinite(v))) {
        issues.push(`${id}: 교정 XY 누락`);
        continue;
      }
      // coords may contain legacy orientation angles. Only verified calibration XY is a location.
      values = measuredXY;
    }
    if (id === "U" && (!values || values[2] === undefined || values[2] === null)) {
      issues.push("U: DB의 XYZ 좌표가 필요합니다");
      continue;
    }
    if (!values) continue;
    positions.set(id, values);
  }
  function position(id: string, fallback: Vec3, needsZ = false): Vec3 {
    const values = positions.get(id);
    const valid = values && (!needsZ || (values[2] !== undefined && values[2] !== null));
    sources[id] = valid ? "DB" : "설정 기본값";
    // A~D loc.z is not a pallet floor; retain place first-contact reference height.
    return valid ? [values[0]!, values[1]!, needsZ ? values[2]! : fallback[2]] : fallback;
  }
  const measure = position("measure_zone", [300, -160, 30.9], true);
  if (!positions.has("U")) sources.U = "좌표 미설정";
  const zones = ZONES.flatMap<Workspace["zones"][number]>(zone => {
    if (zone.id === "U") {
      // Do not invent a U drop location if the DB row is absent or incomplete.
      const values = positions.get("U");
      if (!values) return [];
      sources.U = "DB";
      return [{ ...zone, xyz: [values[0]!, values[1]!, values[2]!] }];
    }
    return [{ ...zone, xyz: position(zone.id, [...zone.xyz] as Vec3) }];
  });
  return { measure, zones, sources, issues };
}
// scan_config.py SCAN_PASSES: Y raster followed by X confirmation raster.
export const SCAN = {
  y: { lanes: Array.from({ length: 11 }, (_, i) => 270 + 30 * i), start: 265, end: -115, z: 40 },
  x: { lanes: Array.from({ length: 11 }, (_, i) => 225 - 29 * i), start: 230, end: 570, z: 35 },
};
