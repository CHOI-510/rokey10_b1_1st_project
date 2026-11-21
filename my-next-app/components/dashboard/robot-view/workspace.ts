import { ZONES, type Vec3 } from "./model";

// BASE coordinates in mm. Defaults: box_comm/common.py FIXED_POSES.
// Live operation overrides these from public.loc via database_work.get_fixed_poses().
export type LocationRow = { name: string; x: unknown; y: unknown; z: unknown };
export type Workspace = {
  measure: Vec3;
  zones: { id: string; color: string; xyz: Vec3; spacing: number; height: number }[];
  sources: Record<string, "DB" | "설정 기본값">;
};
export function workspaceFromLocations(rows: LocationRow[]): Workspace {
  const sources: Workspace["sources"] = {};
  function position(id: string, fallback: Vec3, needsZ = false): Vec3 {
    const row = rows.find(row => row.name === id);
    const valid = row && row.x !== null && row.y !== null && row.x !== "" && row.y !== ""
      && Number.isFinite(Number(row.x)) && Number.isFinite(Number(row.y))
      && (!needsZ || (row.z !== null && row.z !== "" && Number.isFinite(Number(row.z))));
    sources[id] = valid ? "DB" : "설정 기본값";
    // A~D loc.z is not a pallet floor; retain place first-contact reference height.
    return valid ? [Number(row.x), Number(row.y), needsZ ? Number(row.z) : fallback[2]] : fallback;
  }
  const measure = position("measure_zone", [300, -160, 30.9], true);
  const zones = ZONES.map(zone => ({ ...zone, xyz: zone.id === "U" ? [...UNCLASSIFIED_RELEASE] as Vec3
    : position(zone.id, [...zone.xyz] as Vec3) }));
  return { measure, zones, sources };
}
// place_algorithm.py UNCLASSIFIED_RELEASE_POSE. This is a TCP drop point, not a floor.
export const UNCLASSIFIED_RELEASE: Vec3 = [793, -113, 186];
// scan_config.py SCAN_PASSES: Y raster followed by X confirmation raster.
export const SCAN = {
  y: { lanes: Array.from({ length: 11 }, (_, i) => 270 + 30 * i), start: 265, end: -115, z: 40 },
  x: { lanes: Array.from({ length: 11 }, (_, i) => 225 - 29 * i), start: 230, end: 570, z: 35 },
};
