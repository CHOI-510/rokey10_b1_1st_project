"use client";

import { useEffect, useState } from "react";
import { createSupabaseClient } from "@/lib/supabase/client";

const specifications = [
  { loc_id: "A", label: "규격 A", color: "bg-[#12b6a5]" },
  { loc_id: "B", label: "규격 B", color: "bg-[#3d82f6]" },
  { loc_id: "C", label: "규격 C", color: "bg-[#f59a08]" },
  { loc_id: "D", label: "규격 D", color: "bg-[#8b5cf6]" },
] as const;

type Counts = Record<(typeof specifications)[number]["loc_id"], number>;

const emptyCounts: Counts = { A: 0, B: 0, C: 0, D: 0 };

export default function StockStatus({ onConnectionChange }: { onConnectionChange: (connected: boolean) => void }) {
  const [counts, setCounts] = useState<Counts>(emptyCounts);
  const [unmappedLocIds, setUnmappedLocIds] = useState<string[]>([]);
  const [inventoryRowCount, setInventoryRowCount] = useState(0);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [labels, setLabels] = useState<Record<keyof Counts, string>>({ A: "규격 A", B: "규격 B", C: "규격 C", D: "규격 D" });

  useEffect(() => {
    let active = true;

    const loadInventory = async () => {
      try {
        const supabase = createSupabaseClient();
        const { data, error: queryError } = await supabase
          .from("inventory")
          .select("standard, cnt, loc_id")
          .abortSignal(AbortSignal.timeout(8000));

        if (queryError) throw queryError;
        if (!active) return;

        const nextCounts: Counts = { A: 0, B: 0, C: 0, D: 0 };
        const nextLabels: Record<keyof Counts, string> = { A: "규격 A", B: "규격 B", C: "규격 C", D: "규격 D" };
        const unknownLocations = new Set<string>();

        for (const row of data ?? []) {
          const locId = String(row.loc_id ?? "").trim().toUpperCase();
          const count = Number.parseInt(String(row.cnt ?? "0"), 10);
          const specification = specifications.find((item) => item.loc_id === locId);

          if (specification) {
            nextCounts[specification.loc_id] += Number.isFinite(count) ? Math.max(0, count) : 0;
            const standard = String(row.standard ?? "").trim();
            if (standard) nextLabels[specification.loc_id] = standard;
          } else {
            unknownLocations.add(locId || "(빈 loc_id)");
          }
        }

        setCounts(nextCounts);
        setLabels(nextLabels);
        setInventoryRowCount(data?.length ?? 0);
        setUnmappedLocIds([...unknownLocations]);
        setError(null);
        onConnectionChange(true);
      } catch (loadError) {
        if (active) {
          onConnectionChange(false);
          setError(loadError instanceof Error ? loadError.message : "inventory 조회 중 오류가 발생했습니다.");
        }
      } finally {
        if (active) setLoading(false);
      }
    };

    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      await loadInventory();
      if (active) timer = setTimeout(poll, 15_000);
    };
    void poll();

    return () => {
      active = false;
      clearTimeout(timer);
    };
  }, [onConnectionChange]);

  const total = Object.values(counts).reduce((sum, count) => sum + count, 0);

  return (
    <section className="min-h-[300px] min-w-0 rounded-[20px] border border-[#e1e8ef] bg-white p-6 shadow-sm max-[560px]:min-h-0 max-[560px]:rounded-2xl max-[560px]:p-5">
      <div className="flex items-center justify-between gap-3">
        <h2 className="m-0 text-[17px] font-bold text-[#566477]">적재 현황</h2>
        <span className="whitespace-nowrap text-sm text-[#97a3b4]">규격별 수량</span>
      </div>

      <ul className="m-0 mt-[18px] list-none p-0">
        {specifications.map((item) => {
          const count = counts[item.loc_id];

          return (
            <li key={item.loc_id} className="grid min-h-[53px] grid-cols-[12px_minmax(70px,.9fr)_minmax(0,1.1fr)_64px] items-center gap-[11px] border-t border-[#e8edf2] max-[560px]:grid-cols-[10px_minmax(55px,.9fr)_minmax(0,1.1fr)_58px] max-[560px]:gap-[7px]">
              <i className={`h-3 w-3 rounded-[4px] ${item.color}`} />
              <span className="min-w-0 break-keep text-base text-[#202936] max-[560px]:text-sm">{labels[item.loc_id] || item.label}</span>
              <span aria-label={`${count}개 적재`} role="img" className="flex min-w-0 flex-wrap items-center justify-start gap-[3px]">
                {Array.from({ length: count }, (_, index) => (
                  <i key={`${item.loc_id}-${index}`} aria-hidden="true" className={`h-[11px] w-[11px] shrink-0 rounded-[2px] ${item.color}`} />
                ))}
              </span>
              <strong className="w-full whitespace-nowrap text-right text-[17px] text-[#202936]">{count}</strong>
            </li>
          );
        })}
      </ul>

      <div className="mt-[17px] flex items-center justify-between text-sm text-[#8794a6]">
        <span>전체 적재</span>
        <strong className="text-[23px] text-[#202936]">{total} </strong>
      </div>

      {loading && <p className="mb-0 mt-2 text-xs text-[#8794a6]">inventory 불러오는 중...</p>}
      {error && <p role="alert" className="mb-0 mt-3 rounded-lg bg-[#fff2f0] px-3 py-2 text-xs text-[#b42318]">inventory 조회 실패: {error}</p>}
      {!loading && !error && inventoryRowCount === 0 && <p className="mb-0 mt-2 text-xs text-[#8794a6]">inventory 테이블에 적재 데이터가 없습니다.</p>}
      {unmappedLocIds.length > 0 && <p className="mb-0 mt-2 break-words text-xs text-[#b26a00]">규격 매핑이 필요한 loc_id: {unmappedLocIds.join(", ")}</p>}
    </section>
  );
}
