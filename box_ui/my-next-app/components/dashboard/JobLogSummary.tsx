"use client";

import { useEffect, useState } from "react";
import { createSupabaseClient } from "@/lib/supabase/client";

type LogEvent = { id: number | string; timestamp: string; job_id: string; message: string; state: string };

export default function JobLogSummary({ jobId }: { jobId: string }) {
  const [events, setEvents] = useState<LogEvent[]>([]);
  const [counts, setCounts] = useState([0, 0, 0, 0]);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    const controller = new AbortController();
    const poll = async () => {
      try {
        const signal = AbortSignal.any([controller.signal, AbortSignal.timeout(8000)]);
        const db = createSupabaseClient();
        let selectedJob = jobId;
        if (!selectedJob) {
          const latest = await db.from("log").select("job_id").order("timestamp", { ascending: false }).limit(1).abortSignal(signal);
          if (latest.error) throw latest.error;
          selectedJob = latest.data?.[0]?.job_id || "";
        }
        if (!selectedJob) {
          if (active) { setEvents([]); setCounts([0, 0, 0, 0]); setError(null); }
        } else {
          const base = () => db.from("log").select("timestamp", { count: "exact", head: true }).eq("job_id", selectedJob);
          const results = await Promise.all([
            db.from("log").select("id,timestamp,job_id,message,state").eq("job_id", selectedJob).order("timestamp", { ascending: false }).order("id", { ascending: false }).limit(6).abortSignal(signal),
            base().abortSignal(signal),
            base().like("state", "스캔%").abortSignal(signal),
            base().like("state", "판별%").abortSignal(signal),
            base().like("state", "적재%").abortSignal(signal),
          ]);
          const failed = results.find(result => result.error);
          if (failed?.error) throw failed.error;
          if (active) {
            setEvents((results[0].data || []) as unknown as LogEvent[]);
            setCounts(results.slice(1).map(result => result.count || 0));
            setError(null);
          }
        }
      } catch (cause) {
        if (active) setError(cause instanceof Error ? cause.message : "로그 DB를 조회하지 못했습니다.");
      } finally {
        if (active) { setLoading(false); timer = setTimeout(poll, 1000); }
      }
    };
    void poll();
    return () => { active = false; controller.abort(); clearTimeout(timer); };
  }, [jobId]);
  const summary = [
    { label: "전체 로그", value: counts[0], valueClass: "text-[#202936]" },
    { label: "탐지 로그", value: counts[1], valueClass: "text-[#e67d00]" },
    { label: "판별 로그", value: counts[2], valueClass: "text-[#e67d00]" },
    { label: "적재 로그", value: counts[3], valueClass: "text-[#16a453]" },
  ];
  return (
    <section className="min-w-0 rounded-[20px] border border-[#e1e8ef] bg-white p-6 shadow-sm max-[560px]:rounded-2xl max-[560px]:p-5">
      <div className="flex items-center justify-between gap-3">
        <h2 className="m-0 text-[17px] font-bold text-[#566477]">작업 로그 요약</h2>
        <span className="whitespace-nowrap text-sm text-[#97a3b4]">최근 이벤트</span>
      </div>

      <div className="mt-5 grid grid-cols-4 gap-3 max-[560px]:gap-2">
        {summary.map((item) => (
          <div key={item.label} className="min-w-0 rounded-[14px] bg-[#f6f8fa] px-[17px] py-4 max-[560px]:px-3 max-[560px]:py-3">
            <span className="block text-sm text-[#8794a6]">{item.label}</span>
            <strong className={`mt-1 block text-[31px] leading-[1.15] max-[560px]:text-2xl ${item.valueClass}`}>{item.value}</strong>
          </div>
        ))}
      </div>

      {error && <p role="alert" className="mt-3 text-sm text-[#c9232b]">{error}</p>}
      {loading && <p className="mt-3 text-sm text-[#8794a6]">로그 불러오는 중...</p>}
      {!loading && !error && events.length === 0 && <p className="mt-3 text-sm text-[#8794a6]">아직 저장된 작업 로그가 없습니다.</p>}
      <ul className="m-0 mt-[17px] list-none p-0">
        {events.map((event) => (
          <li key={event.id} className="grid min-h-[37px] grid-cols-[72px_64px_minmax(0,1fr)] items-center gap-[10px] border-t border-[#e8edf2] max-[560px]:grid-cols-[60px_56px_minmax(0,1fr)] max-[560px]:gap-[7px]">
            <time className="text-sm tabular-nums text-[#98a4b4] max-[560px]:text-xs">{new Date(event.timestamp).toLocaleTimeString("ko-KR", { timeZone: "Asia/Seoul", hour12: false })}</time>
            <span className={`w-fit rounded-full px-[9px] py-1 text-xs font-bold ${/오류|정지/.test(event.state) ? "bg-[#fff0f0] text-[#c9232b]" : "bg-[#e1f6f2] text-[#128b81]"}`}>{event.state}</span>
            <p title={event.message} className="m-0 overflow-hidden text-ellipsis whitespace-nowrap text-sm text-[#526073] max-[560px]:text-xs">{event.message}</p>
          </li>
        ))}
      </ul>
    </section>
  );
}

