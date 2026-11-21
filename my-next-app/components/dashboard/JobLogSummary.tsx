"use client";

import { useEffect, useRef, useState } from "react";
import { createSupabaseClient } from "@/lib/supabase/client";

type LogEvent = { id: number | string; timestamp: string; job_id: string; message: string; state: string };

const PAGE_SIZE = 50;

function stateBadgeClass(state: string) {
  if (/오류|정지/.test(state)) return "bg-[#fff0f0] text-[#c9232b]";
  if (/탐지|스캔/.test(state)) return "bg-[#fff4cc] text-[#a66a00]";
  if (/판별/.test(state)) return "bg-[#e8f0ff] text-[#2563eb]";
  if (/적재/.test(state)) return "bg-[#e7f8ef] text-[#147a43]";
  return "bg-[#e1f6f2] text-[#128b81]";
}

function mergeLogs(current: LogEvent[], incoming: LogEvent[]) {
  const rows = new Map(current.map(row => [String(row.id), row]));
  incoming.forEach(row => rows.set(String(row.id), row));
  return [...rows.values()].sort((a, b) => {
    const time = Date.parse(b.timestamp) - Date.parse(a.timestamp);
    if (time) return time;
    return BigInt(a.id) === BigInt(b.id) ? 0 : BigInt(a.id) > BigInt(b.id) ? -1 : 1;
  });
}

export default function JobLogSummary() {
  const [events, setEvents] = useState<LogEvent[]>([]);
  const [counts, setCounts] = useState([0, 0, 0, 0]);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [loadingOlder, setLoadingOlder] = useState(false);
  const [hasMore, setHasMore] = useState(false);
  const [olderError, setOlderError] = useState<string | null>(null);
  const loadOlder = useRef<(() => Promise<void>) | null>(null);
  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    const controller = new AbortController();
    let cursor: LogEvent | undefined;
    let more = false;
    let fetchingOlder = false;
    const query = () => createSupabaseClient().from("log")
      .select("id,timestamp,job_id,message,state")
      .order("timestamp", { ascending: false }).order("id", { ascending: false });
    loadOlder.current = async () => {
      if (!cursor || !more || fetchingOlder || !active) return;
      fetchingOlder = true;
      setLoadingOlder(true);
      setOlderError(null);
      try {
        const { data, error: queryError } = await query()
          .or(`timestamp.lt.${cursor.timestamp},and(timestamp.eq.${cursor.timestamp},id.lt.${cursor.id})`)
          .limit(PAGE_SIZE).abortSignal(AbortSignal.any([controller.signal, AbortSignal.timeout(8000)]));
        if (queryError) throw queryError;
        if (!active) return;
        const rows = (data || []) as LogEvent[];
        if (rows.length) cursor = rows[rows.length - 1];
        more = rows.length === PAGE_SIZE;
        setHasMore(more);
        setEvents(current => mergeLogs(current, rows));
      } catch (cause) {
        if (active) setOlderError(cause instanceof Error ? cause.message : "이전 로그를 불러오지 못했습니다.");
      } finally {
        fetchingOlder = false;
        if (active) setLoadingOlder(false);
      }
    };
    const poll = async () => {
      try {
        const signal = AbortSignal.any([controller.signal, AbortSignal.timeout(8000)]);
        const db = createSupabaseClient();
        const base = () => db.from("log").select("timestamp", { count: "exact", head: true });
        const results = await Promise.all([
          query().limit(PAGE_SIZE).abortSignal(signal),
          base().abortSignal(signal),
          base().like("state", "스캔%").abortSignal(signal),
          base().like("state", "판별%").abortSignal(signal),
          base().like("state", "적재%").abortSignal(signal),
        ]);
        const failed = results.find(result => result.error);
        if (failed?.error) throw failed.error;
        if (active) {
          const rows = (results[0].data || []) as unknown as LogEvent[];
          if (!cursor && rows.length) {
            cursor = rows[rows.length - 1];
            more = rows.length === PAGE_SIZE;
            setHasMore(more);
          }
          setEvents(current => mergeLogs(current, rows));
          setCounts(results.slice(1).map(result => result.count || 0));
          setError(null);
        }
      } catch (cause) {
        if (active) setError(cause instanceof Error ? cause.message : "로그 DB를 조회하지 못했습니다.");
      } finally {
        if (active) { setLoading(false); timer = setTimeout(poll, 1000); }
      }
    };
    void poll();
    return () => { active = false; controller.abort(); clearTimeout(timer); loadOlder.current = null; };
  }, []);
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
      <div
        role="region" aria-label="전체 작업 로그" tabIndex={0}
        className="mt-[17px] h-[clamp(200px,32vh,340px)] min-h-0 overflow-y-auto overscroll-contain pr-2 [scrollbar-gutter:stable] focus-visible:rounded-lg focus-visible:outline-2 focus-visible:outline-[#117d75]"
        onScroll={event => {
          const element = event.currentTarget;
          if (element.scrollHeight - element.scrollTop - element.clientHeight < 80) void loadOlder.current?.();
        }}
      >
      <ul className="m-0 list-none p-0">
        {events.map((event) => (
          <li key={event.id} className="grid min-h-[44px] py-2 grid-cols-[72px_64px_minmax(0,1fr)] items-center gap-[10px] border-t border-[#e8edf2] max-[560px]:grid-cols-[60px_56px_minmax(0,1fr)] max-[560px]:gap-[7px]">
            <time className="text-sm tabular-nums text-[#98a4b4] max-[560px]:text-xs">{new Date(event.timestamp).toLocaleTimeString("ko-KR", { timeZone: "Asia/Seoul", hour12: false })}</time>
            <span className={`w-fit rounded-full px-[9px] py-1 text-xs font-bold ${stateBadgeClass(event.state)}`}>{event.state}</span>
            <p title={event.message} className="m-0 min-w-0 break-words text-sm leading-relaxed text-[#526073] [overflow-wrap:anywhere] max-[560px]:text-xs">{event.message}</p>
          </li>
        ))}
      </ul>
      {olderError && <p role="alert" className="text-xs text-[#c9232b]">{olderError}</p>}
      {hasMore && <button type="button" disabled={loadingOlder} onClick={() => void loadOlder.current?.()} className="my-3 w-full rounded-lg bg-[#f2f5f8] px-3 py-2 text-sm text-[#586577] disabled:opacity-50">{loadingOlder ? "이전 로그 불러오는 중..." : "이전 로그 더 보기"}</button>}
      {!loading && !hasMore && events.length > 0 && <p className="my-3 text-center text-xs text-[#8794a6]">모든 로그를 불러왔습니다.</p>}
      </div>
    </section>
  );
}

