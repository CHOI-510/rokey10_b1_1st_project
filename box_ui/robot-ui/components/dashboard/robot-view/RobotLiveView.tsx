"use client";
import { useEffect, useRef, useState } from "react";
import { createSupabaseClient } from "@/lib/supabase/client";
import { useRobotTelemetry } from "./useRobotTelemetry";
import RobotScene from "./RobotScene";
import { demoFrame, HOME, ZONES, type RobotTelemetry } from "./model";

import { workspaceFromLocations, type LocationRow } from "./workspace";

type Props = { telemetry?: RobotTelemetry; stage?: string };
const ROBOT_API_URL = (process.env.NEXT_PUBLIC_ROBOT_API_URL || "http://localhost:8000").replace(/\/$/, "");
const controlClass = "min-h-9 rounded-lg border border-[#dce5eb] bg-white px-3 text-sm font-medium text-[#526073] transition-colors hover:bg-[#f0f6f5] focus-visible:outline-2 focus-visible:outline-[#117d75] disabled:cursor-not-allowed disabled:opacity-40";
export default function RobotLiveView({ telemetry: providedTelemetry, stage }: Props) {
  const stream = useRobotTelemetry(providedTelemetry === undefined);
  const telemetry = providedTelemetry || stream.telemetry;
  const [demo, setDemo] = useState(false);
  const [playing, setPlaying] = useState(false);
  const [seconds, setSeconds] = useState(0);
  const [yaw, setYaw] = useState(-35);
  const [elevation, setElevation] = useState(32);
  const [zoom, setZoom] = useState(1);
  const [now, setNow] = useState(0);
  const [counts, setCounts] = useState<Record<string, number>>({ A:0, B:0, C:0, D:0, U:0 });
  const [inventoryStatus, setInventoryStatus] = useState("재고 조회 중");
  const [workspace, setWorkspace] = useState(() => workspaceFromLocations([]));
  const [locationStatus, setLocationStatus] = useState("설정 기본 좌표 · DB 조회 중");
  const previousTick = useRef(0);
  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    const abort = new AbortController();
    const poll = async () => {
      try {
        const { data, error } = await createSupabaseClient().from("inventory").select("loc_id,cnt")
          .abortSignal(AbortSignal.any([abort.signal, AbortSignal.timeout(8000)]));
        if (error) throw error;
        if (!active) return;
        const stock: Record<string, number> = { A:0, B:0, C:0, D:0, U:0 };
        for (const row of data || []) {
          const key = String(row.loc_id || "").trim().toUpperCase();
          const count = Number(row.cnt);
          if (ZONES.some(zone=>zone.id===key) && Number.isFinite(count)) stock[key] += Math.max(0,Math.floor(count));
        }
        setCounts(stock);
        setInventoryStatus("Supabase 재고");
      } catch {
        if (active) setInventoryStatus("재고 연결 확인 필요");
      } finally {
        if (active) timer = setTimeout(poll, 1000);
      }
    };
    void poll();
    return () => { active=false; abort.abort(); clearTimeout(timer); };
  }, []);
  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    const abort = new AbortController();
    async function poll() {
      try {
        const response = await fetch(`${ROBOT_API_URL}/robot/locations`, {
          cache: "no-store", signal: AbortSignal.any([abort.signal, AbortSignal.timeout(8000)]),
        });
        if (!response.ok) throw new Error("loc API 조회 실패");
        const data: LocationRow[] = await response.json();
        if (!Array.isArray(data)) throw new Error("loc 응답 형식 오류");
        if (!active) return;
        if (!data?.length) {
          setLocationStatus("loc API 조회 결과 없음 · 마지막 좌표/기본값 표시");
          return;
        }
        const next = workspaceFromLocations(data as LocationRow[]);
        setWorkspace(next);
        setLocationStatus(next.issues.length ? `DB 좌표 확인 필요 · ${next.issues.join(" / ")}` : Object.values(next.sources).every(source => source === "DB")
          ? "Supabase loc 좌표" : "DB 좌표 · 일부 구역 기본값/미설정");
      } catch {
        if (active) setLocationStatus("좌표 DB 연결 확인 필요 · 마지막 좌표/기본값 표시");
      } finally {
        if (active) timer = setTimeout(poll, 5000);
      }
    }
    void poll();
    return () => { active = false; abort.abort(); clearTimeout(timer); };
  }, []);
  useEffect(() => {
    previousTick.current = Date.now();
    const timer = setInterval(() => {
      const tick = Date.now();
      const delta = Math.min((tick-previousTick.current)/1000, .1);
      previousTick.current = tick;
      setNow(tick);
      if (demo && playing) setSeconds(value=>value+delta);
    }, demo && playing ? 50 : 500);
    return () => clearInterval(timer);
  }, [demo, playing]);
  const validTelemetry = telemetry && telemetry.jointsDeg.length===6 && telemetry.jointsDeg.every(Number.isFinite);
  const validTcp = telemetry?.tcpMm?.length === 3 && telemetry.tcpMm.every(Number.isFinite);
  const live = Boolean(validTelemetry && telemetry!.receivedAt > 0 && Math.max(0, now-telemetry!.receivedAt)<2000);
  const frame = demoFrame(seconds);
  const joints = demo ? frame.joints : validTelemetry ? telemetry!.jointsDeg : HOME;
  const status = demo ? "데모 · 실제 로봇과 무관" : live ? "실측 데이터 수신 중" : telemetry ? "실측 데이터 끊김" : "실측 데이터 연결 대기";
  const label = demo ? frame.label : live ? (stage || "실측 자세 표시") : "관절 데이터가 연결되면 실제 자세를 표시합니다.";
  return (
    <section className="flex min-w-0 flex-col rounded-[20px] border border-[#e1e8ef] bg-white p-6 shadow-sm max-[560px]:rounded-2xl max-[560px]:p-5">
      <div className="flex flex-wrap items-start justify-between gap-3">
        <div>
          <h2 className="m-0 text-[17px] font-bold text-[#566477]">적재 현황 · 로봇 뷰</h2>
          <p className="mb-0 mt-1 text-xs text-[#8794a6]">Doosan M0609 · OnRobot RG2</p>
        </div>
        <span role="status" className={`rounded-full px-3 py-1.5 text-xs font-semibold ${demo ? "bg-[#fff4da] text-[#a2660a]" : live ? "bg-[#e7f8ef] text-[#147a43]" : "bg-[#f1f5f8] text-[#718096]"}`}>{status}</span>
      </div>
      <div className="mt-4 flex flex-wrap items-center justify-between gap-2">
        <div className="flex rounded-xl bg-[#f1f5f7] p-1" aria-label="로봇 표시 모드">
          <button type="button" aria-pressed={!demo} className={`rounded-lg px-3 py-1.5 text-sm font-semibold ${!demo ? "bg-white text-[#117d75] shadow-sm" : "text-[#8794a6]"}`} onClick={()=>{setDemo(false);setPlaying(false);}}>실시간</button>
          <button type="button" aria-pressed={demo} className={`rounded-lg px-3 py-1.5 text-sm font-semibold ${demo ? "bg-white text-[#ad770d] shadow-sm" : "text-[#8794a6]"}`} onClick={()=>{setDemo(true);setSeconds(0);setPlaying(true);}}>데모 미리보기</button>
        </div>
        <span className="text-xs text-[#8794a6]">{inventoryStatus}</span>
      </div>
      <div className="relative mt-3 h-[340px] min-w-0 overflow-hidden rounded-xl border border-[#e7edf1] bg-[#fbfcfd] sm:h-[380px]">
        <RobotScene workspace={workspace} joints={joints} counts={counts} yaw={yaw} elevation={elevation} zoom={zoom}
          gripperClosed={demo ? frame.holding : live ? telemetry?.gripperClosed : undefined}
          holding={demo ? frame.holding : Boolean(live && telemetry?.gripperClosed)} muted={!demo&&!live}
          target={demo ? "B" : live ? telemetry?.target : undefined}
          tcp={validTcp&&!demo ? telemetry?.tcpMm : undefined} onRotate={setYaw}/>
        {!demo&&!live && <span className="pointer-events-none absolute left-3 top-3 max-w-[85%] rounded-lg bg-white/90 px-3 py-2 text-xs text-[#7b8999] shadow-sm">{telemetry ? "마지막 수신 자세 · 자동 추정 이동 없음" : "대기 자세 예시 · 실제 로봇 위치가 아닙니다"}</span>}
        {demo && <span className="absolute left-3 top-3 rounded-lg bg-[#fff4da] px-3 py-2 text-xs font-semibold text-[#a2660a]">UI 데모 · 동작 명령 전송 없음</span>}
      </div>
      <div className="mt-3 flex flex-wrap items-center justify-between gap-2">
        <div className="flex flex-wrap gap-1.5" aria-label="화면 시점 조절">
          <button type="button" aria-label="시점 왼쪽 회전" className={controlClass} onClick={()=>setYaw(v=>v-15)}>좌회전</button>
          <button type="button" aria-label="시점 오른쪽 회전" className={controlClass} onClick={()=>setYaw(v=>v+15)}>우회전</button>
          <button type="button" aria-label="확대" className={controlClass} disabled={zoom>=1.5} onClick={()=>setZoom(v=>Math.min(1.5,v+.1))}>+</button>
          <button type="button" aria-label="축소" className={controlClass} disabled={zoom<=.5} onClick={()=>setZoom(v=>Math.max(.5,v-.1))}>−</button>
          <button type="button" className={controlClass} onClick={()=>setElevation(v=>v===32?80:32)}>{elevation===32?"위에서 보기":"입체 보기"}</button>
          <button type="button" className={controlClass} onClick={()=>{setYaw(-35);setElevation(32);setZoom(1);}}>시점 초기화</button>
        </div>
        {demo && <button type="button" className="min-h-9 rounded-lg bg-[#117d75] px-4 text-sm font-bold text-white hover:bg-[#0c6c65]" onClick={()=>setPlaying(v=>!v)}>{playing ? "데모 일시정지" : "데모 재생"}</button>}
      </div>
      <div className="mt-4 rounded-xl bg-[#f5f8fa] px-4 py-3">
        <p className="m-0 text-sm font-semibold text-[#526073]">{label}</p>
        {live&&!demo&&validTcp ? <p className="mb-0 mt-1 text-xs tabular-nums text-[#8794a6]">TCP · X {telemetry!.tcpMm![0].toFixed(1)} / Y {telemetry!.tcpMm![1].toFixed(1)} / Z {telemetry!.tcpMm![2].toFixed(1)} mm</p>
          : <p className="mb-0 mt-1 text-xs text-[#8794a6]">{demo?"데모는 미리 정한 관절 경로를 재생합니다. 실제 궤적이 아닙니다.":stream.message}</p>}
      </div>
      {/* {live && !demo && <p className="mb-0 mt-2 break-words text-xs tabular-nums text-[#8794a6]">관절각 · {telemetry!.jointsDeg.map((angle,index) => `J${index+1} ${angle.toFixed(1)}°`).join(" / ")}</p>} */}
      <div className="mt-3 flex flex-wrap items-center gap-x-4 gap-y-2">
        {ZONES.map(zone=><span key={zone.id} className="flex items-center gap-1.5 text-xs text-[#6b788a]"><i className="h-2.5 w-2.5 rounded-[3px]" style={{backgroundColor:zone.color}}/>{zone.id} · {counts[zone.id]||0}개</span>)}
      </div>
      <details className="mt-3 min-w-0 rounded-xl border border-[#e1e8ef] p-3 text-xs text-[#6b788a]">
        <summary className="cursor-pointer font-semibold">구역 좌표 · BASE / mm · {locationStatus}</summary>
        <div className="mt-3 grid min-w-0 grid-cols-1 gap-2 sm:grid-cols-2">
          <p className="m-0 break-words">판별존 · ({workspace.measure.join(", ")}) · {workspace.sources.measure_zone}</p>
          {workspace.zones.map(zone => <p key={zone.id} className="m-0 break-words">{zone.id === "U" ? "미분류 투하 TCP" : `${zone.id} 첫 자리 XY`} · ({(zone.id === "U" ? zone.xyz : zone.xyz.slice(0,2)).join(", ")}) · {workspace.sources[zone.id]}</p>)}
          <p className="m-0">탐지 Y 경로 · X 270~570 / Y -115~265 / Z 40</p>
          <p className="m-0">탐지 X 확인 · X 230~570 / Y -65~225 / Z 35</p>
        </div>
        <p className="mb-0 mt-3 leading-relaxed">A·B 두 번째 자리는 Y −45mm, C·D는 Y −30mm입니다. 미분류는 바닥 위치가 아닌 투하 TCP 좌표입니다. 테두리는 실제 작업대 경계를 뜻하지 않습니다.</p>
      </details>
      <p className="mb-0 mt-3 text-[11px] leading-relaxed text-[#9aa5b4]">로봇 본체는 M0609 URDF 외형을 사용합니다. 그리퍼는 OnRobot RG2 모델입니다. 실측 개방 폭 미수신 시 열린 자세 예시로 표시합니다. 데모 개폐는 예시입니다. WebGL 미지원 시 경량 SVG 모델을 표시하며, 모델 로딩 실패 시 관절 개념도를 표시합니다. 구역 중심·탐색 경로는 BASE 좌표를 사용합니다. 구역 테두리 크기는 개념 형상입니다. A~D는 2자리 × 3층까지 표현합니다.</p>
    </section>
  );
}
