"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import Header from "@/components/common/Header";
import DashboardGrid from "./DashboardGrid";
import type { WorkPhase } from "./types";

const ROBOT_API_URL = (process.env.NEXT_PUBLIC_ROBOT_API_URL || "http://localhost:8000").replace(/\/$/, "");

type RobotStatus = {
  mode: "mock" | "box" | "rosbridge";
  state: "idle" | "countdown" | "running" | "stopped" | "error" | "unknown";
  connected: boolean;
  robot_connected: boolean;
  can_start: boolean;
  stage: string;
  message: string;
  start_at: string | null;
  started_at: string | null;
  ended_at: string | null;
  job_id: string;
  stop: string[] | null;
  box_seq: number;
};

async function requestRobot(command?: "start" | "stop" | "cancel" | "start_now"): Promise<RobotStatus> {
  const response = await fetch(`${ROBOT_API_URL}/robot/${command || "status"}`, {
    method: command ? "POST" : "GET", cache: "no-store",
    signal: AbortSignal.timeout(12000),
  });
  const body = await response.json();
  if (!response.ok) throw new Error(body.detail || `로봇 API 오류 (${response.status})`);
  return body as RobotStatus;
}

type InventoryKey = "a" | "b" | "c" | "d" | "unclassified";
type InventoryCounts = Record<InventoryKey, number>;

const inventoryAreas: { key: InventoryKey; label: string; color: string; max?: number }[] = [
  { key: "a", label: "A 구역", color: "text-[#21b4a5]", max: 6 },
  { key: "b", label: "B 구역", color: "text-[#3d82f6]", max: 6 },
  { key: "c", label: "C 구역", color: "text-[#f59a08]", max: 6 },
  { key: "d", label: "D 구역", color: "text-[#8b5cf6]", max: 6 },
  { key: "unclassified", label: "미분류", color: "text-[#9aa4b2]" },
];

export default function DashboardPopup() {
  const [clock, setClock] = useState({ date: "", time: "" });
  const [now, setNow] = useState(0);
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [startPopupOpen, setStartPopupOpen] = useState(false);
  const [emergencyConfirmOpen, setEmergencyConfirmOpen] = useState(false);
  const [dbConnected, setDbConnected] = useState(false);
  const [apiConnected, setApiConnected] = useState(false);
  const [apiMessage, setApiMessage] = useState("로봇 연결 확인 중");
  const [commandMessage, setCommandMessage] = useState("");
  const [pending, setPending] = useState(false);
  const [immediateStartRequested, setImmediateStartRequested] = useState(false);
  const immediateStartInFlight = useRef(false);
  const [stopAcknowledged, setStopAcknowledged] = useState(false);
  const [startingInventory, setStartingInventory] = useState<InventoryCounts>({ a: 0, b: 0, c: 0, d: 0, unclassified: 0 });
  const [inventoryLoaded, setInventoryLoaded] = useState(false);
  const updateStartingInventory = (key: InventoryKey, delta: number, max?: number) => {
    setStartingInventory(current => ({ ...current, [key]: Math.min(max ?? Infinity, Math.max(0, current[key] + delta)) }));
  };
  const clearStartingInventory = () => setStartingInventory({ a: 0, b: 0, c: 0, d: 0, unclassified: 0 });
  const openStartPopup = async () => {
    setInventoryLoaded(false);
    setStartPopupOpen(true);
    try {
      const response = await fetch(`${ROBOT_API_URL}/robot/inventory`, { cache: "no-store", signal: AbortSignal.timeout(12000) });
      const data = await response.json();
      if (!response.ok) throw new Error(data.detail || "재고 조회 실패");
      setStartingInventory({ a: data.a, b: data.b, c: data.c, d: data.d, unclassified: data.u });
      setInventoryLoaded(true);
    } catch (error) {
      setCommandMessage(error instanceof Error ? error.message : "재고 조회 실패");
    }
  };
  const commandInFlight = useRef(false);
  const requestVersion = useRef(0);

  const checkRobotConnection = useCallback(async () => {
    const version = requestVersion.current;
    try {
      const status = await requestRobot();
      if (version !== requestVersion.current) return;
      setRobotStatus(status);
      if (status.state !== "stopped") setStopAcknowledged(false);
      setApiConnected(true);
      setApiMessage(`${status.stage || status.state} (${status.mode})${status.message ? ` · ${status.message}` : ""}`);
    } catch (error) {
      if (version !== requestVersion.current) return;
      try {
        const health = await fetch(`${ROBOT_API_URL}/health`, { cache: "no-store", signal: AbortSignal.timeout(4000) });
        if (version !== requestVersion.current) return;
        setApiConnected(health.ok);
      } catch {
        if (version !== requestVersion.current) return;
        setApiConnected(false);
      }
      setRobotStatus(null);
      setApiMessage(error instanceof Error ? error.message : "로봇 연결 실패");
    }
  }, []);

  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      await checkRobotConnection();
      if (active) timer = setTimeout(poll, 250);
    };
    void poll();
    return () => { active = false; clearTimeout(timer); requestVersion.current += 1; };
  }, [checkRobotConnection]);

  const sendCommand = async (command: "start" | "stop" | "cancel" | "start_now") => {
    // Stop remains available while another request is waiting.
    if (command !== "stop" && commandInFlight.current) return;
    if (command === "start_now") {
      if (immediateStartInFlight.current) return;
      immediateStartInFlight.current = true;
      setImmediateStartRequested(true);
    }
    if (command === "start") {
      immediateStartInFlight.current = false;
      setImmediateStartRequested(false);
    }
    commandInFlight.current = true;
    setPending(true);
    setStopAcknowledged(false);
    requestVersion.current += 1;
    const version = requestVersion.current;
    setCommandMessage("명령 전송 중…");
    try {
      if (command === "start") {
        const { unclassified, ...counts } = startingInventory;
        const response = await fetch(`${ROBOT_API_URL}/robot/inventory`, {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ ...counts, u: unclassified }), signal: AbortSignal.timeout(12000),
        });
        const data = await response.json();
        if (!response.ok || data.ok !== true) throw new Error(data.detail || data.message || "재고 설정 실패");
        // A stop pressed during inventory setup cancels this pending start.
        if (version !== requestVersion.current) return;
      }
      const status = await requestRobot(command);
      if (version !== requestVersion.current) return;
      setCommandMessage(status.message || "명령 접수");
      await checkRobotConnection();
    } catch (error) {
      if (version !== requestVersion.current) return;
      if (command === "start_now") {
        immediateStartInFlight.current = false;
        setImmediateStartRequested(false);
      }
      setCommandMessage(error instanceof Error ? error.message : "명령 전송 실패");
      await checkRobotConnection();
    } finally {
      if (version === requestVersion.current) {
        commandInFlight.current = false;
        setPending(false);
      }
    }
  };

  useEffect(() => {
    const tick = () => {
      const date = new Date();
      setNow(date.getTime());
      setClock({
        date: new Intl.DateTimeFormat("ko-KR", { timeZone: "Asia/Seoul", year: "numeric", month: "long", day: "numeric", weekday: "long" }).format(date),
        time: new Intl.DateTimeFormat("ko-KR", { timeZone: "Asia/Seoul", hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false }).format(date),
      });
    };
    tick();
    const timer = setInterval(tick, 1000);
    return () => clearInterval(timer);
  }, []);

  const phase: WorkPhase = !apiConnected || !robotStatus?.connected ? "unknown" : robotStatus?.state === "stopped" ? (stopAcknowledged && robotStatus.can_start ? "idle" : "emergency") : robotStatus?.state || "unknown";
  const countdownSeconds = robotStatus?.start_at ? Math.max(0, Math.ceil((Date.parse(robotStatus.start_at) - now) / 1000)) : 0;
  const elapsedSeconds = robotStatus?.started_at ? Math.max(0, Math.floor(((robotStatus.ended_at ? Date.parse(robotStatus.ended_at) : now) - Date.parse(robotStatus.started_at)) / 1000)) : 0;

  return (
    <>
      <Header
        dateLabel={clock.date || "날짜 확인 중"}
        timeLabel={clock.time || "--:--:--"}
        startDisabled={phase !== "idle" || !robotStatus?.can_start || !apiConnected || pending}
        stopDisabled={false}
        apiConnected={apiConnected}
        robotConnected={apiConnected && robotStatus?.robot_connected === true}
        dbConnected={dbConnected}
        onStartClick={() => void openStartPopup()}
        onStopClick={() => setEmergencyConfirmOpen(true)}
      />
      <p role="status" aria-live="polite" className={`mt-2 text-sm ${apiConnected ? "text-[#147a43]" : "text-[#c9232b]"}`}>
        {apiMessage}{commandMessage && ` · ${commandMessage}`}
        {!apiConnected && (
          <button className="ml-3 underline underline-offset-2" type="button" onClick={() => void checkRobotConnection()}>
            연결 다시 확인
          </button>
        )}
      </p>
      <DashboardGrid
        onDbConnectionChange={setDbConnected}
        phase={phase}
        countdownSeconds={countdownSeconds}
        elapsedSeconds={elapsedSeconds}
        onRecover={() => {
          if (apiConnected && robotStatus?.can_start && !pending) {
            setStopAcknowledged(true);
            setCommandMessage("");
          }
        }}
        onStartImmediately={() => {
          if (apiConnected && phase === "countdown" && !pending && (robotStatus?.stage === "STANDBY" || robotStatus?.mode === "mock")) void sendCommand("start_now");
        }}
        recoveryDisabled={!apiConnected || !robotStatus?.can_start || pending}
        immediateStartDisabled={!apiConnected || pending || immediateStartRequested || (robotStatus?.stage !== "STANDBY" && robotStatus?.mode !== "mock")}
        stage={robotStatus?.stage || ""}
        boxSeq={robotStatus?.box_seq || 0}
        jobId={robotStatus?.job_id || ""}
        stop={stopAcknowledged ? null : robotStatus?.stop ?? null}
        reason={stopAcknowledged ? "" : robotStatus?.message || ""}
        onCancel={() => void sendCommand("cancel")}
        commandPending={pending || immediateStartRequested}
      />

      {startPopupOpen && (
        <div className="fixed inset-0 z-50 grid place-items-center overflow-y-auto bg-[#111827]/45 p-4">
          <section
            aria-labelledby="start-popup-title"
            aria-modal="true"
            className="my-auto w-full max-w-[700px] rounded-2xl bg-white p-5 shadow-2xl sm:p-7"
            role="dialog"
          >
            <h2 id="start-popup-title" className="m-0 text-[24px] font-bold text-[#117d75] max-[520px]:text-xl">작업 시작</h2>
            <p className="mb-0 mt-3 text-[18px] font-semibold leading-7 text-[#202936] max-[520px]:text-base">작업을 시작하시겠습니까?</p>
            <p className="mb-0 mt-1 text-[18px] leading-7 text-[#202936] max-[520px]:text-base">사전 점검 후 카운트다운이 끝나면 로봇이 움직입니다.</p>

            <div className="mt-5 flex justify-end gap-2">
              <button
                className="bg-[#117d75] px-4 py-2.5 text-[18px] font-bold text-white hover:bg-[#0e6e67] max-[520px]:text-base"
                type="button"
                disabled={pending || !inventoryLoaded}
                onClick={() => { setStartPopupOpen(false); void sendCommand("start"); }}
              >시작</button>
              <button
                className="border border-[#d9dce1] bg-[#f5f6f8] px-4 py-2.5 text-[18px] font-bold text-[#202936] hover:bg-[#eceff2] max-[520px]:text-base"
                type="button"
                onClick={() => setStartPopupOpen(false)}
              >취소</button>
            </div>
          </section>
        </div>
      )}

      {emergencyConfirmOpen && (
        <div className="fixed inset-0 z-50 grid place-items-center bg-[#111827]/50 p-4">
          <section
            aria-labelledby="emergency-confirm-title"
            aria-modal="true"
            className="w-full max-w-[460px] rounded-2xl bg-white p-6 shadow-2xl sm:p-8"
            role="dialog"
          >
            <h2 id="emergency-confirm-title" className="m-0 text-xl font-bold text-[#c9232b]">비상 정지 확인</h2>
            <p className="mb-0 mt-4 leading-7 text-[#526073]">
              {robotStatus?.mode === "mock" ? "모의 정지 명령을 전송합니다." : "관제에 정지를 요청합니다. 실제 정지 여부를 확인하고, 응답이 없으면 현장의 펜던트 비상정지를 사용하세요."}
            </p>
            <div className="mt-7 flex justify-end gap-3">
              <button
                className="rounded-xl border border-[#d9e1e9] px-5 py-3 font-semibold text-[#586577] hover:bg-[#f6f8fa]"
                type="button"
                onClick={() => setEmergencyConfirmOpen(false)}
              >취소</button>
              <button
                className="rounded-xl bg-[#e52327] px-5 py-3 font-semibold text-white hover:bg-[#c91e22]"
                type="button"
                onClick={() => { setStartPopupOpen(false); setEmergencyConfirmOpen(false); void sendCommand("stop"); }}
              >정지</button>
            </div>
          </section>
        </div>
      )}
    </>
  );
}
