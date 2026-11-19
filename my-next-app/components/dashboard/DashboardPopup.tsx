"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import Header from "@/components/common/Header";
import DashboardGrid from "./DashboardGrid";
import type { WorkPhase } from "./types";

const PRESTART_SECONDS = 10 * 60;
const ROBOT_API_URL = (process.env.NEXT_PUBLIC_ROBOT_API_URL || "http://localhost:8000").replace(/\/$/, "");

type RobotStatus = {
  mode: "mock";
  state: "idle" | "running" | "stopped";
  last_command: "none" | "start" | "stop";
  updated_at: string;
};

async function getRobotStatus(): Promise<RobotStatus> {
  const response = await fetch(`${ROBOT_API_URL}/robot/status`, { cache: "no-store" });
  if (!response.ok) throw new Error(`상태 조회 실패 (${response.status})`);
  return response.json() as Promise<RobotStatus>;
}

async function sendRobotCommand(command: "start" | "stop"): Promise<RobotStatus> {
  const response = await fetch(`${ROBOT_API_URL}/robot/${command}`, { method: "POST" });
  if (!response.ok) throw new Error(`명령 전송 실패 (${response.status})`);
  return response.json() as Promise<RobotStatus>;
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
  const [phase, setPhase] = useState<WorkPhase>("idle");
  const [startPopupOpen, setStartPopupOpen] = useState(false);
  const [emergencyConfirmOpen, setEmergencyConfirmOpen] = useState(false);
  const [startingInventory, setStartingInventory] = useState<InventoryCounts>({ a: 0, b: 0, c: 0, d: 0, unclassified: 0 });
  const [countdownSeconds, setCountdownSeconds] = useState(PRESTART_SECONDS);
  const [elapsedSeconds, setElapsedSeconds] = useState(0);
  const [startDeadline, setStartDeadline] = useState<number | null>(null);
  const [workStartedAt, setWorkStartedAt] = useState<number | null>(null);
  const [apiConnected, setApiConnected] = useState(false);
  const [apiMessage, setApiMessage] = useState("FastAPI 연결 확인 중");
  const startRequestSent = useRef(false);
  const totalStartingInventory = Object.values(startingInventory).reduce((total, count) => total + count, 0);

  const checkRobotConnection = useCallback(async () => {
    setApiMessage("FastAPI 연결 확인 중");
    try {
      const status = await getRobotStatus();
      setApiConnected(true);
      setApiMessage(`현재 상태: ${status.state} (${status.mode})`);
    } catch (error) {
      setApiConnected(false);
      setApiMessage(error instanceof Error ? error.message : "FastAPI에 연결할 수 없습니다.");
    }
  }, []);

  useEffect(() => {
    void checkRobotConnection();
  }, [checkRobotConnection]);

  const startRobot = useCallback(async () => {
    try {
      const status = await sendRobotCommand("start");
      setApiConnected(true);
      setApiMessage(`API 응답: ${status.state} (${status.mode})`);
      setWorkStartedAt(Date.now());
      setPhase("running");
    } catch (error) {
      setApiConnected(false);
      setApiMessage(error instanceof Error ? error.message : "시작 명령에 실패했습니다.");
      setPhase("idle");
      setStartDeadline(null);
    }
  }, []);

  useEffect(() => {
    const updateClock = () => {
      const now = new Date();
      setClock({
        date: new Intl.DateTimeFormat("ko-KR", {
          timeZone: "Asia/Seoul",
          year: "numeric",
          month: "long",
          day: "numeric",
          weekday: "long",
        }).format(now),
        time: new Intl.DateTimeFormat("ko-KR", {
          timeZone: "Asia/Seoul",
          hour: "2-digit",
          minute: "2-digit",
          second: "2-digit",
          hour12: false,
        }).format(now),
      });
    };

    updateClock();
    const interval = window.setInterval(updateClock, 1000);
    return () => window.clearInterval(interval);
  }, []);

  useEffect(() => {
    if (phase !== "countdown" || startDeadline === null) return;

    const updateCountdown = () => {
      const secondsLeft = Math.max(0, Math.ceil((startDeadline - Date.now()) / 1000));
      setCountdownSeconds(secondsLeft);

      if (secondsLeft === 0) {
        if (!startRequestSent.current) {
          startRequestSent.current = true;
          void startRobot();
        }
      }
    };

    updateCountdown();
    const interval = window.setInterval(updateCountdown, 250);
    return () => window.clearInterval(interval);
  }, [phase, startDeadline, startRobot]);

  useEffect(() => {
    if (phase !== "running" || workStartedAt === null) return;

    const updateElapsed = () => setElapsedSeconds(Math.floor((Date.now() - workStartedAt) / 1000));
    updateElapsed();
    const interval = window.setInterval(updateElapsed, 1000);
    return () => window.clearInterval(interval);
  }, [phase, workStartedAt]);

  const updateStartingInventory = (key: InventoryKey, change: number, max?: number) => {
    setStartingInventory((current) => ({
      ...current,
      [key]: Math.max(0, Math.min(max ?? Number.POSITIVE_INFINITY, current[key] + change)),
    }));
  };

  const clearStartingInventory = () => {
    setStartingInventory({ a: 0, b: 0, c: 0, d: 0, unclassified: 0 });
  };

  const startWithPrecheck = () => {
    const deadline = Date.now() + PRESTART_SECONDS * 1000;
    setStartDeadline(deadline);
    setCountdownSeconds(PRESTART_SECONDS);
    setElapsedSeconds(0);
    startRequestSent.current = false;
    setApiMessage("카운트다운 중 · 완료 후 시작 명령을 전송합니다.");
    setStartPopupOpen(false);
    setPhase("countdown");
  };

  const startImmediately = () => {
    if (startRequestSent.current) return;
    startRequestSent.current = true;
    setStartDeadline(null);
    setCountdownSeconds(0);
    setApiMessage("즉시 시작 명령 전송 중…");
    void startRobot();
  };

  const confirmEmergencyStop = async () => {
    setApiMessage("정지 명령 전송 중…");
    try {
      const status = await sendRobotCommand("stop");
      setApiConnected(true);
      setApiMessage(`API 응답: ${status.state} (${status.mode})`);
    } catch (error) {
      setApiConnected(false);
      setApiMessage(error instanceof Error ? error.message : "정지 명령에 실패했습니다.");
      setEmergencyConfirmOpen(false);
      return;
    }
    if (phase === "running" && workStartedAt !== null) {
      setElapsedSeconds(Math.floor((Date.now() - workStartedAt) / 1000));
    }
    setStartPopupOpen(false);
    setEmergencyConfirmOpen(false);
    setStartDeadline(null);
    setWorkStartedAt(null);
    setPhase("emergency");
  };

  const recoverFromEmergency = () => {
    setStartDeadline(null);
    setWorkStartedAt(null);
    setCountdownSeconds(PRESTART_SECONDS);
    setElapsedSeconds(0);
    setPhase("idle");
  };

  return (
    <>
      <Header
        dateLabel={clock.date || "날짜 확인 중"}
        timeLabel={clock.time || "--:--:--"}
        startDisabled={phase !== "idle" || !apiConnected}
        stopDisabled={phase === "emergency"}
        apiConnected={apiConnected}
        onStartClick={() => setStartPopupOpen(true)}
        onStopClick={() => setEmergencyConfirmOpen(true)}
      />
      <p role="status" aria-live="polite" className={`mt-2 text-sm ${apiConnected ? "text-[#147a43]" : "text-[#c9232b]"}`}>
        {apiMessage}
        {!apiConnected && (
          <button className="ml-3 underline underline-offset-2" type="button" onClick={() => void checkRobotConnection()}>
            연결 다시 확인
          </button>
        )}
      </p>
      <DashboardGrid
        phase={phase}
        countdownSeconds={countdownSeconds}
        elapsedSeconds={elapsedSeconds}
        onRecover={recoverFromEmergency}
        onStartImmediately={startImmediately}
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

            <p className="mb-0 mt-6 text-[17px] leading-7 text-[#202936] max-[520px]:mt-5 max-[520px]:text-sm">
              지금 쌓여 있는 재고 <span className="text-[#667386]">(실제와 다르면 맞춰 주세요 — 적재하면 실제로 잰 값으로 바뀜)</span>
            </p>

            <div className="mt-2 grid gap-2">
              {inventoryAreas.map((area) => (
                <div key={area.key} className="grid grid-cols-[90px_58px_42px_58px_52px] items-center gap-2 max-[520px]:grid-cols-[minmax(58px,1fr)_42px_28px_42px_auto] max-[520px]:gap-1.5">
                  <span className={`text-[18px] font-bold max-[520px]:text-base ${area.color}`}>{area.label}</span>
                  <button
                    aria-label={`${area.label} 수량 감소`}
                    className="grid h-[42px] w-[58px] place-items-center border border-[#d9dce1] bg-[#f5f6f8] text-2xl font-bold text-[#202936] disabled:opacity-40 max-[520px]:h-10 max-[520px]:w-[42px]"
                    type="button"
                    onClick={() => updateStartingInventory(area.key, -1, area.max)}
                    disabled={startingInventory[area.key] === 0}
                  >−</button>
                  <strong aria-live="polite" className="text-center text-[28px] leading-none text-[#202936] max-[520px]:text-2xl">{startingInventory[area.key]}</strong>
                  <button
                    aria-label={`${area.label} 수량 증가`}
                    className="grid h-[42px] w-[58px] place-items-center border border-[#d9dce1] bg-[#f5f6f8] text-2xl font-bold text-[#202936] disabled:opacity-40 max-[520px]:h-10 max-[520px]:w-[42px]"
                    type="button"
                    onClick={() => updateStartingInventory(area.key, 1, area.max)}
                    disabled={area.max !== undefined && startingInventory[area.key] >= area.max}
                  >+</button>
                  <span className="whitespace-nowrap text-base text-[#8b96a6]">{area.max ? `/ ${area.max}` : ""}</span>
                </div>
              ))}
            </div>

            <button
              className="mt-3 border border-[#d9dce1] bg-[#f5f6f8] px-2 py-1 text-sm font-semibold text-[#202936] hover:bg-[#eceff2]"
              type="button"
              onClick={clearStartingInventory}
              aria-label="모든 구역의 재고 수량을 0으로 초기화"
            >전부 {totalStartingInventory}</button>

            <div className="mt-5 flex justify-end gap-2">
              <button
                className="bg-[#117d75] px-4 py-2.5 text-[18px] font-bold text-white hover:bg-[#0e6e67] max-[520px]:text-base"
                type="button"
                onClick={startWithPrecheck}
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
              모의 정지 명령을 전송합니다. 현재 API는 실제 로봇을 제어하거나 비상 정지하지 않습니다.
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
                onClick={confirmEmergencyStop}
              >정지</button>
            </div>
          </section>
        </div>
      )}
    </>
  );
}
