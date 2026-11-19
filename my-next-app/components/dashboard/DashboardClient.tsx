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
  if (!response.ok) throw new Error(`로봇 상태 조회 실패 (${response.status})`);
  return response.json() as Promise<RobotStatus>;
}

async function sendRobotCommand(command: "start" | "stop"): Promise<RobotStatus> {
  const response = await fetch(`${ROBOT_API_URL}/robot/${command}`, { method: "POST" });
  if (!response.ok) throw new Error(`명령 전송 실패 (${response.status})`);
  return response.json() as Promise<RobotStatus>;
}

export default function DashboardClient() {
  const [clock, setClock] = useState({ date: "", time: "" });
  const [phase, setPhase] = useState<WorkPhase>("idle");
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [emergencyConfirmOpen, setEmergencyConfirmOpen] = useState(false);
  const [countdownSeconds, setCountdownSeconds] = useState(PRESTART_SECONDS);
  const [elapsedSeconds, setElapsedSeconds] = useState(0);
  const [startDeadline, setStartDeadline] = useState<number | null>(null);
  const [workStartedAt, setWorkStartedAt] = useState<number | null>(null);
  const [apiConnected, setApiConnected] = useState(false);
  const [apiMessage, setApiMessage] = useState("FastAPI 연결 확인 중");
  const startRequestSent = useRef(false);

  const checkRobotConnection = async () => {
    setApiMessage("FastAPI 연결 확인 중");
    try {
      const status = await getRobotStatus();
      setApiConnected(true);
      setApiMessage(`현재 상태: ${status.state} (${status.mode})`);
    } catch (error) {
      setApiConnected(false);
      setApiMessage(error instanceof Error ? error.message : "FastAPI에 연결할 수 없습니다.");
    }
  };

  useEffect(() => {
    let active = true;
    getRobotStatus().then((status) => {
      if (active) {
        setApiConnected(true);
        setApiMessage(`초기 상태: ${status.state} (${status.mode})`);
      }
    }).catch((error: unknown) => {
      if (active) {
        setApiConnected(false);
        setApiMessage(error instanceof Error ? error.message : "FastAPI에 연결할 수 없습니다.");
      }
    });
    return () => {
      active = false;
    };
  }, []);

  const startRobot = useCallback(async () => {
    try {
      const status = await sendRobotCommand("start");
      setApiConnected(true);
      setApiMessage(`API 응답: ${status.state} (${status.mode})`);
      const startedAt = Date.now();
      setWorkStartedAt(startedAt);
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

  const confirmStart = () => {
    const deadline = Date.now() + PRESTART_SECONDS * 1000;
    setStartDeadline(deadline);
    setCountdownSeconds(PRESTART_SECONDS);
    setElapsedSeconds(0);
    startRequestSent.current = false;
    setApiMessage("카운트다운 중 · 완료 후 시작 명령을 전송합니다.");
    setConfirmOpen(false);
    setPhase("countdown");
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
    setConfirmOpen(false);
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
        onStartClick={() => setConfirmOpen(true)}
        onStopClick={() => setEmergencyConfirmOpen(true)}
      />
      <p role="status" aria-live="polite" className={`mt-2 text-sm ${apiConnected ? "text-[#147a43]" : "text-[#c9232b]"}`}>
        {apiMessage}
        {!apiConnected && (
          <button className="ml-3 underline underline-offset-2" type="button" onClick={checkRobotConnection}>
            연결 다시 확인
          </button>
        )}
      </p>
      <DashboardGrid
        phase={phase}
        countdownSeconds={countdownSeconds}
        elapsedSeconds={elapsedSeconds}
        onRecover={recoverFromEmergency}
      />

      {confirmOpen && (
        <div className="fixed inset-0 z-50 grid place-items-center bg-[#111827]/45 p-4">
          <section
            aria-labelledby="start-confirm-title"
            aria-modal="true"
            className="w-full max-w-[460px] rounded-2xl bg-white p-6 shadow-2xl sm:p-8"
            role="dialog"
          >
            <h2 id="start-confirm-title" className="m-0 text-xl font-bold text-[#202936]">작업 시작 전 확인</h2>
            <p className="mb-0 mt-4 leading-7 text-[#526073]">
              10분 후 작업이 시작됩니다. 작업공간 내 사람 또는 파손 위험이 있는 물품이 없는지 확인해주세요.
            </p>
            <div className="mt-7 flex justify-end gap-3">
              <button
                className="rounded-xl border border-[#d9e1e9] px-5 py-3 font-semibold text-[#586577] hover:bg-[#f6f8fa]"
                type="button"
                onClick={() => setConfirmOpen(false)}
              >취소</button>
              <button
                className="rounded-xl bg-[#117d75] px-5 py-3 font-semibold text-white hover:bg-[#0e6e67]"
                type="button"
                onClick={confirmStart}
              >확인</button>
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
