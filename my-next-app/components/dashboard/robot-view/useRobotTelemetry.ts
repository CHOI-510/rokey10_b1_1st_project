"use client";
import { useEffect, useState } from "react";
import type { RobotTelemetry } from "./model";

const API_URL = (process.env.NEXT_PUBLIC_ROBOT_API_URL || "http://localhost:8000").replace(/\/$/, "");
function vector(value: unknown, length: number): value is number[] {
  return Array.isArray(value) && value.length === length && value.every(v => typeof v === "number" && Number.isFinite(v));
}
export function useRobotTelemetry(enabled = true) {
  const [telemetry, setTelemetry] = useState<RobotTelemetry>();
  const [message, setMessage] = useState("실제 로봇 좌표 연결 중");
  useEffect(() => {
    if (!enabled) return;
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    const abort = new AbortController();
    const disconnected = (message: string) => {
      if (!active) return;
      setMessage(message);
      setTelemetry(previous => previous ? { ...previous, receivedAt: 0 } : undefined);
    };
    async function poll() {
      try {
        const response = await fetch(`${API_URL}/robot/telemetry`, {
          cache: "no-store", signal: AbortSignal.any([abort.signal, AbortSignal.timeout(2000)]),
        });
        if (!response.ok) throw new Error(`좌표 API 오류 (${response.status})`);
        const sample = await response.json();
        if (!active) return;
        if (sample.connected !== true || sample.frame !== "BASE" || !vector(sample.joints_deg,6)
          || !vector(sample.tcp_mm,3) || typeof sample.age_ms !== "number" || !Number.isFinite(sample.age_ms)
          || sample.age_ms < 0 || sample.age_ms >= 2000) {
          disconnected(sample.message || "실제 로봇 좌표 연결 대기");
        } else {
          setTelemetry({ jointsDeg: sample.joints_deg as RobotTelemetry["jointsDeg"],
            tcpMm: sample.tcp_mm as RobotTelemetry["tcpMm"],
            // Use server sample age, independent of browser/server clock differences.
            receivedAt: Date.now() - sample.age_ms });
          setMessage("두산 관절각·BASE TCP 실측 수신 중");
        }
      } catch (error) {
        disconnected(error instanceof Error ? error.message : "좌표 API 연결 실패");
      } finally {
        if (active) timer = setTimeout(poll, 100);
      }
    }
    void poll();
    return () => { active = false; abort.abort(); clearTimeout(timer); };
  }, [enabled]);
  return { telemetry, message };
}
