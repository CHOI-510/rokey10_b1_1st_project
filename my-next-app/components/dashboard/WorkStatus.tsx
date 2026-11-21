import type { WorkPhase } from "./types";

type WorkStatusProps = {
  phase: WorkPhase;
  countdownSeconds: number;
  elapsedSeconds: number;
  onRecover: () => void;
  onStartImmediately: () => void;
  recoveryDisabled: boolean;
  immediateStartDisabled: boolean;
  stage: string;
  boxSeq: number;
  jobId: string;
  stop: string[] | null;
  reason: string;
  onCancel: () => void;
  commandPending: boolean;
};

function formatDuration(totalSeconds: number) {
  const minutes = Math.floor(totalSeconds / 60).toString().padStart(2, "0");
  const seconds = (totalSeconds % 60).toString().padStart(2, "0");
  return `${minutes}:${seconds}`;
}

export default function WorkStatus({ phase, countdownSeconds, elapsedSeconds, onRecover, onStartImmediately, recoveryDisabled, immediateStartDisabled, stage, boxSeq, jobId, stop, reason, onCancel, commandPending }: WorkStatusProps) {
  const labels: Record<string, string> = {
    READY: "대기 (시작 가능)", BEFORE: "작업전", PREFLIGHT: "사전점검", STANDBY: "시작대기",
    RUNNING: "작업중", SCANNING: "스캔중", SCAN_DONE: "스캔완료", JUDGING: "판별중",
    JUDGE_DONE: "판별완료", PLACING: "적재중", PLACE_DONE: "적재완료", CLEARING: "측정존 비우기",
    DONE: "작업완료", ERROR: "오류중단", STOPPED: "정지", CANCELED: "시작취소",
  };
  const stopped = phase === "emergency" || Boolean(stop);
  const emergency = stopped && (reason.includes("EMERGENCY") || stop?.[0] === "EMERGENCY_STOP");
  const statusLabel = phase === "unknown" ? "관제 없음" : stopped
    ? `${emergency ? "비상정지" : "정지"}${stage === "READY" ? " · 대기" : ""}`
    : labels[stage] || (phase === "running" ? "작업중" : "대기 중");
  const timeLabel = phase === "countdown" && stage === "STANDBY" ? "남은 시간" : "경과 시간";
  const timeValue = phase === "countdown" ? (stage === "STANDBY" ? formatDuration(countdownSeconds) : "사전 점검") : formatDuration(elapsedSeconds);
  const color = stopped || phase === "error" ? "red" : phase === "unknown" ? "gray"
    : ["PREFLIGHT", "STANDBY", "CLEARING", "CANCELED"].includes(stage) ? "amber"
    : stage === "READY" ? "blue" : stage === "BEFORE" ? "gray" : "teal";
  const indicators: Record<string, string> = {
    red: "bg-[#e52327] shadow-[0_0_0_7px_#ffe1e2]",
    gray: "bg-[#a5afbc] shadow-[0_0_0_7px_#edf1f5]",
    amber: "bg-[#f59a08] shadow-[0_0_0_7px_#fff2cb]",
    blue: "bg-[#3d82f6] shadow-[0_0_0_7px_#e8f0ff]",
    teal: "bg-[#12b6a5] shadow-[0_0_0_7px_#d9f4f0]",
  };
  const indicatorClass = indicators[color];
  const emergencyTextClass = color === "red" ? "text-[#c9232b]" : "text-[#202936]";
  const partIndex = phase === "unknown" ? -1 : ["SCANNING", "SCAN_DONE"].includes(stage) ? 0
    : ["JUDGING", "JUDGE_DONE"].includes(stage) ? 1 : ["PLACING", "PLACE_DONE"].includes(stage) ? 2 : -1;
  return (
    <section className="@container/status flex min-h-[300px] min-w-0 flex-col rounded-[20px] border border-[#e1e8ef] bg-white p-[clamp(14px,5cqi,24px)] shadow-sm max-[560px]:min-h-0 max-[560px]:rounded-2xl">
      <div className="flex items-center justify-between gap-3">
        <h2 className="m-0 text-[clamp(14px,4cqi,17px)] font-bold text-[#566477]">작업 상태</h2>
        <span className="min-w-0 truncate text-[clamp(11px,3.2cqi,14px)] text-[#97a3b4]">{jobId ? `job ${jobId}` : "job —"}</span>
      </div>

      <div aria-live="polite" className={`my-[clamp(18px,7cqi,28px)] flex min-w-0 items-center gap-[clamp(9px,4cqi,15px)] break-keep text-[clamp(22px,8cqi,42px)] font-extrabold tracking-[-.06em] ${emergencyTextClass}`}>
        <i className={`ml-1 h-[clamp(12px,4cqi,18px)] w-[clamp(12px,4cqi,18px)] shrink-0 rounded-full ${indicatorClass}`} />
        {statusLabel}
      </div>
      <p className="mb-4 -mt-3 text-xs text-[#8794a6]">{phase === "unknown" ? "관제 상태를 확인할 수 없습니다." : stage === "STANDBY" ? `시작까지 ${countdownSeconds}초` : stage}</p>

      <div className="grid grid-cols-3 gap-[clamp(5px,2.5cqi,10px)]">
        {["스캔", "판별", "적재"].map((label, index) => (
          <span key={label} className={`grid h-[clamp(36px,11cqi,43px)] min-w-0 place-items-center rounded-xl px-1 text-[clamp(11px,3.4cqi,14px)] ${index === partIndex ? "bg-[#117d75] font-bold text-white" : partIndex > index ? "bg-[#e3f6f3] text-[#108c82]" : "bg-[#f2f5f8] text-[#9ba6b5]"}`}>{label}</span>
        ))}
      </div>

      <div className="mt-[clamp(12px,5cqi,18px)] grid grid-cols-3 gap-[clamp(4px,2.5cqi,10px)]">
        <div className="min-w-0 rounded-[14px] bg-[#f6f8fa] p-[clamp(7px,3cqi,12px)]">
          <span className="block break-keep text-[clamp(10px,3cqi,13px)] leading-tight text-[#8794a6]">처리 완료</span>
          <strong className="mt-1 block whitespace-nowrap text-[clamp(14px,4.5cqi,19px)] text-[#202936]">{boxSeq}개</strong>
        </div>
        <div className="min-w-0 rounded-[14px] bg-[#f6f8fa] p-[clamp(7px,3cqi,12px)]">
          <span className="block break-keep text-[clamp(10px,3cqi,13px)] leading-tight text-[#8794a6]">{timeLabel}</span>
          <strong className={`mt-1 block whitespace-nowrap text-[clamp(14px,4.5cqi,19px)] ${emergencyTextClass}`}>{timeValue}</strong>
        </div>
        <div className="min-w-0 rounded-[14px] bg-[#f6f8fa] p-[clamp(7px,3cqi,12px)]">
          <span className="block break-keep text-[clamp(10px,3cqi,13px)] leading-tight text-[#8794a6]">대기 상태</span>
          <strong className={`mt-1 block whitespace-nowrap text-[clamp(14px,4.5cqi,19px)] ${phase === "emergency" ? "text-[#c9232b]" : "text-[#202936]"}`}>{phase === "emergency" ? "비상 정지" : phase === "unknown" ? "확인 불가" : phase === "error" ? "오류" : "정상"}</strong>
        </div>
      </div>

      {reason && <p className="mt-3 break-words text-xs text-[#8794a6]">{reason}</p>}

      {phase === "emergency" && (
        <div className="mt-auto flex w-full flex-col items-end gap-2 pt-[clamp(14px,5cqi,24px)]">
          <button className="max-w-full rounded-xl bg-[#117d75] px-[clamp(16px,5cqi,24px)] py-[clamp(9px,3cqi,12px)] text-[clamp(12px,3.5cqi,14px)] font-bold text-white hover:bg-[#0e6e67] disabled:cursor-not-allowed disabled:opacity-50" type="button" onClick={onRecover} disabled={recoveryDisabled}>복구</button>
          {recoveryDisabled && <p className="m-0 text-xs text-[#8794a6]">관제가 대기 상태로 돌아오면 복구할 수 있습니다.</p>}
        </div>
      )}

      {phase === "countdown" && (
        <button type="button" onClick={onCancel} disabled={commandPending} className="mt-4 self-end rounded-xl border border-[#d9e1e9] px-4 py-2 text-sm text-[#586577] disabled:opacity-50">시작 취소</button>
      )}
      {phase === "countdown" && stage === "STANDBY" && (
        <div className="mt-auto flex w-full flex-wrap items-center justify-end gap-2 pt-[clamp(14px,5cqi,24px)]">
          <button
            className="max-w-full rounded-xl bg-[#117d75] px-[clamp(16px,5cqi,24px)] py-[clamp(9px,3cqi,12px)] text-[clamp(12px,3.5cqi,14px)] font-bold text-white hover:bg-[#0e6e67] disabled:cursor-not-allowed disabled:opacity-50"
            type="button"
            onClick={onStartImmediately}
            disabled={immediateStartDisabled}
          >바로 시작</button>
          {immediateStartDisabled && <p className="m-0 w-full text-right text-xs text-[#8794a6]">시작 요청 처리 중입니다.</p>}
        </div>
      )}
    </section>
  );
}
