import type { WorkPhase } from "./types";

type WorkStatusProps = {
  phase: WorkPhase;
  countdownSeconds: number;
  elapsedSeconds: number;
  onRecover: () => void;
  onStartImmediately: () => void;
};

function formatDuration(totalSeconds: number) {
  const minutes = Math.floor(totalSeconds / 60).toString().padStart(2, "0");
  const seconds = (totalSeconds % 60).toString().padStart(2, "0");
  return `${minutes}:${seconds}`;
}

export default function WorkStatus({ phase, countdownSeconds, elapsedSeconds, onRecover, onStartImmediately }: WorkStatusProps) {
  const statusLabel = phase === "countdown" ? "시작 대기 중" : phase === "running" ? "작업 중" : phase === "emergency" ? "비상 정지" : "대기 중";
  const timeLabel = phase === "countdown" ? "남은 시간" : phase === "running" ? "경과 시간" : phase === "emergency" ? "정지 상태" : "작업 시간"; 
  const timeValue = phase === "countdown" ? formatDuration(countdownSeconds) : phase === "running" ? formatDuration(elapsedSeconds) : phase === "emergency" ? "정지됨" : "--:--";
  const indicatorClass = phase === "emergency"
    ? "bg-[#e52327] shadow-[0_0_0_clamp(4px,1.5cqi,7px)_#ffe1e2]"
    : phase === "running"
      ? "bg-[#12b6a5] shadow-[0_0_0_clamp(4px,1.5cqi,7px)_#d9f4f0]"
      : phase === "countdown"
        ? "bg-[#f59a08] shadow-[0_0_0_clamp(4px,1.5cqi,7px)_#fff2cb]"
        : "bg-[#a5afbc] shadow-[0_0_0_clamp(4px,1.5cqi,7px)_#edf1f5]";
  const emergencyTextClass = phase === "emergency" ? "text-[#c9232b]" : "text-[#202936]";
  return (
    <section className="@container/status flex min-h-[300px] min-w-0 flex-col rounded-[20px] border border-[#e1e8ef] bg-white p-[clamp(14px,5cqi,24px)] shadow-sm max-[560px]:min-h-0 max-[560px]:rounded-2xl">
      <div className="flex items-center justify-between gap-3">
        <h2 className="m-0 text-[clamp(14px,4cqi,17px)] font-bold text-[#566477]">작업 상태</h2>
        <span className="whitespace-nowrap text-[clamp(11px,3.2cqi,14px)] text-[#97a3b4]">사이클 #14</span>
      </div>

      <div aria-live="polite" className={`my-[clamp(18px,7cqi,28px)] flex min-w-0 items-center gap-[clamp(9px,4cqi,15px)] whitespace-nowrap text-[clamp(22px,10cqi,42px)] font-extrabold tracking-[-.06em] ${emergencyTextClass}`}>
        <i className={`ml-1 h-[clamp(12px,4cqi,18px)] w-[clamp(12px,4cqi,18px)] shrink-0 rounded-full ${indicatorClass}`} />
        {statusLabel}
      </div>

      <div className="grid grid-cols-3 gap-[clamp(5px,2.5cqi,10px)]">
        <span className={`grid h-[clamp(36px,11cqi,43px)] min-w-0 place-items-center rounded-xl px-1 text-[clamp(11px,3.4cqi,14px)] ${phase === "running" ? "bg-[#117d75] font-bold text-white" : "bg-[#f2f5f8] text-[#9ba6b5]"}`}>스캔</span>
        <span className="grid h-[clamp(36px,11cqi,43px)] min-w-0 place-items-center rounded-xl bg-[#f2f5f8] px-1 text-[clamp(11px,3.4cqi,14px)] text-[#9ba6b5]">판별</span>
        <span className="grid h-[clamp(36px,11cqi,43px)] min-w-0 place-items-center rounded-xl bg-[#f2f5f8] px-1 text-[clamp(11px,3.4cqi,14px)] text-[#9ba6b5]">적재</span>
      </div>

      <div className="mt-[clamp(12px,5cqi,18px)] grid grid-cols-3 gap-[clamp(4px,2.5cqi,10px)]">
        <div className="min-w-0 rounded-[14px] bg-[#f6f8fa] p-[clamp(7px,3cqi,12px)]">
          <span className="block break-keep text-[clamp(10px,3cqi,13px)] leading-tight text-[#8794a6]">처리 완료</span>
          <strong className="mt-1 block whitespace-nowrap text-[clamp(14px,4.5cqi,19px)] text-[#202936]">26개</strong>
        </div>
        <div className="min-w-0 rounded-[14px] bg-[#f6f8fa] p-[clamp(7px,3cqi,12px)]">
          <span className="block break-keep text-[clamp(10px,3cqi,13px)] leading-tight text-[#8794a6]">{timeLabel}</span>
          <strong className={`mt-1 block whitespace-nowrap text-[clamp(14px,4.5cqi,19px)] ${emergencyTextClass}`}>{timeValue}</strong>
        </div>
        <div className="min-w-0 rounded-[14px] bg-[#f6f8fa] p-[clamp(7px,3cqi,12px)]">
          <span className="block break-keep text-[clamp(10px,3cqi,13px)] leading-tight text-[#8794a6]">대기 상태</span>
          <strong className={`mt-1 block whitespace-nowrap text-[clamp(14px,4.5cqi,19px)] ${phase === "emergency" ? "text-[#c9232b]" : "text-[#202936]"}`}>{phase === "emergency" ? "비상 정지" : "정상"}</strong>
        </div>
      </div>

      {phase === "emergency" && (
        <div className="mt-auto flex w-full justify-end pt-[clamp(14px,5cqi,24px)]">
          <button
            className="max-w-full rounded-xl bg-[#117d75] px-[clamp(16px,5cqi,24px)] py-[clamp(9px,3cqi,12px)] text-[clamp(12px,3.5cqi,14px)] font-bold text-white hover:bg-[#0e6e67]"
            type="button"
            onClick={onRecover}
          >복구</button>
        </div>
      )}

      {phase === "countdown" && (
        <div className="mt-auto flex w-full justify-end pt-[clamp(14px,5cqi,24px)]">
          <button
            className="max-w-full rounded-xl bg-[#117d75] px-[clamp(16px,5cqi,24px)] py-[clamp(9px,3cqi,12px)] text-[clamp(12px,3.5cqi,14px)] font-bold text-white hover:bg-[#0e6e67]"
            type="button"
            onClick={onStartImmediately}
          >바로 시작</button>
        </div>
      )}
    </section>
  );
}
