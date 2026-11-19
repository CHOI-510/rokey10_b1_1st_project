type HeaderProps = {
  dateLabel: string;
  timeLabel: string;
  startDisabled: boolean;
  stopDisabled: boolean;
  apiConnected: boolean;
  onStartClick: () => void;
  onStopClick: () => void;
};

export default function Header({ dateLabel, timeLabel, startDisabled, stopDisabled, apiConnected, onStartClick, onStopClick }: HeaderProps) {
  return (
    <header className="grid min-h-[104px] w-full grid-cols-[minmax(0,1fr)_auto_minmax(0,1fr)] items-center gap-5 rounded-[20px] border border-[#e1e8ef] bg-white px-[22px] py-[14px] text-[#202936] shadow-sm max-[1020px]:grid-cols-[minmax(0,1fr)_auto_minmax(0,auto)] max-[1020px]:gap-[10px] max-[1020px]:px-[14px] max-[560px]:min-h-0 max-[560px]:grid-cols-[minmax(0,1fr)_auto] max-[560px]:gap-3 max-[560px]:rounded-2xl max-[560px]:px-[13px] max-[560px]:py-[13px]">
      <div className="flex min-w-0 items-center gap-[13px] max-[1020px]:gap-[9px] max-[560px]:gap-2">
        <span className="grid h-[50px] w-[50px] shrink-0 place-items-center rounded-[13px] bg-gradient-to-br from-[#117d75] to-[#14b8a6] max-[1020px]:h-[42px] max-[1020px]:w-[42px] max-[560px]:h-[38px] max-[560px]:w-[38px] max-[560px]:rounded-[10px]" aria-hidden="true">
          <svg className="w-7 stroke-white [stroke-linejoin:round] [stroke-width:2]" viewBox="0 0 32 32" fill="none">
            <path d="m16 3 11 6.3v13.4L16 29 5 22.7V9.3L16 3Z" />
            <path d="M5 9.3 16 16l11-6.7M16 16v13" />
          </svg>
        </span>
        <span className="flex min-w-0 flex-col gap-1">
          <strong className="truncate text-[21px] leading-tight tracking-[-.04em] max-[1020px]:text-[17px] max-[560px]:text-[15px]">박스 분류 관제</strong>
          <small className="truncate text-[13px] text-[#8290a2] max-[1020px]:text-[11px] max-[560px]:text-[9px]">M0609 · RG2 자동 분류 · 재고 관리</small>
        </span>
      </div>

      <div className="flex flex-col items-center whitespace-nowrap max-[560px]:col-start-2 max-[560px]:row-start-1">
        <span className="text-[13px] text-[#78869a] max-[1020px]:text-[11px] max-[560px]:text-[9px]">{dateLabel}</span>
        <strong className="text-[32px] leading-[1.15] tabular-nums text-[#202936] max-[1020px]:text-[25px] max-[560px]:text-[20px]">{timeLabel}</strong>
      </div>

      <div className="flex items-center justify-end gap-[9px] max-[1020px]:gap-[6px] max-[560px]:col-span-full max-[560px]:justify-start max-[560px]:flex-wrap">
        <span className={`inline-flex h-[37px] items-center gap-[7px] whitespace-nowrap rounded-full px-3 text-[13px] max-[1020px]:h-8 max-[1020px]:px-2 max-[1020px]:text-[11px] max-[560px]:h-[30px] ${apiConnected ? "bg-[#e7f8ef] text-[#147a43]" : "bg-[#fff0f0] text-[#c9232b]"}`}><i className={`h-2 w-2 rounded-full ${apiConnected ? "bg-[#12a65a]" : "bg-[#e52327]"}`} />{apiConnected ? "API 연결" : "API 끊김"}</span>
        <span className="inline-flex h-[37px] items-center gap-[7px] whitespace-nowrap rounded-full bg-[#f2f5f8] px-3 text-[13px] text-[#586577] max-[1020px]:h-8 max-[1020px]:px-2 max-[1020px]:text-[11px] max-[560px]:h-[30px]"><i className="h-2 w-2 rounded-full bg-[#12a65a]" />DB 연결</span>
        <button className="h-[46px] whitespace-nowrap rounded-xl border-0 bg-[#117d75] px-[18px] text-[15px] font-bold text-white enabled:hover:bg-[#0e6e67] disabled:cursor-not-allowed disabled:opacity-60 max-[1020px]:h-10 max-[1020px]:px-3 max-[1020px]:text-[13px] max-[560px]:h-9" type="button" onClick={onStartClick} disabled={startDisabled}><span className="mr-[5px] text-[11px]" aria-hidden="true">▶</span>시작</button>
        <button className="grid h-[54px] w-[54px] shrink-0 place-items-center rounded-full border-[6px] border-[#ffe1e2] bg-[#e52327] hover:bg-[#cf1f23] disabled:cursor-not-allowed disabled:opacity-50 max-[1020px]:h-11 max-[1020px]:w-11 max-[1020px]:border-[5px] max-[560px]:h-10 max-[560px]:w-10" type="button" aria-label="비상 정지" onClick={onStopClick} disabled={stopDisabled}><i className="h-3 w-3 rounded-[2px] bg-white" /></button>
      </div>
    </header>
  );
}
