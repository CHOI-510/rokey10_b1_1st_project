const summary = [
  { label: "처리", value: "26", valueClass: "text-[#202936]" },
  { label: "미분류", value: "2", valueClass: "text-[#e67d00]" },
  { label: "재시도", value: "3", valueClass: "text-[#e67d00]" },
  { label: "오류", value: "0", valueClass: "text-[#16a453]" },
];

const events = [
  { time: "14:32:06", type: "판별", tagClass: "bg-[#e8f0ff] text-[#3474e8]", message: "폭 · 높이 · 무게 측정 완료, 규격 B 판정" },
  { time: "14:31:41", type: "적재", tagClass: "bg-[#e1f6f2] text-[#128b81]", message: "규격 A 슬롯 적재 완료, 재고 9개로 갱신" },
  { time: "14:31:12", type: "탐지", tagClass: "bg-[#fff2cb] text-[#d27a00]", message: "파지 실패 1회, 재시도 후 성공" },
  { time: "14:30:27", type: "판별", tagClass: "bg-[#fff2cb] text-[#d27a00]", message: "규격 미확인 1건, 미분류 처리" },
];

export default function JobLogSummary() {
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

      <ul className="m-0 mt-[17px] list-none p-0">
        {events.map((event) => (
          <li key={event.time} className="grid min-h-[37px] grid-cols-[72px_43px_minmax(0,1fr)] items-center gap-[10px] border-t border-[#e8edf2] max-[560px]:grid-cols-[60px_39px_minmax(0,1fr)] max-[560px]:gap-[7px]">
            <time className="text-sm tabular-nums text-[#98a4b4] max-[560px]:text-xs">{event.time}</time>
            <span className={`w-fit rounded-full px-[9px] py-1 text-xs font-bold ${event.tagClass}`}>{event.type}</span>
            <p className="m-0 overflow-hidden text-ellipsis whitespace-nowrap text-sm text-[#526073] max-[560px]:text-xs">{event.message}</p>
          </li>
        ))}
      </ul>
    </section>
  );
}
import type { ApiLogEvent } from "./types";

