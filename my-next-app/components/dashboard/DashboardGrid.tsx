import JobLogSummary from "./JobLogSummary";
import StockStatus from "./StockStatus";
import StorageImage from "./StorageImage";
import WorkStatus from "./WorkStatus";
import type { WorkPhase } from "./types";

type DashboardGridProps = {
  phase: WorkPhase;
  countdownSeconds: number;
  elapsedSeconds: number;
  onRecover: () => void;
  onStartImmediately: () => void;
};

export default function DashboardGrid({ phase, countdownSeconds, elapsedSeconds, onRecover, onStartImmediately }: DashboardGridProps) {
  return (
    <section
      aria-label="대시보드"
      className="mt-5 grid min-h-[calc(100vh-152px)] grid-cols-[minmax(280px,.78fr)_minmax(0,1.62fr)] grid-rows-[minmax(300px,.8fr)_minmax(380px,1.2fr)] gap-5 max-[760px]:grid-cols-1 max-[760px]:grid-rows-none"
    >
      <WorkStatus phase={phase} countdownSeconds={countdownSeconds} elapsedSeconds={elapsedSeconds} onRecover={onRecover} onStartImmediately={onStartImmediately} />
      <JobLogSummary />
      <StockStatus />
      <StorageImage />
    </section>
  );
}
