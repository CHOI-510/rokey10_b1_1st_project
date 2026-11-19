type Palette = {
  top: string;
  left: string;
  right: string;
  label: string;
};

const palettes: Record<string, Palette> = {
  teal: {
    top: "fill-[#4cc7b8] stroke-[#188e84]",
    left: "fill-[#1aa696] stroke-[#13877d]",
    right: "fill-[#128d83] stroke-[#0e766f]",
    label: "fill-[#12b6a5]",
  },
  blue: {
    top: "fill-[#70a7fb] stroke-[#3677dc]",
    left: "fill-[#4285ed] stroke-[#2c6ac6]",
    right: "fill-[#2d6fcf] stroke-[#245cac]",
    label: "fill-[#3d82f6]",
  },
  amber: {
    top: "fill-[#ffc04d] stroke-[#d48a13]",
    left: "fill-[#f8a91e] stroke-[#cf820e]",
    right: "fill-[#df8d0c] stroke-[#b87309]",
    label: "fill-[#f59a08]",
  },
  purple: {
    top: "fill-[#b18af8] stroke-[#8050d7]",
    left: "fill-[#9464e8] stroke-[#7447c1]",
    right: "fill-[#8151d2] stroke-[#6539b0]",
    label: "fill-[#8b5cf6]",
  },
};

function Crate({ x, y, palette }: { x: number; y: number; palette: Palette }) {
  return (
    <g transform={`translate(${x} ${y})`} strokeWidth="1.4" strokeLinejoin="round">
      <path className={palette.top} d="M0 18 35 0 70 18 35 37Z" />
      <path className={palette.left} d="M0 18 35 37 35 76 0 56Z" />
      <path className={palette.right} d="M35 37 70 18 70 56 35 76Z" />
    </g>
  );
}

function Pallet({
  x,
  y,
  color,
  label,
  boxes,
}: {
  x: number;
  y: number;
  color: string;
  label: string;
  boxes: [number, number][];
}) {
  const palette = palettes[color];

  return (
    <g transform={`translate(${x} ${y})`}>
      <path className="fill-[#f4f7fa] stroke-[#dbe3eb]" strokeWidth="2" d="M0 88 105 30 210 88 105 148Z" />
      <path className="fill-[#dce4ec]" d="M0 88 105 148 105 158 0 98ZM105 148 210 88 210 98 105 158Z" />
      {boxes.map(([bx, by], index) => (
        <Crate key={`${bx}-${by}-${index}`} x={bx} y={by} palette={palette} />
      ))}
      <g transform="translate(48 145)">
        <rect className={palette.label} width="88" height="30" rx="15" />
        <text x="44" y="20" fill="white" fontSize="14" fontWeight="700" textAnchor="middle">{label}</text>
      </g>
    </g>
  );
}

export default function StorageImage() {
  return (
    <section className="relative min-h-[380px] min-w-0 overflow-hidden rounded-[20px] border border-[#e1e8ef] bg-white p-6 shadow-sm max-[760px]:min-h-[340px] max-[560px]:rounded-2xl max-[560px]:p-5">
      <div className="flex items-center justify-between gap-3">
        <h2 className="m-0 text-[17px] font-bold text-[#566477]">적재 현황 이미지</h2>
        <div aria-label="규격 범례" className="flex items-center gap-[13px] text-[13px] font-bold text-[#687588] max-[390px]:gap-2">
          <span className="flex items-center gap-1.5"><i className="h-[11px] w-[11px] rounded-[4px] bg-[#12b6a5]" />A</span>
          <span className="flex items-center gap-1.5"><i className="h-[11px] w-[11px] rounded-[4px] bg-[#3d82f6]" />B</span>
          <span className="flex items-center gap-1.5"><i className="h-[11px] w-[11px] rounded-[4px] bg-[#f59a08]" />C</span>
          <span className="flex items-center gap-1.5"><i className="h-[11px] w-[11px] rounded-[4px] bg-[#8b5cf6]" />D</span>
        </div>
      </div>

      <svg className="mx-auto mt-4 block h-[310px] w-full max-w-[700px] max-[760px]:h-[270px] max-[560px]:mt-5 max-[560px]:h-[245px]" viewBox="0 0 700 470" role="img" aria-label="규격별 팔레트 적재 상태">
        <Pallet x={242} y={26} color="teal" label="규격 A" boxes={[[68, 30], [115, 54], [22, 74], [69, 98], [116, 122]]} />
        <Pallet x={440} y={180} color="blue" label="규격 B" boxes={[[20, 45], [67, 69], [114, 93], [67, 117]]} />
        <Pallet x={35} y={174} color="amber" label="규격 C" boxes={[[22, 42], [69, 66], [116, 90], [69, 114]]} />
        <Pallet x={240} y={250} color="purple" label="규격 D" boxes={[[68, 0], [22, 54], [69, 78], [116, 102], [69, 126]]} />
      </svg>
    </section>
  );
}
