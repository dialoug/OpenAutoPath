import { useState } from "react";
export type CamFeature = {
  id: string;
  kind: "pocket" | "profile" | "engrave";
  faces: string[];
};
export type Cutter = {
  id: string;
  name: string;
  type: string;
  diameter: number;
  feed?: number;
  rpm?: number;
};
export type PlanOptions = {
  feature: string;
  tool: string;
  step: number;
  strategy: string;
};
const names = { pocket: "槽腔", profile: "外轮廓", engrave: "雕刻装夹" };
const patterns = {
  pocket: [
    ["offset", "偏置"],
    ["zigzag", "往复"],
    ["spiral", "螺旋"],
    ["freepath", "FreePath"],
  ],
  profile: [
    ["external", "外部清料"],
    ["contour", "轮廓精加工"],
  ],
  engrave: [
    ["finish", "仅精雕"],
    ["rough", "毛坯分层开粗及精雕"],
  ],
};
export function PlanningForm({
  features,
  cutters,
  plan,
}: {
  features: CamFeature[];
  cutters: Cutter[];
  plan: (o: PlanOptions) => void;
}) {
  const [id, setId] = useState("");
  const [strategy, setStrategy] = useState("");
  const [tool, setTool] = useState("");
  const [step, setStep] = useState(1);
  const chosen = features.find((f) => f.id === id);
  const mills = cutters.filter((c) =>
    ["end_mill", "ball_end_mill", "bull_nose"].includes(c.type),
  );
  return (
    <details>
      <summary>规划槽腔、外轮廓或雕刻工序</summary>
      <label>
        加工区域
        <select
          value={id}
          onChange={(e) => {
            setId(e.target.value);
            setStrategy("");
          }}
        >
          <option value="">选择区域或雕刻方向</option>
          {features.map((f) => (
            <option key={f.id} value={f.id}>
              {names[f.kind]} · {f.id}
            </option>
          ))}
        </select>
      </label>
      {chosen && (
        <>
          <label>
            加工策略
            <select
              value={strategy || patterns[chosen.kind][0][0]}
              onChange={(e) => setStrategy(e.target.value)}
            >
              {patterns[chosen.kind].map(([v, l]) => (
                <option value={v} key={v}>
                  {l}
                </option>
              ))}
            </select>
          </label>
          <label>
            规划刀具
            <select value={tool} onChange={(e) => setTool(e.target.value)}>
              <option value="">选择铣刀</option>
              {mills.map((c) => (
                <option value={c.id} key={c.id}>
                  {c.name}
                </option>
              ))}
            </select>
          </label>
          <label>
            {chosen.kind === "engrave" ? "扫描行距" : "每层切深"} (mm)
            <input
              type="number"
              min=".001"
              step="any"
              value={step}
              onChange={(e) => setStep(+e.target.value)}
            />
          </label>
          <button
            disabled={!tool || step <= 0}
            onClick={() =>
              plan({
                feature: id,
                tool,
                step,
                strategy: strategy || patterns[chosen.kind][0][0],
              })
            }
          >
            {chosen.kind === "engrave"
              ? "追加雕刻工序"
              : "更新项目的" + names[chosen.kind] + "工序"}
            并保存新版本
          </button>
        </>
      )}
    </details>
  );
}
