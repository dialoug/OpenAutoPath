import { useState } from "react";
import { Viewport, type Face } from "./Viewport";
import { MillingToolPicker } from "./MillingToolPicker";
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
  autoTool?: boolean;
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
  faces,
  plan,
}: {
  features: CamFeature[];
  faces: Face[];
  cutters: Cutter[];
  plan: (o: PlanOptions) => void;
}) {
  const [expanded, setExpanded] = useState(false);
  const [picked, setPicked] = useState("");
  const [id, setId] = useState("");
  const chosen = features.find((f) => f.id === id);

  const candidates = picked
    ? features.filter((f) => f.faces.includes(picked))
    : [];
  return (
    <details
      className="regionPlanner"
      onToggle={(e) => setExpanded(e.currentTarget.open)}
    >
      <summary>规划槽腔、外轮廓或雕刻工序</summary>
      {expanded && faces.length > 0 && (
        <>
          <Viewport
            faces={faces}
            highlighted={chosen?.faces || []}
            path={[]}
            onPick={(face) => {
              setPicked(face);
              const matches = features.filter((f) => f.faces.includes(face));
              if (matches.length === 1) {
                setId(matches[0].id);
              } else {
                setId("");
              }
            }}
          />
          {picked && (
            <div>
              所选面 {picked} · {candidates.length} 个加工区域
              {candidates.length > 1 &&
                candidates.map((f) => (
                  <button
                    key={f.id}
                    onClick={() => {
                      setId(f.id);
                    }}
                  >
                    {names[f.kind]} · {f.id}
                  </button>
                ))}
            </div>
          )}
        </>
      )}
      <label>
        加工区域
        <select
          value={id}
          onChange={(e) => {
            setPicked("");
            setId(e.target.value);
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
          <PlanningControls
            key={chosen.id}
            feature={chosen}
            cutters={cutters}
            plan={plan}
          />
        </>
      )}
    </details>
  );
}

export function PlanningControls({
  feature,
  cutters,
  plan,
}: {
  feature: CamFeature;
  cutters: Cutter[];
  plan: (o: PlanOptions) => void;
}) {
  const [all, setAll] = useState(false);
  const [autoTool, setAutoTool] = useState(false);
  const [strategy, setStrategy] = useState("");
  const [tool, setTool] = useState("");
  const [step, setStep] = useState(1);
  const canAuto =
    feature.kind === "pocket" ||
    (feature.kind === "profile" && strategy !== "contour");
  const automatic = canAuto && autoTool;
  return (
    <>
      {" "}
      <label>
        加工策略
        <select
          value={strategy || patterns[feature.kind][0][0]}
          onChange={(e) => setStrategy(e.target.value)}
        >
          {patterns[feature.kind].map(([v, l]) => (
            <option value={v} key={v}>
              {l}
            </option>
          ))}
        </select>
      </label>
      {feature.kind !== "engrave" && !feature.id.startsWith("manual-") && (
        <label>
          <input
            type="checkbox"
            checked={all}
            onChange={(e) => setAll(e.target.checked)}
          />{" "}
          重新规划全部{names[feature.kind]}区域
        </label>
      )}
      {canAuto && (
        <label>
          <input
            type="checkbox"
            checked={autoTool}
            onChange={(e) => setAutoTool(e.target.checked)}
          />{" "}
          由活动刀库自动选刀
        </label>
      )}
      {!automatic && (
        <MillingToolPicker
          label="规划刀具"
          cutters={cutters}
          value={tool}
          select={setTool}
        />
      )}
      <label>
        {feature.kind === "engrave" ? "扫描行距" : "每层切深"} (mm)
        <input
          type="number"
          min=".001"
          step="any"
          value={step}
          onChange={(e) => setStep(+e.target.value)}
        />
      </label>
      <button
        disabled={(!automatic && !tool) || step <= 0}
        onClick={() =>
          plan({
            feature: all ? feature.kind + "-all" : feature.id,
            tool: automatic ? "" : tool,
            autoTool: automatic,
            step,
            strategy: strategy || patterns[feature.kind][0][0],
          })
        }
      >
        {all
          ? "更新全部" + names[feature.kind] + "工序"
          : feature.kind === "engrave"
            ? feature.id === "engrave-all"
              ? "重新规划全部六面雕刻"
              : "追加雕刻工序"
            : feature.kind === "pocket"
              ? "追加槽腔工序"
              : "更新项目的外轮廓工序"}
        并保存新版本
      </button>
    </>
  );
}
