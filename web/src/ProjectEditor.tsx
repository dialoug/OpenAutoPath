import { useState } from "react";
import { families, type Family, type Project, type Job } from "./workspace";
import type { Cutter } from "./PlanningForm";
import { MillingToolPicker } from "./MillingToolPicker";
type Props = {
  cutters: Cutter[];
  value: Project;
  change: (doc: Project) => void;
  generated: Job[];
  append: (id: string) => void;
  rebuild: (id: string, tool?: string) => void;
};
export function ProjectEditor({
  value,
  change,
  generated,
  cutters,
  append,
  rebuild,
}: Props) {
  const [appendID, setAppendID] = useState("");
  const [changing, setChanging] = useState("");
  const [replacement, setReplacement] = useState("");
  const def = value.job;
  function edit(fn: (d: Project["job"]) => void) {
    const next = structuredClone(value);
    fn(next.job);
    change(next);
  }
  const ordered = (Object.keys(families) as Family[])
    .flatMap((family) => def[family].map((op) => ({ family, op })))
    .sort((a, b) => a.op.sequence - b.op.sequence);
  function move(i: number, j: number) {
    if (!ordered[j]) return;
    edit((d) => {
      const left = ordered[i],
        right = ordered[j];
      d[left.family].find((o) => o.id === left.op.id)!.sequence =
        right.op.sequence;
      d[right.family].find((o) => o.id === right.op.id)!.sequence =
        left.op.sequence;
    });
  }
  return (
    <>
      <label>
        项目名称
        <input
          value={def.name}
          onChange={(e) =>
            edit((d) => {
              d.name = e.target.value;
            })
          }
        />
      </label>
      <p>零件　{def.part.source_path}</p>
      <label>
        毛坯来源
        <select
          value={def.stock.source_kind}
          onChange={(e) =>
            edit((d) => {
              d.stock.source_kind = +e.target.value;
            })
          }
        >
          <option value={0}>各零件包围盒</option>
          <option value={1}>整体包围盒</option>
          {def.stock.source_kind === 2 && (
            <option value={2}>外部 STEP 毛坯</option>
          )}
        </select>
      </label>
      <div className="docGrid">
        {(["margin_x", "margin_y", "margin_z"] as const).map((k, i) => (
          <label key={k}>
            毛坯余量 {"XYZ"[i]} (mm)
            <input
              type="number"
              min="0"
              step="any"
              disabled={def.stock.source_kind === 2}
              value={def.stock[k]}
              onChange={(e) =>
                edit((d) => {
                  d.stock[k] = +e.target.value;
                })
              }
            />
          </label>
        ))}
      </div>
      <h3>工序顺序 · {ordered.length}</h3>
      {ordered.map(({ op, family }, i) => (
        <article className="opRow" key={op.id}>
          <label>
            <input
              type="checkbox"
              checked={op.enabled}
              onChange={(e) =>
                edit((d) => {
                  d[family].find((o) => o.id === op.id)!.enabled =
                    e.target.checked;
                })
              }
            />{" "}
            {i + 1}. {families[family]}
          </label>
          <small>
            {op.setup_id} ·{" "}
            {
              def.tool_controllers.find((c) => c.id === op.tool_controller_id)
                ?.tool.tool_name
            }
          </small>
          {family === "pocket_operations" && (
            <>
              <p>
                {[op.target?.root_face_id, op.target?.root_wire_id]
                  .filter(Boolean)
                  .join(" / ")}{" "}
                ·{" "}
                {
                  ["偏置", "往复", "螺旋", "FreePath"][
                    (op.parameters || def.pocket_parameters).clearing.pattern
                  ]
                }{" "}
                · 层深{" "}
                {(op.parameters || def.pocket_parameters).depth.step_down} mm
              </p>
              {changing === op.id ? (
                <div>
                  <MillingToolPicker
                    cutters={cutters}
                    label="替换刀具"
                    value={replacement}
                    select={setReplacement}
                  />
                  <button
                    disabled={!op.enabled || !replacement}
                    onClick={() => rebuild(op.id, replacement)}
                  >
                    换刀并重算此工序
                  </button>
                  <button onClick={() => setChanging("")}>取消换刀</button>
                </div>
              ) : (
                <button
                  disabled={!op.enabled}
                  onClick={() => {
                    setChanging(op.id);
                    setReplacement("");
                  }}
                >
                  更换刀具
                </button>
              )}
            </>
          )}
          <div className="docActions">
            <button disabled={!op.enabled} onClick={() => rebuild(op.id)}>
              重算并预览
            </button>
            <button disabled={!i} onClick={() => move(i, i - 1)}>
              上移
            </button>
            <button
              disabled={i === ordered.length - 1}
              onClick={() => move(i, i + 1)}
            >
              下移
            </button>
            <button
              onClick={() =>
                edit((d) => {
                  d[family] = d[family].filter((o) => o.id !== op.id);
                })
              }
            >
              移除
            </button>
          </div>
        </article>
      ))}
      {generated.length > 0 && (
        <div>
          <label>
            已生成的孔加工
            <select
              value={appendID}
              onChange={(e) => setAppendID(e.target.value)}
            >
              <option value="">选择要加入的结果</option>
              {generated.map((j) => (
                <option value={j.id} key={j.id}>
                  {j.name} · {new Date(j.created!).toLocaleString()} ·{" "}
                  {j.id.slice(0, 6)}
                </option>
              ))}
            </select>
          </label>
          <button disabled={!appendID} onClick={() => append(appendID)}>
            加入项目并保存
          </button>
        </div>
      )}
      {def.tool_controllers.length > 0 && (
        <details>
          <summary>项目刀具快照</summary>
          {def.tool_controllers.map((c, i) => (
            <section key={c.id}>
              <h4>{c.tool.tool_name}</h4>
              <div className="docGrid">
                {(
                  ["vertical_feed", "horizontal_feed", "spindle_speed"] as const
                ).map((k, j) => (
                  <label key={k}>
                    {["轴向进给", "水平进给", "主轴转速"][j]}
                    <input
                      type="number"
                      min="0"
                      step="any"
                      value={c.tool[k]}
                      onChange={(e) =>
                        edit((d) => {
                          d.tool_controllers[i].tool[k] = +e.target.value;
                          for (const f of Object.keys(families) as Family[])
                            for (const o of d[f])
                              if (o.tool_controller_id === c.id) {
                                o.state = 1;
                              }
                        })
                      }
                    />
                  </label>
                ))}
              </div>
            </section>
          ))}
        </details>
      )}
    </>
  );
}
