import { useState } from "react";
import { families, type Family, type Project, type Job } from "./workspace";
import type { Cutter } from "./PlanningForm";
import { MachiningParameters } from "./MachiningParameters";
import { MillingToolPicker } from "./MillingToolPicker";
type Props = {
  view: "operations" | "setup";
  cutters: Cutter[];
  value: Project;
  change: (doc: Project) => void;
  generated: Job[];
  append: (id: string) => void;
  assets: Job[];
  bindStock: (id: string | File) => void;
  simulate: (id: string) => void;
  confirmSetups: (ids: string[]) => void;
  rebuild: (id: string, tool?: string) => void;
};
export function ProjectEditor({
  value,
  view,
  change,
  generated,
  cutters,
  append,
  rebuild,
  simulate,
  assets,
  bindStock,
  confirmSetups,
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
      <section hidden={view !== "setup"} className="setupSection">
        <h3>项目与毛坯</h3>
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
                d.stock.source_path = "";
                d.stock.source_fingerprint = "";
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
        <details className="setupRow">
          <summary>
            外部 STEP 毛坯
            {def.stock.source_kind === 2 ? " · " + def.stock.source_path : ""}
          </summary>
          <label>
            已上传的毛坯
            <select
              value=""
              onChange={(e) => {
                if (e.target.value) bindStock(e.target.value);
              }}
            >
              <option value="">选择 STEP 并绑定毛坯</option>
              {assets.map((m) => (
                <option key={m.id} value={m.id}>
                  {m.name} · {m.id.slice(0, 6)}
                </option>
              ))}
            </select>
          </label>
          <label className="upload">
            上传并绑定毛坯
            <input
              type="file"
              accept=".stp,.step"
              onChange={(e) => {
                const f = e.target.files?.[0];
                e.target.value = "";
                if (f) bindStock(f);
              }}
            />
          </label>
        </details>
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
        {def.setups.length > 0 && (
          <section>
            <h3>装夹与工件坐标</h3>
            {def.setups.map((s) => (
              <details key={s.id} className="setupRow">
                <summary>
                  {s.work_offset ? "G" + (53 + s.work_offset) : "未分配坐标"} ·{" "}
                  {s.id} · {s.confirmation.confirmed ? "已确认" : "待确认"}
                </summary>
                <p>
                  原点 (mm)　
                  {[s.origin.x, s.origin.y, s.origin.z]
                    .map((v) => v.toFixed(3))
                    .join(", ")}
                </p>
                <p>{s.axis_mapping}</p>
                <p>{s.operator_instruction}</p>
                <button
                  disabled={!s.valid}
                  onClick={() =>
                    confirmSetups(
                      def.setups
                        .filter((item) =>
                          item.id === s.id
                            ? !s.confirmation.confirmed
                            : item.confirmation.confirmed,
                        )
                        .map((item) => item.id),
                    )
                  }
                >
                  {s.confirmation.confirmed
                    ? "撤销装夹确认"
                    : "确认此装夹与坐标"}
                </button>
              </details>
            ))}
          </section>
        )}
        <details className="setupRow">
          <summary>项目工艺参数与输出设置</summary>
          {(
            [
              "hole_parameters",
              "pocket_parameters",
              "profile_parameters",
            ] as const
          ).map((key, i) =>
            ["depth", ["hole", "clearing", "profile"][i]].map((group, j) => (
              <MachiningParameters
                key={key + group}
                title={
                  ["孔", "槽腔", "外轮廓"][i] +
                  (j === 0 ? "深度与安全高度" : "工艺")
                }
                values={def[key][group]}
                change={(values) =>
                  edit((d) => {
                    Object.assign(d[key][group], values);
                    for (const op of d[
                      (
                        [
                          "hole_operations",
                          "pocket_operations",
                          "profile_operations",
                        ] as const
                      )[i]
                    ])
                      op.state = 1;
                  })
                }
              />
            )),
          )}
          <label>
            参考 NC 程序名
            <input
              value={def.post.program_name}
              onChange={(e) =>
                edit((d) => {
                  d.post.program_name = e.target.value;
                })
              }
            />
          </label>
          <label>
            <input
              type="checkbox"
              checked={def.post.coolant}
              onChange={(e) =>
                edit((d) => {
                  d.post.coolant = e.target.checked;
                })
              }
            />{" "}
            输出冷却液指令
          </label>
        </details>
      </section>
      <section hidden={view !== "operations"}>
        <h3>工序顺序 · {ordered.length}</h3>
        {!ordered.length && (
          <p className="emptyNote">
            尚未添加工序。可在下方规划加工区域，或加入已生成的孔加工。
          </p>
        )}
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
            {family === "surface_finishing_operations" && op.parameters && (
              <MachiningParameters
                title="雕刻工艺参数"
                values={op.parameters}
                change={(values) =>
                  edit((d) => {
                    const item = d[family].find((o) => o.id === op.id)!;
                    Object.assign(item.parameters!, values);
                    item.state = 1;
                  })
                }
              />
            )}
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
                {(["depth", "clearing"] as const).map((group) => (
                  <MachiningParameters
                    key={group}
                    title={
                      group === "depth"
                        ? "此工序深度与安全高度"
                        : "此工序清料与进刀"
                    }
                    values={(op.parameters || def.pocket_parameters)[group]}
                    change={(values) =>
                      edit((d) => {
                        const item = d[family].find((o) => o.id === op.id)!;
                        item.parameters ??= structuredClone(
                          d.pocket_parameters,
                        );
                        Object.assign(item.parameters[group], values);
                        item.state = 1;
                      })
                    }
                  />
                ))}
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
              <button disabled={!op.enabled} onClick={() => simulate(op.id)}>
                仿真至此工序
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
                    [
                      "vertical_feed",
                      "horizontal_feed",
                      "spindle_speed",
                    ] as const
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
      </section>
    </>
  );
}
