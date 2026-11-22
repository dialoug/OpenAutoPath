import { useState } from "react";
import { ToolPreview } from "./ToolPreview";
import { LibraryToolImport } from "./LibraryToolImport";
import type { Library, Bit, Job } from "./workspace";

const cutterNames: Record<string, string> = {
  drill: "钻头",
  center_drill: "中心钻",
  indexable_drill: "可转位钻",
  end_mill: "平底铣刀",
  ball_end_mill: "球头铣刀",
  bull_nose: "圆角铣刀",
  face_mill: "面铣刀",
  chamfer_mill: "倒角刀",
  boring_bar: "镗刀",
  reamer: "铰刀",
  tap: "丝锥",
  probe: "测头",
};
export function LibraryEditor({
  value,
  change,
  versions,
}: {
  value: Library;
  versions: Job[];
  change: (doc: Library) => void;
}) {
  const [chosen, choose] = useState("");
  const [query, setQuery] = useState("");
  const [filter, setFilter] = useState("");
  const [preview, setPreview] = useState(false);
  const lib = value.library;
  const slots = new Map(lib.controllers.map((s) => [s.tool_id, s]));
  const visible = lib.tools.filter(
    (t) =>
      (!filter || t.type === filter) &&
      (
        t.name +
        " " +
        t.id +
        " T" +
        (slots.get(t.id)?.tool_number ?? "") +
        " " +
        t.geometry.diameter
      )
        .toLowerCase()
        .includes(query.trim().toLowerCase()),
  );
  const bit = visible.find((t) => t.id === chosen) || visible[0];
  const slot = lib.controllers.find((s) => s.tool_id === bit?.id);
  function edit(fn: (d: Library["library"]) => void) {
    const next = structuredClone(value);
    fn(next.library);
    change(next);
  }
  function updateBit(fn: (b: Bit) => void) {
    edit((l) => {
      const b = l.tools.find((t) => t.id === bit.id)!;
      fn(b);
    });
  }
  function add() {
    const id = crypto.randomUUID?.() || "tool-" + Date.now();
    edit((l) => {
      l.tools.push({
        id,
        name: "新钻头",
        type: "drill",
        geometry: {
          diameter: 6,
          cutting_edge_length: 24,
          overall_length: 60,
          tip_angle: 118,
        },
      });
      l.controllers.push({
        tool_id: id,
        tool_number:
          Math.max(0, ...l.controllers.map((c) => c.tool_number)) + 1,
        cutting: {
          vertical_feed: 100,
          horizontal_feed: 100,
          spindle_speed: 3000,
        },
      });
    });
    setFilter("");
    setQuery("");
    choose(id);
  }
  return (
    <>
      <label>
        刀库名称
        <input
          value={lib.name}
          onChange={(e) =>
            edit((l) => {
              l.name = e.target.value;
            })
          }
        />
      </label>
      <label>
        机床
        <input
          value={lib.machine}
          onChange={(e) =>
            edit((l) => {
              l.machine = e.target.value;
            })
          }
        />
      </label>
      <div className="docGrid">
        <label>
          搜索刀具
          <input
            type="search"
            value={query}
            placeholder="名称、T 号、直径或 ID"
            onChange={(e) => setQuery(e.target.value)}
          />
        </label>
        <label>
          按类型筛选
          <select value={filter} onChange={(e) => setFilter(e.target.value)}>
            <option value="">全部类型</option>
            {Object.entries(cutterNames).map(([id, name]) => (
              <option key={id} value={id}>
                {name}
              </option>
            ))}
          </select>
        </label>
      </div>
      <LibraryToolImport
        versions={versions}
        onUse={(source, slot) => {
          edit((l) => {
            const imported = structuredClone(source);
            const idx = l.tools.findIndex((t) => t.id === source.id);
            if (idx >= 0) l.tools[idx] = imported;
            else l.tools.push(imported);
            if (slot) {
              const old = l.controllers.find((s) => s.tool_id === source.id);
              const nextNumber =
                old?.tool_number ??
                (l.controllers.some((s) => s.tool_number === slot.tool_number)
                  ? Math.max(0, ...l.controllers.map((s) => s.tool_number)) + 1
                  : slot.tool_number);
              const copy = {
                ...structuredClone(slot),
                tool_number: nextNumber,
              };
              if (old) Object.assign(old, copy);
              else l.controllers.push(copy);
            }
          });
          setFilter("");
          setQuery("");
          choose(source.id);
        }}
      />
      {!bit && <p>没有匹配的刀具。</p>}
      <div className="docActions">
        <button onClick={add}>新增刀具</button>
        {bit && (
          <button
            className="secondary"
            onClick={() =>
              edit((l) => {
                l.tools = l.tools.filter((t) => t.id !== bit.id);
                l.controllers = l.controllers.filter(
                  (s) => s.tool_id !== bit.id,
                );
              })
            }
          >
            移除所选刀具
          </button>
        )}
      </div>
      {bit && (
        <>
          <label>
            刀具列表（{visible.length}）
            <select value={bit.id} onChange={(e) => choose(e.target.value)}>
              {visible.map((t) => (
                <option key={t.id} value={t.id}>
                  {t.name}
                </option>
              ))}
            </select>
          </label>
          <label>
            刀具类型
            <select
              value={bit.type}
              onChange={(e) =>
                updateBit((b) => {
                  b.type = e.target.value;
                  if (b.type === "ball_end_mill")
                    b.geometry.corner_radius = b.geometry.diameter / 2;
                  else if (b.type === "bull_nose")
                    b.geometry.corner_radius = Math.min(
                      1,
                      b.geometry.diameter / 4,
                    );
                  else b.geometry.corner_radius = 0;
                })
              }
            >
              {Object.entries(cutterNames).map(([id, name]) => (
                <option key={id} value={id}>
                  {name}
                </option>
              ))}
            </select>
          </label>
          <label>
            刀具名称
            <input
              value={bit.name}
              onChange={(e) =>
                updateBit((b) => {
                  b.name = e.target.value;
                })
              }
            />
          </label>
          <label>
            刀具备注
            <input
              value={bit.note || ""}
              onChange={(e) =>
                updateBit((b) => {
                  b.note = e.target.value;
                })
              }
            />
          </label>
          <button aria-expanded={preview} onClick={() => setPreview(!preview)}>
            {preview ? "收起尺寸预览" : "预览刀具与夹持器尺寸"}
          </button>
          {preview && <ToolPreview bit={bit} />}
          <div className="docGrid">
            {(
              [
                "diameter",
                "cutting_edge_length",
                "overall_length",
                "shank_diameter",
                "holder_diameter",
                "holder_length",
                "corner_radius",
                "tip_angle",
              ] as const
            ).map((k, i) => (
              <label key={k}>
                {
                  [
                    "直径",
                    "刃长",
                    "总长",
                    "刀柄直径",
                    "夹持器直径",
                    "夹持器长度",
                    "圆角半径",
                    "刀尖角",
                  ][i]
                }{" "}
                ({k === "tip_angle" ? "°" : "mm"})
                <input
                  type="number"
                  min="0"
                  step="any"
                  value={bit.geometry[k] ?? 0}
                  onChange={(e) =>
                    updateBit((b) => {
                      b.geometry[k] = +e.target.value;
                    })
                  }
                />
              </label>
            ))}
            <label>
              刀位 T
              <input
                type="number"
                min="1"
                value={slot?.tool_number ?? ""}
                onChange={(e) =>
                  edit((l) => {
                    const n = Number(e.target.value);
                    const s = l.controllers.find((s) => s.tool_id === bit.id);
                    if (!n) {
                      l.controllers = l.controllers.filter(
                        (s) => s.tool_id !== bit.id,
                      );
                      return;
                    }
                    if (s) s.tool_number = n;
                    else
                      l.controllers.push({
                        tool_id: bit.id,
                        tool_number: n,
                        cutting: {
                          vertical_feed: 100,
                          horizontal_feed: 100,
                          spindle_speed: 3000,
                        },
                      });
                  })
                }
              />
            </label>
            <label>
              <input
                type="checkbox"
                checked={!!bit.through_coolant}
                onChange={(e) =>
                  updateBit((b) => {
                    b.through_coolant = e.target.checked;
                  })
                }
              />
              支持中心内冷
            </label>
            {slot && (
              <>
                <label>
                  冷却方式
                  <select
                    value={slot.coolant || "off"}
                    onChange={(e) =>
                      edit((l) => {
                        l.controllers.find(
                          (s) => s.tool_id === bit.id,
                        )!.coolant = e.target.value;
                      })
                    }
                  >
                    <option value="off">关闭</option>
                    <option value="flood">外冷</option>
                    <option value="through_tool">中心出水</option>
                  </select>
                </label>
                <label>
                  钻削循环
                  <select
                    value={slot.cutting.drill_cycle || "auto"}
                    onChange={(e) =>
                      edit((l) => {
                        l.controllers.find(
                          (s) => s.tool_id === bit.id,
                        )!.cutting.drill_cycle = e.target.value;
                      })
                    }
                  >
                    {["auto", "g81", "g73", "g83"].map((v) => (
                      <option key={v} value={v}>
                        {v === "auto" ? "自动" : v.toUpperCase()}
                      </option>
                    ))}
                  </select>
                </label>
              </>
            )}
            {slot &&
              (
                [
                  "vertical_feed",
                  "horizontal_feed",
                  "spindle_speed",
                  "horizontal_rapid",
                  "vertical_rapid",
                  "peck_depth",
                ] as const
              ).map((k, i) => (
                <label key={k}>
                  {
                    [
                      "轴向进给 (mm/min)",
                      "水平进给 (mm/min)",
                      "主轴转速 (rpm)",
                      "水平快速移动 (mm/min)",
                      "垂直快速移动 (mm/min)",
                      "每次钻深 Q (mm)",
                    ][i]
                  }
                  <input
                    type="number"
                    step="any"
                    min="0"
                    value={slot.cutting[k] ?? 0}
                    onChange={(e) =>
                      edit((l) => {
                        l.controllers.find(
                          (s) => s.tool_id === bit.id,
                        )!.cutting[k] = +e.target.value;
                      })
                    }
                  />
                </label>
              ))}
          </div>
        </>
      )}
    </>
  );
}
