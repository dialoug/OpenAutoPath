import { useState } from "react";
import type { Cutter } from "./PlanningForm";

export function MillingToolPicker({
  cutters,
  value,
  select,
  label,
}: {
  cutters: Cutter[];
  value: string;
  select: (id: string) => void;
  label: string;
}) {
  const [query, setQuery] = useState("");
  const mills = cutters.filter((c) =>
    ["end_mill", "ball_end_mill", "bull_nose"].includes(c.type),
  );
  const matches = mills.filter((c) =>
    (c.name + " " + c.id + " " + c.diameter)
      .toLowerCase()
      .includes(query.trim().toLowerCase()),
  );
  return (
    <>
      <label>
        查找铣刀
        <input
          type="search"
          value={query}
          onChange={(e) => {
            setQuery(e.target.value);
            select("");
          }}
          placeholder="名称、直径或 ID"
        />
      </label>
      <label>
        {label}
        <select value={value} onChange={(e) => select(e.target.value)}>
          <option value="">
            {matches.length ? "选择铣刀" : "没有匹配的铣刀"}
          </option>
          {matches.map((c) => (
            <option key={c.id} value={c.id}>
              {c.name} · Ø{c.diameter} mm
            </option>
          ))}
        </select>
      </label>
    </>
  );
}
