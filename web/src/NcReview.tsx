import { useMemo, useState } from "react";
import { artifact } from "./workspace";

export type NcProgram = {
  filename: string;
  setup: string;
  tool: string;
  operations: string[];
  warnings: { line: number; message: string }[];
  text: string;
  lines: number;
};

export function NcReview({
  job,
  programs,
}: {
  job: string;
  programs: NcProgram[];
}) {
  const [index, setIndex] = useState(0);
  const [checked, setChecked] = useState<string[]>([]);
  const p = programs[index];
  const [page, setPage] = useState(0);
  const lines = useMemo(() => p.text.trimEnd().split(/\r?\n/), [p]);
  const pages = Math.ceil(lines.length / 200);
  return (
    <section aria-label="参考 NC 审查">
      <h3>参考 NC 审查</h3>
      <label>
        程序文件
        <select
          value={index}
          onChange={(e) => {
            setPage(0);
            setIndex(+e.target.value);
          }}
        >
          {programs.map((item, i) => (
            <option key={item.filename} value={i}>
              {checked.includes(item.filename) ? "✓ " : ""}
              {item.filename}
            </option>
          ))}
        </select>
      </label>
      <p>
        {p.setup} · {p.tool} · {p.operations.length} 道工序 · {p.lines} 行
      </p>
      {p.warnings.map((w, i) => (
        <p key={i}>
          {w.line ? "第 " + w.line + " 行 · " : ""}
          {w.message}
        </p>
      ))}
      <pre className="ncPreview">
        {lines
          .slice(page * 200, (page + 1) * 200)
          .map(
            (line, i) => String(page * 200 + i + 1).padStart(4) + "  " + line,
          )
          .join("\n")}
      </pre>
      {pages > 1 && (
        <div className="docActions">
          <button disabled={!page} onClick={() => setPage(page - 1)}>
            上一页
          </button>{" "}
          {page + 1} / {pages}{" "}
          <button
            disabled={page + 1 >= pages}
            onClick={() => setPage(page + 1)}
          >
            下一页
          </button>
        </div>
      )}
      <label>
        <input
          type="checkbox"
          checked={checked.includes(p.filename)}
          onChange={(e) =>
            setChecked(
              e.target.checked
                ? [...checked, p.filename]
                : checked.filter((f) => f !== p.filename),
            )
          }
        />{" "}
        已核对此文件
      </label>
      {checked.length === programs.length && (
        <a href={artifact(job, "programs.zip")} download="programs.zip">
          下载全部参考 NC（ZIP）
        </a>
      )}
    </section>
  );
}
