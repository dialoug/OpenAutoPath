import { useEffect, useMemo, useRef, useState } from "react";
import { artifact, json, response } from "./workspace";

type NcIssue = { line: number; message: string; code?: string };
export type NcProgram = {
  filename: string;
  setup: string;
  tool: string;
  operations: string[];
  warnings: NcIssue[];
  errors?: NcIssue[];
  text: string;
  lines: number;
  historicalPreview?: boolean;
};

const issueLabels: Record<string, string> = {
  reference_nc_only: "参考 NC 缺少机床投产调试数据，请勿直接上机执行。",
  empty_program: "程序为空。",
  missing_units: "缺少单位选择 G20 或 G21。",
  missing_g90: "缺少绝对坐标指令 G90。",
  missing_end: "缺少程序结束指令 M2 或 M30。",
  missing_work_offset: "缺少工件坐标系 G54–G59。",
  missing_safe_modal_reset: "安全模态复位不完整，请核对 G17、G40、G49 和 G80。",
  cycle_missing_r: "钻孔固定循环缺少 R 退刀平面。",
  comment_balance: "NC 注释的左右括号未配对。",
};
function codeLines(text: string) {
  const lines = text.split(/\r?\n/);
  if (lines.at(-1) === "") lines.pop();
  return lines;
}

export function NcReview({
  job,
  programs,
  single = false,
}: {
  job: string;
  programs: NcProgram[];
  single?: boolean;
}) {
  const key = "cam.ncChecked." + job;
  const [checked, setChecked] = useState<string[]>(() => {
    try {
      const saved: unknown = JSON.parse(sessionStorage.getItem(key) || "[]");
      return programs
        .map((p) => p.filename)
        .filter((name) => Array.isArray(saved) && saved.includes(name));
    } catch {
      return [];
    }
  });
  const [index, setIndex] = useState(0);
  const [line, setLine] = useState(1);
  const [jump, setJump] = useState("1");
  const [query, setQuery] = useState("");
  const [download, setDownload] = useState<{
    file: NcProgram;
    url: string;
  } | null>(null);
  const fileUrl =
    download?.file === programs[index] ? download?.url : undefined;
  const codeView = useRef<HTMLDivElement>(null);
  const file = programs[index];
  const rows = useMemo(() => codeLines(file?.text ?? ""), [file]);
  const pageSize = 200,
    page = Math.floor((line - 1) / pageSize);
  const pages = Math.max(1, Math.ceil(rows.length / pageSize));
  const issues = useMemo(
    () =>
      programs.flatMap((p, i) => [
        ...(p.errors ?? []).map((v) => ({ ...v, index: i, error: true })),
        ...p.warnings.map((v) => ({ ...v, index: i, error: false })),
      ]),
    [programs],
  );
  const errors = issues.filter((i) => i.error).length;
  const matches = useMemo(() => {
    const term = query.trim().toUpperCase();
    return term
      ? rows.flatMap((v, i) => (v.toUpperCase().includes(term) ? [i + 1] : []))
      : [];
  }, [rows, query]);
  const downloadable = file && !errors && checked.includes(file.filename);
  // Each object URL belongs to the displayed file and is discarded when it changes.
  useEffect(() => {
    if (!downloadable || single) {
      setDownload(null);
      return;
    }
    const url = URL.createObjectURL(
      new Blob([file.text], { type: "application/octet-stream" }),
    );
    setDownload({ file, url });
    return () => URL.revokeObjectURL(url);
  }, [downloadable, single, file]);
  function centerLine() {
    const code = codeView.current;
    const row = code?.querySelector<HTMLElement>('[aria-current="true"]');
    if (code && row)
      code.scrollTop =
        row.offsetTop - (code.clientHeight - row.offsetHeight) / 2;
  }
  useEffect(centerLine, [index, line]);
  function locate(i: number, n: number, focus = true) {
    if (i === index && n === line) centerLine();
    setIndex(i);
    setLine(n);
    setJump(String(n));
    if (focus) codeView.current?.focus({ preventScroll: true });
  }
  function findMatch(back: boolean) {
    if (!matches.length) return;
    const next = back
      ? (matches.filter((v) => v < line).at(-1) ?? matches.at(-1)!)
      : (matches.find((v) => v > line) ?? matches[0]);
    locate(index, next);
  }
  function mark(value: boolean) {
    const next = value
      ? [...checked, file.filename]
      : checked.filter((n) => n !== file.filename);
    setChecked(next);
    try {
      sessionStorage.setItem(key, JSON.stringify(next));
    } catch {
      /* Still retained in this view. */
    }
  }
  if (!file) return <p role="alert">该版本没有可审查的参考 NC 文件。</p>;
  return (
    <section className="ncReview" aria-label="参考 NC 审查">
      <h3>参考 NC 审查</h3>
      <p className="ncSummary">
        {programs.length} 个文件 ·{" "}
        {file.historicalPreview
          ? "未记录结构审查结果"
          : `${errors} 个错误 · ${issues.length - errors} 个警告`}{" "}
        · 已核对 {checked.length} / {programs.length}
      </p>
      {file.historicalPreview && (
        <p>此历史工序仅保存了 NC 原文。重算工序可生成结构审计记录。</p>
      )}
      {issues.length > 0 && (
        <details className="ncIssues" open>
          <summary>审计问题（{issues.length}）</summary>
          <ul>
            {issues.map((item, i) => (
              <li key={i}>
                <button
                  type="button"
                  title={[item.code, item.message].filter(Boolean).join(" · ")}
                  onClick={() => locate(item.index, item.line || 1)}
                >
                  <strong className={item.error ? "ncError" : "ncWarning"}>
                    {item.error ? "错误" : "警告"}
                  </strong>
                  <span>
                    {programs[item.index].filename} ·{" "}
                    {item.line ? "第 " + item.line + " 行" : "整个文件"}
                  </span>
                  <span>{issueLabels[item.code ?? ""] ?? item.message}</span>
                </button>
              </li>
            ))}
          </ul>
        </details>
      )}
      <label>
        程序文件
        <select
          value={index}
          onChange={(e) => locate(+e.target.value, 1, false)}
        >
          {programs.map((p, i) => (
            <option key={p.filename} value={i}>
              {checked.includes(p.filename) ? "✓ " : ""}
              {p.filename}
            </option>
          ))}
        </select>
      </label>
      <p>
        {[
          file.setup,
          file.tool,
          file.operations.length ? file.operations.length + " 道工序" : "",
          rows.length + " 行",
        ]
          .filter(Boolean)
          .join(" · ")}
      </p>
      {file.operations.length > 0 && (
        <details>
          <summary>关联工序</summary>
          <ul>
            {file.operations.map((id, i) => (
              <li key={i}>{id}</li>
            ))}
          </ul>
        </details>
      )}
      <div className="ncTools">
        <form
          onSubmit={(e) => {
            e.preventDefault();
            findMatch(false);
          }}
        >
          <label>
            查找指令或文本
            <input
              type="search"
              value={query}
              onChange={(e) => setQuery(e.target.value)}
            />
          </label>
          <button
            type="button"
            disabled={!matches.length}
            onClick={() => findMatch(true)}
          >
            上一个匹配
          </button>
          <button disabled={!matches.length}>下一个匹配</button>
          {query && <small aria-live="polite">{matches.length} 行匹配</small>}
        </form>
        <form
          onSubmit={(e) => {
            e.preventDefault();
            const n = Number(jump);
            if (Number.isInteger(n) && n >= 1 && n <= rows.length)
              locate(index, n);
          }}
        >
          <label>
            转到行
            <input
              type="number"
              required
              min={1}
              max={rows.length || 1}
              value={jump}
              onChange={(e) => setJump(e.target.value)}
            />
          </label>
          <button disabled={!rows.length}>跳转</button>
        </form>
      </div>
      <div
        className="ncPreview"
        tabIndex={0}
        ref={codeView}
        aria-label="NC 程序原文"
        role="region"
        onClick={(e) => {
          const row = (e.target as HTMLElement).closest<HTMLElement>(
            "[data-line]",
          );
          if (row) locate(index, Number(row.dataset.line));
        }}
        onKeyDown={(e) => {
          const destinations: Record<string, number> = {
            ArrowDown: line + 1,
            ArrowUp: line - 1,
            Home: 1,
            End: rows.length,
            PageDown: line + pageSize,
            PageUp: line - pageSize,
          };
          if (e.key in destinations && rows.length) {
            e.preventDefault();
            locate(
              index,
              Math.max(1, Math.min(rows.length, destinations[e.key])),
            );
          }
        }}
      >
        {rows.slice(page * pageSize, (page + 1) * pageSize).map((text, i) => {
          const n = page * pageSize + i + 1;
          return (
            <div
              key={n}
              data-line={n}
              aria-current={n === line ? "true" : undefined}
            >
              <span aria-hidden="true">{n}</span>
              <code>{text || " "}</code>
            </div>
          );
        })}
      </div>
      <div className="ncPaging">
        <small>
          第 {line} 行 · 第 {page + 1} / {pages} 页
        </small>
        {pages > 1 && (
          <>
            <button
              disabled={page === 0}
              onClick={() => locate(index, (page - 1) * pageSize + 1)}
            >
              上一页
            </button>
            <button
              disabled={page + 1 === pages}
              onClick={() => locate(index, (page + 1) * pageSize + 1)}
            >
              下一页
            </button>
          </>
        )}
      </div>
      <p>
        {errors
          ? "请修正结构错误并重新生成程序。"
          : "请逐文件核对刀具、工件坐标、转速进给、固定循环和退刀位置。"}
      </p>
      <label>
        <input
          type="checkbox"
          disabled={errors > 0}
          checked={checked.includes(file.filename)}
          onChange={(e) => mark(e.target.checked)}
        />{" "}
        已核对此文件
      </label>
      <div className="docActions">
        {downloadable && (single || fileUrl) && (
          <a
            href={single ? artifact(job, "reference.nc") : fileUrl}
            download={file.filename}
          >
            下载当前 NC 文件
          </a>
        )}
        {!single && !errors && checked.length === programs.length && (
          <a href={artifact(job, "programs.zip")} download="programs.zip">
            下载全部参考 NC（ZIP）
          </a>
        )}
      </div>
    </section>
  );
}

export function SingleNcReview({
  job,
  hasReport,
  onClose,
}: {
  job: string;
  hasReport: boolean;
  onClose: () => void;
}) {
  const dialog = useRef<HTMLDialogElement>(null);
  const [programs, setPrograms] = useState<NcProgram[] | null>(null);
  const [error, setError] = useState("");
  useEffect(() => {
    const el = dialog.current!;
    el.showModal();
    const controller = new AbortController();
    const options = { signal: controller.signal };
    async function load() {
      if (hasReport)
        return json<NcProgram[]>(artifact(job, "programs.json"), options);
      const text = await (
        await response(artifact(job, "reference.nc"), options)
      ).text();
      return [
        {
          filename: "reference.nc",
          setup: "",
          tool: "",
          operations: [],
          warnings: [],
          text,
          lines: codeLines(text).length,
          historicalPreview: true,
        },
      ];
    }
    load()
      .then((v) => {
        if (!controller.signal.aborted) setPrograms(v);
      })
      .catch((e) => {
        if (!controller.signal.aborted) setError(String(e));
      });
    return () => {
      controller.abort();
      el.close();
    };
  }, [job, hasReport]);
  return (
    <dialog
      ref={dialog}
      className="docPanel"
      aria-label="工序参考 NC"
      onCancel={onClose}
    >
      <div className="docTitle">
        <h2>工序参考 NC</h2>
        <button onClick={onClose}>返回工作台</button>
      </div>
      <div className="dialogScroll">
        {error ? (
          <p role="alert">{error}</p>
        ) : programs ? (
          <NcReview job={job} programs={programs} single />
        ) : (
          <p role="status">正在读取 NC 原文…</p>
        )}
      </div>
    </dialog>
  );
}
