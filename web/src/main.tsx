import { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { Viewport, type Face, type Segment } from "./Viewport";
import "./style.css";

type Job = {
  id: string;
  modelId: string;
  action: string;
  state: string;
  stage?: string;
  error?: string;
};
type Hole = { id: string; diameter: number; depth: number; faces: string[] };
type Tool = { id: string; name: string; diameter: number; type: string };
type Review = {
  toolId: string;
  feed: number;
  rpm: number;
  hole: string;
  tool: string;
  validation: string;
  issues: { code: string; message: string }[];
  coverageComplete: boolean;
  sweepComplete: boolean;
  moves: number;
};
const states: Record<string, string> = {
  queued: "等待计算",
  running: "计算中",
  done: "完成",
  failed: "失败",
  cancelled: "已取消",
  analyzing: "正在识别特征",
  meshing: "正在生成网格",
  generating: "正在计算刀路",
};
async function request(url: string, options?: RequestInit) {
  const response = await fetch(url, options);
  if (!response.ok)
    throw new Error((await response.text()).trim() || response.statusText);
  return response;
}
function App() {
  const [saved, setSaved] = useState(
    () => localStorage.getItem("cam.job") || "",
  );
  const [model, setModel] = useState(
    () => localStorage.getItem("cam.model") || "",
  );
  const [name, setName] = useState(
    () => localStorage.getItem("cam.name") || "",
  );
  const [job, setJob] = useState<Job | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [holes, setHoles] = useState<Hole[]>([]);
  const [selected, select] = useState("");
  const [tools, setTools] = useState<Tool[]>([]);
  const [tool, setTool] = useState("");
  const [feed, setFeed] = useState(100);
  const [rpm, setRPM] = useState(3000);
  const [faces, setFaces] = useState<Face[]>([]);
  const [path, setPath] = useState<Segment[]>([]);
  const [review, setReview] = useState<Review | null>(null);
  useEffect(() => {
    if (!review || !holes.length) return;
    select(review.hole);
    setTool(review.toolId);
    setFeed(review.feed);
    setRPM(review.rpm);
  }, [review, holes]);
  const hole = holes.find((h) => h.id === selected);
  const active =
    busy ||
    Boolean(
      saved &&
      (!job || job.id !== saved || ["queued", "running"].includes(job.state)),
    );
  function track(j: Job) {
    setJob(j);
    localStorage.setItem("cam.job", j.id);
    setSaved(j.id);
  }
  useEffect(() => {
    if (!saved) return;
    let live = true;
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const j: Job = await (await request("/api/jobs/" + saved)).json();
        if (!live) return;
        setJob(j);
        if (["queued", "running"].includes(j.state)) {
          timer = setTimeout(poll, 700);
          return;
        }
        if (j.state === "failed") {
          setError(j.error || "任务失败");
          return;
        }
        if (j.state !== "done") return;
        if (j.action === "analyze") {
          localStorage.setItem("cam.model", j.id);
          setModel(j.id);
        } else {
          const base = "/api/jobs/" + j.id + "/files/";
          const [r, p] = await Promise.all(
            ["result.json", "path.json"].map(async (file) =>
              (await request(base + file)).json(),
            ),
          );
          if (live) {
            setReview(r);
            setPath(p);
          }
        }
      } catch (e) {
        if (live) {
          setError(String(e));
          setSaved("");
          localStorage.removeItem("cam.job");
        }
      }
    }
    void poll();
    return () => {
      live = false;
      clearTimeout(timer);
    };
  }, [saved]);
  useEffect(() => {
    if (!model) return;
    let live = true;
    const base = "/api/jobs/" + model + "/files/";
    Promise.all(
      ["result.json", "mesh.json"].map(async (file) =>
        (await request(base + file)).json(),
      ),
    )
      .then(([result, mesh]) => {
        if (live) {
          setTools(result.tools);
          setHoles(result.holes);
          setFaces(mesh);
          select(result.holes[0]?.id || "");
          const match = result.tools.find(
            (t: Tool) =>
              t.type === "drill" &&
              Math.abs(t.diameter - result.holes[0]?.diameter) < 0.001,
          );
          setTool(match?.id || "");
        }
      })
      .catch((e) => {
        if (live) setError(String(e));
      });
    return () => {
      live = false;
    };
  }, [model]);
  async function upload(file: File) {
    setBusy(true);
    setError("");
    try {
      await request("/api/session");
      const j: Job = await (
        await request("/api/models", { method: "POST", body: file })
      ).json();
      setHoles([]);
      select("");
      setFaces([]);
      setModel("");
      localStorage.removeItem("cam.model");
      setName(file.name);
      localStorage.setItem("cam.name", file.name);
      setPath([]);
      setReview(null);
      track(j);
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
    }
  }
  async function generate() {
    setBusy(true);
    setError("");
    try {
      const j = await (
        await request("/api/models/" + model + "/toolpaths", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ hole: selected, tool, feed, rpm }),
        })
      ).json();
      setReview(null);
      setPath([]);
      track(j);
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
    }
  }
  return (
    <>
      <header>
        <b>OpenAutoPath</b>
        <span>孔加工工作台</span>
      </header>
      <main>
        <section className="workspace">
          <div className="bar">
            <h1>{name || "模型视图"}</h1>
            <label className="upload">
              上传 STEP
              <input
                aria-label="上传 STEP"
                disabled={active}
                type="file"
                accept=".step,.stp"
                onChange={(e) => {
                  const f = e.target.files?.[0];
                  if (f) void upload(f);
                  e.target.value = "";
                }}
              />
            </label>
          </div>
          <Viewport
            faces={faces}
            highlighted={hole?.faces || []}
            path={path}
            onPick={(id) => {
              const h =
                holes.find((v) => v.id === id) ||
                holes.find((v) => v.faces.includes(id));
              if (h) select(h.id);
            }}
          />
          <footer>旋转 · 拖动　　缩放 · 滚轮　　选择 · 点击孔壁</footer>
        </section>
        <aside>
          <h2>
            加工特征 <small>{holes.length}</small>
          </h2>
          <label>
            孔段
            <select value={selected} onChange={(e) => select(e.target.value)}>
              <option value="" disabled>
                选择孔段
              </option>
              {holes.map((h) => (
                <option key={h.id} value={h.id}>
                  {h.id}　Ø {h.diameter.toFixed(2)} mm
                </option>
              ))}
            </select>
          </label>
          {hole && <p>孔深 {hole.depth.toFixed(2)} mm</p>}
          <form
            onSubmit={(e) => {
              e.preventDefault();
              void generate();
            }}
          >
            <h2>加工参数</h2>
            <label>
              模拟刀库
              <select
                required
                value={tool}
                onChange={(e) => setTool(e.target.value)}
              >
                <option value="">选择刀具</option>
                {tools.map((t) => (
                  <option key={t.id} value={t.id}>
                    {t.name}
                  </option>
                ))}
              </select>
            </label>
            <label>
              进给 (mm/min)
              <input
                type="number"
                step="any"
                min=".001"
                max="100000"
                required
                value={feed}
                onChange={(e) => setFeed(+e.target.value)}
              />
            </label>
            <label>
              主轴转速 (rpm)
              <input
                type="number"
                step="any"
                min="1"
                max="100000"
                required
                value={rpm}
                onChange={(e) => setRPM(+e.target.value)}
              />
            </label>
            <button disabled={!hole || active}>生成参考刀路</button>
          </form>
          {job && (
            <section aria-live="polite">
              <h2>任务状态</h2>
              <p>
                {
                  states[
                    job.state === "running" && job.stage ? job.stage : job.state
                  ]
                }
              </p>
              {active && (
                <button
                  className="secondary"
                  onClick={() =>
                    request("/api/jobs/" + job.id, { method: "DELETE" }).catch(
                      (e) => setError(String(e)),
                    )
                  }
                >
                  取消任务
                </button>
              )}
            </section>
          )}
          {review && job && (
            <section>
              <h2>参考 NC</h2>
              <p>
                {review.hole} · {review.tool}
              </p>
              <p>
                {review.moves} 段移动 · {review.validation}
              </p>
              <p>
                孔覆盖审计 {review.coverageComplete ? "完成" : "未完成"}
                ，实体扫掠审计 {review.sweepComplete ? "完成" : "未完成"}。
              </p>
              <a href={"/api/jobs/" + job.id + "/files/reference.nc"} download>
                下载本次参考 NC
              </a>
              {!!review.issues?.length && (
                <details>
                  <summary>验证详情</summary>
                  <ul>
                    {review.issues.map((issue, i) => (
                      <li key={i}>
                        {issue.code} · {issue.message}
                      </li>
                    ))}
                  </ul>
                </details>
              )}
            </section>
          )}
          {error && <pre role="alert">{error}</pre>}
        </aside>
      </main>
    </>
  );
}
createRoot(document.getElementById("root")!).render(<App />);
