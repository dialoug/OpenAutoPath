import { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { Viewport, type Face, type Segment } from "./Viewport";
import "./style.css";
import {
  PlanningControls,
  type CamFeature,
  type Cutter,
  type PlanOptions,
} from "./PlanningForm";
import { WorkspacePanel } from "./WorkspacePanel";
import {
  assignedTools,
  artifact,
  json,
  submit,
  type Project,
  type Library,
} from "./workspace";

type Job = {
  name?: string;
  files?: string[];
  id: string;
  modelId: string;
  action: string;
  state: string;
  stage?: string;
  error?: string;
};
type Hole = { id: string; diameter: number; depth: number; faces: string[] };
type Tool = Cutter;
type Review = {
  operation?: string;
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
  const [panel, setPanel] = useState<"project" | "library" | null>(null);
  const [library, setLibrary] = useState(
    () => localStorage.getItem("cam.library") || "",
  );
  const [libraryTools, setLibraryTools] = useState<Tool[]>([]);
  useEffect(() => {
    let live = true;
    if (library)
      json<Library>(artifact(library, "library.oaptl"))
        .then((doc) => {
          if (live) setLibraryTools(assignedTools(doc));
        })
        .catch((e) => {
          if (live) setError(String(e));
        });
    return () => {
      live = false;
    };
  }, [library]);
  const [modelListVersion, refreshModels] = useState(0);
  const [models, setModels] = useState<Job[]>([]);
  useEffect(() => {
    let live = true;
    request("/api/session")
      .then(() => json<Job[]>("/api/workspace"))
      .then((items) => {
        if (live) setModels(items.filter((j) => j.action === "analyze"));
      })
      .catch((e) => {
        if (live) setError(String(e));
      });
    return () => {
      live = false;
    };
  }, [modelListVersion]);
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
  const [features, setFeatures] = useState<CamFeature[]>([]);
  const [mode, setMode] = useState<"hole" | "pocket">("hole");
  const [pocketID, setPocketID] = useState("");
  const [featureState, setFeatureState] = useState<
    "loading" | "ready" | "error"
  >("loading");
  const [featureError, setFeatureError] = useState("");
  const [retryFeatures, retry] = useState(0);
  const [planningJob, setPlanningJob] = useState<Job | null>(null);
  const pockets = features.filter((f) => f.kind === "pocket");
  const pocket = pockets.find((f) => f.id === pocketID) || pockets[0];
  const [holes, setHoles] = useState<Hole[]>([]);
  const [selected, select] = useState("");
  const [tools, setTools] = useState<Tool[]>([]);
  const [tool, setTool] = useState("");
  const [feed, setFeed] = useState(100);
  const [rpm, setRPM] = useState(3000);
  const [faces, setFaces] = useState<Face[]>([]);
  const [path, setPath] = useState<Segment[]>([]);
  const [stock, setStock] = useState<{
    mesh: Face[];
    volume: number;
    operations: number;
  } | null>(null);
  const [showStock, setShowStock] = useState(false);
  const [review, setReview] = useState<Review | null>(null);
  useEffect(() => {
    if (!review || review.operation || !holes.length) return;
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
    setStock(null);
    setShowStock(false);
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
          refreshModels((n) => n + 1);
        } else if (j.action === "simulate") {
          const [r, mesh] = await Promise.all([
            json<{ volume: number; operations: number }>(
              artifact(j.id, "result.json"),
            ),
            json<Face[]>(artifact(j.id, "stock.json")),
          ]);
          if (live) {
            setStock({ ...r, mesh });
            setShowStock(true);
            setReview(null);
            setPath([]);
          }
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
    const controller = new AbortController();
    setFeatureState("loading");
    setFeatureError("");
    setPocketID("");
    const base = "/api/jobs/" + model + "/files/";
    Promise.all(
      ["result.json", "mesh.json"].map(async (file) =>
        (await request(base + file)).json(),
      ),
    )
      .then(async ([result, mesh]) => {
        if (live) {
          setFeatures(result.features ?? []);
          setTools(result.tools);
          setHoles(result.holes);
          setFaces(mesh);
          select(result.holes[0]?.id || "");
          const match =
            !localStorage.getItem("cam.library") &&
            result.tools.find(
              (t: Tool) =>
                t.type === "drill" &&
                Math.abs(t.diameter - result.holes[0]?.diameter) < 0.001,
            );
          setTool(match?.id || "");
          let regions = result;
          if (
            !Array.isArray(result.features) ||
            (result.featureError && retryFeatures > 0)
          ) {
            const signal = controller.signal;
            const previous =
              retryFeatures === 0 &&
              (await json<Job[]>("/api/workspace", { signal })).find(
                (j) => j.action === "features" && j.modelId === model,
              );
            const job =
              previous ||
              (await submit(
                "/api/models/" + model + "/features",
                {},
                signal,
                () => {},
              ));
            regions = await json<{
              features: CamFeature[];
              featureError?: string;
            }>(artifact(job.id, "result.json"), { signal });
          }
          if (!live) return;
          setFeatures(regions.features);
          setFeatureState(regions.featureError ? "error" : "ready");
          setFeatureError(regions.featureError || "");
        }
      })
      .catch((e) => {
        if (live) {
          setError(String(e));
          setFeatureError(String(e));
          setFeatureState("error");
        }
      });
    return () => {
      live = false;
      controller.abort();
    };
  }, [model, retryFeatures]);
  async function upload(file: File) {
    setBusy(true);
    setError("");
    try {
      await request("/api/session");
      const j: Job = await (
        await request("/api/models?name=" + encodeURIComponent(file.name), {
          method: "POST",
          body: file,
        })
      ).json();
      setFeatures([]);
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
  async function planPocket(options: PlanOptions) {
    setBusy(true);
    setError("");
    const signal = new AbortController().signal;
    try {
      const key = "cam.project." + model;
      const version = localStorage.getItem(key);
      const document = version
        ? await json<Project>(artifact(version, "project.oapjob"), { signal })
        : undefined;
      const planned = await submit(
        "/api/models/" + model + "/projects/plan",
        { document, libraryId: library, ...options },
        signal,
        setPlanningJob,
      );
      localStorage.setItem(key, planned.id);
      setPanel("project");
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
      setPlanningJob(null);
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
          body: JSON.stringify({
            hole: selected,
            tool,
            feed,
            rpm,
            libraryId: library,
          }),
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
        <span>加工工作台</span>
        <nav>
          <button
            disabled={active || !model}
            onClick={() => setPanel("project")}
          >
            加工项目
          </button>
          <button disabled={active} onClick={() => setPanel("library")}>
            刀库管理
          </button>
        </nav>
      </header>
      <main>
        <section className="workspace">
          <div className="bar">
            <h1>{name || "模型视图"}</h1>
            {models.length > 0 && (
              <select
                aria-label="历史模型"
                value={model}
                disabled={active}
                onChange={(e) => {
                  const chosen = models.find((m) => m.id === e.target.value);
                  if (!chosen) return;
                  const title = chosen.name || "model.step";
                  setName(title);
                  localStorage.setItem("cam.name", title);
                  setPath([]);
                  setReview(null);
                  setFaces([]);
                  setHoles([]);
                  setFeatures([]);
                  setError("");
                  track(chosen);
                }}
              >
                <option value="" disabled>
                  历史模型
                </option>
                {models.map((m) => (
                  <option key={m.id} value={m.id}>
                    {m.name || "model.step"} · {m.id.slice(0, 6)}
                  </option>
                ))}
              </select>
            )}
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
            faces={showStock && stock ? stock.mesh : faces}
            highlighted={
              (mode === "pocket" ? pocket?.faces : hole?.faces) || []
            }
            path={mode === "pocket" && !review?.operation ? [] : path}
            onPick={(id) => {
              if (mode === "pocket") {
                const hit = pockets.find((f) => f.faces.includes(id));
                if (hit) setPocketID(hit.id);
                return;
              }
              const h =
                holes.find((v) => v.id === id) ||
                holes.find((v) => v.faces.includes(id));
              if (h) select(h.id);
            }}
          />
          <footer>
            旋转 · 拖动　　缩放 · 滚轮　　选择 ·{" "}
            {mode === "pocket" ? "点击槽腔面" : "点击孔壁"}
          </footer>
          {stock && (
            <footer>
              <button onClick={() => setShowStock(!showStock)}>
                {showStock ? "显示零件" : "显示余料"}
              </button>
              已仿真 {stock.operations} 道工序 · 剩余体积{" "}
              {stock.volume.toFixed(2)} mm³
            </footer>
          )}
        </section>
        <aside>
          <h2>加工特征</h2>
          <div className="featureModes" role="group" aria-label="加工类型">
            <button
              aria-pressed={mode === "hole"}
              onClick={() => setMode("hole")}
            >
              孔加工 <small>{holes.length}</small>
            </button>
            <button
              aria-pressed={mode === "pocket"}
              onClick={() => setMode("pocket")}
            >
              槽腔加工{" "}
              <small>{featureState === "ready" ? pockets.length : ""}</small>
            </button>
          </div>
          {mode === "hole" ? (
            <>
              <label>
                孔段
                <select
                  value={selected}
                  onChange={(e) => select(e.target.value)}
                >
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
                  {library ? "活动刀库" : "模拟刀库"}
                  <select
                    required
                    value={tool}
                    onChange={(e) => {
                      const chosen = (library ? libraryTools : tools).find(
                        (t) => t.id === e.target.value,
                      );
                      setTool(e.target.value);
                      if (chosen?.feed) setFeed(chosen.feed);
                      if (chosen?.rpm) setRPM(chosen.rpm);
                    }}
                  >
                    <option value="">选择刀具</option>
                    {(library ? libraryTools : tools).map((t) => (
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
            </>
          ) : (
            <fieldset className="pocketFields" disabled={active || !model}>
              {!model ? (
                <p>上传 STEP 或打开历史模型以识别槽腔。</p>
              ) : featureState === "loading" ? (
                <p role="status">正在识别槽腔…</p>
              ) : featureState === "error" ? (
                <>
                  <p role="alert">{featureError}</p>
                  <button onClick={() => retry((n) => n + 1)}>重新识别</button>
                </>
              ) : pocket ? (
                <>
                  <label>
                    槽腔区域
                    <select
                      value={pocket.id}
                      onChange={(e) => setPocketID(e.target.value)}
                    >
                      {pockets.map((f, i) => (
                        <option key={f.id} value={f.id}>
                          槽腔 {i + 1} · {f.id}
                        </option>
                      ))}
                    </select>
                  </label>
                  <PlanningControls
                    key={pocket.id}
                    feature={pocket}
                    cutters={library ? libraryTools : tools}
                    plan={planPocket}
                  />
                </>
              ) : (
                <p>该模型未识别到槽腔。</p>
              )}
            </fieldset>
          )}
          {planningJob && (
            <div role="status">
              正在规划槽腔…{" "}
              <button
                onClick={() =>
                  void request("/api/jobs/" + planningJob.id, {
                    method: "DELETE",
                  }).catch((e) => setError(String(e)))
                }
              >
                取消槽腔规划
              </button>
            </div>
          )}
          {job && !planningJob && (
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
                {review.operation || review.hole} {review.tool}
              </p>
              <p>
                {review.moves} 段移动 · {review.validation}
              </p>
              {!review.operation && (
                <p>
                  孔覆盖审计 {review.coverageComplete ? "完成" : "未完成"}
                  ，实体扫掠审计 {review.sweepComplete ? "完成" : "未完成"}。
                </p>
              )}
              <a href={"/api/jobs/" + job.id + "/files/reference.nc"} download>
                下载本次参考 NC
              </a>
              {job.files?.includes("project.oapjob") && (
                <a href={artifact(job.id, "project.oapjob")} download>
                  下载加工项目
                </a>
              )}
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
      {panel && (
        <WorkspacePanel
          key={panel + model}
          mode={panel}
          model={model}
          features={features}
          faces={faces}
          cutters={library ? libraryTools : tools}
          library={library}
          close={() => setPanel(null)}
          preview={(j) => {
            setReview(null);
            setPath([]);
            track(j);
          }}
          activate={(id) => {
            setLibrary(id);
            localStorage.setItem("cam.library", id);
            setTool("");
          }}
        />
      )}
    </>
  );
}
createRoot(document.getElementById("root")!).render(<App />);
