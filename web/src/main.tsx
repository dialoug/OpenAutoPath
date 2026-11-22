import { useEffect, useState } from "react";
import {
  Box,
  CircleDot,
  FileBox,
  FolderKanban,
  History,
  Layers3,
  MousePointer2,
  PanelLeft,
  Pentagon,
  Route,
  Settings2,
  Spline,
  Upload,
  Wrench,
} from "lucide-react";
import { ModelStart } from "./ModelStart";
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
type Recognition = {
  throughFaces: string[];
  axes: {
    axis: string;
    roots: number;
    slots: number;
    pockets: number;
    faces: number;
  }[];
};
type Hole = {
  kind?: string;
  id: string;
  diameter: number;
  depth: number;
  faces: string[];
};
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
  const [inspector, setInspector] = useState<"settings" | "results">(
    "settings",
  );
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
  const [mode, setMode] = useState<"hole" | CamFeature["kind"]>("hole");
  const [recognition, setRecognition] = useState<Recognition | null>(null);
  const [colorFeatures, setColorFeatures] = useState(false);
  const [manual, setManual] = useState<"" | "pocket" | "slot">("");
  const [floor, setFloor] = useState("");
  const [regionID, setRegionID] = useState("");
  const [featureState, setFeatureState] = useState<
    "loading" | "ready" | "error"
  >("loading");
  const [featureError, setFeatureError] = useState("");
  const [retryFeatures, retry] = useState(0);
  const [planningJob, setPlanningJob] = useState<Job | null>(null);
  const regionNames = { pocket: "槽腔", profile: "外轮廓", engrave: "雕刻" };
  const regionLabel = mode === "hole" ? "孔" : regionNames[mode];
  const regions = features.filter((f) => f.kind === mode);
  if (mode === "engrave" && regions.length === 6)
    regions.push({ id: "engrave-all", kind: "engrave", faces: [] });
  const region = regions.find((f) => f.id === regionID) || regions[0];
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
    comparison?: {
      allowance: number;
      overcut: number;
      missing: number;
      certainOvercut: number;
      boundaryUncertainty: number;
      attribution: {
        operation: string;
        sequence: number;
        volume: number;
        certain: number;
      }[];
    };
    differences?: Record<string, Face[]>;
    throughOperation?: string;
    removedVolume?: number;
    completedMoves?: number;
    totalMoves?: number;
    operations: number;
  } | null>(null);
  const [stockView, setStockView] = useState("");
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
          const [r, mesh, differences] = await Promise.all([
            json<{ volume: number; operations: number }>(
              artifact(j.id, "result.json"),
            ),
            json<Face[]>(artifact(j.id, "stock.json")),
            j.files?.includes("differences.json")
              ? json<Record<string, Face[]>>(artifact(j.id, "differences.json"))
              : undefined,
          ]);
          if (live) {
            setInspector("results");
            setStockView("");
            setStock({ ...r, mesh, differences });
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
            setInspector("results");
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
    setRegionID("");
    setRecognition(null);
    setManual("");
    setFloor("");
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
            !result.recognition ||
            (result.featureError && retryFeatures > 0)
          ) {
            const signal = controller.signal;
            const previous =
              retryFeatures === 0 &&
              (await json<Job[]>("/api/workspace", { signal })).find(
                (j) => j.action === "features" && j.modelId === model,
              );
            const cached =
              previous &&
              (await json<{ recognition?: Recognition }>(
                artifact(previous.id, "result.json"),
                { signal },
              ));
            const job =
              (previous && cached && cached.recognition && previous) ||
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
          setRecognition(regions.recognition ?? null);
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
      setInspector("settings");
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
  async function plan(options?: PlanOptions) {
    setBusy(true);
    setError("");
    const signal = new AbortController().signal;
    try {
      const key = "cam.project." + model;
      const version = localStorage.getItem(key);
      const document = version
        ? await json<Project>(artifact(version, "project.oapjob"), { signal })
        : undefined;
      if (
        !options &&
        document?.job.hole_operations.length &&
        !window.confirm("按当前刀库重新规划项目中全部孔工序？")
      )
        return;
      const planned = await submit(
        "/api/models/" +
          model +
          (options ? "/projects/plan" : "/projects/holes"),
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
  async function inspectStock(completedMoves: number) {
    if (!job || !review?.operation) return;
    setBusy(true);
    setError("");
    try {
      const document = await json<Project>(artifact(job.id, "project.oapjob"));
      const j = await json<Job>("/api/models/" + model + "/projects/simulate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          document,
          operation: review.operation,
          completedMoves,
        }),
      });
      setReview(null);
      setPath([]);
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
  const modes = [
    { key: "hole", label: "孔加工", icon: CircleDot, count: holes.length },
    {
      key: "pocket",
      label: "槽腔加工",
      icon: Layers3,
      count: features.filter((f) => f.kind === "pocket").length,
    },
    {
      key: "profile",
      label: "外轮廓加工",
      icon: Pentagon,
      count: features.filter((f) => f.kind === "profile").length,
    },
    {
      key: "engrave",
      label: "雕刻加工",
      icon: Spline,
      count: features.filter((f) => f.kind === "engrave").length,
    },
  ] as const;
  const ModeIcon = modes.find((m) => m.key === mode)!.icon;
  const items =
    mode === "hole"
      ? holes.map((h) => ({
          id: h.id,
          label: h.id,
          detail: "Ø " + h.diameter.toFixed(2),
        }))
      : regions.map((f, i) => ({
          id: f.id,
          label:
            f.id === "engrave-all"
              ? "全部六面装夹"
              : regionLabel + " " + (i + 1),
          detail: f.id.replace(/^(pocket|profile|engrave)-/, ""),
        }));
  return (
    <>
      <header className="appHeader">
        <div className="brand">
          <span className="brandMark">
            <Layers3 size={22} />
          </span>
          OpenAutoPath
        </div>
        <nav aria-label="主导航">
          <button
            aria-current={!panel ? "page" : undefined}
            onClick={() => setPanel(null)}
          >
            <Box />
            加工工作台
          </button>
          <button
            aria-current={panel === "project" ? "page" : undefined}
            disabled={active || !model}
            onClick={() => setPanel("project")}
          >
            <FolderKanban />
            加工项目
          </button>
          <button
            aria-current={panel === "library" ? "page" : undefined}
            disabled={active}
            onClick={() => setPanel("library")}
          >
            <Wrench />
            刀库管理
          </button>
        </nav>
      </header>
      <main className="appMain">
        <div className="pageBar">
          <div>
            <h1>加工工作台</h1>
            <div className="documentLine">
              <FileBox size={14} />
              {model || active
                ? name || "正在打开模型"
                : "选择一个 STEP 模型开始加工"}
            </div>
          </div>
          <div className="modelActions">
            {" "}
            {models.length > 0 && (
              <label className="history">
                <History size={15} />
                <select
                  aria-label="历史模型"
                  value={model}
                  disabled={active}
                  onChange={(e) => {
                    const chosen = models.find((m) => m.id === e.target.value);
                    if (!chosen) return;
                    const title = chosen.name || "model.step";
                    setInspector("settings");
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
              </label>
            )}
            {model && (
              <label className="upload primary">
                <Upload size={16} /> 上传 STEP
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
            )}
          </div>
        </div>
        {!model ? (
          <ModelStart busy={active} upload={upload} error={setError} />
        ) : (
          <div className="workGrid">
            <section className="explorer">
              <div className="cardHeading">
                <h2>
                  <PanelLeft />
                  特征浏览器
                </h2>
              </div>
              <div className="explorerScroll">
                <h3 className="groupLabel">加工类型</h3>
                <div
                  role="group"
                  aria-label="加工类型"
                  className="featureModes"
                >
                  {modes.map(({ key, label, icon: Icon, count }) => (
                    <button
                      key={key}
                      aria-pressed={mode === key}
                      onClick={() => {
                        setMode(key);
                        setInspector("settings");
                        setShowStock(false);
                      }}
                    >
                      <Icon size={17} />
                      <span>{label}</span>
                      <small>{count}</small>
                    </button>
                  ))}
                </div>
                <div className="listHeading">
                  <h3 className="groupLabel">
                    {mode === "hole"
                      ? "孔段"
                      : mode === "engrave"
                        ? "装夹方向"
                        : "识别区域"}
                  </h3>
                  <span>{items.length}</span>
                </div>
                <div
                  className="featureList"
                  role="group"
                  aria-label="选择加工特征"
                >
                  {items.map((item) => (
                    <button
                      key={item.id}
                      aria-pressed={
                        (mode === "hole" ? hole?.id : region?.id) === item.id
                      }
                      onClick={() => {
                        if (mode === "hole") select(item.id);
                        else {
                          setRegionID(item.id);
                          setManual("");
                        }
                        setInspector("settings");
                        setShowStock(false);
                      }}
                    >
                      <span className="treeDot" />
                      <span>{item.label}</span>
                      <small>{item.detail}</small>
                    </button>
                  ))}
                  {!items.length && (
                    <p>
                      {featureState === "loading"
                        ? "正在识别…"
                        : "暂无匹配特征"}
                    </p>
                  )}
                </div>
                {recognition && (
                  <details className="setupRow">
                    <summary>三轴识别明细</summary>
                    <label>
                      <input
                        type="checkbox"
                        checked={colorFeatures}
                        onChange={(e) => setColorFeatures(e.target.checked)}
                      />{" "}
                      按特征着色
                    </label>
                    <p>孔壁（绿） · 槽腔（橙） · 外轮廓（蓝） · 通结构（紫）</p>
                    <p>
                      通结构面 {recognition.throughFaces.join("、") || "无"}
                    </p>
                    {recognition.axes.map((a) => (
                      <p key={a.axis}>
                        {a.axis} · 根面 {a.roots} · 盲槽 {a.slots} · 盲腔{" "}
                        {a.pockets} · 跟踪面 {a.faces}
                      </p>
                    ))}
                  </details>
                )}
              </div>
            </section>
            <section className="viewerCard workspace">
              <div className="cardHeading">
                <h2>
                  <Box />
                  {stock && showStock ? "仿真视图" : "模型视图"}
                </h2>
                <span className="viewUnit">
                  {stock && showStock ? "IPW" : "STEP"}
                </span>
              </div>
              <Viewport
                cameraTools
                inspectStock={
                  review?.operation && !active ? inspectStock : undefined
                }
                faces={
                  showStock && stock
                    ? (stock.differences?.[stockView] ?? stock.mesh)
                    : faces
                }
                colors={
                  colorFeatures && !showStock
                    ? Object.fromEntries([
                        ...features.flatMap((f) =>
                          f.faces.map((id) => [
                            id,
                            f.kind === "pocket" ? 0xe8af60 : 0x82a9ce,
                          ]),
                        ),
                        ...(recognition?.throughFaces || []).map((id) => [
                          id,
                          0xc89cbe,
                        ]),
                        ...holes.flatMap((h) =>
                          h.faces.map((id) => [id, 0x59b29d]),
                        ),
                      ])
                    : undefined
                }
                highlighted={
                  mode === "pocket" && manual
                    ? floor
                      ? [floor]
                      : []
                    : (mode === "hole" ? hole?.faces : region?.faces) || []
                }
                path={mode !== "hole" && !review?.operation ? [] : path}
                onPick={(id) => {
                  if (showStock) return;
                  if (mode === "pocket" && manual) {
                    setInspector("settings");
                    setFloor(id);
                    return;
                  }
                  if (mode !== "hole") {
                    const hit = regions.find((f) => f.faces.includes(id));
                    if (hit) {
                      setInspector("settings");
                      setRegionID(hit.id);
                    }
                    return;
                  }
                  const h =
                    holes.find((v) => v.id === id) ||
                    holes.find((v) => v.faces.includes(id));
                  if (h) {
                    setInspector("settings");
                    select(h.id);
                  }
                }}
              />
              <footer>
                <MousePointer2 size={14} /> 拖动旋转 · 滚轮缩放{" "}
                <span className="viewHint">/</span>{" "}
                {mode === "engrave"
                  ? "从列表选择装夹方向"
                  : mode === "hole"
                    ? "点击孔壁"
                    : "点击" + regionLabel + "面"}
              </footer>
            </section>
            <aside className="inspector">
              <div className="cardHeading">
                <h2>
                  <Settings2 />
                  加工面板
                </h2>
              </div>
              <div className="inspectorTabs" role="group" aria-label="加工面板">
                <button
                  aria-pressed={inspector === "settings"}
                  onClick={() => setInspector("settings")}
                >
                  参数设置
                </button>
                <button
                  aria-pressed={inspector === "results"}
                  onClick={() => setInspector("results")}
                >
                  加工结果{(stock || review) && <b className="resultDot" />}
                </button>
              </div>
              <div className="inspectorBody" hidden={inspector !== "settings"}>
                <div className="selectionTitle">
                  <span className="sectionGlyph">
                    <ModeIcon size={19} />
                  </span>
                  <div>
                    <small>当前特征</small>
                    <h3>
                      {mode === "hole"
                        ? hole?.kind || "孔加工"
                        : regionLabel + "加工"}{" "}
                      <code>
                        {mode === "hole"
                          ? hole?.id
                          : manual && mode === "pocket"
                            ? floor
                            : region?.id}
                      </code>
                    </h3>
                  </div>
                </div>
                {mode === "hole" ? (
                  <>
                    {hole && (
                      <p>
                        {hole.kind && hole.kind + " · "}孔深{" "}
                        {hole.depth.toFixed(2)} mm
                      </p>
                    )}
                    <form
                      onSubmit={(e) => {
                        e.preventDefault();
                        void generate();
                      }}
                    >
                      <h3>切削参数</h3>
                      <label>
                        {library ? "活动刀库" : "模拟刀库"}
                        <select
                          required
                          value={tool}
                          onChange={(e) => {
                            const chosen = (
                              library ? libraryTools : tools
                            ).find((t) => t.id === e.target.value);
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
                      <button className="primary" disabled={!hole || active}>
                        <Route size={15} />
                        生成参考刀路
                      </button>
                    </form>
                    <button
                      disabled={!holes.length || active}
                      onClick={() => void plan()}
                    >
                      按刀库自动规划全部孔
                    </button>
                  </>
                ) : (
                  <fieldset
                    className="pocketFields"
                    disabled={active || !model}
                  >
                    {mode === "pocket" && model && (
                      <label>
                        区域来源
                        <select
                          value={manual}
                          onChange={(e) => {
                            setManual(e.target.value as typeof manual);
                            setFloor("");
                            setShowStock(false);
                          }}
                        >
                          <option value="">自动识别</option>
                          <option value="pocket">由底面补腔</option>
                          <option value="slot">由底面补槽</option>
                        </select>
                      </label>
                    )}
                    {mode === "pocket" && manual && model ? (
                      <>
                        <label>
                          加工底面
                          <select
                            value={floor}
                            onChange={(e) => setFloor(e.target.value)}
                          >
                            <option value="">点击模型底面或选择面号</option>
                            {faces.map((f) => (
                              <option key={f.id}>{f.id}</option>
                            ))}
                          </select>
                        </label>
                        {floor && (
                          <PlanningControls
                            key={manual + floor}
                            feature={{
                              id: "manual-" + manual + "-" + floor,
                              kind: "pocket",
                              faces: [floor],
                            }}
                            cutters={library ? libraryTools : tools}
                            plan={plan}
                          />
                        )}
                      </>
                    ) : !model ? (
                      <p>上传 STEP 或打开历史模型以识别{regionLabel}。</p>
                    ) : featureState === "loading" ? (
                      <p role="status">正在识别{regionLabel}…</p>
                    ) : featureState === "error" ? (
                      <>
                        <p role="alert">{featureError}</p>
                        <button onClick={() => retry((n) => n + 1)}>
                          重新识别
                        </button>
                      </>
                    ) : region ? (
                      <>
                        <PlanningControls
                          key={region.id}
                          feature={region}
                          cutters={library ? libraryTools : tools}
                          plan={plan}
                        />
                      </>
                    ) : (
                      <p>该模型未识别到{regionLabel}。</p>
                    )}
                  </fieldset>
                )}
              </div>
              <div
                className="inspectorBody results"
                hidden={inspector !== "results"}
              >
                {!stock && !review && (
                  <div className="emptyResults">
                    <Route />
                    <p>生成刀路或运行仿真后，可在这里查看和下载结果。</p>
                  </div>
                )}
                {stock?.comparison && (
                  <details className="stockAudit">
                    <summary>余量与过切对比</summary>
                    <label>
                      查看区域
                      <select
                        value={stockView}
                        onChange={(e) => {
                          setStockView(e.target.value);
                          setShowStock(true);
                        }}
                      >
                        <option value="">仿真毛坯</option>
                        {Object.keys(stock.differences || {}).map((key) => (
                          <option key={key} value={key}>
                            {
                              {
                                allowance: "剩余加工余量",
                                overcut: "过切区域",
                                missing: "初始毛坯缺料",
                              }[key]
                            }
                          </option>
                        ))}
                      </select>
                    </label>
                    <p>
                      剩余加工余量 {stock.comparison.allowance.toFixed(3)} mm³ ·
                      初始缺料 {stock.comparison.missing.toFixed(3)} mm³
                    </p>
                    <p>
                      过切上界 {stock.comparison.overcut.toFixed(3)}{" "}
                      mm³，其中确定过切{" "}
                      {stock.comparison.certainOvercut.toFixed(3)}{" "}
                      mm³，边界不确定量{" "}
                      {stock.comparison.boundaryUncertainty.toFixed(3)} mm³。
                    </p>
                    {stock.comparison.attribution.map((a) => (
                      <p key={a.operation}>
                        工序 {a.sequence + 1} · {a.operation} · 过切上界{" "}
                        {a.volume.toFixed(3)} mm³
                      </p>
                    ))}
                  </details>
                )}
                {stock && (
                  <div className="stockSummary">
                    <button onClick={() => setShowStock(!showStock)}>
                      {showStock ? "显示零件" : "显示余料"}
                    </button>
                    {stock.throughOperation ? "阶段余料" : "最终余料"} · 已仿真{" "}
                    {stock.operations} 道完整工序
                    {stock.completedMoves !== undefined &&
                      " · 当前工序移动 " +
                        stock.completedMoves +
                        " / " +
                        stock.totalMoves}{" "}
                    · 剩余体积 {stock.volume.toFixed(2)} mm³
                    {stock.removedVolume !== undefined &&
                      " · 去除 " + stock.removedVolume.toFixed(2) + " mm³"}
                  </div>
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
                        ，实体扫掠审计{" "}
                        {review.sweepComplete ? "完成" : "未完成"}。
                      </p>
                    )}
                    <a
                      href={"/api/jobs/" + job.id + "/files/reference.nc"}
                      download
                    >
                      下载本次参考 NC
                    </a>
                    {job.files?.includes("toolpath.jsonl") && (
                      <a href={artifact(job.id, "toolpath.jsonl")} download>
                        下载槽腔刀路 JSONL
                      </a>
                    )}
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
              </div>
            </aside>
          </div>
        )}
        <div className="statusStrip" role="status">
          <span
            className={"statusDot " + (error ? "bad" : active ? "pending" : "")}
          />
          <strong>
            {planningJob
              ? "正在规划" + regionLabel
              : active
                ? (job && states[job.stage || job.state]) || "处理中"
                : error
                  ? "任务提示"
                  : "就绪"}
          </strong>
          {(planningJob || (active && job)) && (
            <button
              onClick={() =>
                void request("/api/jobs/" + (planningJob || job)!.id, {
                  method: "DELETE",
                }).catch((e) => setError(String(e)))
              }
            >
              取消任务
            </button>
          )}
          <span className="statusFile">{model ? name : "当前未打开模型"}</span>
          <span className="statusUnits">
            长度单位 <b>mm</b>
          </span>
        </div>
        {error && <pre role="alert">{error}</pre>}
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
