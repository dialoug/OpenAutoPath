import { useEffect, useRef, useState } from "react";
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
import { loadStockReplay, type StockReplay } from "./stockReplay";
import { SimulationQuality } from "./SimulationQuality";
import { SingleNcReview } from "./NcReview";
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
  jobStatus,
  JobCancelled,
  isPending,
  json,
  submit,
  type Project,
  type Library,
  type Job,
} from "./workspace";
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
  const [ncPreview, setNcPreview] = useState<{
    id: string;
    hasReport: boolean;
  } | null>(null);
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
  const [openedJob, setOpenedJob] = useState("");
  const [error, setError] = useState("");
  const [planCancelled, setPlanCancelled] = useState(false);
  const [busy, setBusy] = useState(false);
  const [features, setFeatures] = useState<CamFeature[]>([]);
  const [mode, setMode] = useState<"hole" | CamFeature["kind"]>("hole");
  const [recognition, setRecognition] = useState<Recognition | null>(null);
  const [colorFeatures, setColorFeatures] = useState(false);
  const [manual, setManual] = useState<"" | "pocket" | "slot">("");
  const [floor, setFloor] = useState("");
  const [regionID, setRegionID] = useState("");
  const [featureState, setFeatureState] = useState<
    "loading" | "ready" | "error" | "cancelled"
  >("loading");
  const [featureError, setFeatureError] = useState("");
  const [featureJob, setFeatureJob] = useState<Job | null>(null);
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
    quality?: number;
    materialSpacing?: number;
    displaySpacing?: number;
    removedVolume?: number;
    completedMoves?: number;
    totalMoves?: number;
    operations: number;
  } | null>(null);
  const [quality, setQuality] = useState(() => {
    const stored = Number(localStorage.getItem("cam.simulationQuality"));
    return [2, 5, 10].includes(stored) ? stored : 10;
  });
  const replayCursor = useRef(0);
  const qualityPicker = (
    <SimulationQuality
      value={quality}
      onChange={(level) => {
        localStorage.setItem("cam.simulationQuality", String(level));
        setQuality(level);
      }}
    />
  );
  const [stockView, setStockView] = useState("");
  const [showStock, setShowStock] = useState(false);
  const [replay, setReplay] = useState<StockReplay | null>(null);
  const replayVisible = inspector === "results" ? replay : null;
  const [review, setReview] = useState<Review | null>(null);
  useEffect(() => {
    if (!review || review.operation || !holes.length) return;
    select(review.hole);
    setTool(review.toolId);
    setFeed(review.feed);
    setRPM(review.rpm);
  }, [review, holes]);
  const hole = holes.find((h) => h.id === selected);
  const trackedJob = job?.id === saved ? job : null;
  const readingResult = trackedJob?.state === "done" && openedJob !== saved;
  const active = busy || Boolean(saved && !trackedJob) || isPending(trackedJob) || readingResult;
  const loadingFeatures = Boolean(model) && featureState === "loading";
  const featureStatus = isPending(featureJob) ? jobStatus(featureJob) : "正在读取模型结果";
  const currentTask = planningJob ?? (isPending(trackedJob) ? trackedJob : null);
  const cancelTask = currentTask ?? (!active && loadingFeatures ? featureJob : null);
  function statusText() {
    if (currentTask) return jobStatus(currentTask);
    if (busy) return "正在处理请求";
    if (saved && !trackedJob) return "正在读取任务状态";
    if (readingResult) return "正在读取任务结果";
    if (error) return trackedJob?.state === "failed" && error === (trackedJob.error || "任务失败")
      ? "任务失败" : "操作出错";
    if (loadingFeatures) return featureStatus;
    if (planCancelled || trackedJob?.state === "cancelled") return "任务已取消";
    if (model && featureState === "cancelled") return "加工信息加载已取消";
    if (model && featureState === "error") return "加工信息加载失败";
    return "就绪";
  }
  function track(j: Job) {
    setPlanCancelled(false);
    setOpenedJob("");
    setReplay(null);
    setReview(null);
    setPath([]);
    setStock(null);
    setShowStock(false);
    setJob(j);
    localStorage.setItem("cam.job", j.id);
    setSaved(j.id);
  }
  useEffect(() => {
    if (!saved) return;
    const controller = new AbortController();
    let live = true;
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const j: Job = await (await request("/api/jobs/" + saved)).json();
        if (!live) return;
        setJob(j);
        if (isPending(j)) {
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
        } else if (j.action === "replay") {
          const data = await loadStockReplay(j.id, controller.signal);
          if (live) {
            const resume = Number(
              sessionStorage.getItem("cam.replayMove." + j.id),
            );
            setReplay({
              ...data,
              startMove:
                Number.isInteger(resume) &&
                resume >= 0 &&
                resume <= data.path.length
                  ? resume
                  : 0,
            });
            setInspector("results");
          }
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
        if (live) setOpenedJob(j.id);
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
      controller.abort();
      clearTimeout(timer);
    };
  }, [saved]);
  useEffect(() => {
    if (!model) return;
    let live = true;
    const controller = new AbortController();
    setFeatureState("loading");
    setFeatureJob(null);
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
                (updated) => { if (live) setFeatureJob(updated); },
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
          if (e instanceof JobCancelled) {
            setFeatureState("cancelled");
            return;
          }
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
    setPlanCancelled(false);
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
      if (e instanceof JobCancelled) setPlanCancelled(true);
      else setError(String(e));
    } finally {
      setBusy(false);
      setPlanningJob(null);
    }
  }
  async function requestStock(
    action: "replay" | "simulate",
    operation?: string,
    completedMoves?: number,
  ) {
    if (!job) return;
    setBusy(true);
    setError("");
    try {
      const document = await json<Project>(artifact(job.id, "project.oapjob"));
      const j = await json<Job>(
        "/api/models/" + model + "/projects/" + action,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            document,
            operation,
            completedMoves,
            quality:
              action === "simulate" &&
              completedMoves !== undefined &&
              replay?.quality
                ? replay.quality
                : quality,
          }),
        },
      );
      if (replay && action === "replay")
        sessionStorage.setItem(
          "cam.replayMove." + j.id,
          String(replayCursor.current),
        );
      sessionStorage.setItem("cam.replayReturn." + j.id, job.id);
      track(j);
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
    }
  }
  async function returnToSource() {
    if (!job) return;
    const id = sessionStorage.getItem("cam.replayReturn." + job.id);
    if (!id) return;
    try {
      track(await json<Job>("/api/jobs/" + id));
    } catch (e) {
      setError(String(e));
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
  const currentMode = modes.find((m) => m.key === mode)!;
  const ModeIcon = currentMode.icon;
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
                  <span>{currentMode.count}</span>
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
                        ? featureStatus
                        : featureState === "error"
                          ? "加工信息加载失败"
                          : featureState === "cancelled" ? "加工信息加载已取消"
                          : mode === "engrave" ? "暂无可用装夹方向" : "暂无匹配特征"}
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
                  {replayVisible
                    ? "余料回放"
                    : stock && showStock
                      ? "仿真视图"
                      : "模型视图"}
                </h2>
                <span className="viewUnit">
                  {replayVisible || (stock && showStock) ? "IPW" : "STEP"}
                </span>
              </div>
              <Viewport
                cameraTools
                replay={replayVisible || undefined}
                onReplayPosition={(move) => {
                  replayCursor.current = move;
                }}
                inspectStock={
                  !active && (review?.operation || replayVisible)
                    ? (moves, op, cursor) => {
                        if (replayVisible && job)
                          sessionStorage.setItem(
                            "cam.replayMove." + job.id,
                            String(cursor),
                          );
                        void requestStock(
                          "simulate",
                          op ?? review?.operation,
                          moves,
                        );
                      }
                    : undefined
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
                path={
                  replayVisible
                    ? replayVisible.path
                    : mode !== "hole" && !review?.operation
                      ? []
                      : path
                }
                onPick={(id) => {
                  if (showStock || replayVisible) return;
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
                {replayVisible
                  ? "拖动进度条检查余料"
                  : stock && showStock
                    ? "查看仿真余料"
                    : mode === "engrave"
                      ? "从列表选择装夹方向"
                      : mode === "hole"
                        ? "点击孔壁"
                        : manual && mode === "pocket" ? "点击加工底面" : "点击" + regionLabel + "面"}
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
                  加工结果
                  {(stock || review || replay) && <b className="resultDot" />}
                </button>
              </div>
              <div className="inspectorBody" hidden={inspector !== "settings"}>
                <div className="selectionTitle">
                  <span className="sectionGlyph">
                    <ModeIcon size={19} />
                  </span>
                  <div>
                    <small>{mode === "engrave" ? "当前装夹方向" : "当前特征"}</small>
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
                      <p>上传 STEP 或打开历史模型以选择加工目标。</p>
                    ) : featureState === "loading" ? (
                      <p role="status">{featureStatus}</p>
                    ) : featureState === "error" || featureState === "cancelled" ? (
                      <>
                        {featureState === "cancelled"
                          ? <p>加工信息加载已取消</p>
                          : <p role="alert">{featureError}</p>}
                        <button onClick={() => { setError(""); retry((n) => n + 1); }}>
                          重试
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
                      <p>{mode === "engrave" ? "暂无可用装夹方向。" : `该模型未识别到${regionLabel}。`}</p>
                    )}
                  </fieldset>
                )}
              </div>
              <div
                className="inspectorBody results"
                hidden={inspector !== "results"}
              >
                {(review || stock || replay) && qualityPicker}
                {replay && replay.quality !== quality && (
                  <button
                    className="primary"
                    disabled={active}
                    onClick={() =>
                      void requestStock(
                        "replay",
                        (replay.selectedOnly ?? replay.operations.length === 1)
                          ? replay.operations[0].id
                          : undefined,
                      )
                    }
                  >
                    按所选精度重新回放
                  </button>
                )}
                {stock && stock.quality !== quality && (
                  <button
                    disabled={active}
                    onClick={() =>
                      void requestStock(
                        "simulate",
                        stock.throughOperation || undefined,
                        stock.completedMoves,
                      )
                    }
                  >
                    按所选精度重算余料
                  </button>
                )}
                {stock?.materialSpacing && stock.displaySpacing && (
                  <p>
                    材料间距 {stock.materialSpacing.toFixed(4)} mm · 显示间距{" "}
                    {stock.displaySpacing.toFixed(4)} mm
                  </p>
                )}
                {(review || stock) && (
                  <button
                    className="primary"
                    disabled={active}
                    onClick={() =>
                      void requestStock(
                        "replay",
                        review?.operation || stock?.throughOperation,
                      )
                    }
                  >
                    {stock && !stock.throughOperation
                      ? "回放项目余料"
                      : "回放加工余料"}
                  </button>
                )}
                {job &&
                  sessionStorage.getItem("cam.replayReturn." + job.id) && (
                    <button
                      disabled={active}
                      onClick={() => void returnToSource()}
                    >
                      返回上次结果
                    </button>
                  )}
                {replay && (
                  <section>
                    <h2>余料回放</h2>
                    <p>
                      {replay.operations.length} 道工序 · {replay.path.length}{" "}
                      段移动 · {replay.frames.length} 帧余料
                    </p>
                    <p>
                      回放显示当前移动处或之前最近的余料帧。选择“查看此步余料”可按当前回放精度计算到这一步。
                    </p>
                  </section>
                )}
                {!stock && !review && !replay && (
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
                    <button
                      onClick={() =>
                        setNcPreview({
                          id: job.id,
                          hasReport: !!job.files?.includes("programs.json"),
                        })
                      }
                    >
                      审查本次参考 NC
                    </button>
                    {(!job.files?.length ||
                      job.files.includes("reference.nc")) && (
                      <a href={artifact(job.id, "reference.nc")} download>
                        下载本次参考 NC
                      </a>
                    )}
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
            className={"statusDot " + (error ? "bad" : active || loadingFeatures ? "pending" : "")}
          />
          <strong>{statusText()}</strong>
          {isPending(cancelTask) && (
            <button
              onClick={() =>
                void request("/api/jobs/" + cancelTask.id, {
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
      {ncPreview && (
        <SingleNcReview
          job={ncPreview.id}
          hasReport={ncPreview.hasReport}
          onClose={() => setNcPreview(null)}
        />
      )}
      {panel && (
        <WorkspacePanel
          key={panel + model}
          mode={panel}
          quality={quality}
          qualityPicker={qualityPicker}
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
