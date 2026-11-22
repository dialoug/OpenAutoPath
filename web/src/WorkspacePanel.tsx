import { useEffect, useRef, useState } from "react";
import {
  artifact,
  json,
  submit,
  response,
  type Job,
  type Project,
  type Library,
} from "./workspace";
import type { Face } from "./Viewport";
import { LibraryEditor } from "./LibraryEditor";
import { ProjectEditor } from "./ProjectEditor";
import { PlanningForm, type CamFeature, type Cutter } from "./PlanningForm";

type Props = {
  mode: "project" | "library";
  model: string;
  library: string;
  features: CamFeature[];
  faces: Face[];
  cutters: Cutter[];
  close: () => void;
  activate: (id: string) => void;
  preview: (job: Job) => void;
};
export function WorkspacePanel({
  mode,
  model,
  library,
  features,
  faces,
  cutters,
  close,
  activate,
  preview,
}: Props) {
  const isLib = mode === "library";
  const key = isLib ? "cam.libraryEditor" : "cam.project." + model;
  const [items, itemsSet] = useState<Job[]>([]);
  const [doc, setDoc] = useState<Library | Project | null>(null);
  const [version, setVersion] = useState("");
  const [dirty, setDirty] = useState(false),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const [pending, setPending] = useState<Job | null>(null);
  const life = useRef(new AbortController());
  const file = isLib ? "library.oaptl" : "project.oapjob";
  const saved = items.filter((j) =>
    isLib
      ? j.action === "library"
      : j.modelId === model && j.files?.includes("project.oapjob"),
  );
  const generated = items.filter(
    (j) =>
      j.action === "generate" &&
      j.modelId === model &&
      j.files?.includes("project.oapjob"),
  );
  const url = isLib ? "/api/libraries" : "/api/models/" + model + "/projects";
  async function reload() {
    itemsSet(
      await json<Job[]>("/api/workspace", { signal: life.current.signal }),
    );
  }
  async function open(id: string) {
    const value = await json<Library | Project>(artifact(id, file), {
      signal: life.current.signal,
    });
    setDoc(value);
    setVersion(id);
    setDirty(false);
    localStorage.setItem(key, id);
  }
  async function work(fn: () => Promise<void>) {
    setBusy(true);
    setError("");
    try {
      await fn();
    } catch (e) {
      if (!life.current.signal.aborted) setError(String(e));
    } finally {
      setPending(null);
      setBusy(false);
    }
  }
  useEffect(() => {
    const ctrl = new AbortController();
    life.current = ctrl;
    void work(async () => {
      await response("/api/session", { signal: ctrl.signal });
      await reload();
      const id = localStorage.getItem(key) || (isLib ? library : "");
      if (id) await open(id);
    });
    return () => ctrl.abort();
  }, []);
  const discard = () => !dirty || window.confirm("放弃当前未保存的修改？");
  function edit(value: Library | Project) {
    setDoc(value);
    setDirty(true);
  }
  async function save(value?: Library | Project, appendId?: string) {
    if (isLib && value && "library" in value) {
      const slots = value.library.controllers.map((s) => s.tool_number);
      if (
        slots.some((n) => !Number.isInteger(n) || n < 1) ||
        new Set(slots).size !== slots.length
      )
        throw Error("每个刀位须为唯一的正整数");
    }
    const job = await submit(
      url,
      { document: value, appendId },
      life.current.signal,
      setPending,
    );
    await open(job.id);
    await reload();
  }
  return (
    <div className="docBackdrop">
      <section
        className="docPanel"
        role="dialog"
        aria-modal="true"
        aria-label={isLib ? "刀库管理" : "加工项目"}
      >
        <div className="docTitle">
          <h2>{isLib ? "刀库管理" : "加工项目"}</h2>
          <button
            className="secondary"
            disabled={busy}
            onClick={() => {
              if (discard()) close();
            }}
          >
            返回工作台
          </button>
        </div>
        <fieldset disabled={busy}>
          <label>
            服务器版本
            <select
              value={version}
              onChange={(e) => {
                if (e.target.value && discard())
                  void work(() => open(e.target.value));
              }}
            >
              <option value="">选择已保存版本</option>
              {version && !saved.some((j) => j.id === version) && (
                <option value={version}>当前版本</option>
              )}
              {saved.map((j) => (
                <option key={j.id} value={j.id}>
                  {j.name || j.id.slice(0, 8)} ·{" "}
                  {j.created ? new Date(j.created).toLocaleString() : ""} ·{" "}
                  {j.id.slice(0, 6)}
                </option>
              ))}
            </select>
          </label>
          <div className="docActions">
            <button
              onClick={() => {
                if (discard()) void work(() => save());
              }}
            >
              {isLib ? "从内置刀库新建" : "新建项目"}
            </button>
            <label className="upload">
              导入 {isLib ? ".oaptl" : ".oapjob"}
              <input
                aria-label="导入文件"
                type="file"
                accept={isLib ? ".oaptl,.fftl" : ".oapjob,.ffjob"}
                onChange={(e) => {
                  const f = e.target.files?.[0];
                  e.target.value = "";
                  if (f && discard())
                    void work(async () => {
                      if (f.size > 32 * 1024 * 1024)
                        throw Error("文件超过 32 MiB");
                      await save(JSON.parse(await f.text()));
                    });
                }}
              />
            </label>
          </div>
          {doc && (
            <>
              {isLib ? (
                <LibraryEditor
                  value={doc as Library}
                  change={edit}
                  versions={saved.filter((j) => j.id !== version)}
                />
              ) : (
                <ProjectEditor
                  value={doc as Project}
                  change={edit}
                  cutters={cutters}
                  generated={generated}
                  append={(id) => void work(() => save(doc, id))}
                  rebuild={(operation, tool) =>
                    void work(async () => {
                      const j = await submit(
                        url + "/rebuild",
                        {
                          document: doc,
                          operation,
                          tool,
                          libraryId: tool ? library : undefined,
                        },
                        life.current.signal,
                        setPending,
                      );
                      localStorage.setItem(key, j.id);
                      preview(j);
                      close();
                    })
                  }
                />
              )}
              <div className="docActions">
                {!isLib && (
                  <PlanningForm
                    features={features}
                    faces={faces}
                    cutters={cutters}
                    plan={(options) =>
                      void work(async () => {
                        const j = await submit(
                          url + "/plan",
                          { document: doc, libraryId: library, ...options },
                          life.current.signal,
                          setPending,
                        );
                        await open(j.id);
                        await reload();
                      })
                    }
                  />
                )}
                {!isLib && (
                  <button
                    onClick={() =>
                      void work(async () => {
                        const j = await submit(
                          url + "/simulate",
                          { document: doc },
                          life.current.signal,
                          setPending,
                        );
                        localStorage.setItem(key, j.id);
                        preview(j);
                        close();
                      })
                    }
                  >
                    仿真已启用工序
                  </button>
                )}
                <button
                  disabled={!dirty}
                  onClick={() => void work(() => save(doc))}
                >
                  保存新版本
                </button>
                {version && !dirty && (
                  <a href={artifact(version, file)} download={file}>
                    下载 {isLib ? ".oaptl" : ".oapjob"}
                  </a>
                )}
                {isLib && (
                  <button
                    disabled={dirty || !version}
                    onClick={() => {
                      activate(version);
                      close();
                    }}
                  >
                    用于加工
                  </button>
                )}
              </div>
            </>
          )}
        </fieldset>
        {busy && (
          <p role="status">
            正在处理…{" "}
            {pending && (
              <button
                onClick={() =>
                  void response("/api/jobs/" + pending.id, {
                    method: "DELETE",
                    signal: life.current.signal,
                  }).catch((e) => setError(String(e)))
                }
              >
                取消任务
              </button>
            )}
          </p>
        )}
        {error && <pre role="alert">{error}</pre>}
      </section>
    </div>
  );
}
