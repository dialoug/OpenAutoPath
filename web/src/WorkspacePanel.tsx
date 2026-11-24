import { FolderKanban, Wrench, X, Save } from "lucide-react";
import { useEffect, useRef, useState, type ReactNode } from "react";
import {
  artifact,
  jobStatus,
  JobCancelled,
  isPending,
  json,
  submit,
  waitForJob,
  response,
  type Job,
  type Project,
  type Library,
} from "./workspace";
import type { Face } from "./Viewport";
import { NcReview, type NcProgram } from "./NcReview";
import { LibraryEditor } from "./LibraryEditor";
import { ProjectEditor } from "./ProjectEditor";
import { PlanningForm, type CamFeature, type Cutter } from "./PlanningForm";

type Props = {
  quality: number;
  qualityPicker: ReactNode;
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
  quality,
  qualityPicker,
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
  type Page = "operations" | "setup" | "output" | "files";
  const [page, setPage] = useState<Page>("operations");
  const dialog = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    const el = dialog.current!;
    el.showModal();
    return () => el.close();
  }, []);
  const isLib = mode === "library";
  const key = isLib ? "cam.libraryEditor" : "cam.project." + model;
  const [items, itemsSet] = useState<Job[]>([]);
  const [doc, setDoc] = useState<Library | Project | null>(null);
  type Coverage = {
    covered: number;
    eligible: number;
    input: number;
    issues: string[];
  };
  const [coverage, setCoverage] = useState<Coverage | null>(null);
  const [nc, setNC] = useState<{ job: string; programs: NcProgram[] } | null>(
    null,
  );
  const [version, setVersion] = useState("");
  const [dirty, setDirty] = useState(false),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const [pending, setPending] = useState<Job | null>(null);
  const [cancelled, setCancelled] = useState(false);
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
    const details =
      !isLib &&
      (await json<{ holeCoverage?: Coverage; programCount?: number }>(
        artifact(id, "result.json"),
        {
          signal: life.current.signal,
        },
      ));
    setCoverage((details && details.holeCoverage) || null);
    setNC(
      details && details.programCount
        ? {
            job: id,
            programs: await json<NcProgram[]>(artifact(id, "programs.json"), {
              signal: life.current.signal,
            }),
          }
        : null,
    );
    setDoc(value);
    setVersion(id);
    setDirty(false);
    localStorage.setItem(key, id);
    return details && details.programCount ? "output" : "operations";
  }
  async function work(fn: () => Promise<void>) {
    setBusy(true);
    setError("");
    setCancelled(false);
    try {
      await fn();
    } catch (e) {
      if (!life.current.signal.aborted) {
        if (e instanceof JobCancelled) setCancelled(true);
        else setError(String(e));
      }
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
    setNC(null);
    setDoc(value);
    setCoverage(null);
    setDirty(true);
  }
  function editPost(patch: Partial<Project["job"]["post"]>) {
    if (doc && "job" in doc)
      edit({
        ...doc,
        job: { ...doc.job, post: { ...doc.job.post, ...patch } },
      });
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
    if (!value || !doc) setPage("operations");
    await reload();
  }
  function simulate(operation?: string, replay = false) {
    void work(async () => {
      const j = await submit(
        url + (replay ? "/replay" : "/simulate"),
        { document: doc, operation, quality },
        life.current.signal,
        setPending,
      );
      localStorage.setItem(key, j.id);
      preview(j);
      close();
    });
  }
  const visible = doc ? page : "files";
  return (
    <dialog
      ref={dialog}
      onCancel={(e) => {
        e.preventDefault();
        if (!busy && discard()) close();
      }}
      className="docPanel"
      role="dialog"
      aria-modal="true"
      aria-label={isLib ? "刀库管理" : "加工项目"}
    >
      <div className="docTitle">
        <h2>
          {isLib ? <Wrench /> : <FolderKanban />}
          {isLib ? "刀库管理" : "加工项目"}
        </h2>
        <button
          className="secondary"
          disabled={busy}
          onClick={() => {
            if (discard()) close();
          }}
        >
          <X size={16} /> 返回工作台
        </button>
      </div>
      <div className="dialogTabs" role="group" aria-label="管理分区">
        {(isLib
          ? [
              ["operations", "刀具目录"],
              ["files", "文件版本"],
            ]
          : [
              ["operations", "加工工序"],
              ["setup", "装夹与毛坯"],
              ["output", "仿真与 NC"],
              ["files", "文件版本"],
            ]
        ).map(([p, label]) => (
          <button
            key={p}
            disabled={busy || (!doc && p !== "files")}
            aria-pressed={visible === p}
            onClick={() => setPage(p as Page)}
          >
            {label}
          </button>
        ))}
      </div>
      <div className="dialogScroll">
        <fieldset disabled={busy}>
          <div hidden={visible !== "files"}>
            <h3>{isLib ? "刀库文件" : "项目文件"}</h3>
            <label>
              服务器版本
              <select
                value={version}
                onChange={(e) => {
                  if (e.target.value && discard())
                    void work(async () => {
                      setPage(await open(e.target.value));
                    });
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
          </div>
          {!isLib &&
            doc &&
            (visible === "output" || visible === "operations") &&
            qualityPicker}
          {coverage && visible === "operations" && (
            <section role="status">
              <h3>孔覆盖审计</h3>
              <p>
                识别 {coverage.input} 个孔段，六面装夹范围内已覆盖{" "}
                {coverage.covered} / {coverage.eligible}。
              </p>
              {coverage.issues.map((message, i) => (
                <p key={i}>{message}</p>
              ))}
            </section>
          )}
          {doc && (
            <>
              {isLib ? (
                <div hidden={visible !== "operations"}>
                  <LibraryEditor
                    value={doc as Library}
                    change={edit}
                    versions={saved.filter((j) => j.id !== version)}
                  />
                </div>
              ) : (
                <div hidden={visible !== "operations" && visible !== "setup"}>
                  <ProjectEditor
                    view={visible === "setup" ? "setup" : "operations"}
                    value={doc as Project}
                    change={edit}
                    cutters={cutters}
                    assets={items.filter((j) => j.action === "analyze")}
                    bindStock={(asset) =>
                      void work(async () => {
                        const signal = life.current.signal;
                        const stockId =
                          typeof asset === "string"
                            ? asset
                            : (
                                await waitForJob(
                                  await json<Job>(
                                    "/api/models?name=" +
                                      encodeURIComponent(asset.name),
                                    { method: "POST", body: asset, signal },
                                  ),
                                  signal,
                                  setPending,
                                )
                              ).id;
                        const j = await submit(
                          url + "/stock",
                          { document: doc, stockId },
                          signal,
                          setPending,
                        );
                        await open(j.id);
                        await reload();
                      })
                    }
                    simulate={simulate}
                    confirmSetups={(confirmedSetups) =>
                      void work(async () => {
                        const j = await submit(
                          url + "/setups",
                          { document: doc, confirmedSetups },
                          life.current.signal,
                          setPending,
                        );
                        await open(j.id);
                        await reload();
                      })
                    }
                    key={version}
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
                </div>
              )}
              <div className="docActions">
                {!isLib && visible === "operations" && (
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
                {"job" in doc && visible === "output" && (
                  <div className="outputActions">
                    <h3>项目仿真</h3>
                    <p>按当前顺序计算已启用工序的余料。</p>
                    <button onClick={() => simulate()}>仿真已启用工序</button>
                    <button onClick={() => simulate(undefined, true)}>
                      回放已启用工序
                    </button>
                    <h3>参考 NC</h3>
                    <label>
                      参考 NC 程序名
                      <input
                        value={doc.job.post.program_name}
                        onChange={(e) =>
                          editPost({ program_name: e.target.value })
                        }
                      />
                    </label>
                    <label>
                      <input
                        type="checkbox"
                        checked={doc.job.post.coolant}
                        onChange={(e) =>
                          editPost({ coolant: e.target.checked })
                        }
                      />{" "}
                      输出冷却液指令
                    </label>
                    <p>核对各工序的装夹后，生成程序并逐文件审查。</p>
                    <button
                      onClick={() =>
                        void work(async () => {
                          setNC(null);
                          const j = await submit(
                            url + "/export",
                            { document: doc },
                            life.current.signal,
                            setPending,
                          );
                          await open(j.id);
                          await reload();
                        })
                      }
                    >
                      生成项目参考 NC 并审查
                    </button>
                  </div>
                )}
              </div>
            </>
          )}
        </fieldset>
        {nc && visible === "output" && <NcReview key={nc.job} {...nc} />}
      </div>
      <div className="dialogBottom">
        <small>
          {dirty
            ? "有未保存的修改"
            : version
              ? "版本 " + version.slice(0, 8)
              : "尚未打开版本"}
        </small>
        <fieldset disabled={busy} className="docActions">
          {doc && (
            <>
              <button
                className="primary"
                disabled={!dirty}
                onClick={() => void work(() => save(doc))}
              >
                <Save size={15} />
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
            </>
          )}
        </fieldset>
      </div>
      {busy && (
        <p role="status">
          {pending ? pending.state === "done" ? "正在读取任务结果" : jobStatus(pending) : "正在处理请求"}{" "}
          {isPending(pending) && (
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
      {cancelled && <p role="status">任务已取消</p>}
      {error && <pre role="alert">{error}</pre>}
    </dialog>
  );
}
