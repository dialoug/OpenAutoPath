export type Job = {
  files?: string[];
  id: string;
  name?: string;
  created?: string;
  modelId: string;
  action: string;
  state: string;
  stage?: string;
  error?: string;
};

const stageLabels: Record<string, string> = {
  reading_step: "正在读取 STEP 文件",
  transferring_step: "正在转换 STEP 几何",
  indexing: "正在建立模型拓扑索引",
  previewing: "正在准备模型预览",
  preparing_project: "正在准备加工项目",
  recognizing_holes: "正在识别孔特征",
  recognizing_regions: "正在识别槽腔和外轮廓",
  meshing: "正在生成模型预览网格",
  generating: "正在计算刀路",
  engraving: "正在计算雕刻刀路",
  rebuilding: "正在重建工序刀路",
  preparing_stock: "正在准备仿真毛坯",
  simulating: "正在仿真加工余料",
  building_stock_view: "正在生成余料视图",
  comparing_stock: "正在比较余料与成品",
  building_replay: "正在生成余料回放帧",
  exporting_nc: "正在生成参考 NC",
  reviewing_nc: "正在检查参考 NC",
  saving_project: "正在保存加工项目",
};

export function isPending(job: Job | null): job is Job & { state: "queued" | "running" } {
  return job?.state === "queued" || job?.state === "running";
}

export function jobStatus(job: Job): string {
  switch (job.state) {
    case "queued": return "任务排队中";
    case "running": return stageLabels[job.stage ?? ""] ?? "任务执行中";
    case "done": return "任务已完成";
    case "cancelled": return "任务已取消";
    case "failed": return "任务失败";
    default: return "任务状态未知";
  }
}

export class JobCancelled extends Error {
  constructor() { super("任务已取消"); }
}
export type Bit = {
  id: string;
  name: string;
  type: string;
  note?: string;
  through_coolant?: boolean;
  geometry: {
    diameter: number;
    cutting_edge_length: number;
    overall_length: number;
    tip_angle?: number;
    corner_radius?: number;
    shank_diameter?: number;
    holder_diameter?: number;
    holder_length?: number;
  };
};
export type Slot = {
  tool_number: number;
  tool_id: string;
  coolant?: string;
  cutting: {
    vertical_feed: number;
    horizontal_feed: number;
    horizontal_rapid?: number;
    vertical_rapid?: number;
    peck_depth?: number;
    drill_cycle?: string;
    spindle_speed: number;
  };
};
export type Library = {
  schema: string;
  version: number;
  library: {
    id: string;
    name: string;
    machine: string;
    tools: Bit[];
    controllers: Slot[];
  };
};
export type PocketSettings = {
  [group: string]: Record<string, unknown>;
  depth: { step_down: number };
  clearing: { pattern: number };
};
export type Op = {
  target?: { root_face_id?: string; root_wire_id?: string };
  parameters?: PocketSettings;
  id: string;
  enabled: boolean;
  sequence: number;
  tool_controller_id: string;
  setup_id: string;
  state: number;
  input_fingerprint: string;
};
export const families = {
  hole_operations: "孔加工",
  pocket_operations: "槽腔",
  profile_operations: "外轮廓",
  surface_finishing_operations: "曲面雕刻",
} as const;
export type Family = keyof typeof families;
export type Definition = Record<Family, Op[]> & {
  name: string;
  pocket_parameters: PocketSettings;
  profile_parameters: PocketSettings;
  hole_parameters: PocketSettings;
  post: { program_name: string; coolant: boolean };
  setups: {
    id: string;
    work_offset: number;
    origin: { x: number; y: number; z: number };
    valid: boolean;
    axis_mapping: string;
    operator_instruction: string;
    confirmation: { confirmed: boolean };
  }[];
  part: { source_path: string };
  stock: {
    source_kind: number;
    source_path: string;
    source_fingerprint: string;
    margin_x: number;
    margin_y: number;
    margin_z: number;
  };
  tool_controllers: {
    id: string;
    tool: {
      tool_name: string;
      vertical_feed: number;
      horizontal_feed: number;
      spindle_speed: number;
    };
  }[];
};
export type Project = {
  format: string;
  schema_version: number;
  job: Definition;
};
export async function response(url: string, init?: RequestInit) {
  const r = await fetch(url, init);
  if (!r.ok) throw Error((await r.text()).trim() || r.statusText);
  return r;
}
export async function json<T>(url: string, init?: RequestInit): Promise<T> {
  return (await response(url, init)).json();
}
export const artifact = (id: string, file: string) =>
  "/api/jobs/" + id + "/files/" + file;
export async function submit(
  url: string,
  body: unknown,
  signal: AbortSignal,
  onUpdate: (job: Job) => void,
): Promise<Job> {
  await response("/api/session", { signal });
  let job = await json<Job>(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    signal,
  });
  return waitForJob(job, signal, onUpdate);
}
export async function waitForJob(
  job: Job,
  signal: AbortSignal,
  onUpdate: (job: Job) => void,
) {
  onUpdate(job);
  while (isPending(job)) {
    await new Promise((resolve) => setTimeout(resolve, 350));
    job = await json<Job>("/api/jobs/" + job.id, { signal });
    onUpdate(job);
  }
  if (job.state === "cancelled") throw new JobCancelled();
  if (job.state !== "done")
    throw Error(
      job.error || jobStatus(job),
    );
  return job;
}
export function assignedTools(doc: Library) {
  const assigned = new Map(
    doc.library.controllers.map((s) => [s.tool_id, s.cutting]),
  );
  return doc.library.tools
    .filter((t) => assigned.has(t.id))
    .map((t) => ({
      id: t.id,
      name: t.name,
      type: t.type,
      diameter: t.geometry.diameter,
      feed: assigned.get(t.id)?.vertical_feed,
      rpm: assigned.get(t.id)?.spindle_speed,
    }));
}
