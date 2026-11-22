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
type PocketSettings = {
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
  part: { source_path: string };
  stock: {
    source_kind: number;
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
  onQueued: (job: Job) => void,
): Promise<Job> {
  await response("/api/session", { signal });
  let job = await json<Job>(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    signal,
  });
  onQueued(job);
  while (job.state === "queued" || job.state === "running") {
    await new Promise((resolve) => setTimeout(resolve, 350));
    job = await json<Job>("/api/jobs/" + job.id, { signal });
  }
  if (job.state !== "done")
    throw Error(
      job.error || (job.state === "cancelled" ? "任务已取消" : job.state),
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
