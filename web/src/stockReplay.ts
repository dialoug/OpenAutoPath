import { artifact, json, response } from "./workspace";
import type { Segment } from "./Viewport";

type FrameInfo = {
  move: number;
  volume: number;
  vertices: number;
  triangles: number;
  materialSpacing?: number;
  displaySpacing?: number;
};
type Manifest = {
  moves: number;
  quality?: number;
  selectedOnly?: boolean;
  frames: FrameInfo[];
  operations: { id: string; end: number }[];
  tools?: {
    end: number;
    axis: [number, number, number];
    outline: [number, number][];
  }[];
};
type FrameMesh = { positions: Float32Array; indices: Uint32Array };
export type StockReplay = {
  startMove?: number;
  frames: FrameInfo[];
  firstMesh: FrameMesh;
  loadFrame: (frame: FrameInfo, signal: AbortSignal) => Promise<FrameMesh>;
  quality?: number;
  selectedOnly?: boolean;
  operations: Manifest["operations"];
  tools?: Manifest["tools"];
  path: Segment[];
};

export async function loadStockReplay(
  job: string,
  signal: AbortSignal,
): Promise<StockReplay> {
  const [info, path] = await Promise.all([
    json<Manifest>(artifact(job, "result.json"), { signal }),
    json<Segment[]>(artifact(job, "path.json"), { signal }),
  ]);
  if (
    !path.length ||
    info.moves !== path.length ||
    info.frames[0]?.move !== 0 ||
    info.frames.at(-1)?.move !== path.length ||
    info.operations.at(-1)?.end !== path.length
  )
    throw Error("余料帧与刀路长度不一致");
  const ranges = new Map<FrameInfo, { offset: number; size: number }>();
  let offset = 0;
  for (const frame of info.frames) {
    const size = 12 * (frame.vertices + frame.triangles);
    if (!Number.isSafeInteger(size) || size <= 0) throw Error("余料帧长度无效");
    ranges.set(frame, { offset, size });
    offset += size;
  }
  const cache = new Map<FrameInfo, FrameMesh>();
  async function loadFrame(frame: FrameInfo, signal: AbortSignal) {
    const hit = cache.get(frame);
    if (hit) return hit;
    const { offset, size } = ranges.get(frame)!;
    const r = await response(artifact(job, "replay.bin"), {
      headers: { Range: `bytes=${offset}-${offset + size - 1}` },
      signal,
    });
    if (r.status !== 206) throw Error("服务未返回请求的余料帧");
    const buffer = await r.arrayBuffer();
    if (buffer.byteLength !== size) throw Error("余料帧数据不完整");
    const positions = new Float32Array(buffer, 0, frame.vertices * 3);
    const indices = new Uint32Array(
      buffer,
      positions.byteLength,
      frame.triangles * 3,
    );
    const mesh = { positions, indices };
    cache.set(frame, mesh);
    if (cache.size > 3) cache.delete(cache.keys().next().value!);
    return mesh;
  }
  const firstMesh = await loadFrame(info.frames[0], signal);
  return { ...info, path, firstMesh, loadFrame };
}
