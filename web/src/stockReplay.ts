import { artifact, json, response } from "./workspace";
import type { Segment } from "./Viewport";

type FrameInfo = {
  move: number;
  volume: number;
  vertices: number;
  triangles: number;
};
type Manifest = {
  moves: number;
  frames: FrameInfo[];
  operations: { id: string; end: number }[];
  tools?: {
    end: number;
    axis: [number, number, number];
    outline: [number, number][];
  }[];
};
export type StockFrame = {
  move: number;
  volume: number;
  positions: Float32Array;
  indices: Uint32Array;
};
export type StockReplay = {
  startMove?: number;
  frames: StockFrame[];
  operations: Manifest["operations"];
  tools?: Manifest["tools"];
  path: Segment[];
};

export async function loadStockReplay(
  job: string,
  signal: AbortSignal,
): Promise<StockReplay> {
  const [info, path, buffer] = await Promise.all([
    json<Manifest>(artifact(job, "result.json"), { signal }),
    json<Segment[]>(artifact(job, "path.json"), { signal }),
    response(artifact(job, "replay.bin"), { signal }).then((r) =>
      r.arrayBuffer(),
    ),
  ]);
  if (
    !path.length ||
    info.moves !== path.length ||
    info.frames[0]?.move !== 0 ||
    info.frames.at(-1)?.move !== path.length ||
    info.operations.at(-1)?.end !== path.length
  )
    throw Error("余料帧与刀路长度不一致");
  let offset = 0;
  const frames = info.frames.map((frame) => {
    const positions = new Float32Array(buffer, offset, 3 * frame.vertices);
    offset += positions.byteLength;
    const indices = new Uint32Array(buffer, offset, 3 * frame.triangles);
    offset += indices.byteLength;
    return { move: frame.move, volume: frame.volume, positions, indices };
  });
  if (offset !== buffer.byteLength) throw Error("余料回放文件长度不一致");
  return { frames, operations: info.operations, tools: info.tools, path };
}
