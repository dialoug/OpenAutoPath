import { useMemo } from "react";
import { LatheGeometry, Vector2 } from "three";
import { Viewport, type Face } from "./Viewport";
import type { Bit } from "./workspace";

function toolEnvelope(bit: Bit): Face[] {
  const g = bit.geometry;
  const radius = g.diameter / 2;
  if (!(radius > 0 && g.overall_length > 0)) return [];
  const profile: Vector2[] = [new Vector2(0, 0)];
  const corner =
    bit.type === "ball_end_mill"
      ? radius
      : bit.type === "bull_nose"
        ? Math.min(g.corner_radius || 0, radius)
        : 0;
  if (corner > 0) {
    for (let i = 0; i <= 12; i++) {
      const angle = ((Math.PI / 2) * i) / 12;
      profile.push(
        new Vector2(
          radius - corner + corner * Math.sin(angle),
          corner * (1 - Math.cos(angle)),
        ),
      );
    }
  } else {
    const angle = g.tip_angle || 0;
    const tip =
      bit.type.includes("drill") && angle > 0 && angle < 180
        ? Math.min(
            g.cutting_edge_length,
            radius / Math.tan((angle * Math.PI) / 360),
          )
        : 0;
    profile.push(new Vector2(radius, tip));
  }
  const stickout = g.overall_length;
  const flute = Math.min(
    stickout,
    Math.max(profile[profile.length - 1].y, g.cutting_edge_length),
  );
  const shank = (g.shank_diameter || g.diameter) / 2;
  profile.push(
    new Vector2(radius, flute),
    new Vector2(shank, flute),
    new Vector2(shank, stickout),
  );
  let top = stickout;
  if (g.holder_diameter && g.holder_length) {
    top += g.holder_length;
    profile.push(
      new Vector2(g.holder_diameter / 2, stickout),
      new Vector2(g.holder_diameter / 2, top),
    );
  }
  profile.push(new Vector2(0, top));
  const mesh = new LatheGeometry(profile, 48);
  mesh.rotateX(Math.PI / 2);
  const face = {
    id: bit.id,
    positions: Array.from(mesh.getAttribute("position").array),
    indices: Array.from(mesh.getIndex()!.array),
  };
  mesh.dispose();
  return [face];
}
export function ToolPreview({ bit }: { bit: Bit }) {
  const faces = useMemo(() => toolEnvelope(bit), [bit]);
  return (
    <section aria-label="刀具尺寸预览">
      {faces.length ? (
        <Viewport faces={faces} path={[]} highlighted={[]} onPick={() => {}} />
      ) : (
        <p>填写有效的直径和总长后可预览刀具。</p>
      )}
    </section>
  );
}
