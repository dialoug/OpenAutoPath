import { useEffect, useRef, useState } from "react";
import { Box, Focus, Grid2X2, Minus, Plus, Square } from "lucide-react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export type Face = { id: string; positions: number[]; indices: number[] };
export type Segment = [[number, number, number], [number, number, number]];
type Props = {
  faces: Face[];
  highlighted: string[];
  colors?: Record<string, number>;
  path: Segment[];
  cameraTools?: boolean;
  inspectStock?: (moves: number) => void;
  onPick: (face: string) => void;
};

export function Viewport({
  faces,
  highlighted,
  colors,
  path,
  onPick,
  inspectStock,
  cameraTools = false,
}: Props) {
  const host = useRef<HTMLDivElement>(null);
  const [gridVisible, setGridVisible] = useState(true);
  const cameraAction = useRef<
    ((action: "fit" | "iso" | "top" | "in" | "out") => void) | null
  >(null);
  const gridRef = useRef<THREE.GridHelper | null>(null);
  const state = useRef<{
    scene: THREE.Scene;
    meshes: THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>[];
    draw: () => void;
  } | null>(null);
  const pick = useRef(onPick);
  const renderMove = useRef<((count: number) => void) | null>(null);
  const [position, setPosition] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  pick.current = onPick;
  const [error, setError] = useState("");
  useEffect(() => {
    const root = host.current!;
    if (!faces.length) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
      setError("");
    } catch {
      setError("浏览器无法创建三维视口。");
      return;
    }
    root.appendChild(renderer.domElement);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeff2f4);
    const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 1e6);
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x68757c, 2.3));
    const light = new THREE.DirectionalLight(0xffffff, 2.2);
    light.position.set(1, -2, 4);
    scene.add(light);
    const meshes = faces.map((face) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(face.positions, 3),
      );
      geo.setIndex(face.indices);
      geo.computeVertexNormals();
      const mesh = new THREE.Mesh(
        geo,
        new THREE.MeshStandardMaterial({
          color: 0x859aaa,
          roughness: 0.68,
          side: THREE.DoubleSide,
        }),
      );
      mesh.name = face.id;
      scene.add(mesh);
      return mesh;
    });
    const box = new THREE.Box3();
    meshes.forEach((mesh) => box.expandByObject(mesh));
    const center = box.getCenter(new THREE.Vector3());
    const span = Math.max(box.getSize(new THREE.Vector3()).length(), 1);
    const grid = new THREE.GridHelper(span * 4, 80, 0xafbdc5, 0xc4cfd6);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(center.x, center.y, box.min.z - span * 0.015);
    grid.material.transparent = true;
    grid.material.opacity = 0.47;
    gridRef.current = grid;
    if (cameraTools) scene.add(grid);
    camera.position
      .copy(center)
      .add(
        new THREE.Vector3(1, -1.4, 1).normalize().multiplyScalar(span * 1.7),
      );
    camera.near = span / 10000;
    camera.far = span * 100;
    controls.target.copy(center);
    controls.update();
    const draw = () => renderer.render(scene, camera);
    const fit = () => {
      const direction = camera.position
        .clone()
        .sub(controls.target)
        .normalize();
      const distance =
        (span / 2 / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2))) *
        Math.max(1, 1 / camera.aspect) *
        (camera.aspect < 1.15 ? 1.12 : 0.93);
      controls.target.copy(center);
      camera.position.copy(center).addScaledVector(direction, distance);
      controls.update();
      draw();
    };
    cameraAction.current = (action) => {
      if (action === "in" || action === "out") {
        camera.position
          .sub(controls.target)
          .multiplyScalar(action === "in" ? 0.8 : 1.25)
          .add(controls.target);
        controls.update();
        draw();
        return;
      }
      if (action === "iso" || action === "top") {
        controls.target.copy(center);
        camera.position
          .copy(center)
          .add(
            action === "iso"
              ? new THREE.Vector3(1, -1.6, 1.1)
              : new THREE.Vector3(0, -0.00001, 1),
          );
      }
      fit();
    };
    let previousAspect = 0;
    const resize = () => {
      const { width, height } = root.getBoundingClientRect();
      if (!width || !height) return;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      if (!previousAspect || previousAspect < 1.15 !== camera.aspect < 1.15)
        fit();
      else draw();
      previousAspect = camera.aspect;
    };
    controls.addEventListener("change", draw);
    const observer = new ResizeObserver(resize);
    observer.observe(root);
    resize();
    let down = new THREE.Vector2();
    const start = (e: PointerEvent) => {
      down.set(e.clientX, e.clientY);
    };
    const end = (e: PointerEvent) => {
      if (
        e.button !== 0 ||
        down.distanceTo(new THREE.Vector2(e.clientX, e.clientY)) > 4
      )
        return;
      const rect = root.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      ray.setFromCamera(
        new THREE.Vector2(
          (2 * (e.clientX - rect.left)) / rect.width - 1,
          1 - (2 * (e.clientY - rect.top)) / rect.height,
        ),
        camera,
      );
      const hit = ray.intersectObjects(meshes)[0];
      if (hit) pick.current(hit.object.name);
    };
    renderer.domElement.addEventListener("pointerdown", start);
    renderer.domElement.addEventListener("pointerup", end);
    state.current = { scene, meshes, draw };
    return () => {
      renderer.domElement.removeEventListener("pointerdown", start);
      renderer.domElement.removeEventListener("pointerup", end);
      observer.disconnect();
      controls.dispose();
      meshes.forEach((m) => {
        m.geometry.dispose();
        m.material.dispose();
      });
      cameraAction.current = null;
      gridRef.current = null;
      grid.geometry.dispose();
      grid.material.dispose();
      renderer.dispose();
      renderer.domElement.remove();
      state.current = null;
    };
  }, [faces, cameraTools]);
  useEffect(() => {
    if (gridRef.current) {
      gridRef.current.visible = gridVisible;
      state.current?.draw();
    }
  }, [gridVisible, faces, cameraTools]);
  useEffect(() => {
    const view = state.current;
    if (!view) return;
    for (const mesh of view.meshes)
      mesh.material.color.setHex(
        highlighted.includes(mesh.name)
          ? 0x069486
          : (colors?.[mesh.name] ?? 0x859aaa),
      );
    view.draw();
  }, [highlighted, colors, faces]);
  useEffect(() => {
    const view = state.current;
    setPlaying(false);
    setPosition(path.length);
    if (!view || !path.length) return;
    const geometry = new THREE.BufferGeometry().setFromPoints(
      path.flatMap(([a, b]) => [
        new THREE.Vector3(...a),
        new THREE.Vector3(...b),
      ]),
    );
    const line = new THREE.LineSegments(
      geometry,
      new THREE.LineBasicMaterial({ color: 0xd95a17, depthTest: false }),
    );
    line.renderOrder = 1;
    view.scene.add(line);
    view.draw();
    const marker = new THREE.Points(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3()]),
      new THREE.PointsMaterial({
        color: 0xbd3617,
        size: 9,
        sizeAttenuation: false,
        depthTest: false,
      }),
    );
    marker.renderOrder = 2;
    view.scene.add(marker);
    renderMove.current = (count) => {
      geometry.setDrawRange(0, 2 * count);
      marker.position.set(...(count ? path[count - 1][1] : path[0][0]));
      view.draw();
    };
    renderMove.current(path.length);
    return () => {
      renderMove.current = null;
      view.scene.remove(marker);
      marker.geometry.dispose();
      marker.material.dispose();
      view.scene.remove(line);
      geometry.dispose();
      line.material.dispose();
      if (state.current === view) view.draw();
    };
  }, [path, faces]);
  useEffect(() => {
    if (position <= path.length) renderMove.current?.(position);
  }, [position, path]);
  useEffect(() => {
    if (!playing || !path.length) return;
    let handle = 0,
      previous = performance.now(),
      remainder = 0,
      cursor = startPosition.current;
    const duration = Math.max(1500, Math.min(12000, path.length * 90));
    const frame = (now: number) => {
      remainder += ((now - previous) * speed * path.length) / duration;
      previous = now;
      const count = Math.floor(remainder);
      remainder -= count;
      cursor = Math.min(path.length, cursor + count);
      setPosition(cursor);
      if (cursor < path.length) handle = requestAnimationFrame(frame);
      else setPlaying(false);
    };
    handle = requestAnimationFrame(frame);
    return () => cancelAnimationFrame(handle);
  }, [playing, path, speed]);
  const startPosition = useRef(0);
  startPosition.current = position;
  const seek = (count: number) => {
    setPlaying(false);
    setPosition(count);
  };
  return (
    <div className="viewport" ref={host}>
      {cameraTools && faces.length > 0 && !error && (
        <div
          className="cameraTools"
          aria-label="视图控制"
          onPointerDown={(e) => e.stopPropagation()}
          onPointerUp={(e) => e.stopPropagation()}
        >
          <button
            title="等轴测视图"
            aria-label="等轴测视图"
            onClick={() => cameraAction.current?.("iso")}
          >
            <Box />
          </button>
          <button
            title="俯视图"
            aria-label="俯视图"
            onClick={() => cameraAction.current?.("top")}
          >
            <Square />
          </button>
          <i />
          <button
            title="放大"
            aria-label="放大"
            onClick={() => cameraAction.current?.("in")}
          >
            <Plus />
          </button>
          <button
            title="缩小"
            aria-label="缩小"
            onClick={() => cameraAction.current?.("out")}
          >
            <Minus />
          </button>
          <button
            title="适应视口"
            aria-label="适应视口"
            onClick={() => cameraAction.current?.("fit")}
          >
            <Focus />
          </button>
          <i />
          <button
            title="显示网格"
            aria-label="显示网格"
            aria-pressed={gridVisible}
            onClick={() => setGridVisible((v) => !v)}
          >
            <Grid2X2 />
          </button>
        </div>
      )}
      {error && <span>{error}</span>}
      {path.length > 0 && !error && (
        <div
          className="replay"
          onPointerDown={(e) => e.stopPropagation()}
          onPointerUp={(e) => e.stopPropagation()}
        >
          <button
            onClick={() => {
              if (position === path.length) {
                startPosition.current = 0;
                setPosition(0);
              }
              setPlaying(!playing);
            }}
          >
            {playing
              ? "暂停"
              : position === path.length
                ? "回放刀路"
                : "继续播放"}
          </button>
          <button disabled={position === 0} onClick={() => seek(position - 1)}>
            上一步
          </button>
          <button
            disabled={position === path.length}
            onClick={() => seek(position + 1)}
          >
            下一步
          </button>
          <select
            aria-label="播放速度"
            value={speed}
            onChange={(e) => setSpeed(+e.target.value)}
          >
            {[0.25, 0.5, 1, 2, 4].map((v) => (
              <option key={v} value={v}>
                {v}×
              </option>
            ))}
          </select>
          <label>
            移动 {position} / {path.length}
            <input
              aria-label="刀路进度"
              type="range"
              min="0"
              max={path.length}
              value={position}
              onChange={(e) => seek(+e.target.value)}
            />
          </label>
          {inspectStock && (
            <button
              onClick={() => {
                setPlaying(false);
                inspectStock(position);
              }}
            >
              查看此步余料
            </button>
          )}
        </div>
      )}
      {!faces.length && <span>上传 STEP 模型以查看几何与加工特征</span>}
    </div>
  );
}
