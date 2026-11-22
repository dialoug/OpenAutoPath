import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export type Face = { id: string; positions: number[]; indices: number[] };
export type Segment = [[number, number, number], [number, number, number]];
type Props = {
  faces: Face[];
  highlighted: string[];
  path: Segment[];
  onPick: (face: string) => void;
};

export function Viewport({ faces, highlighted, path, onPick }: Props) {
  const host = useRef<HTMLDivElement>(null);
  const state = useRef<{
    scene: THREE.Scene;
    meshes: THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>[];
    draw: () => void;
  } | null>(null);
  const pick = useRef(onPick);
  const replay = useRef<(() => void) | null>(null);
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
      setError("浏览器无法创建三维视口，可从右侧列表选择孔。");
      return;
    }
    root.appendChild(renderer.domElement);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xe9eff1);
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
          color: 0x93a5af,
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
    const resize = () => {
      const { width, height } = root.getBoundingClientRect();
      if (!width || !height) return;
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      draw();
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
    root.addEventListener("pointerdown", start);
    root.addEventListener("pointerup", end);
    state.current = { scene, meshes, draw };
    return () => {
      root.removeEventListener("pointerdown", start);
      root.removeEventListener("pointerup", end);
      observer.disconnect();
      controls.dispose();
      meshes.forEach((m) => {
        m.geometry.dispose();
        m.material.dispose();
      });
      renderer.dispose();
      renderer.domElement.remove();
      state.current = null;
    };
  }, [faces]);
  useEffect(() => {
    const view = state.current;
    if (!view) return;
    for (const mesh of view.meshes)
      mesh.material.color.setHex(
        highlighted.includes(mesh.name) ? 0x069486 : 0x93a5af,
      );
    view.draw();
  }, [highlighted, faces]);
  useEffect(() => {
    const view = state.current;
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
    let animation = 0;
    replay.current = () => {
      cancelAnimationFrame(animation);
      const start = performance.now();
      const duration = Math.max(1500, Math.min(12000, path.length * 90));
      const frame = (now: number) => {
        const progress = Math.min(1, (now - start) / duration);
        geometry.setDrawRange(0, 2 * Math.ceil(progress * path.length));
        view.draw();
        if (progress < 1) animation = requestAnimationFrame(frame);
      };
      animation = requestAnimationFrame(frame);
    };
    return () => {
      cancelAnimationFrame(animation);
      replay.current = null;
      view.scene.remove(line);
      geometry.dispose();
      line.material.dispose();
      if (state.current === view) view.draw();
    };
  }, [path, faces]);
  return (
    <div className="viewport" ref={host}>
      {error && <span>{error}</span>}
      {path.length > 0 && !error && (
        <button
          className="replay"
          onPointerDown={(e) => e.stopPropagation()}
          onPointerUp={(e) => e.stopPropagation()}
          onClick={() => replay.current?.()}
        >
          回放刀路
        </button>
      )}
      {!faces.length && <span>上传 STEP 模型以查看几何与加工特征</span>}
    </div>
  );
}
