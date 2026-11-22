import { useState } from "react";
import { ArrowUpRight, Box, FileBox, FileUp, FolderOpen } from "lucide-react";
import { Viewport } from "./Viewport";
import sampleFaces from "./assets/cam-feature-preview.json";
import sampleUrl from "../../samples/cam_feature_test.stp?url";

type Props = {
  busy: boolean;
  upload: (file: File) => Promise<void>;
  error: (message: string) => void;
};
export function ModelStart({ busy, upload, error }: Props) {
  const [drag, setDrag] = useState(false);
  const [opening, setOpening] = useState(false);
  const disabled = busy || opening;
  function accept(file?: File) {
    if (!file || disabled) return;
    if (!/\.(stp|step)$/i.test(file.name)) {
      error("请选择 .step 或 .stp 文件。");
      return;
    }
    void upload(file);
  }
  async function openSample() {
    setOpening(true);
    try {
      const res = await fetch(sampleUrl);
      if (!res.ok) throw Error("无法读取示例零件");
      await upload(new File([await res.blob()], "cam_feature_test.stp"));
    } catch (e) {
      error(String(e));
    } finally {
      setOpening(false);
    }
  }
  return (
    <div className="startLayout">
      <section className="viewerCard">
        <div className="cardHeading">
          <h2>
            <Box />
            模型预览
          </h2>
          <span className="viewUnit">mm</span>
        </div>
        <div className="startScene">
          <Viewport
            faces={sampleFaces}
            highlighted={[]}
            path={[]}
            onPick={() => {}}
            cameraTools
          />
        </div>
        <div className="sampleCaption">
          <span className="fileTile">
            <FileBox />
          </span>
          <div>
            <strong>cam_feature_test.stp</strong>
            <small>示例零件 · 120 × 80 × 20 mm</small>
          </div>
        </div>
      </section>
      <section className="startActions">
        <h2>
          <FolderOpen />
          打开模型
        </h2>
        <div
          className={"dropZone" + (drag ? " dragging" : "")}
          onDragOver={(e) => {
            e.preventDefault();
            setDrag(true);
          }}
          onDragLeave={() => setDrag(false)}
          onDrop={(e) => {
            e.preventDefault();
            setDrag(false);
            accept(e.dataTransfer.files[0]);
          }}
        >
          <div className="uploadGlyph">
            <FileUp size={28} />
          </div>
          <strong>将 STEP 文件拖到这里</strong>
          <small>.step / .stp · 最大 128 MiB</small>
          <label className="upload primary">
            <FolderOpen size={16} />
            选择文件
            <input
              aria-label="上传 STEP"
              type="file"
              accept=".step,.stp"
              disabled={disabled}
              onChange={(e) => {
                accept(e.target.files?.[0]);
                e.target.value = "";
              }}
            />
          </label>
        </div>
        <div className="sampleEntry">
          <h3>示例零件</h3>
          <p>用这个测试件体验孔加工、槽腔和外轮廓规划。</p>
          <button onClick={() => void openSample()} disabled={disabled}>
            打开示例零件 <ArrowUpRight size={16} />
          </button>
        </div>
      </section>
    </div>
  );
}
