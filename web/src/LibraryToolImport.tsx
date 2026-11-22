import { useEffect, useState } from "react";
import {
  artifact,
  json,
  type Bit,
  type Library,
  type Job,
  type Slot,
} from "./workspace";
export function LibraryToolImport({
  versions,
  onUse,
}: {
  versions: Job[];
  onUse: (tool: Bit, slot?: Slot) => void;
}) {
  const [source, setSource] = useState("");
  const [library, setLibrary] = useState<Library | null>(null);
  const [tool, setTool] = useState("");
  const [error, setError] = useState("");
  useEffect(() => {
    if (!source) return;
    const controller = new AbortController();
    json<Library>(artifact(source, "library.oaptl"), {
      signal: controller.signal,
    })
      .then((doc) => {
        if (!controller.signal.aborted) setLibrary(doc);
      })
      .catch((e) => {
        if (!controller.signal.aborted) setError(String(e));
      });
    return () => controller.abort();
  }, [source]);
  const chosen = library?.library.tools.find((t) => t.id === tool);
  return (
    <details>
      <summary>从其他刀库版本取用刀具</summary>
      <label>
        来源版本
        <select
          value={source}
          onChange={(e) => {
            setSource(e.target.value);
            setLibrary(null);
            setTool("");
            setError("");
          }}
        >
          <option value="">选择刀库版本</option>
          {versions.map((j) => (
            <option key={j.id} value={j.id}>
              {j.name} · {new Date(j.created!).toLocaleString()} ·{" "}
              {j.id.slice(0, 6)}
            </option>
          ))}
        </select>
      </label>
      {library && (
        <label>
          来源刀具
          <select value={tool} onChange={(e) => setTool(e.target.value)}>
            <option value="">选择刀具</option>
            {library.library.tools.map((t) => (
              <option key={t.id} value={t.id}>
                {t.name}
              </option>
            ))}
          </select>
        </label>
      )}
      {chosen && (
        <button
          onClick={() =>
            onUse(
              chosen,
              library?.library.controllers.find((s) => s.tool_id === tool),
            )
          }
        >
          将所选刀具参数应用到当前草稿
        </button>
      )}
      {error && <p role="alert">{error}</p>}
    </details>
  );
}
