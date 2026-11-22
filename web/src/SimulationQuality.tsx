export function SimulationQuality({
  value,
  onChange,
}: {
  value: number;
  onChange: (level: number) => void;
}) {
  return (
    <label>
      仿真与回放精度
      <select value={value} onChange={(e) => onChange(Number(e.target.value))}>
        <option value={2}>交互 · 快速预览</option>
        <option value={5}>平衡</option>
        <option value={10}>详细 · 检查槽壁和圆角</option>
      </select>
    </label>
  );
}
