type Values = Record<string, unknown>;
type Field = { name: string; label: string; options?: string[] };
const labels: Record<string, string> = {
  clearance_height: "清空高度 (mm)",
  safe_height: "安全高度 (mm)",
  use_feature_depth: "采用特征深度",
  start_depth: "起始深度 (mm)",
  final_depth: "最终深度 (mm)",
  step_down: "每层切深 (mm)",
  finish_depth: "底部精加工余量 (mm)",
  expansion_direction: "清料方向",
  pattern: "清料图案",
  cut_mode: "铣削方向",
  entry_mode: "进刀方式",
  ramp_start_reference: "斜坡起点",
  ramp_angle_deg: "斜坡角度 (°)",
  maximum_ramp_length: "最大斜坡长度 (mm)",
  helix_diameter: "螺旋直径 (mm)",
  zigzag_ramp_width: "往复斜坡宽度 (mm)",
  ramp_feed: "斜坡进给 (mm/min)",
  step_over_percent: "横向步距 (%)",
  extra_offset: "附加偏置 (mm)",
  precision: "精度",
  arc_tolerance: "圆弧公差 (mm)",
  zigzag_angle: "往复角度 (°)",
  perimeter_pass: "边界走刀",
  start_corner: "访问起始角",
  allowance: "加工余量 (mm)",
  small_hole_diameter_tolerance: "小孔直径容差 (mm)",
  center_spot_depth: "定位钻深 (mm)",
  through_hole_breakthrough: "通孔突破量 (mm)",
  medium_hole_ratio: "中孔刀径比",
  drill_cycle_mode: "钻削循环",
  drill_retract_mode: "退刀平面",
  peck_depth: "啄钻深度 (mm)",
  dwell_seconds: "孔底停留 (s)",
  drill_tip_angle_deg: "钻尖角度 (°)",
  g81_max_ratio: "G81 最大深径比",
  g73_max_ratio: "G73 最大深径比",
  g73_peck_depth: "G73 啄深 (mm)",
  g83_peck_depth: "G83 啄深 (mm)",
  g73_chip_break_retract: "G73 断屑退距 (mm)",
  g83_reentry_clearance: "G83 再入间隙 (mm)",
  collision_envelope_mode: "碰撞包络",
  collision_sample_step: "碰撞采样步长 (mm)",
  helix_pitch: "螺距 (mm)",
  radial_step: "径向步距 (mm)",
  sample_angle_deg: "采样角 (°)",
  helix_chord_tolerance: "螺旋弦差 (mm)",
  min_sample_angle_deg: "最小采样角 (°)",
  max_sample_angle_deg: "最大采样角 (°)",
  max_sample_points: "最大采样点数",
  machining_mode: "加工方式",
  side_allowance: "侧壁余量 (mm)",
  roughing_allowance: "粗加工余量 (mm)",
  radial_pass_count: "径向圈数",
  radial_step_over: "径向步距 (mm)",
  lead_in_length: "切入长度 (mm)",
  lead_out_length: "切出长度 (mm)",
  finish_pass: "精加工走刀",
  step_over: "扫描行距 (mm)",
  sample_spacing: "采样间距 (mm)",
  mesh_deflection: "离散弦差 (mm)",
  safe_clearance: "抬刀间隙 (mm)",
  max_step_down: "最大层深 (mm)",
  mode: "雕刻方式",
  max_moves: "最大移动数",
  max_samples: "最大采样数",
  max_triangles: "最大三角面数",
  target_engagement: "目标材料占比",
  target_angle_deg: "目标包角 (°)",
  maximum_angle_deg: "最大包角 (°)",
  angle_samples: "角度采样数",
  minimum_turn_radius_factor: "转弯半径系数",
  remaining_tolerance: "余料公差 (mm)",
  material_cell_size: "余料网格尺寸 (mm)",
  maximum_passes: "最大走刀数",
  lookahead_depth: "前瞻深度",
  beam_width: "搜索宽度",
};
const options: Record<string, string[]> = {
  pattern: ["偏置", "往复", "螺旋", "FreePath"],
  expansion_direction: ["由外向内", "由内向外"],
  cut_mode: ["顺铣", "逆铣"],
  entry_mode: ["垂直下刀", "沿轮廓斜坡", "螺旋斜坡", "直线斜坡", "往复斜坡"],
  ramp_start_reference: ["上一层", "开口平面", "安全平面"],
  start_corner: ["左下", "右下", "左上", "右上"],
  drill_cycle_mode: ["自动", "直接钻削", "深孔啄钻", "断屑钻削"],
  drill_retract_mode: ["G98 初始平面", "G99 R 平面"],
  collision_envelope_mode: ["刀刃", "刀具装配"],
  machining_mode: ["外部清料", "轮廓精加工"],
  mode: ["仅精雕", "分层开粗及精雕"],
};
export function MachiningParameters({
  title,
  values,
  change,
}: {
  title: string;
  values: Values;
  change: (values: Values) => void;
}) {
  const fields: Field[] = Object.keys(values)
    .filter((key) => key in labels)
    .map((name) => ({ name, label: labels[name], options: options[name] }));
  return (
    <details className="setupRow">
      <summary>{title}</summary>
      <div className="docGrid">
        {fields.map((f) => {
          const v = values[f.name];
          const set = (value: number | boolean) =>
            change({ ...values, [f.name]: value });
          return (
            <label key={f.name}>
              {f.label}
              {f.options ? (
                <select
                  value={Number(v)}
                  onChange={(e) => set(+e.target.value)}
                >
                  {f.options.map((opt, i) => (
                    <option key={opt} value={i}>
                      {opt}
                    </option>
                  ))}
                </select>
              ) : typeof v === "boolean" ? (
                <input
                  type="checkbox"
                  checked={v}
                  onChange={(e) => set(e.target.checked)}
                />
              ) : (
                <input
                  type="number"
                  step="any"
                  value={Number(v)}
                  onChange={(e) => set(+e.target.value)}
                />
              )}
            </label>
          );
        })}
      </div>
      {values.adaptive && typeof values.adaptive === "object" ? (
        <MachiningParameters
          title="FreePath 自适应参数"
          values={values.adaptive as Values}
          change={(adaptive) => change({ ...values, adaptive })}
        />
      ) : null}
    </details>
  );
}
