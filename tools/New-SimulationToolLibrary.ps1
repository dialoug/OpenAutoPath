param(
    [string]$OutputPath = (Join-Path $PSScriptRoot '../resources/tool-libraries/general-simulation.oaptl')
)

$ErrorActionPreference = 'Stop'
$toolBits = [System.Collections.Generic.List[object]]::new()
$slots = [System.Collections.Generic.List[object]]::new()
$culture = [System.Globalization.CultureInfo]::InvariantCulture

function Add-SimulationTool([string]$Type, [string]$Label, [double]$Diameter, [int]$Number) {
    $size = $Diameter.ToString('0.###', $culture)
    $id = "sim-v1-$Type-d$size"
    $isMill = $Type -eq 'end_mill'
    $isCenter = $Type -eq 'center_drill'
    $edge = if ($isCenter) { [Math]::Max(3, $Diameter * 2) }
        elseif ($isMill) { [Math]::Max(5, [Math]::Min(80, $Diameter * 5)) }
        else { [Math]::Min(200, $Diameter * 8) }
    $shank = [Math]::Max(6, $Diameter)
    $holder = [Math]::Max(32, $shank + 12)
    $speed = if ($isMill) { 6000 } else { 3000 }
    $toolBits.Add([ordered]@{
        id = $id
        name = "【模拟】$Label D$size"
        type = $Type
        geometry = [ordered]@{
            diameter = $Diameter; cutting_edge_length = $edge
            overall_length = $edge + 30; corner_radius = 0
            tip_angle = $(if ($isCenter) { 90 } elseif ($isMill) { 0 } else { 118 })
            tip_compensation = 0; shank_diameter = $shank
            holder_diameter = $holder; holder_length = 40
            spindle_envelope_diameter = [Math]::Max(80, $holder + 10)
            spindle_envelope_length = 80
        }
        note = '仅用于软件模拟和回归测试。几何、装夹和 S/F 为合成参数，不代表实物刀具或已验证加工条件。'
        through_coolant = $false
        attributes = [ordered]@{ simulation_only = 'true'; units = 'mm'; flutes = $(if ($isMill) { '4' } else { '2' }) }
    })
    $slots.Add([ordered]@{
        tool_number = $Number; tool_id = $id; coolant = 'off'
        cutting = [ordered]@{
            spindle_speed = $speed; horizontal_feed = $(if ($isMill) { 600 } else { 150 })
            vertical_feed = $(if ($isMill) { 150 } else { 100 })
            horizontal_rapid = 3000; vertical_rapid = 1500
            peck_depth = [Math]::Min(5, $Diameter); drill_cycle = 'auto'
        }
    })
}

$number = 1
foreach ($diameter in @(0.5, 0.8, 1, 2, 3, 4, 6, 8, 10)) {
    Add-SimulationTool 'center_drill' '中心钻' $diameter $number
    $number++
}
$drillSizes = [System.Collections.Generic.List[double]]::new()
foreach ($diameter in @(0.5, 0.6, 0.8)) { $drillSizes.Add($diameter) }
for ($i = 10; $i -le 200; $i++) { $drillSizes.Add($i / 10.0) }
for ($i = 41; $i -le 80; $i++) { $drillSizes.Add($i / 2.0) }
for ($i = 41; $i -le 60; $i++) { $drillSizes.Add($i) }
$number = 101
foreach ($diameter in $drillSizes) {
    Add-SimulationTool 'drill' '钻头' $diameter $number
    $number++
}
$number = 1001
foreach ($diameter in @(0.5, 0.8, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12, 16, 20, 25, 32, 40)) {
    Add-SimulationTool 'end_mill' '平底铣刀' $diameter $number
    $number++
}
$document = [ordered]@{
    schema = 'oap-tool-library'; version = 1
    library = [ordered]@{
        id = 'simulation-general-mm-v1'
        name = '【模拟测试】通用毫米制刀库（281 把）'
        machine = '虚拟三轴铣床 / 非实机刀位表'
        description = '用于常见毫米制 STEP 零件的孔、槽腔和外轮廓软件测试。9 把中心钻、254 把钻头、18 把平底铣刀，均已分配虚拟 T 号。仅模拟测试，不能直接作为实机加工依据。'
        attributes = [ordered]@{ simulation_only = 'true'; units = 'mm'; revision = '1'; generator = 'tools/New-SimulationToolLibrary.ps1' }
        tools = $toolBits.ToArray(); controllers = $slots.ToArray()
    }
}
$targetPath = [System.IO.Path]::GetFullPath($OutputPath)
[System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($targetPath)) | Out-Null
[System.IO.File]::WriteAllText($targetPath, ($document | ConvertTo-Json -Depth 12) + "`n", [System.Text.UTF8Encoding]::new($false))
Write-Output "Created $($toolBits.Count) assigned simulation tools: $targetPath"
