# Verify the distributed ZIP and start it without developer SDK search paths.
# A hosted runner is not a pristine Windows installation or a visual CAD test.
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Archive,
    [Parameter(Mandatory)][string]$OutputDir,
    [string]$ExpectedCommit,
    [ValidateRange(10,180)][int]$StartupTimeoutSeconds = 60
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$archivePath = (Resolve-Path -LiteralPath $Archive).Path
$output = [IO.Path]::GetFullPath($OutputDir)
if(Test-Path -LiteralPath $output){throw 'Use a new package validation directory'}
New-Item -ItemType Directory -Path $output | Out-Null
$checksum = Join-Path (Split-Path $archivePath -Parent) 'SHA256SUMS.txt'
$name = [IO.Path]::GetFileName($archivePath)
$line = @(Get-Content $checksum | Where-Object { $_ -match ('^[0-9a-fA-F]{64}\s+'+[regex]::Escape($name)+'$') })
if($line.Count -ne 1 -or (Get-FileHash $archivePath).Hash -ne $line[0].Substring(0,64)){throw 'ZIP checksum mismatch'}
Expand-Archive -LiteralPath $archivePath -DestinationPath $output
$package = Join-Path $output ([IO.Path]::GetFileNameWithoutExtension($archivePath))
$bin = Join-Path $package 'bin'
foreach($file in @('qtbase_zh_CN.ts', 'README.md', 'licenseRule.json', 'LGPL-3.0-only.txt', 'GPL-3.0-only.txt')) {
    if(-not (Test-Path -LiteralPath (Join-Path $package "share/licenses/OpenAutoPath/qttranslations/$file") -PathType Leaf)) {
        throw "Qt translation source or notice missing: $file"
    }
}
$info = Get-Content (Join-Path $package 'build-info.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if($ExpectedCommit -and $info.commit -ne $ExpectedCommit){throw 'Package commit mismatch'}
$inventory = Get-Content (Join-Path $package 'share/licenses/OpenAutoPath/runtime-inventory.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if($inventory.sourceBuilds.Count -ne 2 -or $inventory.sources.Count -ne 5){throw 'Unexpected dependency source inventory'}
if(@($inventory.binaries | Where-Object component -eq 'ffmpeg').Count){throw 'FFmpeg is present in the runtime inventory'}
foreach($binary in $inventory.binaries){
    if((Get-FileHash (Join-Path $package $binary.file)).Hash -ne $binary.sha256){throw "Runtime hash mismatch: $($binary.file)"}
}
foreach($source in $inventory.sources){
    if((Get-FileHash (Join-Path $package "share/sources/OpenAutoPath/$($source.file)")).Hash -ne $source.sha256){throw "Source hash mismatch: $($source.file)"}
}
foreach($record in $inventory.sourceBuilds){
    $recipe = Join-Path $package "share/sources/OpenAutoPath/OpenAutoPath-build-support/$($record.recipe)"
    if((Get-FileHash $recipe).Hash -ne $record.recipeSha256){throw 'Build recipe hash mismatch'}
}
$occt = $inventory.sourceBuilds | Where-Object component -eq 'occt'
if(-not $occt -or -not $occt.PSObject.Properties['ffmpegEnabled'] -or $occt.ffmpegEnabled -ne $false){throw 'OCCT video support is not disabled'}
$typeName = 'ReleasePackageNative_' + [guid]::NewGuid().ToString('N')
$code = @'
using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public static class __TYPE__ {
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern IntPtr LoadLibraryEx(string file,IntPtr reserved,uint flags);
    [DllImport("kernel32.dll",CharSet=CharSet.Ansi,ExactSpelling=true)] static extern IntPtr GetProcAddress(IntPtr module,string name);
    [DllImport("kernel32.dll")] static extern bool FreeLibrary(IntPtr module);
    public static void CheckSystemCompiler(string file) {
        IntPtr module=LoadLibraryEx(file,IntPtr.Zero,0x800);
        if(module==IntPtr.Zero) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error(),"Cannot load Windows D3D compiler");
        try { if(GetProcAddress(module,"D3DCompile")==IntPtr.Zero) throw new Exception("System D3DCompile entry point is missing"); }
        finally { FreeLibrary(module); }
    }
    delegate bool EnumProc(IntPtr h,IntPtr p);
    [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc callback,IntPtr p);
    [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h,out uint pid);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetWindowText(IntPtr h,StringBuilder s,int n);
    [DllImport("user32.dll",SetLastError=true)] static extern IntPtr SendMessageTimeout(IntPtr h,uint m,IntPtr w,IntPtr l,uint flags,uint timeout,out IntPtr result);
    [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr h,uint m,IntPtr w,IntPtr l);
    public static IntPtr MainWindow(uint pid) {
        IntPtr found=IntPtr.Zero;
        EnumWindows(delegate(IntPtr h,IntPtr p) { uint owner;GetWindowThreadProcessId(h,out owner);
            var title=new StringBuilder(512);GetWindowText(h,title,title.Capacity);
            if(owner==pid && title.ToString().StartsWith("OpenAutoPath")){found=h;return false;}return true;
        },IntPtr.Zero);
        return found;
    }
    public static bool Responsive(IntPtr h) { IntPtr result;return SendMessageTimeout(h,0,IntPtr.Zero,IntPtr.Zero,2,2000,out result)!=IntPtr.Zero; }
    public static string[] Imports(string file) {
        using(var stream=File.OpenRead(file)) using(var b=new BinaryReader(stream)) {
            if(b.ReadUInt16()!=0x5a4d) throw new Exception("Not a PE file: "+file);
            stream.Position=0x3c;uint pe=b.ReadUInt32();stream.Position=pe;
            if(b.ReadUInt32()!=0x4550) throw new Exception("Invalid PE header");
            b.ReadUInt16();ushort count=b.ReadUInt16();stream.Position=pe+20;ushort optionalSize=b.ReadUInt16();
            long optional=pe+24;stream.Position=optional;ushort magic=b.ReadUInt16();
            if(magic!=0x20b && magic!=0x10b) throw new Exception("Unsupported PE format");
            stream.Position=optional+(magic==0x20b?24:28);ulong imageBase=magic==0x20b?b.ReadUInt64():b.ReadUInt32();
            long dirs=optional+(magic==0x20b?112:96);
            var sections=new List<uint[]>();stream.Position=optional+optionalSize;
            for(int i=0;i<count;i++){stream.Position+=8;uint size=b.ReadUInt32(),va=b.ReadUInt32(),rawSize=b.ReadUInt32(),raw=b.ReadUInt32();sections.Add(new uint[]{va,Math.Max(size,rawSize),raw});stream.Position+=16;}
            Func<uint,long> offset=delegate(uint rva){foreach(var s in sections)if(rva>=s[0] && (ulong)rva<s[0]+(ulong)s[1])return s[2]+(long)rva-s[0];throw new Exception("Invalid PE RVA");};
            var names=new List<string>();
            foreach(int index in new int[]{1,13}) {
                stream.Position=dirs+index*8;uint rva=b.ReadUInt32();if(rva==0)continue;
                long table=offset(rva);int stride=index==1?20:32;
                for(int n=0;n<4096;n++) {
                    stream.Position=table+n*stride;uint first=b.ReadUInt32();
                    stream.Position=table+n*stride+(index==1?12:4);uint name=b.ReadUInt32();if(name==0)break;
                    if(index==13 && (first&1)==0){if(name<imageBase)throw new Exception("Invalid delay import VA");name=(uint)(name-imageBase);}
                    stream.Position=offset(name);var text=new StringBuilder();byte ch;
                    while((ch=b.ReadByte())!=0){if(text.Length>=512)throw new Exception("Invalid import name");text.Append((char)ch);}
                    names.Add(text.ToString());
                }
            }
            return names.ToArray();
        }
    }
}
'@
$native = Add-Type -TypeDefinition $code.Replace('__TYPE__',$typeName) -PassThru | Where-Object Name -eq $typeName
$forbidden = '^(avcodec|avformat|avutil|swscale|avdevice|avfilter|swresample)(-[0-9]+)?\.dll$'
$imports = @()
foreach($file in Get-ChildItem -LiteralPath $package -Recurse -File | Where-Object Extension -in '.dll','.exe'){
    if($file.Name -match $forbidden){throw "FFmpeg binary in package: $($file.Name)"}
    if($file.Name -ieq 'D3Dcompiler_47.dll'){throw 'Use the Windows system D3D compiler, not a bundled SDK copy'}
    $dependencies = @($native::Imports($file.FullName))
    if(@($dependencies | Where-Object {$_ -match $forbidden}).Count){throw "FFmpeg import in $($file.Name)"}
    $imports += @{file=$file.FullName.Substring($package.Length+1);imports=$dependencies}
}
$imports | ConvertTo-Json -Depth 5 | Set-Content (Join-Path $output 'imports.json') -Encoding UTF8
$saved = @{}
$envNames = @('PATH','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH','QT_QPA_PLATFORM','QML_IMPORT_PATH','QML2_IMPORT_PATH')
foreach($key in $envNames){$saved[$key]=[Environment]::GetEnvironmentVariable($key,'Process')}
$app = $null
$window = [IntPtr]::Zero
try {
    $env:PATH = "$env:SystemRoot\System32;$env:SystemRoot"
    foreach($key in $envNames | Where-Object {$_ -ne 'PATH'}){[Environment]::SetEnvironmentVariable($key,$null,'Process')}
    $systemCompiler = Join-Path ([Environment]::SystemDirectory) 'D3Dcompiler_47.dll'
    $native::CheckSystemCompiler($systemCompiler)
    $working = Join-Path $output 'empty-working-directory'
    New-Item -ItemType Directory -Path $working | Out-Null
    $launcher = Start-Process -FilePath (Join-Path $package 'OpenAutoPath.exe') -WorkingDirectory $working -WindowStyle Hidden -PassThru
    $watch = [Diagnostics.Stopwatch]::StartNew()
    while($watch.Elapsed.TotalSeconds -lt $StartupTimeoutSeconds){
        if(-not $app){
            $child = Get-CimInstance Win32_Process -Filter "Name='OpenAutoPathApp.exe'" | Where-Object ExecutablePath -eq (Join-Path $bin 'OpenAutoPathApp.exe') | Select-Object -First 1
            if($child){$app=Get-Process -Id $child.ProcessId}
        }
        if($app){
            $app.Refresh()
            if($app.HasExited){throw 'Packaged application exited during startup'}
            $window=$native::MainWindow([uint32]$app.Id)
            if($window -ne [IntPtr]::Zero -and $native::Responsive($window)){break}
        }
        Start-Sleep -Milliseconds 500
    }
    if($window -eq [IntPtr]::Zero){throw 'Packaged main window did not appear'}
    Start-Sleep -Seconds 5
    $app.Refresh()
    if($app.HasExited -or -not $native::Responsive($window)){throw 'Packaged main window is not responsive'}
    $modules = @($app.Modules | Select-Object ModuleName,FileName)
    $bundledNames = @{}
    foreach($binary in $inventory.binaries){$bundledNames[[IO.Path]::GetFileName($binary.file)]=$true}
    foreach($module in $modules){
        if($module.ModuleName -match $forbidden){throw 'Application loaded FFmpeg'}
        if($module.ModuleName -ieq 'D3Dcompiler_47.dll' -and $module.FileName -ine $systemCompiler){
            throw 'Application loaded a non-system D3D compiler'
        }
        if($bundledNames.ContainsKey($module.ModuleName) -and
            -not $module.FileName.StartsWith($bin+'\',[StringComparison]::OrdinalIgnoreCase)){
            throw "Application loaded an SDK module outside its package: $($module.ModuleName)"
        }
    }
    @{commit=$info.commit;result='passed';systemD3DCompiler=$systemCompiler;scope='ZIP integrity, PE imports, isolated PATH and responsive main-window startup; not a pristine-OS or visual CAD test';modules=$modules} |
        ConvertTo-Json -Depth 6 | Set-Content (Join-Path $output 'startup-result.json') -Encoding UTF8
    Write-Host "Package integrity, FFmpeg-free imports and isolated startup passed: $($info.commit)"
} finally {
    if($app){
        $app.Refresh()
        if(-not $app.HasExited){
            if($window -ne [IntPtr]::Zero){[void]$native::PostMessage($window,0x10,[IntPtr]::Zero,[IntPtr]::Zero)}
            if(-not $app.WaitForExit(5000)){Stop-Process -Id $app.Id}
        }
    }
    foreach($key in $envNames){[Environment]::SetEnvironmentVariable($key,$saved[$key],'Process')}
}
