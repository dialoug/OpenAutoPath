[CmdletBinding()]
param([Parameter(Mandatory)][string]$BinDir, [Parameter(Mandatory)][string]$OutputDir)
$ErrorActionPreference = 'Stop'
$bin = (Resolve-Path -LiteralPath $BinDir).Path
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$output = [IO.Path]::GetFullPath($OutputDir)
$image = Join-Path $output 'input.ppm'
$pixels = New-Object byte[] (16 * 16 * 3)
for ($i = 0; $i -lt $pixels.Length; $i++) { $pixels[$i] = ($i * 13) % 256 }
$header = [Text.Encoding]::ASCII.GetBytes("P6`n16 16`n255`n")
[IO.File]::WriteAllBytes($image, [byte[]]($header + $pixels))
$ffprobe = Join-Path $bin 'ffprobe.exe'
$info = & $ffprobe -v error -show_frames -show_entries frame=width,height,pix_fmt -of json $image
if ($LASTEXITCODE -ne 0) { throw 'FFprobe PPM decode failed' }
$frame = ($info -join "`n" | ConvertFrom-Json).frames[0]
if ($frame.width -ne 16 -or $frame.height -ne 16 -or $frame.pix_fmt -ne 'rgb24') { throw 'Unexpected decoded frame' }
$info = & $ffprobe -v error -show_packets -show_data_hash sha256 -show_entries packet=data_hash -of json $image
if ($LASTEXITCODE -ne 0) { throw 'FFprobe packet inspection failed' }
$packet = ($info -join "`n" | ConvertFrom-Json).packets[0]
if ($packet.data_hash -ne ('SHA256:' + (Get-FileHash $image).Hash)) { throw 'Demuxed packet differs from source pixels/header' }
$typeName = 'FFmpegScaleSmoke_' + [guid]::NewGuid().ToString('N')
$code = @'
using System;
using System.Runtime.InteropServices;
public static class __TYPE__ {
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode)] static extern IntPtr LoadLibrary(string name);
    [DllImport(@"__DLL__",CallingConvention=CallingConvention.Cdecl)] static extern IntPtr sws_getContext(int sw,int sh,int sf,int dw,int dh,int df,int flags,IntPtr a,IntPtr b,IntPtr c);
    [DllImport(@"__DLL__",CallingConvention=CallingConvention.Cdecl)] static extern int sws_scale(IntPtr ctx,IntPtr[] src,int[] ss,int y,int h,IntPtr[] dst,int[] ds);
    [DllImport(@"__DLL__",CallingConvention=CallingConvention.Cdecl)] static extern void sws_freeContext(IntPtr ctx);
    public static void Run(string avutil, byte[] pixels) {
        if(LoadLibrary(avutil)==IntPtr.Zero) throw new Exception("Cannot load avutil dependency");
        IntPtr input=Marshal.AllocHGlobal(pixels.Length),output=Marshal.AllocHGlobal(pixels.Length),ctx=IntPtr.Zero;
        try {
            Marshal.Copy(pixels,0,input,pixels.Length);
            // FFmpeg 3.3.4: RGB24=2, BGR24=3; SWS_POINT=16.
            ctx=sws_getContext(16,16,2,16,16,3,16,IntPtr.Zero,IntPtr.Zero,IntPtr.Zero);
            if(ctx==IntPtr.Zero) throw new Exception("sws_getContext failed");
            var src=new IntPtr[]{input,IntPtr.Zero,IntPtr.Zero,IntPtr.Zero};
            var dst=new IntPtr[]{output,IntPtr.Zero,IntPtr.Zero,IntPtr.Zero};
            if(sws_scale(ctx,src,new int[]{48,0,0,0},0,16,dst,new int[]{48,0,0,0})!=16) throw new Exception("Incomplete conversion");
            byte[] result=new byte[pixels.Length]; Marshal.Copy(output,result,0,result.Length);
            for(int i=0;i<result.Length;i+=3)
                if(result[i]!=pixels[i+2] || result[i+1]!=pixels[i+1] || result[i+2]!=pixels[i]) throw new Exception("RGB/BGR pixel mismatch");
        } finally { if(ctx!=IntPtr.Zero)sws_freeContext(ctx);Marshal.FreeHGlobal(input);Marshal.FreeHGlobal(output); }
    }
}
'@
$type = Add-Type -TypeDefinition $code.Replace('__TYPE__',$typeName).Replace('__DLL__',(Join-Path $bin 'swscale-4.dll').Replace('"','""')) -PassThru
$type::Run((Join-Path $bin 'avutil-55.dll'), $pixels)
Write-Host 'FFmpeg PPM decode, packet checksum and RGB/BGR conversion: OK'
