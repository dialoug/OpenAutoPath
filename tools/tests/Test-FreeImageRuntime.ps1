[CmdletBinding()]
param([Parameter(Mandatory)][string]$Dll, [Parameter(Mandatory)][string]$OutputDir)
$ErrorActionPreference = 'Stop'
$dllPath = (Resolve-Path -LiteralPath $Dll).Path.Replace('"', '""')
$typeName = 'FreeImageSmoke_' + [guid]::NewGuid().ToString('N')
$code = @'
using System;
using System.Runtime.InteropServices;
public static class __TYPE__ {
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern IntPtr FreeImage_GetVersion();
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern void FreeImage_Initialise(bool local);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern void FreeImage_DeInitialise();
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern IntPtr FreeImage_Allocate(int w,int h,int bpp,uint r,uint g,uint b);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern IntPtr FreeImage_GetScanLine(IntPtr dib,int row);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern uint FreeImage_GetWidth(IntPtr dib);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern uint FreeImage_GetHeight(IntPtr dib);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern uint FreeImage_GetBPP(IntPtr dib);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall,CharSet=CharSet.Unicode)] static extern bool FreeImage_SaveU(int format,IntPtr dib,string file,int flags);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall,CharSet=CharSet.Unicode)] static extern IntPtr FreeImage_LoadU(int format,string file,int flags);
    [DllImport(@"__DLL__", CallingConvention=CallingConvention.StdCall)] static extern void FreeImage_Unload(IntPtr dib);
    public static void Run(string file) {
        if(Marshal.PtrToStringAnsi(FreeImage_GetVersion()) != "3.18.0") throw new Exception("Unexpected FreeImage version");
        FreeImage_Initialise(false);
        IntPtr image=IntPtr.Zero, loaded=IntPtr.Zero;
        try {
            image=FreeImage_Allocate(16,16,24,0,0,0);
            if(image==IntPtr.Zero) throw new Exception("Allocation failed");
            for(int y=0;y<16;y++) for(int x=0;x<48;x++) Marshal.WriteByte(FreeImage_GetScanLine(image,y),x,(byte)(x+y*3));
            if(!FreeImage_SaveU(13,image,file,0)) throw new Exception("PNG save failed");
            loaded=FreeImage_LoadU(13,file,0);
            if(loaded==IntPtr.Zero || FreeImage_GetWidth(loaded)!=16 || FreeImage_GetHeight(loaded)!=16 || FreeImage_GetBPP(loaded)!=24)
                throw new Exception("PNG load dimensions/format mismatch");
            for(int y=0;y<16;y++) for(int x=0;x<48;x++)
                if(Marshal.ReadByte(FreeImage_GetScanLine(loaded,y),x)!=(byte)(x+y*3)) throw new Exception("PNG pixel mismatch");
        } finally {
            if(loaded!=IntPtr.Zero) FreeImage_Unload(loaded);
            if(image!=IntPtr.Zero) FreeImage_Unload(image);
            FreeImage_DeInitialise();
        }
    }
}
'@
$type = Add-Type -TypeDefinition $code.Replace('__TYPE__', $typeName).Replace('__DLL__', $dllPath) -PassThru
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$type::Run((Join-Path ([IO.Path]::GetFullPath($OutputDir)) 'roundtrip.png'))
Write-Host 'FreeImage 3.18.0 PNG round-trip: OK'
