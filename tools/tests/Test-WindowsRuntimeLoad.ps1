# Check the deployed DLL dependency closure in a fresh PowerShell process.
[CmdletBinding()]
param([Parameter(Mandatory)][string]$BinDir)
$ErrorActionPreference = 'Stop'
$bin = (Resolve-Path -LiteralPath $BinDir).Path
if (-not (Test-Path (Join-Path $bin 'OpenAutoPathApp.exe'))) { throw 'Application runtime directory is missing' }
$typeName = 'RuntimeLoad_' + [guid]::NewGuid().ToString('N')
$code = @'
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices;
public static class __TYPE__ {
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern bool SetDllDirectory(string path);
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern IntPtr LoadLibrary(string path);
    [DllImport("kernel32.dll")] static extern bool FreeLibrary(IntPtr module);
    public static void Run(string dir,string[] files) {
        var handles=new List<IntPtr>();
        if(!SetDllDirectory(dir)) throw new Win32Exception(Marshal.GetLastWin32Error());
        try {
            foreach(string file in files) {
                IntPtr handle=LoadLibrary(file);
                if(handle==IntPtr.Zero) throw new Win32Exception(Marshal.GetLastWin32Error(),"Cannot load deployed DLL: "+file);
                handles.Add(handle);
            }
        } finally {
            for(int i=handles.Count-1;i>=0;i--) FreeLibrary(handles[i]);
            SetDllDirectory(null);
        }
    }
}
'@
$type = Add-Type -TypeDefinition $code.Replace('__TYPE__', $typeName) -PassThru
$files = @(Get-ChildItem -LiteralPath $bin -Filter '*.dll' -File | Sort-Object Name | Select-Object -ExpandProperty FullName)
if ($files.Count -eq 0) { throw 'No runtime DLLs found' }
$type::Run($bin, [string[]]$files)
Write-Host "Loaded $($files.Count) deployed DLLs and their import dependencies."
