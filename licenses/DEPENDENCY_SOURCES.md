# 原生依赖源码

[构建说明](../BUILDING.md)提供当前 worker 使用的官方 OCCT Windows SDK、第三方运行库和 CMake 依赖安装入口。SDK 许可见 [runtime](runtime/README.md)。固定源码归档的 URL 和 SHA-256 见 `windows-runtime.json`。

OpenCAMLib 按 CMake 锁定的提交从上游构建为共享库。可在 ABI 与架构匹配的前提下更换 OCCT 和 OpenCAMLib 动态库。分发二进制时，根据各库的许可条件随包提供对应声明和源码获取方式。

## 重新构建 FreeImage 和 OCCT

以下脚本可用于准备自己构建的 OCCT SDK。FreeImage 使用仓库的 OpenEXR 兼容修补，并运行 PNG 读写回归。OCCT 保留图像和 TBB 支持，关闭 FFmpeg，采用 NATIVE 分配器。两个脚本根据清单校验输入归档。

先运行 `tools/Install-WindowsOcct.ps1`，获取 FreeType、oneTBB 等其余第三方 SDK。把清单中注明的源码归档保存到 `external/release-sources`，然后从 Visual Studio Developer PowerShell 运行：

```powershell
./tools/Build-WindowsFreeImage.ps1 -SourceArchive external/release-sources/FreeImage3180-mirror.zip -OutputDir build/dependencies/freeimage -Parallel 4
./tools/Build-WindowsOCCT.ps1 -SourceArchive external/release-sources/OCCT-V7_9_3.tar.gz -ThirdPartyRoot external/3rdparty-vc14-64 -FreeImageRoot build/dependencies/freeimage -OutputDir build/dependencies/occt -Generator 'Visual Studio 18 2026' -Parallel 4
cmake --preset windows-core-vs2026 -DOAP_FREEIMAGE_ROOT="$PWD/build/dependencies/freeimage" -DOAP_OCCT_ROOT="$PWD/build/dependencies/occt/install"
```

VS 2022 使用 `Visual Studio 17 2022` 和 `windows-core`。脚本在各自输出目录写入构建记录。
