# 原始许可副本来源

此目录保存第三方声明，保留原始编码与换行；不适用项目自有代码的 MIT。
组件与运行库文件的对应关系见上一级 `windows-runtime.json`，源码获取说明见
上一级 `DEPENDENCY_SOURCES.md`。源文件核对日期：2026-11-11。

| 本目录文件或目录 | 原始来源 |
| --- | --- |
| OCCT | OCCT 7.9.3 官方 Windows SDK 的 LICENSE_LGPL_21.txt 和 OCCT_LGPL_EXCEPTION.txt |
| FreeImage | FreeImage3180.zip 中根目录许可及 Source 目录的许可、版权、专利和 README 文件；额外保留 TIFF/JXR/zlib 含许可的源文件 |
| FreeType | freetype-2.13.3.tar.xz 的许可、FTL/GPL 及 BDF/PCF/zlib/HarfBuzz 相关声明源文件 |
| oneTBB-LICENSE.txt | OCCT 第三方 SDK 中 tbb-2021.13.0-x64/LICENSE.txt |
| jemalloc-COPYRIGHT.txt | OCCT 第三方 SDK 中 jemalloc-vc14-64/share/jemalloc/copyright |
| OpenVR-LICENSE.txt | https://raw.githubusercontent.com/ValveSoftware/openvr/v1.14.15/LICENSE |
| Microsoft-RUNTIME.txt | 本项目提供的运行库识别与微软原始条款入口说明；实际许可保留在微软安装器及适用的开发工具许可中 |

其余随包源码归档哈希见 `windows-runtime.json`。更新组件时应从相应上游版本重新核对
原始声明，并更新二进制指纹和源码归档，不能把旧声明直接套用于新 SDK。
