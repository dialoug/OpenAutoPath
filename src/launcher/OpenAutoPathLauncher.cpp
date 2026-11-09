// OpenAutoPath 根目录启动器：为 bin/ 中的 Qt/OCCT 应用准备运行环境。
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <string>
#include <vector>

namespace {

std::wstring moduleFilePath()
{
    std::vector<wchar_t> buffer(1024);
    for (;;) {
        const DWORD length = GetModuleFileNameW(
            nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (length == 0)
            return {};
        if (length < buffer.size() - 1)
            return std::wstring(buffer.data(), length);
        if (buffer.size() >= 32768)
            return {};
        buffer.resize(buffer.size() * 2);
    }
}

std::wstring parentDirectory(const std::wstring& path)
{
    const std::wstring::size_type separator = path.find_last_of(L"\\/");
    return separator == std::wstring::npos ? std::wstring{} : path.substr(0, separator);
}

std::wstring quoteArgument(const std::wstring& argument)
{
    std::wstring quoted;
    quoted.reserve(argument.size() + 2);
    quoted.push_back(L'"');
    std::size_t slashCount = 0;
    for (const wchar_t ch : argument) {
        if (ch == L'\\') {
            ++slashCount;
            continue;
        }
        if (ch == L'"') {
            quoted.append(slashCount * 2 + 1, L'\\');
            quoted.push_back(ch);
        } else {
            quoted.append(slashCount, L'\\');
            quoted.push_back(ch);
        }
        slashCount = 0;
    }
    quoted.append(slashCount * 2, L'\\');
    quoted.push_back(L'"');
    return quoted;
}

int showLaunchError(const wchar_t* message, DWORD error = 0)
{
    std::wstring detail(message);
    if (error != 0) {
        detail += L"\n\nWindows error: ";
        detail += std::to_wstring(error);
    }
    MessageBoxW(nullptr, detail.c_str(), L"OpenAutoPath", MB_OK | MB_ICONERROR);
    return 1;
}

} // namespace

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR arguments, int)
{
    const std::wstring launcherPath = moduleFilePath();
    const std::wstring applicationRoot = parentDirectory(launcherPath);
    if (applicationRoot.empty())
        return showLaunchError(L"无法确定程序主目录。", GetLastError());

    const std::wstring applicationPath =
        applicationRoot + L"\\bin\\OpenAutoPathApp.exe";
    if (GetFileAttributesW(applicationPath.c_str()) == INVALID_FILE_ATTRIBUTES) {
        return showLaunchError(
            L"未找到 bin\\OpenAutoPathApp.exe，请重新构建或完整部署程序。",
            GetLastError());
    }
    if (!SetEnvironmentVariableW(L"OAP_APP_ROOT", applicationRoot.c_str()))
        return showLaunchError(L"无法设置应用主目录环境。", GetLastError());

    std::wstring commandLine = quoteArgument(applicationPath);
    if (arguments && *arguments) {
        commandLine.push_back(L' ');
        commandLine += arguments;
    }
    std::vector<wchar_t> mutableCommandLine(commandLine.begin(), commandLine.end());
    mutableCommandLine.push_back(L'\0');

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};
    if (!CreateProcessW(applicationPath.c_str(), mutableCommandLine.data(),
                        nullptr, nullptr, FALSE, 0, nullptr,
                        applicationRoot.c_str(), &startup, &process)) {
        return showLaunchError(L"无法启动 OpenAutoPath 主程序。", GetLastError());
    }

    CloseHandle(process.hThread);
    const DWORD waitResult = WaitForSingleObject(process.hProcess, INFINITE);
    DWORD exitCode = 1;
    if (waitResult == WAIT_OBJECT_0)
        GetExitCodeProcess(process.hProcess, &exitCode);
    CloseHandle(process.hProcess);
    return static_cast<int>(exitCode);
}
