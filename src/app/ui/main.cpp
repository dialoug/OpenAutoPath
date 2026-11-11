// OpenAutoPath — 使用 Qt6 + OpenCASCADE 打开并显示 STEP (.stp/.step) 文件。
#include "app/ui/shell/MainWindow.h"

#include "app/ui/LanguageSettings.h"
#include "recognition/FeatureRecognizer.h"

#include <QApplication>
#include <QLocale>
#include <QStringList>
#include <OSD.hxx>

int main(int argc, char* argv[])
{
    // 安装 OCCT 信号/异常处理:在 Windows MSVC 上注册 SE translator,使 OCCT 内部的
    // 访问越界(SEH)被翻译成 Standard_Failure。否则即便代码写了 catch(Standard_Failure)
    // / catch(...),这类异常仍会绕过所有 C++ catch 直接闪退(如 Split 导出路径)。
    // 必须在任何 OCCT 调用前、于主线程执行一次。
    OSD::SetSignal(Standard_False);

    QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("OAP"));
    QCoreApplication::setApplicationName(QStringLiteral("OpenAutoPath"));
    QApplication::setApplicationDisplayName(QStringLiteral("OpenAutoPath"));
    const bool useEnglish = oap::app::preferredLanguage() == QStringLiteral("en");
    oap::app::LanguageTranslator translator(useEnglish);
    if (!translator.loadCatalog() || !app.installTranslator(&translator))
        qFatal("%s", useEnglish ? "Cannot load the embedded language catalogs"
                               : "无法加载内置语言目录");
    QLocale::setDefault(useEnglish ? QLocale(QLocale::English)
                                  : QLocale(QLocale::Chinese, QLocale::China));
    oap::app::previousLanguageMessageHandler = qInstallMessageHandler(oap::app::localizedMessageHandler);
    oap::FeatureRecognizer::setDiagnosticHandler([](const char* prefix, const char* detail) {
        qWarning().noquote() << QString::fromUtf8(prefix) + QString::fromUtf8(detail);
    });
    int exitCode = 0;
    {
        MainWindow w;
        w.show();
        // 可选：在命令行传入 .stp 路径以在启动时打开它。
        // Windows 下 QApplication::arguments() 从宽字符命令行恢复参数，避免窄字符
        // argv 在非 UTF-8 活动代码页上先丢失罕见汉字或 Emoji。
        const QStringList arguments = QApplication::arguments();
        if (arguments.size() > 1)
            w.loadFile(arguments.at(1));
        exitCode = app.exec();
    } // Stop windows and their workers while translation is still installed.
    oap::FeatureRecognizer::setDiagnosticHandler(nullptr);
    qInstallMessageHandler(oap::app::previousLanguageMessageHandler);
    return exitCode;
}
