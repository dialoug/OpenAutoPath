#include "app/ui/LanguageSettings.h"
#include "app/dialogs/ToolpathParameterDialog.h"
#include "tooling/library/ToolLibrary.h"

#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QJsonArray>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <iostream>
#include <stdexcept>

namespace {
QString capturedLog;
void captureLog(QtMsgType, const QMessageLogContext&, const QString& text)
{
    capturedLog = text;
}

void require(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

QStringList placeholders(const QString& text)
{
    const QRegularExpression expression(QStringLiteral("%L?[0-9]+|%n"));
    auto matches = expression.globalMatch(text);
    QStringList result;
    while (matches.hasNext())
        result.append(matches.next().captured());
    result.sort();
    return result;
}

void checkCatalog(const QString& sourceRoot)
{
    QFile file(QStringLiteral(":/i18n/en.json"));
    require(file.open(QIODevice::ReadOnly), "embedded catalog unavailable");
    const auto catalog = QJsonDocument::fromJson(file.readAll()).object();
    require(catalog.size() > 1900, "catalog unexpectedly incomplete");
    const QRegularExpression han(QStringLiteral("[\\x{4e00}-\\x{9fff}]"));
    for (auto it = catalog.begin(); it != catalog.end(); ++it) {
        require(it.value().isString() && !it.value().toString().isEmpty(), "empty translation");
        require(!han.match(it.value().toString()).hasMatch(), "Chinese text in English catalog");
        require(placeholders(it.key()) == placeholders(it.value().toString()), "placeholder mismatch");
    }
    for (const auto* resource : {":/i18n/diagnostics_zh_CN.json", ":/i18n/logs_zh_CN.json"}) {
        QFile supplemental(QString::fromUtf8(resource));
        require(supplemental.open(QIODevice::ReadOnly), "diagnostic catalog unavailable");
        const auto entries = QJsonDocument::fromJson(supplemental.readAll()).object();
        require(!entries.isEmpty(), "diagnostic catalog empty");
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            require(it.value().isString() && !it.value().toString().isEmpty(), "empty diagnostic translation");
            require(placeholders(it.key()) == placeholders(it.value().toString()), "diagnostic placeholder mismatch");
        }
    }
    // Check every static translation call, including adjacent C++ string literals.
    const QRegularExpression call(QString::fromLatin1(
        R"rx(QCoreApplication::translate\("OpenAutoPath",\s*((?:"(?:[^"\\]|\\.)*"\s*)+))rx"));
    const QRegularExpression literal(QString::fromLatin1(R"rx("(?:[^"\\]|\\.)*")rx"));
    QDirIterator sources(sourceRoot + QStringLiteral("/src"),
                         {QStringLiteral("*.cpp"), QStringLiteral("*.h")},
                         QDir::Files, QDirIterator::Subdirectories);
    int checked = 0;
    while (sources.hasNext()) {
        QFile source(sources.next());
        require(source.open(QIODevice::ReadOnly), "cannot read source");
        auto calls = call.globalMatch(QString::fromUtf8(source.readAll()));
        while (calls.hasNext()) {
            QString key;
            auto pieces = literal.globalMatch(calls.next().captured(1));
            while (pieces.hasNext()) {
                const auto json = QByteArray("[") + pieces.next().captured().toUtf8() + "]";
                key += QJsonDocument::fromJson(json).array().at(0).toString();
            }
            if (!catalog.contains(key)) {
                std::cerr << "Missing translation: " << key.toStdString() << '\n';
                require(false, "source text missing from catalog");
            }
            ++checked;
        }
    }
    require(checked > 2000, "source coverage scanner did not run");
}

void checkDiagnostics(bool english)
{
    const auto display = [](const char* text) {
        return QCoreApplication::translate("OpenAutoPathDiagnostic", text);
    };
    require(display("Target surface meshing failed.") ==
            (english ? QStringLiteral("Target surface meshing failed.") : QStringLiteral("目标曲面网格划分失败。")),
            "English domain error was not localized");
    require(display("兼容区域组 [region_17] 规划失败：Target surface meshing failed.") ==
            (english ? QStringLiteral("Compatible region group [region_17] planning failed: Target surface meshing failed.")
                     : QStringLiteral("兼容区域组 [region_17] 规划失败：目标曲面网格划分失败。")),
            "composed diagnostic did not preserve region identity or language");
    const auto message = display("cannot open output file: \"C:/中文/program does not end with M2 or M30.nc\"");
    require(message.endsWith(QStringLiteral("\"C:/中文/program does not end with M2 or M30.nc\"")),
            "diagnostic translation changed a quoted path");
    require(display("cannot open output file: Target surface meshing failed.nc").endsWith(QStringLiteral("Target surface meshing failed.nc")),
            "diagnostic translation changed an unquoted relative path");
    require(display("vendor_code_321: untranslated detail") == QStringLiteral("vendor_code_321: untranslated detail"),
            "unknown third-party error was lost");
    require(QCoreApplication::translate("OpenAutoPathLabel", "unknown_role") ==
            (english ? QStringLiteral("Unknown role") : QStringLiteral("未知角色")), "classification not localized");
    require(QCoreApplication::translate("Unrelated", "unknown_role") == QStringLiteral("unknown_role"),
            "label translation leaked into another context");
    const auto log = QCoreApplication::translate("OpenAutoPathLog",
        "[OpenAutoPath] worker_end request=17 wall_ms=12 ok=true cancelled=false");
    require(log.contains(english ? QStringLiteral("worker_end") : QStringLiteral("后台任务结束")), "log language incorrect");
    require(log.contains(QStringLiteral("17")) && log.contains(QStringLiteral("12")), "log lost numeric data");
    const auto logPath = QCoreApplication::translate("OpenAutoPathLog",
        "[OpenAutoPath] load \"C:/worker_end request=17.stp\"\n  faces=12");
    require(logPath.contains(QStringLiteral("\"C:/worker_end request=17.stp\"")), "log altered path");
    const auto previous = qInstallMessageHandler(oap::app::localizedMessageHandler);
    oap::app::previousLanguageMessageHandler = captureLog;
    qWarning().noquote() << "Target surface meshing failed.";
    const auto warningOutput = capturedLog;
    const QString sourcePath = QStringLiteral("C:/中文/worker_end request=17.stp");
    qDebug().nospace() << "[OpenAutoPath] load " << sourcePath << "\n  faces=12";
    const auto debugOutput = capturedLog;
    qInstallMessageHandler(previous);
    oap::app::previousLanguageMessageHandler = nullptr;
    require(warningOutput == display("Target surface meshing failed."), "message handler did not localize warning");
    require(debugOutput.contains(sourcePath) &&
            debugOutput.contains(english ? QStringLiteral("faces=") : QStringLiteral("面数=")),
            "real debug output lost language or path");
    QDialogButtonBox buttons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    const auto cancel = buttons.button(QDialogButtonBox::Cancel)->text().remove(QLatin1Char('&'));
    require(cancel == (english ? QStringLiteral("Cancel") : QStringLiteral("取消")), "Qt standard button language incorrect");
    require(QCoreApplication::translate("QFileDialog", "Look in:") ==
            (english ? QStringLiteral("Look in:") : QStringLiteral("查找范围：")), "Qt file dialog language incorrect");
}

void checkDialog(bool english)
{
    oap::app::ToolpathParameterEditorState initial;
    bool inspected = false;
    bool valid = true;
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog)
            return;
        inspected = true;
        const auto* tabs = dialog->findChild<QTabWidget*>();
        valid = tabs && tabs->count() == 6 &&
            tabs->tabText(0) == (english ? QStringLiteral("Hole parameters") : QStringLiteral("孔参数"));
        const QRegularExpression han(QStringLiteral("[\\x{4e00}-\\x{9fff}]"));
        if (english) {
            for (const auto* label : dialog->findChildren<QLabel*>())
                valid &= !han.match(label->text()).hasMatch();
            for (const auto* combo : dialog->findChildren<QComboBox*>()) {
                for (int index = 0; index < combo->count(); ++index)
                    valid &= !han.match(combo->itemText(index)).hasMatch();
            }
        }
        int fields = 0;
        for (const auto* form : dialog->findChildren<QFormLayout*>()) {
            for (int row = 0; row < form->rowCount(); ++row) {
                auto* item = form->itemAt(row, QFormLayout::LabelRole);
                const auto* label = item ? qobject_cast<QLabel*>(item->widget()) : nullptr;
                if (!label || label->text() != (english ? QStringLiteral("Depth per layer") : QStringLiteral("每层切深")))
                    continue;
                auto* field = qobject_cast<QDoubleSpinBox*>(form->itemAt(row, QFormLayout::FieldRole)->widget());
                if (field) {
                    field->setValue(1.25);
                    ++fields;
                }
            }
        }
        valid &= fields == 3;
        auto* buttons = dialog->findChild<QDialogButtonBox*>();
        if (buttons && valid)
            buttons->button(QDialogButtonBox::Ok)->click();
        else
            dialog->reject();
    });
    const auto edited = oap::app::editToolpathParameters(nullptr, initial);
    require(inspected && valid && edited.has_value(), "localized dialog labels or controls incorrect");
    require(edited->holeParams.depth.stepDown == 1.25 &&
            edited->pocketParams.depth.stepDown == 1.25 &&
            edited->profileParams.depth.stepDown == 1.25,
            "language changed parameter editing behavior");
}
} // namespace

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QTemporaryDir settingsDirectory;
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, settingsDirectory.path());
    QCoreApplication::setOrganizationName(QStringLiteral("OAPLanguageTest"));
    QCoreApplication::setApplicationName(QStringLiteral("LanguageSmoke"));
    try {
        require(app.arguments().size() == 2, "source root argument required");
        checkCatalog(app.arguments().at(1));
        require(oap::app::preferredLanguage() == QStringLiteral("zh_CN"), "default language changed");
        oap::app::LanguageTranslator chinese(false);
        require(chinese.loadCatalog() && app.installTranslator(&chinese), "Chinese translator install failed");
        checkDiagnostics(false);
        checkDialog(false);
        app.removeTranslator(&chinese);
        QSettings().setValue(QStringLiteral("ui/language"), QStringLiteral("en"));
        require(oap::app::preferredLanguage() == QStringLiteral("en"), "English preference not saved");
        oap::app::LanguageTranslator translator;
        require(translator.loadCatalog() && app.installTranslator(&translator), "translator install failed");
        require(QCoreApplication::translate("OpenAutoPath", "文件(&F)") == QStringLiteral("&File"), "menu not translated");
        require(QCoreApplication::translate("Unrelated", "文件(&F)") == QStringLiteral("文件(&F)"), "context leaked");
        require(QCoreApplication::translate("OpenAutoPath", "Not in catalog") == QStringLiteral("Not in catalog"), "fallback failed");
        require(oap::tooling::toolTypeDisplayName(oap::tooling::ToolType::Drill) == QStringLiteral("Drill"), "tool type not translated");
        const auto stableId = oap::tooling::toolTypeId(oap::tooling::ToolType::Drill);
        checkDiagnostics(true);
        checkDialog(true);
        app.removeTranslator(&translator);
        require(oap::tooling::toolTypeId(oap::tooling::ToolType::Drill) == stableId, "persistent tool ID changed");
        QSettings().setValue(QStringLiteral("ui/language"), QStringLiteral("zh_CN"));
        require(app.installTranslator(&chinese), "Chinese translator reinstall failed");
        checkDiagnostics(false);
        checkDialog(false);
        app.removeTranslator(&chinese);
        QSettings().setValue(QStringLiteral("ui/language"), QStringLiteral("unsupported"));
        require(oap::app::preferredLanguage() == QStringLiteral("zh_CN"), "unsupported language fallback failed");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "Language catalog, dialogs, settings and stable IDs passed\n";
    return 0;
}
