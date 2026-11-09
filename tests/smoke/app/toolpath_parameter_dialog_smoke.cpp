#include "app/dialogs/ToolpathParameterDialog.h"
#include "app/dialogs/QtWorkflowInteraction.h"
#include "app/dialogs/StockSimulationDialogs.h"

#include "app/workflows/project/EngravingSettingsStorage.h"
#include <QSettings>
#include <QTemporaryDir>
#include <QSpinBox>
#include <QTabWidget>
#include <QApplication>
#include <QComboBox>
#include <QCursor>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

bool projectTimelineSmoke()
{
    // Empty results must never implicitly request an animation window.
    if (oap::app::showIpwTimelineDialog(nullptr, {}, {}, {}, true))
        return false;
    for (bool allowOpenGl : {false, true}) {
      for (int action : {0, 1, 2}) { // Explicit animation, keep result, window close.
        bool inspected = false;
        bool selected = false;
        bool frameShown = false;
        QTimer::singleShot(0, [&] {
            auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
            if (!dialog)
                return;
            auto* buttons = dialog->findChild<QDialogButtonBox*>();
            bool mentionsOpenGl = false;
            for (const auto* label : dialog->findChildren<QLabel*>())
                mentionsOpenGl |= label->text().contains(QStringLiteral("OpenGL"));
            inspected = buttons && mentionsOpenGl == allowOpenGl &&
                (buttons->button(QDialogButtonBox::Ok) != nullptr) == allowOpenGl &&
                (buttons->button(QDialogButtonBox::Close) != nullptr) == !allowOpenGl;
            if (inspected && allowOpenGl)
                inspected = !buttons->button(QDialogButtonBox::Ok)->isDefault() &&
                    buttons->button(QDialogButtonBox::Cancel)->isDefault();
            if (buttons && inspected && action != 2)
                buttons->button(allowOpenGl
                    ? (action == 0 ? QDialogButtonBox::Ok : QDialogButtonBox::Cancel)
                    : QDialogButtonBox::Close)->click();
            else
                dialog->reject();
        });
        oap::app::IpwTimelineOperation operation;
        operation.operationIndex = 3;
        operation.moveCount = 7;
        operation.formalOperationId = QStringLiteral("engraving-operation");
        operation.replayAvailable = true;
        const bool continueOpenGl = oap::app::showIpwTimelineDialog(
            nullptr, {operation},
            [&](int index, int moves) { frameShown = index == 3 && moves == 7; },
            [&](const QString& id) { selected = id == operation.formalOperationId; },
            allowOpenGl);
        if (!inspected || !selected || !frameShown || continueOpenGl != (allowOpenGl && action == 0))
            return false;
      }
    }
    return true;
}

bool workflowInteractionSmoke()
{
    // Retain another owner's cursor while testing both nested explicit busy
    // scopes and the progress-only entry used by stock simulation.
    QApplication::setOverrideCursor(Qt::CrossCursor);
    std::vector<bool> transitions;
    bool externalCancellation = false;
    auto interaction = oap::app::makeQtWorkflowInteraction(
        nullptr,
        [&](bool busy) { transitions.push_back(busy); },
        [&] { return externalCancellation; });
    interaction.setBusy(true);
    interaction.setBusy(true);
    bool finished = false;
    bool busyDuringProgress = true;
    QTimer complete;
    complete.setSingleShot(true);
    QObject::connect(&complete, &QTimer::timeout, [&] { finished = true; });
    complete.start(0);
    oap::app::WorkflowProgressRequest request;
    request.title = QStringLiteral("交互生命周期回归");
    request.ready = [&] {
        busyDuringProgress &= transitions == std::vector<bool>{true} &&
            QApplication::overrideCursor() &&
            QApplication::overrideCursor()->shape() == Qt::WaitCursor;
        return finished;
    };
    interaction.showProgress(request);
    interaction.setBusy(false);
    if (!busyDuringProgress || transitions != std::vector<bool>{true})
        return false;
    interaction.setBusy(false);
    interaction.setBusy(false); // Unmatched release must not pop CrossCursor.
    if (transitions != std::vector<bool>({true, false}) ||
        !QApplication::overrideCursor() ||
        QApplication::overrideCursor()->shape() != Qt::CrossCursor)
        return false;

    // Each source can cancel independently. No setBusy call surrounds this
    // path: showProgress must keep the window busy until the worker is ready.
    for (bool cancelExternally : {true, false}) {
        transitions.clear();
        externalCancellation = cancelExternally;
        int cancellations = 0;
        finished = false;
        bool cancelledWhileBusy = false;
        request.ready = [&] { return finished; };
        request.cancellationRequested = [=] { return !cancelExternally; };
        request.cancel = [&] {
            ++cancellations;
            cancelledWhileBusy = transitions == std::vector<bool>{true};
            finished = true;
        };
        QTimer timeout;
        timeout.setSingleShot(true);
        QObject::connect(&timeout, &QTimer::timeout, [&] { finished = true; });
        timeout.start(1000);
        interaction.showProgress(request);
        if (cancellations != 1 || !cancelledWhileBusy ||
            transitions != std::vector<bool>({true, false}))
            return false;
    }
    // A synchronous failure before the modal event loop must also release its
    // busy scope, so the window is not permanently disabled after an error.
    transitions.clear();
    request.ready = []() -> bool { throw std::runtime_error("ready failure"); };
    bool caught = false;
    try {
        interaction.showProgress(request);
    } catch (const std::runtime_error&) {
        caught = true;
    }
    const bool restored = QApplication::overrideCursor() &&
        QApplication::overrideCursor()->shape() == Qt::CrossCursor;
    QApplication::restoreOverrideCursor();
    return caught && restored && transitions == std::vector<bool>({true, false});
}

} // namespace

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    if (!projectTimelineSmoke()) {
        std::cerr << "FAIL: project timeline must offer only supported continuation actions\n";
        return 1;
    }
    if (!workflowInteractionSmoke()) {
        std::cerr << "FAIL: workflow interaction busy scopes/cancellation must remain balanced\n";
        return 1;
    }
    oap::app::ToolpathParameterEditorState initial;
    bool inspected = false;
    int manualToolSources = 0;
    bool automaticGeometryHidden = true;
    int depthFields = 0;
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog)
            return;
        inspected = true;
        for (const auto* combo : dialog->findChildren<QComboBox*>()) {
            for (int index = 0; index < combo->count(); ++index)
                manualToolSources += combo->itemText(index).contains(QStringLiteral("手动单刀")) ? 1 : 0;
        }
        for (const QString& prefix : {QStringLiteral("hole"), QStringLiteral("pocket"), QStringLiteral("profile")}) {
            const auto* geometry = dialog->findChild<QDoubleSpinBox*>(prefix + "ToolDiameter");
            automaticGeometryHidden &= geometry && geometry->isHidden();
        }
        for (const auto* form : dialog->findChildren<QFormLayout*>()) {
            for (int row = 0; row < form->rowCount(); ++row) {
                const auto* labelItem = form->itemAt(row, QFormLayout::LabelRole);
                const auto* label = labelItem ? qobject_cast<QLabel*>(labelItem->widget()) : nullptr;
                if (!label)
                    continue;
                if (label->text() != QStringLiteral("每层切深"))
                    continue;
                auto* field = qobject_cast<QDoubleSpinBox*>(form->itemAt(row, QFormLayout::FieldRole)->widget());
                if (field) {
                    field->setValue(1.25);
                    ++depthFields;
                }
            }
        }
        const auto* buttons = dialog->findChild<QDialogButtonBox*>();
        if (buttons)
            buttons->button(QDialogButtonBox::Ok)->click();
        else
            dialog->reject();
    });
    const auto edited = oap::app::editToolpathParameters(nullptr, initial);
    if (!inspected || manualToolSources != 3 || !automaticGeometryHidden || depthFields != 3 || !edited ||
        std::abs(edited->holeParams.depth.stepDown - 1.25) > 1e-9 ||
        std::abs(edited->pocketParams.depth.stepDown - 1.25) > 1e-9 ||
        std::abs(edited->profileParams.depth.stepDown - 1.25) > 1e-9 ||
        edited->holeParams.tool.diameter != initial.holeParams.tool.diameter ||
        !oap::app::EngravingSettingsStorage::equal(edited->engraving, initial.engraving)) {
        std::cerr << "FAIL: automatic process settings must preserve hidden direct-tool snapshots\n";
        return 1;
    }

    bool directInspected = false;
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog) return;
        directInspected = true;
        for (const QString& prefix : {QStringLiteral("hole"), QStringLiteral("pocket"), QStringLiteral("profile")}) {
            auto* mode = dialog->findChild<QComboBox*>(prefix + "ToolMode");
            mode->setCurrentIndex(mode->findData(static_cast<int>(oap::app::ToolSelectionMode::SingleTool)));
            dialog->findChild<QDoubleSpinBox*>(prefix + "ToolDiameter")->setValue(4.5);
            dialog->findChild<QDoubleSpinBox*>(prefix + "ToolLength")->setValue(22);
            dialog->findChild<QDoubleSpinBox*>(prefix + "ToolFeed")->setValue(321);
            dialog->findChild<QDoubleSpinBox*>(prefix + "ToolPlunge")->setValue(123);
            dialog->findChild<QDoubleSpinBox*>(prefix + "ToolSpindle")->setValue(9000);
            dialog->findChild<QSpinBox*>(prefix + "ToolNumber")->setValue(8);
        }
        dialog->findChild<QDoubleSpinBox*>("holeToolTipAngle")->setValue(130);
        dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
    });
    const auto direct = oap::app::editToolpathParameters(nullptr, initial);
    if (!direct || !directInspected || !direct->activeToolLibraryId.empty() ||
        direct->holeToolSelectionMode != oap::app::ToolSelectionMode::SingleTool ||
        direct->pocketToolSelectionMode != oap::app::ToolSelectionMode::SingleTool ||
        direct->profileToolSelectionMode != oap::app::ToolSelectionMode::SingleTool ||
        direct->holeSingleTool.toolType != "drill" || direct->holeSingleTool.tipAngle != 130 ||
        direct->pocketSingleTool.toolType != "end_mill" || direct->profileSingleTool.toolType != "end_mill") {
        std::cerr << "FAIL: all process pages must configure direct tools without an active library\n";
        return 1;
    }
    for (const auto* tool : {&direct->holeSingleTool, &direct->pocketSingleTool, &direct->profileSingleTool}) {
        if (!tool->valid || tool->toolNumber != 8 || tool->diameter != 4.5 || tool->cuttingEdgeLength != 22 ||
            tool->horizontalFeed != 321 || tool->verticalFeed != 123 || tool->spindleSpeed != 9000) {
            std::cerr << "FAIL: direct tool geometry and feeds must be retained\n";
            return 1;
        }
    }
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog) return;
        dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
    });
    const auto retained = oap::app::editToolpathParameters(nullptr, *direct, oap::app::ToolpathParameterEditorPage::Engraving);
    if (!retained ||
        oap::toolpath::toolpathToolContextKey(retained->holeSingleTool) != oap::toolpath::toolpathToolContextKey(direct->holeSingleTool) ||
        oap::toolpath::toolpathToolContextKey(retained->pocketSingleTool) != oap::toolpath::toolpathToolContextKey(direct->pocketSingleTool) ||
        oap::toolpath::toolpathToolContextKey(retained->profileSingleTool) != oap::toolpath::toolpathToolContextKey(direct->profileSingleTool)) {
        std::cerr << "FAIL: editing another page must retain every direct-tool snapshot\n";
        return 1;
    }
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog) return;
        dialog->findChild<QDoubleSpinBox*>("holeToolDiameter")->setValue(7);
        auto* mode = dialog->findChild<QComboBox*>("holeToolMode");
        mode->setCurrentIndex(mode->findData(static_cast<int>(oap::app::ToolSelectionMode::AutomaticLibrary)));
        dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
    });
    const auto automatic = oap::app::editToolpathParameters(nullptr, *direct);
    if (!automatic || automatic->holeToolSelectionMode != oap::app::ToolSelectionMode::AutomaticLibrary ||
        oap::toolpath::toolpathToolContextKey(automatic->holeSingleTool) != oap::toolpath::toolpathToolContextKey(direct->holeSingleTool)) {
        std::cerr << "FAIL: automatic mode must not overwrite its stored direct tool\n";
        return 1;
    }


    for (const auto& cutterType : {QStringLiteral("ball_end_mill"), QStringLiteral("bull_nose")}) {
        bool available = true;
        QTimer::singleShot(0, [&] {
            auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
            if (!dialog) { available = false; return; }
            for (const auto* prefix : {"pocket", "profile"}) {
                auto* type = dialog->findChild<QComboBox*>(QString(prefix) + "ToolType");
                const int index = type->findData(cutterType);
                available &= index >= 0;
                type->setCurrentIndex(index);
                dialog->findChild<QDoubleSpinBox*>(QString(prefix) + "ToolDiameter")->setValue(4);
                dialog->findChild<QDoubleSpinBox*>(QString(prefix) + "ToolCornerRadius")->setValue(0.5);
            }
            dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
        });
        const auto rounded = oap::app::editToolpathParameters(nullptr, *direct);
        const double radius = cutterType == "ball_end_mill" ? 2.0 : 0.5;
        if (!rounded || !available || rounded->pocketSingleTool.toolType != cutterType.toStdString() ||
            rounded->profileSingleTool.toolType != cutterType.toStdString() ||
            rounded->pocketSingleTool.cornerRadius != radius || rounded->profileSingleTool.cornerRadius != radius) {
            std::cerr << "FAIL: pocket/profile rounded single-tool fields lost geometry\n";
            return 1;
        }
        QTimer::singleShot(0, [&] {
            auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
            dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
        });
        const auto restored = oap::app::editToolpathParameters(nullptr, *rounded);
        if (!restored || oap::toolpath::toolpathToolContextKey(restored->pocketSingleTool) !=
                oap::toolpath::toolpathToolContextKey(rounded->pocketSingleTool) ||
            oap::toolpath::toolpathToolContextKey(restored->profileSingleTool) !=
                oap::toolpath::toolpathToolContextKey(rounded->profileSingleTool)) return 1;
    }

    initial.activeToolLibraryId = "roughing";
    initial.engravingLibraries = {{"engraving", "Detail tools"}};
    bool engravingInspected = false;
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog) return;
        auto* tabs = dialog->findChild<QTabWidget*>();
        engravingInspected = tabs && tabs->tabText(tabs->currentIndex()) == QStringLiteral("雕刻参数");
        dialog->findChild<QComboBox*>("engravingToolMode")->setCurrentIndex(1);
        dialog->findChild<QComboBox*>("engravingScope")->setCurrentIndex(1);
        dialog->findChild<QComboBox*>("engravingLibrary")->setCurrentIndex(1);
        dialog->findChild<QDoubleSpinBox*>("engravingDiameter")->setValue(1.5);
        dialog->findChild<QDoubleSpinBox*>("engravingFeed")->setValue(432);
        dialog->findChild<QSpinBox*>("engravingToolNumber")->setValue(7);
        dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
    });
    const auto single = oap::app::editToolpathParameters(nullptr, initial, oap::app::ToolpathParameterEditorPage::Engraving);
    if (!single || !engravingInspected || single->engraving.scope != oap::app::EngravingSetupScope::PositiveZ ||
        single->engraving.toolMode != oap::app::EngravingToolMode::SingleTool ||
        !single->engraving.manualTool.valid || single->engraving.manualTool.toolNumber != 7 ||
        single->engraving.manualTool.diameter != 1.5 || single->engraving.manualTool.horizontalFeed != 432 ||
        single->engraving.libraryId != "engraving" || single->activeToolLibraryId != "roughing") {
        std::cerr << "FAIL: integrated engraving single-tool settings\n";
        return 1;
    }
    for (const auto direction : {oap::app::EngravingSetupScope::NegativeX, oap::app::EngravingSetupScope::PositiveX,
            oap::app::EngravingSetupScope::NegativeY, oap::app::EngravingSetupScope::PositiveY,
            oap::app::EngravingSetupScope::NegativeZ}) {
        bool directionAvailable = false;
        QTimer::singleShot(0, [&] {
            auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
            if (!dialog) return;
            auto* scope = dialog->findChild<QComboBox*>("engravingScope");
            const int index = scope->findData(static_cast<int>(direction));
            directionAvailable = index >= 0;
            if (index >= 0) scope->setCurrentIndex(index);
            dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
        });
        const auto selected = oap::app::editToolpathParameters(nullptr, initial, oap::app::ToolpathParameterEditorPage::Engraving);
        if (!selected || !directionAvailable || selected->engraving.scope != direction) {
            std::cerr << "FAIL: every individual engraving direction must be selectable\n";
            return 1;
        }
    }
    bool roughingFieldsToggled = false;
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (!dialog) return;
        auto* mode = dialog->findChild<QComboBox*>("engravingMachiningMode");
        auto* stepDown = dialog->findChild<QDoubleSpinBox*>("engravingMaxStepDown");
        auto* roughingAllowance = dialog->findChild<QDoubleSpinBox*>("engravingRoughingAllowance");
        roughingFieldsToggled = mode && stepDown && roughingAllowance && stepDown->isHidden() &&
            roughingAllowance->isHidden() && mode->currentData().toInt() ==
                static_cast<int>(oap::toolpath::EngravingMachiningMode::FinishOnly);
        mode->setCurrentIndex(mode->findData(static_cast<int>(oap::toolpath::EngravingMachiningMode::StockRoughingAndFinishing)));
        roughingFieldsToggled &= !stepDown->isHidden() && !roughingAllowance->isHidden();
        stepDown->setValue(2);
        roughingAllowance->setValue(0.4);
        dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
    });
    const auto roughing = oap::app::editToolpathParameters(nullptr, initial, oap::app::ToolpathParameterEditorPage::Engraving);
    if (!roughing || !roughingFieldsToggled ||
        roughing->engraving.parameters.mode != oap::toolpath::EngravingMachiningMode::StockRoughingAndFinishing ||
        roughing->engraving.parameters.maxStepDown != 2 || roughing->engraving.parameters.roughingAllowance != 0.4) {
        std::cerr << "FAIL: stock roughing must be configurable inside the shared engraving page\n";
        return 1;
    }
    for (const bool invalidAllowance : {true, false}) {
        bool validationPromptSeen = false;
        QTimer::singleShot(0, [&] {
            auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
            if (!dialog) return;
            if (invalidAllowance) {
                dialog->findChild<QDoubleSpinBox*>("engravingRoughingAllowance")->setValue(3);
            } else {
                dialog->findChild<QComboBox*>("engravingToolMode")->setCurrentIndex(1);
                dialog->findChild<QDoubleSpinBox*>("engravingLength")->setValue(1);
            }
            // The public dialog must show a validation message and remain open.
            // Close that nested prompt, then cancel the unchanged editor transaction.
            QTimer::singleShot(0, dialog, [&, dialog] {
                auto* prompt = qobject_cast<QMessageBox*>(QApplication::activeModalWidget());
                validationPromptSeen = prompt != nullptr;
                if (prompt) prompt->accept();
                QTimer::singleShot(0, dialog, &QDialog::reject);
            });
            dialog->findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Ok)->click();
        });
        const auto rejected = oap::app::editToolpathParameters(
            nullptr, *roughing, oap::app::ToolpathParameterEditorPage::Engraving);
        if (rejected || !validationPromptSeen) {
            std::cerr << "FAIL: invalid roughing allowance/cutting length must keep the editor uncommitted\n";
            return 1;
        }
    }
    QTemporaryDir directory;
    if (!directory.isValid()) return 1;
    QSettings settings(directory.filePath("engraving.ini"), QSettings::IniFormat);
    QString error;
    auto roundtrip = initial.engraving;
    auto saved = single->engraving;
    saved.parameters.mode = roughing->engraving.parameters.mode;
    saved.parameters.maxStepDown = roughing->engraving.parameters.maxStepDown;
    saved.parameters.roughingAllowance = roughing->engraving.parameters.roughingAllowance;
    saved.parameters.maxMoves = 123456;
    saved.scope = oap::app::EngravingSetupScope::PreserveProjectFaces;
    saved.projectFaces = {oap::MoldSetupFace::NegativeX, oap::MoldSetupFace::PositiveZ};
    if (!oap::app::EngravingSettingsStorage::save(settings, saved, &error) ||
        !oap::app::EngravingSettingsStorage::load(settings, roundtrip, &error) ||
        !oap::app::EngravingSettingsStorage::equal(saved, roundtrip)) {
        std::cerr << "FAIL: engraving settings roundtrip " << error.toStdString() << '\n';
        return 1;
    }
    const QByteArray validBytes = settings.value("toolpath/engraving_parameters").toByteArray();
    const auto validObject = QJsonDocument::fromJson(validBytes).object();
    auto legacyObject = validObject;
    auto legacyParameters = legacyObject.value("parameters").toObject();
    for (const char* key : {"machining_mode", "max_step_down", "roughing_allowance", "max_moves"})
        legacyParameters.remove(QString::fromLatin1(key));
    legacyObject.insert("parameters", legacyParameters);
    settings.setValue("toolpath/engraving_parameters", QJsonDocument(legacyObject).toJson(QJsonDocument::Compact));
    auto legacy = saved;
    const oap::toolpath::SurfaceFinishingParams defaults;
    if (!oap::app::EngravingSettingsStorage::load(settings, legacy, &error) ||
        legacy.parameters.mode != oap::toolpath::EngravingMachiningMode::FinishOnly ||
        legacy.parameters.maxStepDown != defaults.maxStepDown ||
        legacy.parameters.roughingAllowance != defaults.roughingAllowance ||
        legacy.parameters.maxMoves != defaults.maxMoves ||
        oap::toolpath::toolpathToolContextKey(legacy.manualTool) != oap::toolpath::toolpathToolContextKey(saved.manualTool) ||
        legacy.scope != saved.scope || legacy.projectFaces != saved.projectFaces) {
        std::cerr << "FAIL: old engraving preferences must restore finish-only with existing tool and faces intact\n";
        return 1;
    }
    const std::pair<const char*, QJsonValue> invalidFields[] = {
        {"machining_mode", 2}, {"machining_mode", 0.5}, {"machining_mode", "roughing"},
        {"max_step_down", 0}, {"max_step_down", -1}, {"max_step_down", 1e10},
        {"max_step_down", QJsonValue(QJsonValue::Null)},
        {"roughing_allowance", -1}, {"roughing_allowance", 3},
        {"max_moves", 3}, {"max_moves", 4.5},
        {"max_moves", static_cast<double>(std::numeric_limits<int>::max()) + 1}
    };
    for (const auto& [key, invalidValue] : invalidFields) {
        auto invalidObject = validObject;
        auto invalidParameters = invalidObject.value("parameters").toObject();
        invalidParameters.insert(QString::fromLatin1(key), invalidValue);
        invalidObject.insert("parameters", invalidParameters);
        settings.setValue("toolpath/engraving_parameters", QJsonDocument(invalidObject).toJson(QJsonDocument::Compact));
        if (oap::app::EngravingSettingsStorage::load(settings, roundtrip, &error) || error.isEmpty() ||
            !oap::app::EngravingSettingsStorage::equal(saved, roundtrip)) {
            std::cerr << "FAIL: invalid engraving preference must be rejected transactionally: " << key << '\n';
            return 1;
        }
    }
    settings.setValue("toolpath/engraving_parameters", validBytes);
    auto invalidSave = saved;
    invalidSave.parameters.maxMoves = 3;
    if (oap::app::EngravingSettingsStorage::save(settings, invalidSave, &error) || error.isEmpty() ||
        settings.value("toolpath/engraving_parameters").toByteArray() != validBytes) {
        std::cerr << "FAIL: invalid new settings must not overwrite saved engraving preferences\n";
        return 1;
    }
    settings.setValue("toolpath/engraving_parameters", QByteArray("{broken"));
    if (oap::app::EngravingSettingsStorage::load(settings, roundtrip, &error) ||
        !oap::app::EngravingSettingsStorage::equal(saved, roundtrip) || error.isEmpty()) {
        std::cerr << "FAIL: corrupt engraving settings must not replace current values\n";
        return 1;
    }
    std::cout << "PASS: unified process editor, engraving single tool and transactional settings\n";
    return 0;
}
