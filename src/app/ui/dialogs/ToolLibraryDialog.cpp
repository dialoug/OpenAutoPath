// 刀库编辑对话框：在 Qt 表格模型与 ToolLibrary 领域对象之间同步用户修改。
#include "app/ui/dialogs/ToolLibraryDialog.h"

#include "app/ui/view/OcctView.h"
#include "app/view/ToolPreviewGeometryBuilder.h"
#include "tooling/library/ToolLibraryStorage.h"

#include <QAction>
#include <QCheckBox>
#include <QAbstractItemView>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QKeySequence>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QShowEvent>
#include <QSignalBlocker>
#include <QSplitter>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QUuid>
#include <QVBoxLayout>
#include <QTimer>

#include <AIS_Shape.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Quantity_Color.hxx>
#include <V3d_TypeOfOrientation.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

using oap::tooling::CoolantMode;
using oap::tooling::DrillCycle;
using oap::tooling::ToolBit;
using oap::tooling::ToolGeometry;
using oap::tooling::LibraryToolSlot;
using oap::tooling::ToolLibrary;
using oap::tooling::ToolType;

namespace {

constexpr int kToolIdRole = Qt::UserRole + 1;
constexpr int kToolNumberRole = Qt::UserRole + 2;
const QString kAllToolsLibraryId = QStringLiteral("__all_tool_assets__");

QTableWidgetItem* tableItem(const QString& text)
{
    auto* item = new QTableWidgetItem(text);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    return item;
}

QString numberText(double value, int decimals = 3)
{
    if (value <= 0.0) return QStringLiteral("—");
    QString text = QString::number(value, 'f', decimals);
    if (decimals > 0) {
        text.remove(QRegularExpression(QStringLiteral("0+$")));
        text.remove(QRegularExpression(QStringLiteral("\\.$")));
    }
    return text;
}

class ToolProfilePreviewWidget final : public QWidget
{
public:
    explicit ToolProfilePreviewWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(240, 300);
        setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        setToolTip(QStringLiteral(
            "按当前参数示意刀刃、刀杆、刀柄/夹头和主轴头包络；仅用于参数检查，不是碰撞计算模型。"));
    }

    void setToolGeometry(const ToolGeometry& geometry)
    {
        m_geometry = geometry;
        if (!m_hasReferenceGeometry) {
            const double diameter = geometry.diameter > 0.0
                ? geometry.diameter : 6.0;
            const double overallLength = geometry.overallLength > 0.0
                ? geometry.overallLength
                : std::max(30.0, geometry.cuttingEdgeLength * 1.5);
            const double shankDiameter = geometry.shankDiameter > 0.0
                ? geometry.shankDiameter : diameter;
            const double holderDiameter = geometry.holderDiameter > 0.0
                ? geometry.holderDiameter : shankDiameter * 2.2;
            const double holderLength = geometry.holderLength > 0.0
                ? geometry.holderLength : overallLength * 0.35;
            const double spindleDiameter =
                geometry.spindleEnvelopeDiameter > 0.0
                ? geometry.spindleEnvelopeDiameter : holderDiameter * 1.7;
            const double spindleLength =
                geometry.spindleEnvelopeLength > 0.0
                ? geometry.spindleEnvelopeLength : holderLength * 0.75;
            m_referenceTotalLength = std::max(
                60.0,
                (overallLength + holderLength + spindleLength) * 1.25);
            m_referenceMaximumDiameter = std::max(
                20.0,
                std::max({diameter, shankDiameter, holderDiameter,
                          spindleDiameter}) * 1.25);
            m_hasReferenceGeometry = true;
        }
        update();
    }

    void setToolType(ToolType type)
    {
        m_type = type;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), palette().base());
        const QRectF viewport = rect().adjusted(18, 18, -18, -58);
        if (viewport.width() <= 0.0 || viewport.height() <= 0.0)
            return;

        const double diameter = m_geometry.diameter > 0.0
            ? m_geometry.diameter : 6.0;
        const double overallLength = m_geometry.overallLength > 0.0
            ? m_geometry.overallLength
            : std::max(30.0, m_geometry.cuttingEdgeLength * 1.5);
        const double cuttingLength = m_geometry.cuttingEdgeLength > 0.0
            ? std::min(m_geometry.cuttingEdgeLength, overallLength)
            : overallLength * 0.4;
        const double shankDiameter = m_geometry.shankDiameter > 0.0
            ? m_geometry.shankDiameter : diameter;
        const double holderDiameter = m_geometry.holderDiameter > 0.0
            ? m_geometry.holderDiameter : shankDiameter * 2.2;
        const double holderLength = m_geometry.holderLength > 0.0
            ? m_geometry.holderLength : overallLength * 0.35;
        const double spindleDiameter =
            m_geometry.spindleEnvelopeDiameter > 0.0
            ? m_geometry.spindleEnvelopeDiameter : holderDiameter * 1.7;
        const double spindleLength =
            m_geometry.spindleEnvelopeLength > 0.0
            ? m_geometry.spindleEnvelopeLength : holderLength * 0.75;
        const double totalLength = overallLength + holderLength + spindleLength;
        const double maximumDiameter = std::max(
            {diameter, shankDiameter, holderDiameter, spindleDiameter});
        // 使用打开编辑器时建立的参考尺度；小范围修改会直接表现为宽度/长度变化，
        // 只有当前几何确实超出画布时才扩大显示范围，避免“每次自动铺满”掩盖变化。
        const double displayedTotalLength = std::max(
            m_referenceTotalLength, totalLength * 1.05);
        const double displayedMaximumDiameter = std::max(
            m_referenceMaximumDiameter, maximumDiameter * 1.05);
        const double lengthScale = viewport.height() / displayedTotalLength;
        const double diameterScale = std::min(
            viewport.width() * 0.78 / displayedMaximumDiameter,
            lengthScale * 2.8);
        const double centerX = viewport.center().x();
        double y = viewport.top() +
            (viewport.height() - totalLength * lengthScale) * 0.5;

        const auto drawSection = [&](double sectionDiameter,
                                     double sectionLength,
                                     const QColor& color) {
            const double width = sectionDiameter * diameterScale;
            const double height = sectionLength * lengthScale;
            const QRectF section(centerX - width * 0.5, y, width, height);
            painter.setPen(QPen(palette().mid().color(), 1.0));
            painter.setBrush(color);
            painter.drawRoundedRect(section, 2.0, 2.0);
            y += height;
        };
        drawSection(spindleDiameter, spindleLength,
                    QColor(92, 103, 112));
        drawSection(holderDiameter, holderLength,
                    QColor(125, 137, 146));
        const double shankLength = std::max(0.0, overallLength - cuttingLength);
        drawSection(shankDiameter, shankLength,
                    QColor(164, 174, 181));
        const double cuttingTop = y;
        drawSection(diameter, cuttingLength, QColor(54, 145, 205));
        const double cuttingBottom = y;
        const double cuttingWidth = diameter * diameterScale;
        const QRectF cuttingRect(centerX - cuttingWidth * 0.5,
                                 cuttingTop, cuttingWidth,
                                 cuttingBottom - cuttingTop);

        if (m_type == ToolType::BallEndMill && cuttingRect.height() > 0.0) {
            const double ballHeight = std::min(
                cuttingRect.height(), diameter * 0.5 * lengthScale);
            painter.fillRect(
                QRectF(cuttingRect.left() - 1.0,
                       cuttingBottom - ballHeight - 1.0,
                       cuttingRect.width() + 2.0, ballHeight + 2.0),
                palette().base());
            QPainterPath ballTip;
            ballTip.moveTo(cuttingRect.left(), cuttingBottom - ballHeight);
            ballTip.quadTo(cuttingRect.left(), cuttingBottom,
                           centerX, cuttingBottom);
            ballTip.quadTo(cuttingRect.right(), cuttingBottom,
                           cuttingRect.right(), cuttingBottom - ballHeight);
            ballTip.closeSubpath();
            painter.setPen(QPen(palette().mid().color(), 1.0));
            painter.setBrush(QColor(54, 145, 205));
            painter.drawPath(ballTip);
        } else if (m_type == ToolType::BullNose &&
                   m_geometry.cornerRadius > 0.0) {
            painter.fillRect(cuttingRect.adjusted(-1.0, -1.0, 1.0, 1.0),
                             palette().base());
            painter.setPen(QPen(palette().mid().color(), 1.0));
            painter.setBrush(QColor(54, 145, 205));
            painter.drawRoundedRect(
                cuttingRect,
                std::min(cuttingWidth * 0.5,
                         m_geometry.cornerRadius * diameterScale),
                std::min(cuttingRect.height() * 0.5,
                         m_geometry.cornerRadius * lengthScale));
        }

        if (m_geometry.tipAngle > 0.0 &&
            m_geometry.tipAngle < 180.0) {
            const double tipHeight = std::min(
                cuttingLength * lengthScale * 0.45,
                diameter * 0.5 /
                    std::tan(m_geometry.tipAngle * 0.5 *
                             3.14159265358979323846 / 180.0) * lengthScale);
            y -= std::max(2.0, tipHeight);
            QPolygonF tip;
            tip << QPointF(centerX - diameter * diameterScale * 0.5, y)
                << QPointF(centerX + diameter * diameterScale * 0.5, y)
                << QPointF(centerX, y + std::max(2.0, tipHeight));
            painter.setBrush(QColor(54, 145, 205));
            painter.drawPolygon(tip);
        }

        painter.setPen(palette().text().color());
        painter.drawText(
            QRectF(8, height() - 50, width() - 16, 40),
            Qt::AlignCenter,
            QStringLiteral("%1  ·  Ø%2 mm  ·  刃长 %3 mm  ·  总长 %4 mm")
                .arg(oap::tooling::toolTypeDisplayName(m_type),
                     QString::number(m_geometry.diameter, 'f', 3),
                     QString::number(m_geometry.cuttingEdgeLength, 'f', 3),
                     QString::number(m_geometry.overallLength, 'f', 3)));
    }

private:
    ToolGeometry m_geometry;
    ToolType m_type = ToolType::Unknown;
    double m_referenceTotalLength = 60.0;
    double m_referenceMaximumDiameter = 20.0;
    bool m_hasReferenceGeometry = false;
};

class ToolSolidPreviewWidget final : public QWidget
{
public:
    explicit ToolSolidPreviewWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(280, 300);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        m_view = new OcctView(this);
        m_view->setBackgroundGray(0.18);
        m_view->setFaceSelectionEnabled(false);
        m_view->setToolTip(QStringLiteral(
            "中键旋转，右键拖动平移，滚轮缩放；实体仅用于刀具参数预览。"));
        layout->addWidget(m_view, 1);

        m_refreshTimer.setSingleShot(true);
        m_refreshTimer.setInterval(80);
        QObject::connect(&m_refreshTimer, &QTimer::timeout,
                         this, [this] { updatePresentation(); });
        QObject::connect(m_view, &OcctView::viewerInitialized,
                         this, [this] { updatePresentation(); });
    }

    void setTool(ToolType type, const ToolGeometry& geometry)
    {
        m_type = type;
        m_geometry = oap::tooling::normalizedToolGeometry(type, geometry);
        m_refreshTimer.start();
    }

protected:
    void showEvent(QShowEvent* event) override
    {
        QWidget::showEvent(event);
        m_refreshTimer.start();
    }

private:
    void displayShape(const TopoDS_Shape& shape,
                      const Quantity_Color& color,
                      double transparency = 0.0)
    {
        if (shape.IsNull() || m_view->context().IsNull())
            return;
        Handle(AIS_Shape) presentation = new AIS_Shape(shape);
        presentation->SetDisplayMode(AIS_Shaded);
        presentation->SetMaterial(Graphic3d_NOM_SATIN);
        presentation->SetColor(color);
        if (transparency > 0.0)
            presentation->SetTransparency(transparency);
        m_view->context()->Display(presentation, Standard_False);
    }

    void updatePresentation()
    {
        if (m_view->context().IsNull() || m_view->view().IsNull())
            return;
        m_view->context()->RemoveAll(Standard_False);
        const oap::app::view::ToolPreviewGeometry preview =
            oap::app::view::buildToolPreviewGeometry(m_type, m_geometry);
        displayShape(preview.cuttingPart,
                     Quantity_Color(0.10, 0.53, 0.86,
                                    Quantity_TOC_sRGB));
        displayShape(preview.shank,
                     Quantity_Color(0.62, 0.67, 0.71,
                                    Quantity_TOC_sRGB));
        displayShape(preview.holder,
                     Quantity_Color(0.38, 0.43, 0.47,
                                    Quantity_TOC_sRGB));
        displayShape(preview.spindleEnvelope,
                     Quantity_Color(0.24, 0.28, 0.31,
                                    Quantity_TOC_sRGB),
                     0.18);
        m_view->view()->SetProj(V3d_XposYnegZpos);
        m_view->view()->FitAll(0.08, Standard_False);
        m_view->view()->ZFitAll();
        m_view->view()->Redraw();
    }

    OcctView* m_view = nullptr;
    QTimer m_refreshTimer;
    ToolType m_type = ToolType::Unknown;
    ToolGeometry m_geometry;
};

void fillToolTypes(QComboBox* combo)
{
    for (ToolType type : {ToolType::CenterDrill, ToolType::Drill,
                          ToolType::IndexableDrill, ToolType::EndMill,
                          ToolType::BallEndMill, ToolType::BullNose,
                          ToolType::FaceMill, ToolType::ChamferMill,
                          ToolType::BoringBar, ToolType::Reamer,
                          ToolType::Tap, ToolType::Probe, ToolType::Unknown}) {
        combo->addItem(oap::tooling::toolTypeDisplayName(type), static_cast<int>(type));
    }
}

bool editToolValues(QWidget* parent, ToolBit& tool,
                    LibraryToolSlot& slot, bool& assigned, bool isNew,
                    bool hasLibraryContext)
{
    // 对话框编辑副本，只有用户确认且字段校验通过后才回写领域对象。
    QDialog dialog(parent);
    dialog.setWindowTitle(isNew ? QStringLiteral("新增刀具") : QStringLiteral("编辑刀具"));
    dialog.setWindowFlag(Qt::WindowMaximizeButtonHint, true);
    dialog.setSizeGripEnabled(true);
    dialog.setMinimumSize(520, 420);
    dialog.resize(760, 720);

    // FreeCAD 风格地按职责拆分参数；每个页签独立滚动，按钮保持在页签外。
    auto* dialogLayout = new QVBoxLayout(&dialog);
    auto* tabs = new QTabWidget(&dialog);
    const auto makeFormTab = [&](const QString& title) {
        auto* scrollArea = new QScrollArea(tabs);
        scrollArea->setWidgetResizable(true);
        scrollArea->setFrameShape(QFrame::NoFrame);
        auto* content = new QWidget(scrollArea);
        auto* form = new QFormLayout(content);
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setRowWrapPolicy(QFormLayout::WrapLongRows);
        scrollArea->setWidget(content);
        tabs->addTab(scrollArea, title);
        return std::pair<QWidget*, QFormLayout*>(content, form);
    };
    const auto geometryTab = makeFormTab(QStringLiteral("几何与装配"));
    const auto cuttingTab = makeFormTab(QStringLiteral("刀位与切削默认值"));
    const auto drillingTab = makeFormTab(QStringLiteral("钻削与备注"));
    QWidget* geometryEditor = geometryTab.first;
    QWidget* cuttingEditor = cuttingTab.first;
    QWidget* drillingEditor = drillingTab.first;
    QFormLayout* geometryForm = geometryTab.second;
    QFormLayout* cuttingForm = cuttingTab.second;
    QFormLayout* drillingForm = drillingTab.second;
    dialogLayout->addWidget(tabs, 1);

    auto* name = new QLineEdit(tool.name, geometryEditor);
    geometryForm->addRow(QStringLiteral("名称"), name);

    auto* type = new QComboBox(geometryEditor);
    fillToolTypes(type);
    type->setCurrentIndex(std::max(0, type->findData(static_cast<int>(tool.type))));
    geometryForm->addRow(QStringLiteral("类型"), type);
    auto* typeHelp = new QLabel(geometryEditor);
    typeHelp->setWordWrap(true);
    typeHelp->setStyleSheet(QStringLiteral("color: palette(mid);"));
    geometryForm->addRow(QString(), typeHelp);

    auto makeLength = [&](QWidget* owner, double value) {
        auto* input = new QDoubleSpinBox(owner);
        input->setRange(0.0, 10000.0);
        input->setDecimals(3);
        input->setKeyboardTracking(true);
        input->setSuffix(QStringLiteral(" mm"));
        input->setValue(value);
        return input;
    };
    auto* diameter = makeLength(geometryEditor, tool.geometry.diameter);
    auto* cuttingLength = makeLength(geometryEditor, tool.geometry.cuttingEdgeLength);
    auto* overallLength = makeLength(geometryEditor, tool.geometry.overallLength);
    auto* shankDiameter = makeLength(geometryEditor, tool.geometry.shankDiameter);
    auto* holderDiameter = makeLength(geometryEditor, tool.geometry.holderDiameter);
    auto* holderLength = makeLength(geometryEditor, tool.geometry.holderLength);
    auto* spindleEnvelopeDiameter = makeLength(
        geometryEditor, tool.geometry.spindleEnvelopeDiameter);
    auto* spindleEnvelopeLength = makeLength(
        geometryEditor, tool.geometry.spindleEnvelopeLength);
    auto* cornerRadius = makeLength(geometryEditor, tool.geometry.cornerRadius);
    auto* diameterLabel = new QLabel(QStringLiteral("直径"), geometryEditor);
    auto* cuttingLengthLabel = new QLabel(
        QStringLiteral("刀刃长度"), geometryEditor);
    auto* cornerRadiusLabel = new QLabel(
        QStringLiteral("刀尖圆角"), geometryEditor);
    geometryForm->addRow(diameterLabel, diameter);
    geometryForm->addRow(cuttingLengthLabel, cuttingLength);
    geometryForm->addRow(QStringLiteral("总长"), overallLength);
    geometryForm->addRow(QStringLiteral("刀杆直径"), shankDiameter);
    geometryForm->addRow(QStringLiteral("刀柄/夹头直径"), holderDiameter);
    geometryForm->addRow(QStringLiteral("刀柄/夹头长度"), holderLength);
    geometryForm->addRow(QStringLiteral("主轴头包络直径"),
                 spindleEnvelopeDiameter);
    geometryForm->addRow(QStringLiteral("主轴头包络长度"),
                 spindleEnvelopeLength);
    geometryForm->addRow(cornerRadiusLabel, cornerRadius);

    auto* fluteCount = new QSpinBox(geometryEditor);
    fluteCount->setRange(0, 100);
    fluteCount->setSpecialValueText(QStringLiteral("未填写"));
    bool fluteCountOk = false;
    const int storedFluteCount = tool.attributes
        .value(QStringLiteral("flutes")).toInt(&fluteCountOk);
    fluteCount->setValue(fluteCountOk ? storedFluteCount : 0);
    auto* fluteCountLabel = new QLabel(QStringLiteral("刃数"), geometryEditor);
    geometryForm->addRow(fluteCountLabel, fluteCount);

    auto* tipAngle = new QDoubleSpinBox(geometryEditor);
    tipAngle->setRange(0.0, 180.0);
    tipAngle->setDecimals(2);
    tipAngle->setSuffix(QStringLiteral("°"));
    tipAngle->setValue(tool.geometry.tipAngle);
    auto* tipCompensation = makeLength(
        geometryEditor, tool.geometry.tipCompensation);
    tipCompensation->setSpecialValueText(QStringLiteral("未填写"));
    tipCompensation->setToolTip(
        QStringLiteral("实测或厂家给定的钻尖高度 h；大于 0 时优先于刀径和刀尖角公式"));
    auto* tipGeometryWidget = new QWidget(geometryEditor);
    auto* tipGeometryRow = new QHBoxLayout(tipGeometryWidget);
    tipGeometryRow->setContentsMargins(0, 0, 0, 0);
    tipGeometryRow->addWidget(tipAngle);
    auto* tipCompensationLabel = new QLabel(
        QStringLiteral("直接补偿 h"), tipGeometryWidget);
    tipGeometryRow->addWidget(tipCompensationLabel);
    tipGeometryRow->addWidget(tipCompensation);
    auto* tipAngleLabel = new QLabel(QStringLiteral("刀尖角"), geometryEditor);
    geometryForm->addRow(tipAngleLabel, tipGeometryWidget);

    // FreeCAD ToolBitEditor 的几何属性 + Shape 预览布局；这里使用当前项目
    // 已有的包络参数绘制轻量二维装配示意，不引入额外 OCCT 文档对象。
    auto* geometryScroll = qobject_cast<QScrollArea*>(tabs->widget(0));
    tabs->removeTab(0);
    auto* geometryPage = new QWidget(tabs);
    auto* geometryPageLayout = new QVBoxLayout(geometryPage);
    geometryPageLayout->setContentsMargins(0, 0, 0, 0);
    auto* geometrySplitter = new QSplitter(Qt::Horizontal, geometryPage);
    geometryScroll->setMinimumWidth(360);
    geometryScroll->setSizePolicy(
        QSizePolicy::Expanding, QSizePolicy::Expanding);
    geometrySplitter->addWidget(geometryScroll);
    auto* previewPane = new QWidget(geometrySplitter);
    auto* previewPaneLayout = new QVBoxLayout(previewPane);
    previewPaneLayout->setContentsMargins(0, 0, 0, 0);
    auto* previewModeRow = new QHBoxLayout;
    previewModeRow->addWidget(new QLabel(QStringLiteral("预览"), previewPane));
    auto* previewMode = new QComboBox(previewPane);
    previewMode->addItem(QStringLiteral("二维装配"));
    previewMode->addItem(QStringLiteral("三维刀具"));
    previewModeRow->addWidget(previewMode, 1);
    previewPaneLayout->addLayout(previewModeRow);
    auto* previewStack = new QStackedWidget(previewPane);
    auto* geometryPreview = new ToolProfilePreviewWidget(previewStack);
    auto* solidPreview = new ToolSolidPreviewWidget(previewStack);
    previewStack->addWidget(geometryPreview);
    previewStack->addWidget(solidPreview);
    previewPaneLayout->addWidget(previewStack, 1);
    geometrySplitter->addWidget(previewPane);
    QObject::connect(previewMode,
                     QOverload<int>::of(&QComboBox::currentIndexChanged),
                     previewStack, &QStackedWidget::setCurrentIndex);
    geometrySplitter->setChildrenCollapsible(false);
    geometrySplitter->setHandleWidth(6);
    geometrySplitter->setStretchFactor(0, 3);
    geometrySplitter->setStretchFactor(1, 2);
    geometrySplitter->setSizes({500, 340});
    geometryPageLayout->addWidget(geometrySplitter);
    tabs->insertTab(0, geometryPage, QStringLiteral("几何与装配"));
    // QTabWidget::removeTab() 会显式隐藏原页面；重新放入 splitter 后必须恢复
    // 可见状态，否则只剩右侧预览，用户无法编辑几何参数。
    geometryScroll->show();
    tabs->setCurrentIndex(0);

    const auto refreshGeometryPreview = [=] {
        ToolGeometry geometry;
        geometry.diameter = diameter->value();
        geometry.cuttingEdgeLength = cuttingLength->value();
        geometry.overallLength = overallLength->value();
        geometry.shankDiameter = shankDiameter->value();
        geometry.holderDiameter = holderDiameter->value();
        geometry.holderLength = holderLength->value();
        geometry.spindleEnvelopeDiameter =
            spindleEnvelopeDiameter->value();
        geometry.spindleEnvelopeLength =
            spindleEnvelopeLength->value();
        geometry.cornerRadius = cornerRadius->value();
        geometry.tipAngle = tipAngle->value();
        geometry.tipCompensation = tipCompensation->value();
        const ToolType previewType = static_cast<ToolType>(
            type->currentData().toInt());
        geometryPreview->setToolType(previewType);
        geometryPreview->setToolGeometry(
            oap::tooling::normalizedToolGeometry(previewType, geometry));
        solidPreview->setTool(previewType, geometry);
    };
    for (QDoubleSpinBox* input : {
            diameter, cuttingLength, overallLength, shankDiameter,
            holderDiameter, holderLength, spindleEnvelopeDiameter,
            spindleEnvelopeLength, cornerRadius, tipAngle,
            tipCompensation}) {
        QObject::connect(
            input, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            geometryPreview,
            [refreshGeometryPreview](double) { refreshGeometryPreview(); });
    }
    refreshGeometryPreview();

    auto* toolNumber = new QSpinBox(cuttingEditor);
    toolNumber->setRange(0, 9999);
    toolNumber->setSpecialValueText(QStringLiteral("未分配"));
    toolNumber->setValue(assigned ? slot.toolNumber : 0);
    cuttingForm->addRow(QStringLiteral("刀号"), toolNumber);

    auto makeCutting = [&](double value, const QString& suffix) {
        auto* input = new QDoubleSpinBox(cuttingEditor);
        input->setRange(0.0, 1000000.0);
        input->setDecimals(2);
        input->setSpecialValueText(QStringLiteral("沿用当前值"));
        input->setSuffix(suffix);
        input->setValue(value);
        return input;
    };
    auto* spindle = makeCutting(
        slot.cuttingDefaults.spindleSpeed, QStringLiteral(" rpm"));
    auto* horizontalFeed =
        makeCutting(slot.cuttingDefaults.horizontalFeed,
                    QStringLiteral(" mm/min"));
    auto* verticalFeed =
        makeCutting(slot.cuttingDefaults.verticalFeed,
                    QStringLiteral(" mm/min"));
    auto* horizontalRapid =
        makeCutting(slot.cuttingDefaults.horizontalRapid,
                    QStringLiteral(" mm/min"));
    auto* verticalRapid =
        makeCutting(slot.cuttingDefaults.verticalRapid,
                    QStringLiteral(" mm/min"));
    cuttingForm->addRow(QStringLiteral("主轴转速"), spindle);
    cuttingForm->addRow(QStringLiteral("水平进给"), horizontalFeed);
    cuttingForm->addRow(QStringLiteral("垂直进给"), verticalFeed);
    cuttingForm->addRow(QStringLiteral("水平快速移动"), horizontalRapid);
    cuttingForm->addRow(QStringLiteral("垂直快速移动"), verticalRapid);

    auto* coolant = new QComboBox(cuttingEditor);
    for (CoolantMode mode : {CoolantMode::Off, CoolantMode::Flood,
                             CoolantMode::ThroughTool}) {
        coolant->addItem(oap::tooling::coolantModeDisplayName(mode), static_cast<int>(mode));
    }
    coolant->setCurrentIndex(
        std::max(0, coolant->findData(
            static_cast<int>(slot.coolantDefault))));
    cuttingForm->addRow(QStringLiteral("冷却"), coolant);

    auto* throughCoolant = new QCheckBox(
        QStringLiteral("刀具结构支持中心内冷"), drillingEditor);
    throughCoolant->setChecked(tool.throughCoolant);
    drillingForm->addRow(QStringLiteral("刀具能力"), throughCoolant);

    auto* peckDepth = makeLength(
        drillingEditor, slot.cuttingDefaults.peckDepth);
    auto* peckDepthLabel = new QLabel(
        QStringLiteral("每次钻深 Q"), drillingEditor);
    drillingForm->addRow(peckDepthLabel, peckDepth);

    auto* drillCycle = new QComboBox(drillingEditor);
    for (DrillCycle cycle : {DrillCycle::Auto, DrillCycle::G81Direct,
                             DrillCycle::G73Peck, DrillCycle::G83Peck}) {
        drillCycle->addItem(oap::tooling::drillCycleDisplayName(cycle),
                            static_cast<int>(cycle));
    }
    drillCycle->setCurrentIndex(
        std::max(0, drillCycle->findData(
            static_cast<int>(slot.cuttingDefaults.drillCycle))));
    auto* drillCycleLabel = new QLabel(
        QStringLiteral("钻孔循环"), drillingEditor);
    drillingForm->addRow(drillCycleLabel, drillCycle);

    auto* threadSize = new QLineEdit(tool.attributes.value(QStringLiteral("thread_size")),
                                     drillingEditor);
    auto* note = new QLineEdit(tool.note, drillingEditor);
    auto* threadSizeLabel = new QLabel(
        QStringLiteral("螺纹规格"), drillingEditor);
    drillingForm->addRow(threadSizeLabel, threadSize);
    drillingForm->addRow(QStringLiteral("备注"), note);

    const auto setRowVisible = [](QLabel* label, QWidget* field,
                                  bool visible) {
        label->setVisible(visible);
        field->setVisible(visible);
    };
    const auto selectedToolType = [type] {
        return static_cast<ToolType>(type->currentData().toInt());
    };
    const auto refreshToolTypeParameters = [=] {
        const ToolType selectedType = selectedToolType();
        const oap::tooling::ToolTypeParameterSchema schema =
            oap::tooling::toolTypeParameterSchema(selectedType);
        diameterLabel->setText(schema.diameterLabel);
        cuttingLengthLabel->setText(schema.cuttingEdgeLengthLabel);
        fluteCountLabel->setText(schema.fluteCountLabel);
        tipAngleLabel->setText(schema.tipAngleLabel);
        setRowVisible(cuttingLengthLabel, cuttingLength,
                      schema.cuttingEdgeLengthVisible);
        setRowVisible(fluteCountLabel, fluteCount,
                      schema.fluteCountVisible);
        setRowVisible(cornerRadiusLabel, cornerRadius,
                      schema.cornerRadiusVisible);
        setRowVisible(tipAngleLabel, tipGeometryWidget,
                      schema.tipAngleVisible);
        tipCompensationLabel->setVisible(schema.tipCompensationVisible);
        tipCompensation->setVisible(schema.tipCompensationVisible);
        setRowVisible(peckDepthLabel, peckDepth,
                      schema.drillingDefaultsVisible);
        setRowVisible(drillCycleLabel, drillCycle,
                      schema.drillingDefaultsVisible);
        setRowVisible(threadSizeLabel, threadSize,
                      schema.threadSpecificationVisible);

        if (schema.defaultTipAngle > 0.0 && tipAngle->value() <= 0.0)
            tipAngle->setValue(schema.defaultTipAngle);
        if (schema.cornerRadiusDerivedFromDiameter) {
            cornerRadius->setValue(diameter->value() * 0.5);
            cornerRadius->setEnabled(false);
            cornerRadius->setToolTip(
                QStringLiteral("球刀圆角由刀具直径自动计算。"));
        } else {
            cornerRadius->setEnabled(true);
            cornerRadius->setToolTip(QString());
        }

        QStringList activeParameters;
        activeParameters << schema.diameterLabel;
        if (schema.cuttingEdgeLengthVisible)
            activeParameters << schema.cuttingEdgeLengthLabel;
        if (schema.cornerRadiusVisible)
            activeParameters << (schema.cornerRadiusDerivedFromDiameter
                ? QStringLiteral("球半径（自动）")
                : QStringLiteral("刀尖圆角"));
        if (schema.tipAngleVisible)
            activeParameters << schema.tipAngleLabel;
        if (schema.fluteCountVisible)
            activeParameters << schema.fluteCountLabel;
        typeHelp->setText(QStringLiteral("当前刀型参数：%1。装配包络参数始终可编辑。")
                              .arg(activeParameters.join(QStringLiteral("、"))));
        tabs->setTabText(2,
            schema.drillingDefaultsVisible
                ? QStringLiteral("钻削与备注")
                : schema.threadSpecificationVisible
                    ? QStringLiteral("螺纹与备注")
                    : QStringLiteral("能力与备注"));
        refreshGeometryPreview();
    };
    QObject::connect(type, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     &dialog, [refreshToolTypeParameters](int) {
                         refreshToolTypeParameters();
                     });
    QObject::connect(
        diameter, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        &dialog, [=](double value) {
            const auto schema = oap::tooling::toolTypeParameterSchema(
                selectedToolType());
            if (schema.cornerRadiusDerivedFromDiameter)
                cornerRadius->setValue(value * 0.5);
        });
    refreshToolTypeParameters();

    if (!hasLibraryContext) {
        tabs->setTabEnabled(1, false);
        tabs->setTabToolTip(
            1, QStringLiteral("请进入具体刀库设置 T 刀号与切削默认值。"));
        peckDepth->setEnabled(false);
        drillCycle->setEnabled(false);
        peckDepth->setToolTip(
            QStringLiteral("钻孔循环参数属于刀库刀位。"));
        drillCycle->setToolTip(
            QStringLiteral("钻孔循环参数属于刀库刀位。"));
    }

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                         &dialog);
    dialogLayout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        const ToolType selectedType = static_cast<ToolType>(type->currentData().toInt());
        if (name->text().trimmed().isEmpty()) {
            QMessageBox::warning(&dialog, QStringLiteral("刀具"),
                                 QStringLiteral("刀具名称不能为空。"));
            return;
        }
        ToolGeometry candidateGeometry;
        candidateGeometry.diameter = diameter->value();
        candidateGeometry.cuttingEdgeLength = cuttingLength->value();
        candidateGeometry.overallLength = overallLength->value();
        candidateGeometry.shankDiameter = shankDiameter->value();
        candidateGeometry.holderDiameter = holderDiameter->value();
        candidateGeometry.holderLength = holderLength->value();
        candidateGeometry.spindleEnvelopeDiameter =
            spindleEnvelopeDiameter->value();
        candidateGeometry.spindleEnvelopeLength =
            spindleEnvelopeLength->value();
        candidateGeometry.cornerRadius = cornerRadius->value();
        candidateGeometry.tipAngle = tipAngle->value();
        candidateGeometry.tipCompensation = tipCompensation->value();
        candidateGeometry = oap::tooling::normalizedToolGeometry(
            selectedType, candidateGeometry);
        const QStringList geometryErrors =
            oap::tooling::toolGeometryValidationErrors(
                selectedType, candidateGeometry);
        if (!geometryErrors.isEmpty()) {
            QMessageBox::warning(&dialog, QStringLiteral("刀具"),
                                 geometryErrors.join(QLatin1Char('\n')));
            return;
        }
        dialog.accept();
    });
    if (dialog.exec() != QDialog::Accepted) return false;

    tool.name = name->text().trimmed();
    tool.type = static_cast<ToolType>(type->currentData().toInt());
    ToolGeometry editedGeometry;
    editedGeometry.diameter = diameter->value();
    editedGeometry.cuttingEdgeLength = cuttingLength->value();
    editedGeometry.overallLength = overallLength->value();
    editedGeometry.shankDiameter = shankDiameter->value();
    editedGeometry.holderDiameter = holderDiameter->value();
    editedGeometry.holderLength = holderLength->value();
    editedGeometry.spindleEnvelopeDiameter =
        spindleEnvelopeDiameter->value();
    editedGeometry.spindleEnvelopeLength =
        spindleEnvelopeLength->value();
    editedGeometry.cornerRadius = cornerRadius->value();
    editedGeometry.tipAngle = tipAngle->value();
    editedGeometry.tipCompensation = tipCompensation->value();
    tool.geometry = oap::tooling::normalizedToolGeometry(
        tool.type, editedGeometry);
    tool.note = note->text().trimmed();
    const oap::tooling::ToolTypeParameterSchema selectedSchema =
        oap::tooling::toolTypeParameterSchema(tool.type);
    if (!selectedSchema.threadSpecificationVisible ||
        threadSize->text().trimmed().isEmpty()) {
        tool.attributes.remove(QStringLiteral("thread_size"));
    } else {
        tool.attributes.insert(QStringLiteral("thread_size"), threadSize->text().trimmed());
    }
    if (!selectedSchema.fluteCountVisible || fluteCount->value() <= 0) {
        tool.attributes.remove(QStringLiteral("flutes"));
    } else {
        tool.attributes.insert(QStringLiteral("flutes"),
                               QString::number(fluteCount->value()));
    }

    if (hasLibraryContext) {
        assigned = toolNumber->value() > 0;
        slot.toolNumber = toolNumber->value();
        slot.toolId = tool.id;
        slot.cuttingDefaults.spindleSpeed = spindle->value();
        slot.cuttingDefaults.horizontalFeed = horizontalFeed->value();
        slot.cuttingDefaults.verticalFeed = verticalFeed->value();
        slot.cuttingDefaults.horizontalRapid = horizontalRapid->value();
        slot.cuttingDefaults.verticalRapid = verticalRapid->value();
        slot.cuttingDefaults.peckDepth =
            selectedSchema.drillingDefaultsVisible
                ? peckDepth->value() : 0.0;
        slot.cuttingDefaults.drillCycle =
            selectedSchema.drillingDefaultsVisible
                ? static_cast<DrillCycle>(
                      drillCycle->currentData().toInt())
                : DrillCycle::Auto;
        slot.coolantDefault =
            static_cast<CoolantMode>(coolant->currentData().toInt());
    } else {
        assigned = false;
    }
    tool.throughCoolant = throughCoolant->isChecked() ||
        (hasLibraryContext &&
         slot.coolantDefault == CoolantMode::ThroughTool);
    return true;
}

} // namespace

ToolLibraryDialog::ToolLibraryDialog(oap::tooling::ToolAssetCatalog catalog,
                                     const QString& activeLibraryId,
                                     QWidget* parent)
    : QDialog(parent),
      m_catalog(std::move(catalog))
{
    // 主对话框只维护内存副本；是否保存目录由调用方在 accept 后统一决定。
    setWindowTitle(QStringLiteral("刀库管理"));
    setMinimumSize(760, 480);
    resize(1180, 720);
    auto* layout = new QVBoxLayout(this);

    // 采用 FreeCAD Library Manager 的双栏结构：左侧管理刀库，右侧浏览
    // 当前刀库内容。QSplitter 允许用户按工作内容调整两栏宽度。
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    auto* libraryPane = new QWidget(splitter);
    auto* libraryLayout = new QVBoxLayout(libraryPane);
    libraryLayout->setContentsMargins(0, 0, 0, 0);
    auto* libraryTitle = new QLabel(QStringLiteral("刀库"), libraryPane);
    QFont libraryTitleFont = libraryTitle->font();
    libraryTitleFont.setBold(true);
    libraryTitle->setFont(libraryTitleFont);
    libraryLayout->addWidget(libraryTitle);
    m_libraryList = new QListWidget(libraryPane);
    m_libraryList->setSelectionMode(QAbstractItemView::SingleSelection);
    libraryLayout->addWidget(m_libraryList, 1);

    auto* libraryActionRow = new QHBoxLayout;
    auto* newLibrary = new QPushButton(QStringLiteral("新建"), libraryPane);
    auto* renameLibraryButton = new QPushButton(QStringLiteral("重命名"), libraryPane);
    auto* deleteLibraryButton = new QPushButton(QStringLiteral("删除"), libraryPane);
    libraryActionRow->addWidget(newLibrary);
    libraryActionRow->addWidget(renameLibraryButton);
    libraryActionRow->addWidget(deleteLibraryButton);
    libraryLayout->addLayout(libraryActionRow);

    auto* transferRow = new QHBoxLayout;
    auto* importButton = new QPushButton(QStringLiteral("导入刀库"), libraryPane);
    auto* exportButton = new QPushButton(QStringLiteral("导出刀库"), libraryPane);
    transferRow->addWidget(importButton);
    transferRow->addWidget(exportButton);
    libraryLayout->addLayout(transferRow);

    m_description = new QLabel(libraryPane);
    m_description->setWordWrap(true);
    m_description->setTextInteractionFlags(Qt::TextSelectableByMouse);
    libraryLayout->addWidget(m_description);

    auto* toolPane = new QWidget(splitter);
    auto* toolLayout = new QVBoxLayout(toolPane);
    toolLayout->setContentsMargins(0, 0, 0, 0);

    auto* toolActionRow = new QHBoxLayout;
    auto* addToolButton = new QPushButton(QStringLiteral("新增刀具"), toolPane);
    m_duplicateToolButton = new QPushButton(QStringLiteral("复制刀具"), toolPane);
    m_copyToLibraryButton =
        new QPushButton(QStringLiteral("复制到刀库…"), toolPane);
    m_moveToLibraryButton =
        new QPushButton(QStringLiteral("移动到刀库…"), toolPane);
    m_editButton = new QPushButton(QStringLiteral("编辑刀具"), toolPane);
    m_removeToolButton = new QPushButton(QStringLiteral("删除刀具"), toolPane);
    toolActionRow->addWidget(addToolButton);
    toolActionRow->addWidget(m_duplicateToolButton);
    toolActionRow->addWidget(m_copyToLibraryButton);
    toolActionRow->addWidget(m_moveToLibraryButton);
    toolActionRow->addWidget(m_editButton);
    toolActionRow->addWidget(m_removeToolButton);
    toolActionRow->addStretch(1);
    toolLayout->addLayout(toolActionRow);

    auto* filterRow = new QHBoxLayout;
    filterRow->addWidget(new QLabel(QStringLiteral("搜索"), toolPane));
    m_filter = new QLineEdit(toolPane);
    m_filter->setPlaceholderText(QStringLiteral("刀号、名称、类型、规格或备注"));
    filterRow->addWidget(m_filter, 1);
    m_typeFilter = new QComboBox(toolPane);
    m_typeFilter->addItem(QStringLiteral("全部类型"), -1);
    fillToolTypes(m_typeFilter);
    filterRow->addWidget(m_typeFilter);
    m_sortMode = new QComboBox(toolPane);
    m_sortMode->addItem(QStringLiteral("按刀号排序"), 0);
    m_sortMode->addItem(QStringLiteral("按名称排序"), 1);
    m_sortMode->addItem(QStringLiteral("按类型排序"), 2);
    m_sortMode->addItem(QStringLiteral("按直径排序"), 3);
    filterRow->addWidget(m_sortMode);
    toolLayout->addLayout(filterRow);

    m_table = new QTableWidget(toolPane);
    m_table->setColumnCount(11);
    m_table->setHorizontalHeaderLabels({
        QStringLiteral("刀号"), QStringLiteral("名称"), QStringLiteral("类型"),
        QStringLiteral("直径/mm"), QStringLiteral("刃长/mm"),
        QStringLiteral("转速/rpm"), QStringLiteral("水平进给"),
        QStringLiteral("垂直进给"), QStringLiteral("冷却"), QStringLiteral("备注"),
        QStringLiteral("所属刀库")});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(9, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(10, QHeaderView::Stretch);
    m_table->setContextMenuPolicy(Qt::ActionsContextMenu);
    toolLayout->addWidget(m_table, 1);

    m_editToolAction = new QAction(QStringLiteral("编辑刀具"), m_table);
    m_editToolAction->setShortcut(QKeySequence(Qt::Key_F2));
    m_editToolAction->setShortcutContext(Qt::WidgetShortcut);
    m_table->addAction(m_editToolAction);
    m_duplicateToolAction = new QAction(QStringLiteral("复制刀具"), m_table);
    m_duplicateToolAction->setShortcut(QKeySequence::Copy);
    m_duplicateToolAction->setShortcutContext(Qt::WidgetShortcut);
    m_table->addAction(m_duplicateToolAction);
    m_copyToLibraryAction =
        new QAction(QStringLiteral("复制到其他刀库…"), m_table);
    m_table->addAction(m_copyToLibraryAction);
    m_moveToLibraryAction =
        new QAction(QStringLiteral("移动到其他刀库…"), m_table);
    m_table->addAction(m_moveToLibraryAction);
    m_removeToolAction = new QAction(QStringLiteral("删除刀具"), m_table);
    m_removeToolAction->setShortcut(QKeySequence::Delete);
    m_removeToolAction->setShortcutContext(Qt::WidgetShortcut);
    m_table->addAction(m_removeToolAction);

    m_libraryList->setContextMenuPolicy(Qt::ActionsContextMenu);
    auto* renameLibraryAction =
        new QAction(QStringLiteral("重命名刀库"), m_libraryList);
    renameLibraryAction->setShortcut(QKeySequence(Qt::Key_F2));
    renameLibraryAction->setShortcutContext(Qt::WidgetShortcut);
    m_libraryList->addAction(renameLibraryAction);
    auto* deleteLibraryAction =
        new QAction(QStringLiteral("删除刀库"), m_libraryList);
    deleteLibraryAction->setShortcut(QKeySequence::Delete);
    deleteLibraryAction->setShortcutContext(Qt::WidgetShortcut);
    m_libraryList->addAction(deleteLibraryAction);

    splitter->addWidget(libraryPane);
    splitter->addWidget(toolPane);
    splitter->setCollapsible(0, false);
    splitter->setCollapsible(1, false);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({250, 930});
    layout->addWidget(splitter, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                         this);
    m_useButton = buttons->button(QDialogButtonBox::Ok);
    m_useButton->setText(QStringLiteral("保存更改"));
    layout->addWidget(buttons);

    connect(m_libraryList, &QListWidget::currentRowChanged,
            this, [this] { refreshTable(); });
    connect(m_filter, &QLineEdit::textChanged, this, [this] { refreshTable(); });
    connect(m_typeFilter, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this] { refreshTable(); });
    connect(m_sortMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this] { refreshTable(); });
    connect(m_table, &QTableWidget::itemSelectionChanged,
            this, [this] { updateSelectionState(); });
    connect(m_table, &QTableWidget::cellDoubleClicked, this, [this](int, int) {
        editTool();
    });
    connect(newLibrary, &QPushButton::clicked, this, [this] { createLibrary(); });
    connect(renameLibraryButton, &QPushButton::clicked,
            this, [this] { renameLibrary(); });
    connect(deleteLibraryButton, &QPushButton::clicked, this, [this] { deleteLibrary(); });
    connect(importButton, &QPushButton::clicked, this, [this] { importLibrary(); });
    connect(exportButton, &QPushButton::clicked, this, [this] { exportLibrary(); });
    connect(addToolButton, &QPushButton::clicked, this, [this] { addTool(); });
    connect(m_duplicateToolButton, &QPushButton::clicked,
            this, [this] { duplicateTool(); });
    connect(m_copyToLibraryButton, &QPushButton::clicked,
            this, [this] { copyToolToLibrary(); });
    connect(m_moveToLibraryButton, &QPushButton::clicked,
            this, [this] { moveToolToLibrary(); });
    connect(m_editButton, &QPushButton::clicked, this, [this] { editTool(); });
    connect(m_removeToolButton, &QPushButton::clicked, this, [this] { removeTool(); });
    connect(m_editToolAction, &QAction::triggered,
            this, [this] { editTool(); });
    connect(m_duplicateToolAction, &QAction::triggered,
            this, [this] { duplicateTool(); });
    connect(m_copyToLibraryAction, &QAction::triggered,
            this, [this] { copyToolToLibrary(); });
    connect(m_moveToLibraryAction, &QAction::triggered,
            this, [this] { moveToolToLibrary(); });
    connect(m_removeToolAction, &QAction::triggered,
            this, [this] { removeTool(); });
    connect(renameLibraryAction, &QAction::triggered,
            this, [this] { renameLibrary(); });
    connect(deleteLibraryAction, &QAction::triggered,
            this, [this] { deleteLibrary(); });
    connect(buttons, &QDialogButtonBox::accepted, this, &ToolLibraryDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    refreshLibraryList(activeLibraryId);
}

ToolLibrary* ToolLibraryDialog::currentLibrary()
{
    const QListWidgetItem* selected = m_libraryList->currentItem();
    const QString id = selected ? selected->data(Qt::UserRole).toString()
                                : QString();
    for (ToolLibrary& library : editableLibraries())
        if (library.id() == id) return &library;
    return nullptr;
}

const ToolLibrary* ToolLibraryDialog::currentLibrary() const
{
    const QListWidgetItem* selected = m_libraryList->currentItem();
    const QString id = selected ? selected->data(Qt::UserRole).toString()
                                : QString();
    for (const ToolLibrary& library : m_catalog.libraries())
        if (library.id() == id) return &library;
    return nullptr;
}

QString ToolLibraryDialog::selectedToolId() const
{
    const QStringList ids = selectedToolIds();
    return ids.isEmpty() ? QString() : ids.front();
}

QStringList ToolLibraryDialog::selectedToolIds() const
{
    QStringList ids;
    if (!m_table->selectionModel())
        return ids;
    const QModelIndexList rows =
        m_table->selectionModel()->selectedRows(0);
    ids.reserve(rows.size());
    for (const QModelIndex& index : rows) {
        const QTableWidgetItem* item = m_table->item(index.row(), 0);
        if (item) {
            const QString id = item->data(kToolIdRole).toString();
            if (!id.isEmpty() && !ids.contains(id))
                ids.push_back(id);
        }
    }
    return ids;
}

QString ToolLibraryDialog::selectedLibraryId() const
{
    const ToolLibrary* library = currentLibrary();
    return library ? library->id() : QString();
}

void ToolLibraryDialog::refreshLibraryList(const QString& preferredId)
{
    QString target = preferredId;
    if (target.isEmpty() && m_libraryList->currentItem()) {
        target = m_libraryList->currentItem()
                     ->data(Qt::UserRole).toString();
    }
    const QSignalBlocker signalBlocker(m_libraryList);
    m_libraryList->clear();
    auto* allTools = new QListWidgetItem(
        QStringLiteral("全部刀具\n跨刀库资产视图"), m_libraryList);
    allTools->setData(Qt::UserRole, kAllToolsLibraryId);
    QFont allToolsFont = allTools->font();
    allToolsFont.setBold(true);
    allTools->setFont(allToolsFont);
    int selectedRow = -1;
    for (const ToolLibrary& library : m_catalog.libraries()) {
        auto* item = new QListWidgetItem(
            library.machine().isEmpty()
                ? library.name()
                : QStringLiteral("%1\n%2").arg(library.name(), library.machine()),
            m_libraryList);
        item->setData(Qt::UserRole, library.id());
        item->setToolTip(QStringLiteral("ID: %1").arg(library.id()));
        if (library.id() == target)
            selectedRow = m_libraryList->count() - 1;
    }
    if (selectedRow < 0 && m_libraryList->count() > 0)
        selectedRow = 0;
    m_libraryList->setCurrentRow(selectedRow);
    refreshTable();
}

void ToolLibraryDialog::refreshTable(const QString& preferredToolId)
{
    // 稳定 ID 存在 UserRole 中而非可见文本；排序和类型筛选由浏览器工具栏控制。
    QString target = preferredToolId.isEmpty() ? selectedToolId() : preferredToolId;
    m_table->setRowCount(0);
    const QListWidgetItem* libraryItem = m_libraryList->currentItem();
    if (libraryItem &&
        libraryItem->data(Qt::UserRole).toString() ==
            kAllToolsLibraryId) {
        refreshAllToolsTable(target);
        return;
    }
    const ToolLibrary* library = currentLibrary();
    if (!library) {
        m_description->clear();
        updateSelectionState();
        return;
    }
    const int assignedCount = library->assignedTools().size();
    const int unassignedCount = library->tools().size() - assignedCount;
    m_description->setText(
        library->description() +
        QStringLiteral("\n当前刀库已分配 T 刀具 %1 把，未分配备用刀具 %2 把。"
                       "启用后只把前者作为可用刀具集合，暂不自动匹配孔。")
            .arg(assignedCount).arg(unassignedCount));

    QVector<QString> ids;
    ids.reserve(library->tools().size());
    for (auto it = library->tools().cbegin(); it != library->tools().cend(); ++it)
        ids.push_back(it.key());
    const int sortMode = m_sortMode ? m_sortMode->currentData().toInt() : 0;
    std::sort(ids.begin(), ids.end(), [&](const QString& left, const QString& right) {
        const ToolBit* leftTool = library->toolById(left);
        const ToolBit* rightTool = library->toolById(right);
        const int leftNo = library->toolNumberFor(left);
        const int rightNo = library->toolNumberFor(right);
        const int leftKey = leftNo > 0 ? leftNo : std::numeric_limits<int>::max();
        const int rightKey = rightNo > 0 ? rightNo : std::numeric_limits<int>::max();
        if (sortMode == 1) {
            const int nameOrder = leftTool->name.localeAwareCompare(rightTool->name);
            return nameOrder == 0 ? left < right : nameOrder < 0;
        }
        if (sortMode == 2) {
            const QString leftType =
                oap::tooling::toolTypeDisplayName(leftTool->type);
            const QString rightType =
                oap::tooling::toolTypeDisplayName(rightTool->type);
            const int typeOrder = leftType.localeAwareCompare(rightType);
            if (typeOrder != 0)
                return typeOrder < 0;
            const int nameOrder = leftTool->name.localeAwareCompare(rightTool->name);
            return nameOrder == 0 ? left < right : nameOrder < 0;
        }
        if (sortMode == 3 &&
            leftTool->geometry.diameter != rightTool->geometry.diameter) {
            return leftTool->geometry.diameter < rightTool->geometry.diameter;
        }
        if (leftKey != rightKey)
            return leftKey < rightKey;
        const int nameOrder = leftTool->name.localeAwareCompare(rightTool->name);
        return nameOrder == 0 ? left < right : nameOrder < 0;
    });

    const QString filter = m_filter->text().trimmed();
    const int typeFilter = m_typeFilter
        ? m_typeFilter->currentData().toInt() : -1;
    int selectedRow = -1;
    for (const QString& id : ids) {
        const ToolBit* tool = library->toolById(id);
        if (typeFilter >= 0 &&
            static_cast<int>(tool->type) != typeFilter) {
            continue;
        }
        const LibraryToolSlot* slot = library->slotForTool(id);
        const int number = slot ? slot->toolNumber : 0;
        const QString haystack = QStringLiteral("%1 %2 %3 %4 %5")
            .arg(number).arg(tool->name, oap::tooling::toolTypeDisplayName(tool->type),
                             tool->note, tool->attributes.value(QStringLiteral("thread_size")));
        if (!filter.isEmpty() && !haystack.contains(filter, Qt::CaseInsensitive))
            continue;

        const int row = m_table->rowCount();
        m_table->insertRow(row);
        auto* numberItem = tableItem(number > 0 ? QStringLiteral("T%1").arg(number)
                                                : QStringLiteral("未分配"));
        numberItem->setData(kToolIdRole, id);
        numberItem->setData(kToolNumberRole, number);
        m_table->setItem(row, 0, numberItem);
        m_table->setItem(row, 1, tableItem(tool->name));
        m_table->setItem(row, 2, tableItem(oap::tooling::toolTypeDisplayName(tool->type)));
        m_table->setItem(row, 3, tableItem(numberText(tool->geometry.diameter)));
        m_table->setItem(row, 4, tableItem(numberText(tool->geometry.cuttingEdgeLength)));
        m_table->setItem(row, 5, tableItem(slot
            ? numberText(slot->cuttingDefaults.spindleSpeed, 0) : QStringLiteral("—")));
        m_table->setItem(row, 6, tableItem(slot
            ? numberText(slot->cuttingDefaults.horizontalFeed, 1) : QStringLiteral("—")));
        m_table->setItem(row, 7, tableItem(slot
            ? numberText(slot->cuttingDefaults.verticalFeed, 1) : QStringLiteral("—")));
        m_table->setItem(row, 8, tableItem(slot
            ? oap::tooling::coolantModeDisplayName(slot->coolantDefault)
            : QStringLiteral("—")));
        m_table->setItem(row, 9, tableItem(tool->note));
        m_table->setItem(row, 10, tableItem(library->name()));
        if (id == target) {
            selectedRow = row;
        }
    }
    if (selectedRow >= 0)
        m_table->selectRow(selectedRow);
    updateSelectionState();
}

void ToolLibraryDialog::refreshAllToolsTable(
    const QString& preferredToolId)
{
    const QMap<QString, ToolBit>& assets = m_catalog.assets();

    m_description->setText(QStringLiteral(
        "全局刀具资产库，共 %1 个独立 ToolBit。刀库仅引用这些资产并维护 T 刀号与切削默认值；"
        "未被刀库引用的资产也会在此保留。")
        .arg(assets.size()));
    QVector<QString> ids;
    ids.reserve(assets.size());
    for (auto it = assets.cbegin(); it != assets.cend(); ++it)
        ids.push_back(it.key());
    const int sortMode = m_sortMode ? m_sortMode->currentData().toInt() : 1;
    std::sort(ids.begin(), ids.end(), [&](const QString& left,
                                          const QString& right) {
        const ToolBit* leftTool = m_catalog.assetById(left);
        const ToolBit* rightTool = m_catalog.assetById(right);
        if (sortMode == 2) {
            const int typeOrder = oap::tooling::toolTypeDisplayName(leftTool->type)
                .localeAwareCompare(
                    oap::tooling::toolTypeDisplayName(rightTool->type));
            if (typeOrder != 0)
                return typeOrder < 0;
        } else if (sortMode == 3 &&
                   leftTool->geometry.diameter !=
                       rightTool->geometry.diameter) {
            return leftTool->geometry.diameter <
                   rightTool->geometry.diameter;
        }
        const int nameOrder = leftTool->name.localeAwareCompare(rightTool->name);
        return nameOrder == 0 ? left < right : nameOrder < 0;
    });

    const QString filter = m_filter->text().trimmed();
    const int typeFilter = m_typeFilter
        ? m_typeFilter->currentData().toInt() : -1;
    int selectedRow = -1;
    for (const QString& id : ids) {
        const ToolBit* tool = m_catalog.assetById(id);
        const QStringList assetLibraries =
            m_catalog.referencingLibraryNames(id);
        if (!tool || (typeFilter >= 0 &&
            static_cast<int>(tool->type) != typeFilter)) {
            continue;
        }
        const QString haystack = QStringLiteral("%1 %2 %3 %4 %5")
            .arg(tool->name,
                 oap::tooling::toolTypeDisplayName(tool->type),
                 tool->note,
                 tool->attributes.value(QStringLiteral("thread_size")),
                 assetLibraries.join(QLatin1Char(' ')));
        if (!filter.isEmpty() &&
            !haystack.contains(filter, Qt::CaseInsensitive)) {
            continue;
        }
        const int row = m_table->rowCount();
        m_table->insertRow(row);
        auto* idItem = tableItem(QStringLiteral("—"));
        idItem->setData(kToolIdRole, id);
        idItem->setData(kToolNumberRole, 0);
        m_table->setItem(row, 0, idItem);
        m_table->setItem(row, 1, tableItem(tool->name));
        m_table->setItem(row, 2, tableItem(
            oap::tooling::toolTypeDisplayName(tool->type)));
        m_table->setItem(row, 3, tableItem(numberText(tool->geometry.diameter)));
        m_table->setItem(row, 4, tableItem(
            numberText(tool->geometry.cuttingEdgeLength)));
        for (int column = 5; column <= 8; ++column)
            m_table->setItem(row, column, tableItem(QStringLiteral("—")));
        m_table->setItem(row, 9, tableItem(tool->note));
        m_table->setItem(row, 10, tableItem(
            assetLibraries.isEmpty()
                ? QStringLiteral("未加入刀库")
                : assetLibraries.join(QStringLiteral("、"))));
        if (id == preferredToolId)
            selectedRow = row;
    }
    if (selectedRow >= 0)
        m_table->selectRow(selectedRow);
    updateSelectionState();
}

void ToolLibraryDialog::updateSelectionState()
{
    const ToolLibrary* library = currentLibrary();
    const QListWidgetItem* libraryItem = m_libraryList->currentItem();
    const bool globalAssetsSelected = libraryItem &&
        libraryItem->data(Qt::UserRole).toString() == kAllToolsLibraryId;
    const QStringList ids = selectedToolIds();
    const bool hasTools = (library || globalAssetsSelected) && !ids.isEmpty();
    const bool hasSingleTool = hasTools && ids.size() == 1 &&
        (library ? library->toolById(ids.front())
                 : m_catalog.assetById(ids.front()));
    const bool canCopy = hasTools &&
        (globalAssetsSelected ? !m_catalog.libraries().isEmpty()
                              : m_catalog.libraries().size() > 1);
    m_copyToLibraryButton->setText(
        globalAssetsSelected ? QStringLiteral("加入刀库…")
                             : QStringLiteral("加入其他刀库…"));
    m_moveToLibraryButton->setText(QStringLiteral("移至其他刀库…"));
    m_removeToolButton->setText(
        globalAssetsSelected ? QStringLiteral("删除资产")
                             : QStringLiteral("从刀库移除"));
    m_copyToLibraryAction->setText(m_copyToLibraryButton->text());
    m_moveToLibraryAction->setText(m_moveToLibraryButton->text());
    m_removeToolAction->setText(m_removeToolButton->text());
    m_duplicateToolButton->setEnabled(hasSingleTool);
    m_copyToLibraryButton->setEnabled(canCopy);
    m_moveToLibraryButton->setEnabled(
        hasTools && !globalAssetsSelected &&
        m_catalog.libraries().size() > 1);
    m_editButton->setEnabled(hasSingleTool);
    m_removeToolButton->setEnabled(hasTools);
    m_editToolAction->setEnabled(hasSingleTool);
    m_duplicateToolAction->setEnabled(hasSingleTool);
    m_copyToLibraryAction->setEnabled(canCopy);
    m_moveToLibraryAction->setEnabled(
        hasTools && !globalAssetsSelected &&
        m_catalog.libraries().size() > 1);
    m_removeToolAction->setEnabled(hasTools);
    m_useButton->setEnabled(!m_catalog.libraries().isEmpty());
}

void ToolLibraryDialog::createLibrary()
{
    bool ok = false;
    const QString name = QInputDialog::getText(
        this, QStringLiteral("新建刀库"), QStringLiteral("刀库名称"),
        QLineEdit::Normal, QStringLiteral("新刀库"), &ok).trimmed();
    if (!ok || name.isEmpty()) return;
    const QString id = QUuid::createUuid().toString(QUuid::WithoutBraces);
    QString error;
    if (!m_catalog.addLibrary(ToolLibrary(id, name), &error)) {
        QMessageBox::critical(this, QStringLiteral("新建刀库"), error);
        return;
    }
    refreshLibraryList(id);
}

void ToolLibraryDialog::renameLibrary()
{
    ToolLibrary* library = currentLibrary();
    if (!library)
        return;
    bool ok = false;
    const QString name = QInputDialog::getText(
        this, QStringLiteral("重命名刀库"), QStringLiteral("刀库名称"),
        QLineEdit::Normal, library->name(), &ok).trimmed();
    if (!ok || name.isEmpty() || name == library->name())
        return;
    const QString id = library->id();
    library->setName(name);
    refreshLibraryList(id);
}

void ToolLibraryDialog::deleteLibrary()
{
    const ToolLibrary* library = currentLibrary();
    if (!library) return;
    if (m_catalog.libraries().size() <= 1) {
        QMessageBox::information(
            this, QStringLiteral("删除刀库"),
            QStringLiteral("当前项目至少需要保留一个刀库。"));
        return;
    }
    if (QMessageBox::question(this, QStringLiteral("删除刀库"),
                              QStringLiteral("确定删除刀库“%1”吗？").arg(library->name()))
        != QMessageBox::Yes) return;
    const QString id = library->id();
    QString error;
    if (!m_catalog.removeLibrary(id, &error)) {
        QMessageBox::critical(this, QStringLiteral("删除刀库"), error);
        return;
    }
    refreshLibraryList();
}

void ToolLibraryDialog::importLibrary()
{
    // 同 ID 导入采用显式替换，避免目录中出现两个逻辑上相同的刀库。
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("导入刀库"), QString(),
        QStringLiteral("刀库文件 (*.oaptl *.fftl *.fctl);;所有文件 (*.*)"));
    if (path.isEmpty()) return;
    ToolLibrary imported;
    QString error;
    if (!oap::tooling::ToolLibraryStorage::loadLibrary(path, imported, &error)) {
        QMessageBox::critical(this, QStringLiteral("导入刀库"), error);
        return;
    }
    if (m_catalog.libraryById(imported.id())) {
        if (QMessageBox::question(this, QStringLiteral("导入刀库"),
                                  QStringLiteral("同 ID 刀库已存在，是否替换？"))
            != QMessageBox::Yes) return;
        if (!m_catalog.replaceLibrary(imported, &error)) {
            QMessageBox::critical(this, QStringLiteral("导入刀库"), error);
            return;
        }
        refreshLibraryList(imported.id());
        return;
    }
    if (!m_catalog.addLibrary(imported, &error)) {
        QMessageBox::critical(this, QStringLiteral("导入刀库"), error);
        return;
    }
    refreshLibraryList(imported.id());
}

void ToolLibraryDialog::exportLibrary()
{
    const ToolLibrary* library = currentLibrary();
    if (!library) return;
    QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("导出刀库"), library->name() + QStringLiteral(".oaptl"),
        QStringLiteral("OpenAutoPath 刀库 (*.oaptl);;JSON 文件 (*.json)"));
    if (path.isEmpty()) return;
    if (QFileInfo(path).suffix().isEmpty()) path += QStringLiteral(".oaptl");
    QString error;
    if (!oap::tooling::ToolLibraryStorage::saveLibrary(path, *library, &error))
        QMessageBox::critical(this, QStringLiteral("导出刀库"), error);
}

void ToolLibraryDialog::addTool()
{
    ToolLibrary* library = currentLibrary();
    const QString libraryId = library ? library->id() : QString();
    ToolBit tool;
    tool.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
    tool.name = QStringLiteral("新刀具");
    tool.type = ToolType::Drill;
    tool.geometry.diameter = 6.0;
    LibraryToolSlot slot;
    slot.toolId = tool.id;
    slot.toolNumber = library ? library->nextAvailableToolNumber() : 0;
    bool assigned = library != nullptr;
    if (!editToolValues(this, tool, slot, assigned, true,
                        library != nullptr)) return;
    QString error;
    if (!m_catalog.upsertAsset(tool, &error) ||
        (!libraryId.isEmpty() &&
         !m_catalog.attachAsset(libraryId, tool.id, &error))) {
        QMessageBox::critical(this, QStringLiteral("新增刀具"), error);
        return;
    }
    library = m_catalog.libraryById(libraryId);
    if (library && assigned && !library->assignToolToSlot(slot, &error)) {
        m_catalog.detachAsset(libraryId, tool.id);
        QMessageBox::critical(this, QStringLiteral("新增刀具"), error);
        return;
    }
    refreshTable(tool.id);
}

void ToolLibraryDialog::duplicateTool()
{
    ToolLibrary* library = currentLibrary();
    const QString libraryId = library ? library->id() : QString();
    const QString sourceId = selectedToolId();
    const ToolBit* source = library ? library->toolById(sourceId)
                                    : m_catalog.assetById(sourceId);
    if (!source)
        return;

    ToolBit duplicate = *source;
    duplicate.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
    duplicate.name += QStringLiteral("（副本）");
    QString error;
    if (!m_catalog.upsertAsset(duplicate, &error) ||
        (!libraryId.isEmpty() && !m_catalog.attachAsset(
             libraryId, duplicate.id, &error))) {
        QMessageBox::critical(this, QStringLiteral("复制刀具"), error);
        return;
    }

    library = m_catalog.libraryById(libraryId);
    if (library) {
        if (const LibraryToolSlot* sourceSlot =
            library->slotForTool(sourceId)) {
            LibraryToolSlot duplicateSlot = *sourceSlot;
            duplicateSlot.toolId = duplicate.id;
            duplicateSlot.toolNumber = library->nextAvailableToolNumber();
            if (!library->assignToolToSlot(duplicateSlot, &error)) {
                library->removeTool(duplicate.id);
                m_catalog.removeAsset(duplicate.id);
                QMessageBox::critical(this, QStringLiteral("复制刀具"), error);
                return;
            }
        }
    }
    refreshTable(duplicate.id);
}

void ToolLibraryDialog::copyToolToLibrary()
{
    transferToolToLibrary(false);
}

void ToolLibraryDialog::moveToolToLibrary()
{
    transferToolToLibrary(true);
}

void ToolLibraryDialog::transferToolToLibrary(bool removeSource)
{
    ToolLibrary* sourceLibrary = currentLibrary();
    const QStringList toolIds = selectedToolIds();
    const QListWidgetItem* selectedLibraryItem = m_libraryList->currentItem();
    const bool fromGlobalAssets = !sourceLibrary && selectedLibraryItem &&
        selectedLibraryItem->data(Qt::UserRole).toString() ==
            kAllToolsLibraryId;
    if ((!sourceLibrary && !fromGlobalAssets) || toolIds.isEmpty() ||
        (!fromGlobalAssets && m_catalog.libraries().size() <= 1) ||
        (fromGlobalAssets && m_catalog.libraries().isEmpty()))
        return;
    if (fromGlobalAssets && removeSource)
        return;
    const QString sourceLibraryId = sourceLibrary ? sourceLibrary->id()
                                                  : QString();

    QStringList labels;
    QVector<QString> targetIds;
    for (const ToolLibrary& library : m_catalog.libraries()) {
        if (!sourceLibraryId.isEmpty() && library.id() == sourceLibraryId)
            continue;
        const QString displayName = library.machine().isEmpty()
            ? library.name()
            : QStringLiteral("%1（%2）")
                  .arg(library.name(), library.machine());
        labels.push_back(
            QStringLiteral("%1 · %2")
                .arg(displayName, library.id().left(8)));
        targetIds.push_back(library.id());
    }
    bool ok = false;
    const QString selectedLabel = QInputDialog::getItem(
        this,
        removeSource ? QStringLiteral("移动刀具")
                     : QStringLiteral("复制刀具"),
        QStringLiteral("目标刀库"), labels, 0, false, &ok);
    if (!ok || selectedLabel.isEmpty())
        return;
    const int selectedIndex = labels.indexOf(selectedLabel);
    if (selectedIndex < 0 || selectedIndex >= targetIds.size())
        return;

    const QString targetLibraryId = targetIds.at(selectedIndex);
    const ToolLibrary* existingTarget = nullptr;
    for (const ToolLibrary& library : m_catalog.libraries()) {
        if (library.id() == targetLibraryId) {
            existingTarget = &library;
            break;
        }
    }
    if (!existingTarget)
        return;

    if (fromGlobalAssets) {
        QString error;
        for (const QString& toolId : toolIds) {
            if (!m_catalog.attachAsset(targetLibraryId, toolId, &error)) {
                QMessageBox::critical(
                    this, QStringLiteral("加入刀库"), error);
                return;
            }
        }
        refreshLibraryList(kAllToolsLibraryId);
        if (!toolIds.isEmpty())
            refreshTable(toolIds.front());
        return;
    }

    QStringList conflicts;
    for (const QString& toolId : toolIds) {
        if (existingTarget->toolById(toolId)) {
            const ToolBit* tool = sourceLibrary->toolById(toolId);
            conflicts.push_back(tool ? tool->name : toolId);
        }
    }
    if (!conflicts.isEmpty() &&
        QMessageBox::question(
            this, QStringLiteral("目标刀库已有刀具"),
            QStringLiteral(
                "目标刀库已包含以下同 ID 刀具：\n%1\n"
                "是否用当前定义更新它们？目标刀号保持不变。")
                .arg(conflicts.join(QStringLiteral("、"))))
            != QMessageBox::Yes) {
        return;
    }

    QVector<ToolLibrary> candidate = m_catalog.libraries();
    ToolLibrary* candidateSource = nullptr;
    ToolLibrary* candidateTarget = nullptr;
    for (ToolLibrary& library : candidate) {
        if (library.id() == sourceLibraryId)
            candidateSource = &library;
        if (library.id() == targetLibraryId)
            candidateTarget = &library;
    }
    if (!candidateSource || !candidateTarget)
        return;

    QString error;
    for (const QString& toolId : toolIds) {
        const ToolBit* sourceTool = candidateSource->toolById(toolId);
        if (!sourceTool)
            continue;
        const ToolBit toolCopy = *sourceTool;
        const LibraryToolSlot* sourceSlot =
            candidateSource->slotForTool(toolId);
        const std::optional<LibraryToolSlot> slotCopy = sourceSlot
            ? std::optional<LibraryToolSlot>(*sourceSlot)
            : std::nullopt;
        if (!candidateTarget->upsertTool(toolCopy, &error)) {
            QMessageBox::critical(this, QStringLiteral("跨刀库操作"), error);
            return;
        }
        if (!candidateTarget->slotForTool(toolId) && slotCopy) {
            LibraryToolSlot targetSlot = *slotCopy;
            targetSlot.toolNumber = candidateTarget->nextAvailableToolNumber();
            if (!candidateTarget->assignToolToSlot(targetSlot, &error)) {
                QMessageBox::critical(
                    this, QStringLiteral("跨刀库操作"), error);
                return;
            }
        }
        if (removeSource &&
            !candidateSource->removeTool(toolId, &error)) {
            QMessageBox::critical(this, QStringLiteral("移动刀具"), error);
            return;
        }
    }

    editableLibraries() = std::move(candidate);
    refreshLibraryList(sourceLibraryId);
    if (!removeSource && !toolIds.isEmpty())
        refreshTable(toolIds.front());
}

void ToolLibraryDialog::editTool()
{
    ToolLibrary* library = currentLibrary();
    const QString libraryId = library ? library->id() : QString();
    const QStringList selectedIds = selectedToolIds();
    if (selectedIds.size() != 1)
        return;
    const QString id = selectedIds.front();
    const ToolBit* stored = library ? library->toolById(id)
                                    : m_catalog.assetById(id);
    if (!stored) return;
    // 先编辑刀具和刀位默认值副本；更新时重建刀号分配以复用冲突校验。
    ToolBit tool = *stored;
    const LibraryToolSlot* existing = library
        ? library->slotForTool(id) : nullptr;
    LibraryToolSlot slot = existing ? *existing : LibraryToolSlot{};
    slot.toolId = id;
    bool assigned = existing != nullptr;
    if (!editToolValues(this, tool, slot, assigned, false,
                        library != nullptr)) return;
    QString error;
    if (!m_catalog.upsertAsset(tool, &error)) {
        QMessageBox::critical(this, QStringLiteral("编辑刀具"), error);
        return;
    }
    library = m_catalog.libraryById(libraryId);
    if (library)
        library->unassignToolSlot(id);
    if (library && assigned && !library->assignToolToSlot(slot, &error)) {
        QMessageBox::critical(this, QStringLiteral("编辑刀具"), error);
        return;
    }
    refreshTable(id);
}

void ToolLibraryDialog::removeTool()
{
    ToolLibrary* library = currentLibrary();
    const QStringList ids = selectedToolIds();
    if (ids.isEmpty())
        return;
    QStringList names;
    for (const QString& id : ids) {
        const ToolBit* tool = library ? library->toolById(id)
                                      : m_catalog.assetById(id);
        if (tool)
            names.push_back(tool->name);
    }
    if (names.isEmpty())
        return;
    if (QMessageBox::question(this, QStringLiteral("删除刀具"),
                              library
                                  ? QStringLiteral(
                                        "确定从当前刀库移除 %1 把刀具吗？全局资产仍会保留。\n%2")
                                        .arg(names.size())
                                        .arg(names.join(QStringLiteral("、")))
                                  : QStringLiteral(
                                        "确定永久删除 %1 个全局刀具资产吗？所有刀库引用和 T 刀位也会解除。\n%2")
                                        .arg(names.size())
                                        .arg(names.join(QStringLiteral("、"))))
        != QMessageBox::Yes) return;
    QString error;
    oap::tooling::ToolAssetCatalog candidate = m_catalog;
    for (const QString& id : ids) {
        const bool removed = library
            ? candidate.detachAsset(library->id(), id, &error)
            : candidate.removeAsset(id, &error);
        if (!removed) {
            QMessageBox::critical(this, QStringLiteral("删除刀具"), error);
            return;
        }
    }
    m_catalog = std::move(candidate);
    refreshTable();
}

void ToolLibraryDialog::accept()
{
    QString error;
    if (!m_catalog.synchronize(&error)) {
        QMessageBox::critical(this, QStringLiteral("保存刀具资产目录"), error);
        return;
    }
    QDialog::accept();
}

ToolLibrarySelectionDialog::ToolLibrarySelectionDialog(
    const QVector<ToolLibrary>& libraries,
    const QString& activeLibraryId,
    QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("选择 CAM 活动刀库"));
    setMinimumSize(520, 380);
    resize(640, 500);
    auto* layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(
        QStringLiteral("选择参与自动选刀的刀库。已生成的 Job 仍保留自己的不可变刀具快照。"),
        this));

    m_list = new QListWidget(this);
    m_list->setSelectionMode(QAbstractItemView::SingleSelection);
    int activeRow = -1;
    for (const ToolLibrary& library : libraries) {
        const int assignedCount = library.assignedTools().size();
        auto* item = new QListWidgetItem(
            QStringLiteral("%1\n%2 把已分配 T 刀具%3")
                .arg(library.name())
                .arg(assignedCount)
                .arg(library.machine().isEmpty()
                    ? QString()
                    : QStringLiteral(" · %1").arg(library.machine())),
            m_list);
        item->setData(Qt::UserRole, library.id());
        item->setData(Qt::UserRole + 1, assignedCount);
        item->setData(Qt::UserRole + 2, library.description());
        if (library.id() == activeLibraryId)
            activeRow = m_list->count() - 1;
    }
    if (activeRow < 0 && m_list->count() > 0)
        activeRow = 0;
    m_list->setCurrentRow(activeRow);
    layout->addWidget(m_list, 1);

    m_summary = new QLabel(this);
    m_summary->setWordWrap(true);
    layout->addWidget(m_summary);
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_selectButton = buttons->button(QDialogButtonBox::Ok);
    m_selectButton->setText(QStringLiteral("用于 CAM"));
    layout->addWidget(buttons);

    connect(m_list, &QListWidget::currentRowChanged,
            this, [this] { updateSelectionState(); });
    connect(m_list, &QListWidget::itemDoubleClicked,
            this, [this](QListWidgetItem*) { accept(); });
    connect(buttons, &QDialogButtonBox::accepted,
            this, &ToolLibrarySelectionDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected,
            this, &QDialog::reject);
    updateSelectionState();
}

QString ToolLibrarySelectionDialog::selectedLibraryId() const
{
    const QListWidgetItem* selected = m_list->currentItem();
    return selected ? selected->data(Qt::UserRole).toString() : QString();
}

void ToolLibrarySelectionDialog::updateSelectionState()
{
    const QListWidgetItem* selected = m_list->currentItem();
    const int assignedCount = selected
        ? selected->data(Qt::UserRole + 1).toInt() : 0;
    m_selectButton->setEnabled(selected && assignedCount > 0);
    m_summary->setText(!selected
        ? QStringLiteral("没有可选刀库。")
        : assignedCount > 0
            ? selected->data(Qt::UserRole + 2).toString()
            : QStringLiteral("该刀库没有已分配 T 号的刀具，不能用于自动选刀。"));
}

void ToolLibrarySelectionDialog::accept()
{
    const QListWidgetItem* selected = m_list->currentItem();
    if (!selected || selected->data(Qt::UserRole + 1).toInt() <= 0) {
        QMessageBox::information(
            this, QStringLiteral("选择刀库"),
            QStringLiteral("请选择至少包含一把已分配 T 刀具的刀库。"));
        return;
    }
    QDialog::accept();
}
