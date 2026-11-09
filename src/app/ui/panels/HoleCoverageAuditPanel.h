// HoleCoverageAuditPanel — 展示参考 NC 导出复核所使用的孔覆盖审计明细。
#pragma once

#include <QWidget>

#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "toolpath/hole/HoleCoverage.h"

class QLabel;
class QTreeWidget;

// 孔覆盖审计面板：展示孔覆盖审计结果。
class HoleCoverageAuditPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit HoleCoverageAuditPanel(QWidget* parent = nullptr);

    void setReport(
        const oap::toolpath::HoleCoverageReport& report,
        const QVector<oap::app::HoleCoverageOperationBinding>& bindings);
    void clear();
    void selectFormalOperation(const QString& formalOperationId);
    QString selectedFormalOperationId() const;

signals:
    // 用户选中具体孔段行时发出；分类和汇总行不触发定位。
    void sourceHoleSelected(int sourceHoleIndex);
    void formalOperationSelected(const QString& formalOperationId);

private:
    QLabel* m_statusLabel = nullptr;
    QTreeWidget* m_tree = nullptr;
};
