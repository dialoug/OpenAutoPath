// ShortcutSettingsDialog 实现 — 参见 ShortcutSettingsDialog.h。
#include "app/ui/dialogs/ShortcutSettingsDialog.h"
#include <QCoreApplication>

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

namespace {
QString mouseButtonText(Qt::MouseButton button)
{
    switch (button) {
        case Qt::LeftButton: return QCoreApplication::translate("OpenAutoPath", "左键");
        case Qt::RightButton: return QCoreApplication::translate("OpenAutoPath", "右键");
        case Qt::MiddleButton: return QCoreApplication::translate("OpenAutoPath", "中键");
        case Qt::BackButton: return QCoreApplication::translate("OpenAutoPath", "侧键1");
        case Qt::ForwardButton: return QCoreApplication::translate("OpenAutoPath", "侧键2");
        default: return QCoreApplication::translate("OpenAutoPath", "无");
    }
}

void fillMouseCombo(QComboBox* combo, Qt::MouseButton current)
{
    for (const auto& item : std::initializer_list<std::pair<QString, Qt::MouseButton>>{
             {QCoreApplication::translate("OpenAutoPath", "无"), Qt::NoButton},
             {QCoreApplication::translate("OpenAutoPath", "左键"), Qt::LeftButton},
             {QCoreApplication::translate("OpenAutoPath", "右键"), Qt::RightButton},
             {QCoreApplication::translate("OpenAutoPath", "中键"), Qt::MiddleButton},
             {QCoreApplication::translate("OpenAutoPath", "侧键1"), Qt::BackButton},
             {QCoreApplication::translate("OpenAutoPath", "侧键2"), Qt::ForwardButton}})
        combo->addItem(item.first, int(item.second));
    const int index = combo->findData(int(current));
    if (index >= 0) combo->setCurrentIndex(index);
}
} // namespace

ShortcutSettingsDialog::ShortcutSettingsDialog(const std::vector<Row>& rows, QWidget* parent)
    : QDialog(parent), m_rows(rows)
{
    setWindowTitle(QCoreApplication::translate("OpenAutoPath", "修改按键"));
    resize(620, 420);
    auto* layout = new QVBoxLayout(this);
    auto* table = new QTableWidget(int(m_rows.size()), 3, this);
    table->setHorizontalHeaderLabels(
        {QCoreApplication::translate("OpenAutoPath", "操作"), QCoreApplication::translate("OpenAutoPath", "当前按键"), QCoreApplication::translate("OpenAutoPath", "修改按键")});
    table->verticalHeader()->setVisible(false);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    for (int index = 0; index < int(m_rows.size()); ++index) {
        const Row& row = m_rows[size_t(index)];
        table->setItem(index, 0, new QTableWidgetItem(row.label));
        if (row.kind == RowKind::MouseButton) {
            table->setItem(index, 1, new QTableWidgetItem(mouseButtonText(row.currentMouse)));
            auto* combo = new QComboBox(table);
            fillMouseCombo(combo, row.currentMouse);
            table->setCellWidget(index, 2, combo);
            m_keyEditors.push_back(nullptr);
            m_mouseEditors.push_back(combo);
        } else {
            table->setItem(index, 1, new QTableWidgetItem(
                                         row.current.toString(QKeySequence::NativeText)));
            auto* editor = new QKeySequenceEdit(row.current, table);
            editor->setClearButtonEnabled(true);
            table->setCellWidget(index, 2, editor);
            m_keyEditors.push_back(editor);
            m_mouseEditors.push_back(nullptr);
        }
    }
    layout->addWidget(table);
    auto* hint = new QLabel(
        QCoreApplication::translate("OpenAutoPath", "同一鼠标键最多绑定一个单击动作和一个拖动动作，例如左键可同时用于拾取和框选。"),
        this);
    hint->setWordWrap(true);
    layout->addWidget(hint);

    auto* row = new QHBoxLayout;
    auto* reset = new QPushButton(QCoreApplication::translate("OpenAutoPath", "全部恢复默认"), this);
    connect(reset, &QPushButton::clicked, this, [this] {
        for (size_t i = 0; i < m_rows.size(); ++i) {
            if (m_rows[i].kind == RowKind::MouseButton) {
                QComboBox* combo = m_mouseEditors[i];
                const int index = combo ? combo->findData(int(m_rows[i].defaultMouse)) : -1;
                if (index >= 0) combo->setCurrentIndex(index);
            } else if (m_keyEditors[i]) {
                m_keyEditors[i]->setKeySequence(m_rows[i].defaultSequence);
            }
        }
    });
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    row->addWidget(reset);
    row->addStretch();
    row->addWidget(buttons);
    layout->addLayout(row);
}

std::vector<ShortcutSettingsDialog::Row> ShortcutSettingsDialog::rows() const
{
    std::vector<Row> result = m_rows;
    for (size_t i = 0; i < result.size(); ++i) {
        if (result[i].kind == RowKind::MouseButton) {
            if (m_mouseEditors[i])
                result[i].editedMouse = Qt::MouseButton(m_mouseEditors[i]->currentData().toInt());
        } else if (m_keyEditors[i]) {
            result[i].edited = m_keyEditors[i]->keySequence();
        }
    }
    return result;
}
