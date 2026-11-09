// GCodeReviewDialog 实现:导出前 NC 审查对话框。每个文件一页只读代码,并附
// 结构审计树;审计有错误时禁用确认写出,操作者勾选逐行核对后才允许落盘。
#include "app/ui/dialogs/GCodeReviewDialog.h"

#include "toolpath/program/validation/GCodeReview.h"

#include <algorithm>

#include <QCheckBox>
#include <QBrush>
#include <QColor>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFontDatabase>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTextBlock>
#include <QTextEdit>
#include <QTextFormat>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace oap::app {
namespace {

constexpr int kDocumentRole = Qt::UserRole + 1;
constexpr int kLineRole = Qt::UserRole + 2;

class CodePage final : public QWidget {
public:
    CodePage(const GCodeReviewDocument& document, QWidget* parent = nullptr)
        : QWidget(parent)
    {
        auto *layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        auto *path = new QLabel(document.outputPath, this);
        path->setTextInteractionFlags(Qt::TextSelectableByMouse);
        layout->addWidget(path);
        m_editor = new QPlainTextEdit(this);
        m_editor->setObjectName(QStringLiteral("gcodeReviewEditor"));
        m_editor->setReadOnly(true);
        m_editor->setLineWrapMode(QPlainTextEdit::NoWrap);
        m_editor->setFont(QFontDatabase::systemFont(
            QFontDatabase::FixedFont));
        m_editor->setPlainText(document.text);
        layout->addWidget(m_editor, 1);
        m_position = new QLabel(this);
        layout->addWidget(m_position);
        connect(m_editor, &QPlainTextEdit::cursorPositionChanged,
                this, [this] { highlightCurrentLine(); });
        highlightCurrentLine();
    }

    void goToLine(int oneBasedLine)
    {
        const int line = std::max(1, oneBasedLine);
        QTextBlock block = m_editor->document()->findBlockByNumber(line - 1);
        if (!block.isValid())
            block = m_editor->document()->lastBlock();
        QTextCursor cursor(block);
        m_editor->setTextCursor(cursor);
        m_editor->centerCursor();
        m_editor->setFocus();
        highlightCurrentLine();
    }

private:
    void highlightCurrentLine()
    {
        QTextEdit::ExtraSelection selection;
        selection.cursor = m_editor->textCursor();
        selection.cursor.clearSelection();
        selection.format.setBackground(QColor(255, 244, 190));
        selection.format.setProperty(QTextFormat::FullWidthSelection, true);
        m_editor->setExtraSelections({selection});
        const QTextCursor cursor = m_editor->textCursor();
        m_position->setText(
            QStringLiteral("行 %1，列 %2")
                .arg(cursor.blockNumber() + 1)
                .arg(cursor.positionInBlock() + 1));
    }

    QPlainTextEdit* m_editor = nullptr;
    QLabel* m_position = nullptr;
};

} // namespace

bool confirmGCodeReview(QWidget* parent,
                        const QVector<GCodeReviewDocument>& documents)
{
    // 先把每个文档交给结构审计(GCodeReview),把错误/警告汇总进审计树;
    // 有错误或未勾选核对确认时,确认写出按钮保持禁用。
    if (documents.isEmpty())
        return false;

    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("导出前 NC 审查"));
    dialog.resize(1050, 760);
    auto *layout = new QVBoxLayout(&dialog);
    auto *summary = new QLabel(&dialog);
    summary->setWordWrap(true);
    layout->addWidget(summary);

    auto *tabs = new QTabWidget(&dialog);
    QVector<CodePage*> pages;
    pages.reserve(documents.size());
    int totalErrors = 0;
    int totalWarnings = 0;
    QVector<oap::toolpath::GCodeReviewReport> reports;
    reports.reserve(documents.size());
    for (const GCodeReviewDocument& document : documents) {
        auto *page = new CodePage(document, tabs);
        pages.push_back(page);
        tabs->addTab(page, document.title);
        oap::toolpath::GCodeReviewReport report =
            oap::toolpath::GCodeReview{}.review(document.text.toStdString());
        totalErrors += report.errorCount;
        totalWarnings += report.warningCount;
        reports.push_back(std::move(report));
    }
    layout->addWidget(tabs, 3);

    auto *audit = new QTreeWidget(&dialog);
    audit->setObjectName(QStringLiteral("gcodeReviewAudit"));
    audit->setHeaderLabels({QStringLiteral("级别"), QStringLiteral("文件"),
                            QStringLiteral("行"), QStringLiteral("审计项")});
    audit->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    audit->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    audit->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    audit->header()->setSectionResizeMode(3, QHeaderView::Stretch);
    for (int documentIndex = 0; documentIndex < reports.size();
         ++documentIndex) {
        const auto& report = reports[documentIndex];
        if (report.issues.empty()) {
            auto *item = new QTreeWidgetItem(audit);
            item->setText(0, QStringLiteral("通过"));
            item->setText(1, documents[documentIndex].title);
            item->setText(3, QStringLiteral("结构审计未发现问题"));
            item->setForeground(0, QBrush(QColor(0, 120, 0)));
            continue;
        }
        for (const oap::toolpath::GCodeReviewIssue& issue : report.issues) {
            auto *item = new QTreeWidgetItem(audit);
            const bool error = issue.severity ==
                oap::toolpath::GCodeReviewSeverity::Error;
            item->setText(0, error ? QStringLiteral("错误")
                                   : QStringLiteral("警告"));
            item->setText(1, documents[documentIndex].title);
            item->setText(2, issue.line > 0
                ? QString::number(issue.line) : QStringLiteral("全局"));
            item->setText(3, QString::fromStdString(issue.message));
            item->setToolTip(3, QString::fromStdString(issue.code));
            item->setData(0, kDocumentRole, documentIndex);
            item->setData(0, kLineRole, issue.line);
            item->setForeground(0, QBrush(error ? QColor(180, 0, 0)
                                                : QColor(170, 100, 0)));
        }
    }
    layout->addWidget(audit, 1);
    const auto goToIssue = [tabs, pages](QTreeWidgetItem* item) {
        if (!item)
            return;
        const int documentIndex = item->data(0, kDocumentRole).toInt();
        if (documentIndex < 0 || documentIndex >= pages.size())
            return;
        tabs->setCurrentIndex(documentIndex);
        pages[documentIndex]->goToLine(item->data(0, kLineRole).toInt());
    };
    QObject::connect(audit, &QTreeWidget::itemActivated, &dialog,
                     [goToIssue](QTreeWidgetItem* item) { goToIssue(item); });
    QObject::connect(audit, &QTreeWidget::itemClicked, &dialog,
                     [goToIssue](QTreeWidgetItem* item) { goToIssue(item); });

    summary->setText(
        totalErrors == 0
            ? QStringLiteral("%1 个参考 NC 文件结构审计通过；警告 %2。请逐行核对刀具信息、G54–G59、S/F、固定循环和退刀位置。")
                  .arg(documents.size()).arg(totalWarnings)
            : QStringLiteral("结构审计发现错误 %1、警告 %2；修正前不能写出 NC。")
                  .arg(totalErrors).arg(totalWarnings));
    summary->setStyleSheet(totalErrors == 0
        ? QStringLiteral("color:#166534")
        : QStringLiteral("color:#b91c1c;font-weight:bold"));

    auto *confirmed = new QCheckBox(
        QStringLiteral("我已逐行核对所有文件的刀具信息、工件坐标、转速进给、循环和安全退刀"),
        &dialog);
    confirmed->setEnabled(totalErrors == 0);
    layout->addWidget(confirmed);
    auto *buttons = new QDialogButtonBox(
        QDialogButtonBox::Save | QDialogButtonBox::Cancel, &dialog);
    QPushButton* save = buttons->button(QDialogButtonBox::Save);
    save->setText(QStringLiteral("确认并写出"));
    save->setEnabled(false);
    QObject::connect(confirmed, &QCheckBox::toggled,
                     save, &QPushButton::setEnabled);
    QObject::connect(buttons, &QDialogButtonBox::accepted,
                     &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected,
                     &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    return dialog.exec() == QDialog::Accepted;
}

} // namespace oap::app
