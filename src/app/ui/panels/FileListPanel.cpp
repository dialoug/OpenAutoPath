// FileListPanel 实现:按文件名规则把 STEP 文件分组的侧边栏。分组键由
// categoryForFile 从文件名推断(项目号前缀/数字/其他),组内按自然顺序排序;
// 相邻文件导航 activateRelativeFile 供 MainWindow 的 W/S 快捷键使用。
#include "app/ui/panels/FileListPanel.h"

#include <QCollator>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QHeaderView>
#include <QLabel>
#include <QMap>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

#include <algorithm>
#include <vector>

namespace {
constexpr int kRolePath = Qt::UserRole + 1;
constexpr int kRoleKind = Qt::UserRole + 2;
constexpr int kCountColumnWidth = 48;
constexpr const char* kKindGroup = "group";
constexpr const char* kKindFile = "file";

struct FileEntry {
    QFileInfo info;
    QString category;
};

QString projectKey(const QString& token)
{
    const QStringList parts = token.trimmed().split(QLatin1Char('-'), Qt::SkipEmptyParts);
    return parts.size() >= 2 ? parts[0] + QLatin1Char('-') + parts[1] : token.trimmed();
}

bool isLooseNumeric(QString text)
{
    text.remove(QRegularExpression(QStringLiteral("[.\\-_\\s]")));
    if (text.isEmpty()) return false;
    return std::all_of(text.cbegin(), text.cend(), [](QChar ch) { return ch.isDigit(); });
}

QString leadingLetters(const QString& token)
{
    QString result;
    for (QChar ch : token.trimmed()) {
        if (!ch.isLetter()) break;
        result.append(ch.toUpper());
    }
    return result;
}

// 由文件名推断分组键。命名约定 `<前缀>__<零件>__<来源>`:零件与来源的项目号
// 前缀一致时归入该项目组;否则按纯数字、已知前缀(DIE/LP/MI/PH...)兜底归类,
// 保证同一零件的多个版本/相配文件在侧边栏中聚在一起。
QString categoryForFile(const QFileInfo& info)
{
    const QStringList parts = info.completeBaseName().split(QStringLiteral("__"));
    if (parts.size() < 3) return QStringLiteral("其他");
    const QString part = parts[1].trimmed();
    const QString source = parts[2].trimmed();
    if (part.isEmpty() || source.isEmpty()) return QStringLiteral("其他");
    const QString partProject = projectKey(part);
    if (!partProject.isEmpty() &&
        QString::compare(partProject, projectKey(source), Qt::CaseInsensitive) == 0)
        return partProject;
    if (isLooseNumeric(part)) return QStringLiteral("数字");

    const QString prefix = leadingLetters(part);
    static const QStringList known{
        QStringLiteral("DIE"), QStringLiteral("LP"), QStringLiteral("MI"),
        QStringLiteral("PG"), QStringLiteral("PH"), QStringLiteral("PS"),
        QStringLiteral("PU"), QStringLiteral("UB"), QStringLiteral("UP"),
        QStringLiteral("B"), QStringLiteral("U")};
    for (const QString& value : known)
        if (QString::compare(prefix, value, Qt::CaseInsensitive) == 0) return value;
    return QStringLiteral("其他");
}

int categoryRank(const QString& category)
{
    static const QStringList fixed{
        QStringLiteral("B"), QStringLiteral("DIE"), QStringLiteral("LP"),
        QStringLiteral("MI"), QStringLiteral("PG"), QStringLiteral("PH"),
        QStringLiteral("PS"), QStringLiteral("PU"), QStringLiteral("U"),
        QStringLiteral("UB"), QStringLiteral("UP")};
    const int index = fixed.indexOf(category);
    if (index >= 0) return index;
    if (category == QStringLiteral("数字")) return 10000;
    if (category == QStringLiteral("其他")) return 10001;
    return 5000;
}

QCollator naturalCollator()
{
    QCollator collator;
    collator.setNumericMode(true);
    collator.setCaseSensitivity(Qt::CaseInsensitive);
    return collator;
}
} // namespace

FileListPanel::FileListPanel(QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);
    m_header = new QLabel(QStringLiteral("未打开文件夹"), this);
    m_header->setWordWrap(true);
    m_header->setStyleSheet(QStringLiteral("color:#888; font-size:11px;"));
    layout->addWidget(m_header);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabels({QStringLiteral("分类 / 文件"), QStringLiteral("数量")});
    m_tree->setAlternatingRowColors(true);
    m_tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_tree->setTextElideMode(Qt::ElideRight);
    m_tree->header()->setStretchLastSection(false);
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Interactive);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::Interactive);
    m_tree->viewport()->installEventFilter(this);
    fitColumnsToViewport(kCountColumnWidth);
    connect(m_tree->header(), &QHeaderView::sectionResized, this,
            [this](int section, int, int size) {
                if (section == 0) fitColumnsToViewport(m_tree->viewport()->width() - size);
                else if (section == 1) fitColumnsToViewport(size);
            });
    connect(m_tree, &QTreeWidget::currentItemChanged,
            this, &FileListPanel::onCurrentItemChanged);
    layout->addWidget(m_tree);
}

int FileListPanel::setFolder(const QString& dir)
{
    m_tree->clear();
    const QFileInfoList files = QDir(dir).entryInfoList(
        {"*.stp", "*.step", "*.STP", "*.STEP"}, QDir::Files, QDir::Name);
    QMap<QString, std::vector<FileEntry>> groups;
    for (const QFileInfo& info : files)
        groups[categoryForFile(info)].push_back({info, categoryForFile(info)});

    std::vector<QString> categories;
    for (auto it = groups.cbegin(); it != groups.cend(); ++it) categories.push_back(it.key());
    QCollator collator = naturalCollator();
    std::sort(categories.begin(), categories.end(), [&](const QString& a, const QString& b) {
        const int ar = categoryRank(a), br = categoryRank(b);
        return ar != br ? ar < br : collator.compare(a, b) < 0;
    });

    const QSignalBlocker signalBlocker(m_tree);
    for (const QString& category : categories) {
        auto entries = groups[category];
        std::sort(entries.begin(), entries.end(), [&](const FileEntry& a, const FileEntry& b) {
            return collator.compare(a.info.fileName(), b.info.fileName()) < 0;
        });
        auto* group = new QTreeWidgetItem(m_tree);
        group->setText(0, category);
        group->setText(1, QString::number(entries.size()));
        group->setData(0, kRoleKind, QString::fromLatin1(kKindGroup));
        for (const FileEntry& entry : entries) {
            auto* item = new QTreeWidgetItem(group);
            item->setText(0, entry.info.fileName());
            item->setData(0, kRoleKind, QString::fromLatin1(kKindFile));
            item->setData(0, kRolePath, entry.info.absoluteFilePath());
            item->setToolTip(0, entry.info.absoluteFilePath());
        }
    }
    m_tree->expandToDepth(0);
    fitColumnsToViewport(m_tree->header()->sectionSize(1));
    m_header->setText(QStringLiteral("%1\n共 %2 个 STEP 文件")
                          .arg(QDir::toNativeSeparators(dir)).arg(files.size()));
    return files.size();
}

void FileListPanel::setCurrentFile(const QString& path)
{
    for (QTreeWidgetItemIterator it(m_tree); *it; ++it) {
        QTreeWidgetItem* item = *it;
        if (item->data(0, kRolePath).toString() != path) continue;
        const QSignalBlocker signalBlocker(m_tree);
        m_tree->clearSelection();
        item->setSelected(true);
        if (item->parent()) item->parent()->setExpanded(true);
        m_tree->setCurrentItem(item);
        m_tree->scrollToItem(item);
        return;
    }
}

QStringList FileListPanel::filePaths() const
{
    QStringList paths;
    for (QTreeWidgetItemIterator it(m_tree); *it; ++it) {
        const QString path = (*it)->data(0, kRolePath).toString();
        if (!path.isEmpty()) paths << path;
    }
    return paths;
}

QString FileListPanel::currentFilePath() const
{
    QTreeWidgetItem* item = m_tree ? m_tree->currentItem() : nullptr;
    return item ? item->data(0, kRolePath).toString() : QString();
}

bool FileListPanel::canActivateRelativeFile(int delta) const
{
    const QStringList paths = filePaths();
    const int index = paths.indexOf(currentFilePath());
    return delta != 0 && index >= 0 && index + delta >= 0 && index + delta < paths.size();
}

bool FileListPanel::activateRelativeFile(int delta)
{
    if (!canActivateRelativeFile(delta)) return false;
    const QStringList paths = filePaths();
    const QString path = paths[paths.indexOf(currentFilePath()) + delta];
    setCurrentFile(path);
    emit fileActivated(path);
    return true;
}

void FileListPanel::onCurrentItemChanged()
{
    const QString path = currentFilePath();
    if (!path.isEmpty()) emit fileActivated(path);
}

bool FileListPanel::eventFilter(QObject* watched, QEvent* event)
{
    if (m_tree && watched == m_tree->viewport() && event->type() == QEvent::Resize)
        fitColumnsToViewport(m_tree->header()->sectionSize(1));
    return QWidget::eventFilter(watched, event);
}

void FileListPanel::fitColumnsToViewport(int preferredCountWidth)
{
    if (!m_tree || m_adjustingColumns || m_tree->viewport()->width() <= 0) return;
    m_adjustingColumns = true;
    const int total = m_tree->viewport()->width();
    const int count = std::clamp(preferredCountWidth < 0 ? kCountColumnWidth : preferredCountWidth,
                                 0, total);
    m_tree->header()->resizeSection(1, count);
    m_tree->header()->resizeSection(0, total - count);
    m_adjustingColumns = false;
}
