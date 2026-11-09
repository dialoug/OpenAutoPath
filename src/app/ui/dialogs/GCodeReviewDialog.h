// 导出前 NC 文本审查窗口：展示实际将写入文件的后处理结果，支持审计项跳转
// 和逐行高亮；只有结构审计通过且操作者明确确认后才允许继续写文件。
#pragma once

#include <QVector>
#include <QString>

class QWidget;

namespace oap::app {

// GCode 审查文档：程序/工序的审查视图模型。
struct GCodeReviewDocument {
    QString title;
    QString outputPath;
    QString text;
};

bool confirmGCodeReview(QWidget* parent,
                        const QVector<GCodeReviewDocument>& documents);

} // namespace oap::app
