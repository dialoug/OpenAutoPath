// 快捷键与鼠标按键绑定设置对话框:以表格列出全部可编辑项,键盘项用
// QKeySequenceEdit、鼠标项用下拉框编辑;冲突校验与持久化由 MainWindow 在
// dialog.exec() 返回后依据 rows() 统一完成。
#pragma once

#include <QDialog>
#include <QKeySequence>
#include <Qt>

#include <vector>

class QComboBox;
class QKeySequenceEdit;

// 快捷键设置对话框：编辑键盘快捷键与鼠标绑定。
class ShortcutSettingsDialog : public QDialog
{
    Q_OBJECT
public:
    enum class RowKind { Keyboard, MouseButton };
    // 快捷键表格行：一个动作与按键的绑定条目。
    struct Row {
        QString id;
        QString label;
        RowKind kind = RowKind::Keyboard;
        QKeySequence current;
        QKeySequence edited;
        QKeySequence defaultSequence;
        Qt::MouseButton currentMouse = Qt::NoButton;
        Qt::MouseButton editedMouse = Qt::NoButton;
        Qt::MouseButton defaultMouse = Qt::NoButton;
    };

    explicit ShortcutSettingsDialog(const std::vector<Row>& rows,
                                    QWidget* parent = nullptr);
    std::vector<Row> rows() const;

private:
    std::vector<Row> m_rows;
    std::vector<QKeySequenceEdit*> m_keyEditors;
    std::vector<QComboBox*> m_mouseEditors;
};
