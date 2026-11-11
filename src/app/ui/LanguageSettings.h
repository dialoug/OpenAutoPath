// Translation state is immutable after startup and shared safely by worker threads.
#pragma once
#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QTranslator>
#include <QXmlStreamReader>
#include <QHash>
#include <QVector>
#include <QDebug>
#include <algorithm>
#include <cstdio>

namespace oap::app {
inline QString preferredLanguage()
{
    const auto value = QSettings().value(QStringLiteral("ui/language"), QStringLiteral("zh_CN")).toString();
    return value == QStringLiteral("en") ? value : QStringLiteral("zh_CN");
}

// Keep the original diagnostic in the domain model. Translate only presentation
// copies, never IDs, persisted values, filenames, tool names or NC program text.
class LanguageTranslator final : public QTranslator
{
public:
    explicit LanguageTranslator(bool english = true) : m_english(english) {}
    bool loadCatalog()
    {
        if (!readObject(":/i18n/en.json", m_messages) ||
            !readObject(":/i18n/diagnostics_zh_CN.json", m_diagnostics) ||
            !readObject(":/i18n/labels.json", m_labels) ||
            !readObject(":/i18n/logs_zh_CN.json", m_logs)) return false;
        m_rules.clear();
        m_qt.clear();
        m_logRules.clear();
        for (auto it = m_logs.begin(); it != m_logs.end(); ++it)
            m_logRules.push_back({it.key(), it.value().toString()});
        std::sort(m_logRules.begin(), m_logRules.end(), [](const Rule& a, const Rule& b) {
            return a.source.size() > b.source.size();
        });
        const auto& source = m_english ? m_messages : m_diagnostics;
        for (auto it = source.begin(); it != source.end(); ++it)
            m_rules.push_back({it.key(), it.value().toString()});
        std::sort(m_rules.begin(), m_rules.end(), [](const Rule& a, const Rule& b) {
            return a.source.size() > b.source.size();
        });
        if (!m_english) {
            QFile file(QStringLiteral(":/i18n/qt/qtbase_zh_CN.ts"));
            if (!file.open(QIODevice::ReadOnly)) return false;
            QXmlStreamReader xml(&file);
            QString context;
            while (!xml.atEnd()) {
                xml.readNext();
                if (!xml.isStartElement()) continue;
                if (xml.name() == QLatin1String("context")) context.clear();
                else if (xml.name() == QLatin1String("name")) context = xml.readElementText();
                else if (xml.name() == QLatin1String("message")) {
                    QString sourceText, translation, comment;
                    bool finished = true;
                    while (xml.readNextStartElement()) {
                        if (xml.name() == QLatin1String("source")) sourceText = xml.readElementText();
                        else if (xml.name() == QLatin1String("comment")) comment = xml.readElementText();
                        else if (xml.name() == QLatin1String("translation")) {
                            finished = !xml.attributes().hasAttribute(QLatin1String("type"));
                            translation = xml.readElementText(QXmlStreamReader::IncludeChildElements).trimmed();
                        } else xml.skipCurrentElement();
                    }
                    if (finished && !translation.isEmpty())
                        m_qt.insert(context + QChar(4) + sourceText + QChar(4) + comment, translation);
                }
            }
            if (xml.hasError() || m_qt.isEmpty()) return false;
        }
        return true;
    }
    bool isEmpty() const override { return m_messages.isEmpty(); }
    QString translate(const char* context, const char* sourceText,
                      const char* disambiguation = nullptr, int n = -1) const override
    {
        Q_UNUSED(n);
        if (!context || !sourceText) return {};
        const QString text = QString::fromUtf8(sourceText);
        if (qstrcmp(context, "OpenAutoPath") == 0)
            return m_english ? m_messages.value(text).toString() : QString();
        if (qstrcmp(context, "OpenAutoPathLabel") == 0) {
            return m_labels.value(text).toObject().value(m_english ? "en" : "zh").toString();
        }
        if (qstrcmp(context, "OpenAutoPathDiagnostic") == 0)
            return diagnostic(text);
        if (qstrcmp(context, "OpenAutoPathLog") == 0) {
            if (m_english) return diagnostic(text);
            // Log tokens are intentionally a separate context: small words such
            // as "read" must never be substituted in user-visible error data.
            return replaceSegments(diagnostic(text), m_logRules);
        }
        if (!m_english) {
            const QString key = QString::fromUtf8(context) + QChar(4) + text + QChar(4);
            auto it = m_qt.constFind(key + QString::fromUtf8(disambiguation ? disambiguation : ""));
            if (it != m_qt.cend()) return *it;
            it = m_qt.constFind(key);
            if (it != m_qt.cend()) return *it;
        }
        return {};
    }
private:
    struct Rule { QString source; QString target; };
    static bool readObject(const char* path, QJsonObject& result)
    {
        QFile file(QString::fromUtf8(path));
        if (!file.open(QIODevice::ReadOnly)) return false;
        QJsonParseError error;
        const auto doc = QJsonDocument::fromJson(file.readAll(), &error);
        if (error.error != QJsonParseError::NoError || !doc.isObject()) return false;
        result = doc.object();
        return !result.isEmpty();
    }
    QString diagnostic(const QString& text) const
    {
        const auto& catalog = m_english ? m_messages : m_diagnostics;
        const auto exact = catalog.value(text);
        if (exact.isString()) return exact.toString();
        return replaceSegments(text, m_rules);
    }
    static bool wordCharacter(QChar c)
    {
        return c.isLetterOrNumber() || c == QLatin1Char('_');
    }
    static QString replaceSegments(const QString& text, const QVector<Rule>& rules)
    {
        QString result;
        for (qsizetype pos = 0; pos < text.size();) {
            // Quoted parameters (notably paths and user names) remain verbatim.
            if (text.at(pos) == QLatin1Char('"')) {
                qsizetype end = pos + 1;
                while (end < text.size()) {
                    if (text.at(end) == QLatin1Char('\\')) { end += 2; continue; }
                    if (text.at(end++) == QLatin1Char('"')) break;
                }
                result += text.mid(pos, end - pos);
                pos = end;
                continue;
            }
            bool matched = false;
            for (const auto& rule : rules) {
                if (rule.source.isEmpty() || !QStringView(text).mid(pos).startsWith(rule.source)) continue;
                // Do not translate pieces of identifiers or words. Chinese
                // message fragments may legitimately adjoin a dynamic value.
                const auto first = rule.source.front(), last = rule.source.back();
                const qsizetype end = pos + rule.source.size();
                if (first.unicode() < 128 && wordCharacter(first) && pos > 0 && wordCharacter(text.at(pos - 1))) continue;
                if (last.unicode() < 128 && wordCharacter(last) && end < text.size() && wordCharacter(text.at(end))) continue;
                result += rule.target;
                // These diagnostics append an opaque path, not another message.
                // Preserve relative paths containing spaces as well as absolute paths.
                if (rule.source == QLatin1String("cannot open output file: ") ||
                    rule.source == QLatin1String("cannot write output file: ")) {
                    result += text.mid(end);
                    return result;
                }
                pos = end;
                matched = true;
                break;
            }
            if (!matched) result += text.at(pos++);
        }
        return result;
    }
    bool m_english;
    QJsonObject m_messages, m_diagnostics, m_labels, m_logs;
    QVector<Rule> m_rules, m_logRules;
    QHash<QString, QString> m_qt;
};

inline QtMessageHandler previousLanguageMessageHandler = nullptr;
inline void defaultLanguageMessageOutput(QtMsgType type, const QMessageLogContext& context, const QString& message)
{
    const auto bytes = qFormatLogMessage(type, context, message).toLocal8Bit();
    std::fwrite(bytes.constData(), 1, static_cast<std::size_t>(bytes.size()), stderr);
    std::fputc('\n', stderr);
    std::fflush(stderr);
}
inline void localizedMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& message)
{
    // A failing handler must not recurse or lose the original diagnostic.
    static thread_local bool active = false;
    if (active) { defaultLanguageMessageOutput(type, context, message); return; }
    active = true;
    try {
        const auto translated = QCoreApplication::translate("OpenAutoPathLog", message.toUtf8().constData());
        if (previousLanguageMessageHandler) previousLanguageMessageHandler(type, context, translated);
        else defaultLanguageMessageOutput(type, context, translated);
    } catch (...) {
        defaultLanguageMessageOutput(type, context, message);
    }
    active = false;
}
} // namespace oap::app
