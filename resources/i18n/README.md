# Interface translations

The source UI language is Simplified Chinese. Language choice is stored in QSettings under `ui/language` (`zh_CN` or `en`) and applies on the next launch. The active translator is immutable during a session, including background jobs and logs.

The executable embeds these resources:

- [en.json](en.json): Chinese UI and domain messages to English.
- [diagnostics_zh_CN.json](diagnostics_zh_CN.json): English domain diagnostics to Chinese.
- [logs_zh_CN.json](logs_zh_CN.json): debug prefixes and timing fields to Chinese.
- [labels.json](labels.json): localized display names for stable model classification IDs.
- [Qt translation source](qt/qtbase_zh_CN.ts): Qt 6.11.1 standard controls, messages and dialogs in Chinese; see [provenance and licensing](qt/README.md).

Use `QCoreApplication::translate("OpenAutoPath", "中文源文本")` for static UI text. Preserve placeholders, line breaks, units and file-filter patterns.

For domain messages, convert only the presentation copy using `QCoreApplication::translate("OpenAutoPathDiagnostic", message.c_str())`. Core modules keep original diagnostics and do not depend on Qt. Add complete messages and any concatenated message fragments to the corresponding catalogs. The translator first matches complete messages, then uses longest nonrecursive fragment matches with word boundaries. Quoted parameters and output-file path tails are preserved. Unknown vendor messages remain verbatim so their diagnostic content is never discarded; they need explicit catalog entries to be localized.

Use `OpenAutoPathLabel` only for display copies of known classification IDs. Never translate persistent IDs, keys, user-entered names, file paths, tool names or NC program text. NC syntax and original generated NC comments are preserved for interoperability; the review UI translates its explanatory messages separately.

Qt debug/warning output goes through a localized message handler; recognition stderr diagnostics have a Qt-independent optional presentation sink. The default core-only behavior remains stderr. Raw output emitted internally by third-party libraries is outside this handler. File dialogs use Qt widgets so their language follows the app rather than the operating system.

The `language` CTest verifies static catalog coverage and placeholders, both languages of real parameter dialogs and Qt standard buttons, composed domain errors, model labels, logs, preserved path values, unknown-message fallback, settings and stable tool IDs. It does not claim exhaustive fault injection for all third-party exceptions.

After building `oap_test_language`, run:

```sh
ctest --test-dir build -C RelWithDebInfo -R "^language$" --output-on-failure
```
