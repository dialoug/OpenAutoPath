#include "support/Utf8Path.h"
#include "support/Files.h"
#include "support/Identity.h"

#include <algorithm>
#include <array>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#endif

namespace oap {
namespace {

bool fail(std::string* error, std::string text)
{
    if (error) *error = std::move(text);
    return false;
}

} // namespace

bool readFile(std::string_view path, std::string& data, std::string* error, std::size_t limit)
{
    data.clear();
    std::ifstream input(utf8Path(path), std::ios::binary);
    if (!input) return fail(error, "Cannot open file: " + std::string(path));

    std::array<char, 16384> buffer;
    while (input) {
        input.read(buffer.data(), buffer.size());
        const auto count = static_cast<std::size_t>(input.gcount());
        if (count > limit - data.size()) return fail(error, "File exceeds size limit");
        data.append(buffer.data(), count);
    }
    return input.eof() && !input.bad() ? true : fail(error, "Cannot read file: " + std::string(path));
}

AtomicFile::AtomicFile(std::string_view filename)
    : m_target(utf8Path(filename)),
      m_temporary(m_target.parent_path() / (".oap-" + randomUuid() + ".tmp")),
      m_stream(m_temporary, std::ios::binary | std::ios::trunc)
{
}

AtomicFile::~AtomicFile()
{
    if (m_committed) return;
    if (m_stream.is_open()) m_stream.close();
    std::error_code ignored;
    std::filesystem::remove(m_temporary, ignored);
}

bool AtomicFile::commit(std::string* error)
{
    m_stream.close();
    if (!m_stream) return fail(error, "Cannot write temporary file");
    std::error_code failure;
#ifdef _WIN32
    if (!MoveFileExW(m_temporary.c_str(), m_target.c_str(), MOVEFILE_REPLACE_EXISTING))
        failure = std::error_code(static_cast<int>(GetLastError()), std::system_category());
#else
    std::filesystem::rename(m_temporary, m_target, failure);
#endif
    if (failure) return fail(error, failure.message());
    m_committed = true;
    return true;
}

bool writeFileAtomically(std::string_view path, std::string_view data, std::string* error)
{
    AtomicFile file(path);
    if (!file.stream()) return fail(error, "Cannot open temporary file: " + std::string(path));
    file.stream().write(data.data(), static_cast<std::streamsize>(data.size()));
    return file.commit(error);
}

} // namespace oap
