#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>

namespace oap {

bool readFile(std::string_view path, std::string& data, std::string* error = nullptr,
              std::size_t limit = std::numeric_limits<std::size_t>::max());
bool writeFileAtomically(std::string_view path, std::string_view data, std::string* error = nullptr);

class AtomicFile {
public:
    explicit AtomicFile(std::string_view filename);
    ~AtomicFile();
    AtomicFile(const AtomicFile&) = delete;
    AtomicFile& operator=(const AtomicFile&) = delete;

    std::ostream& stream() { return m_stream; }
    bool commit(std::string* error = nullptr);

private:
    std::filesystem::path m_target;
    std::filesystem::path m_temporary;
    std::ofstream m_stream;
    bool m_committed = false;
};

} // namespace oap
