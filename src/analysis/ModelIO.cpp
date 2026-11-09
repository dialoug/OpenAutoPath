// ModelIO 实现 —— 参见 ModelIO.h。紧凑的小端二进制格式,自带魔数 + 版本。
#include "ModelIO.h"
#include "Utf8Path.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace oap {
namespace {

constexpr std::array<char, 8> kMagic = {'F','F','M','D','L','B','I','N'};
constexpr uint32_t kMaxStringBytes = 16u * 1024u * 1024u;
constexpr uint32_t kMaxVectorElems = 1'000'000u;
constexpr uint64_t kMaxModelFileBytes = 512ull * 1024ull * 1024ull;
constexpr size_t kWriterBufferBytes = 1u << 20;

// ---- 写出原语(小端;在常见的小端目标平台上直写即可)。----
// 使用固定 1 MiB 缓冲分块写盘，避免大模型序列化时再复制一份完整缓存。
struct Writer {
    std::ostream& os;
    std::vector<char> buf;
    bool bad = false;
    uint64_t bytes = 0;

    explicit Writer(std::ostream& output) : os(output) {
        buf.reserve(kWriterBufferBytes);
    }
    void flush() {
        if (bad || buf.empty()) return;
        os.write(buf.data(), static_cast<std::streamsize>(buf.size()));
        if (!os) bad = true;
        buf.clear();
    }
    void raw(const void* d, size_t n) {
        if (bad) return;
        if (n > kMaxModelFileBytes || bytes > kMaxModelFileBytes - n) {
            bad = true;
            return;
        }
        bytes += n;
        const char* c = static_cast<const char*>(d);
        while (n > 0 && !bad) {
            const size_t available = kWriterBufferBytes - buf.size();
            const size_t chunk = std::min(n, available);
            buf.insert(buf.end(), c, c + chunk);
            c += chunk;
            n -= chunk;
            if (buf.size() == kWriterBufferBytes)
                flush();
        }
    }
    void u8(uint8_t v)  { raw(&v, sizeof(v)); }
    void u32(uint32_t v){ raw(&v, sizeof(v)); }
    void u64(uint64_t v){ raw(&v, sizeof(v)); }
    void i32(int32_t v) { raw(&v, sizeof(v)); }
    void dbl(double v)  { raw(&v, sizeof(v)); }
    void boolean(bool v){ u8(v ? 1 : 0); }
    void str(const std::string& s) {
        if (s.size() > kMaxStringBytes) { bad = true; return; }
        u32(static_cast<uint32_t>(s.size()));
        if (!s.empty()) raw(s.data(), s.size());
    }
    void vstr(const std::vector<std::string>& v) {
        if (v.size() > kMaxVectorElems) { bad = true; return; }
        u32(static_cast<uint32_t>(v.size()));
        for (const auto& s : v) str(s);
    }
    void bbox(const Bbox& b) {
        dbl(b.xmin); dbl(b.ymin); dbl(b.zmin);
        dbl(b.xmax); dbl(b.ymax); dbl(b.zmax);
    }
    void arr3(const std::array<double,3>& a) { dbl(a[0]); dbl(a[1]); dbl(a[2]); }
};

void wUnit(Writer& w, const Unit& u) {
    w.str(u.unit_id); w.str(u.unit_name); w.str(u.decoded_name);
    w.str(u.name_prefix); w.str(u.name_family); w.str(u.unit_kind);
    w.str(u.unit_geometry_class); w.str(u.unit_role_hint); w.str(u.role_hint_source);
    w.dbl(u.role_hint_confidence); w.vstr(u.role_hint_diagnostics);
    w.bbox(u.bbox); w.vstr(u.face_ids);
}
void wFace(Writer& w, const Face& f) {
    w.str(f.face_id); w.str(f.unit_id); w.str(f.surface_type);
    w.dbl(f.area); w.bbox(f.bbox);
    w.boolean(f.has_center); w.arr3(f.center);
    w.boolean(f.has_normal); w.arr3(f.normal);
    w.dbl(f.radius); w.vstr(f.edge_ids); w.vstr(f.wire_ids);
}
void wEdge(Writer& w, const Edge& e) {
    w.str(e.edge_id); w.str(e.unit_id); w.str(e.curve_type);
    w.dbl(e.length); w.bbox(e.bbox); w.vstr(e.face_ids); w.vstr(e.wire_ids);
    w.boolean(e.has_start); w.arr3(e.start_point);
    w.boolean(e.has_end); w.arr3(e.end_point);
}
void wWire(Writer& w, const Wire& wi) {
    w.str(wi.wire_id); w.str(wi.unit_id); w.str(wi.face_id); w.str(wi.wire_role);
    w.boolean(wi.is_closed); w.vstr(wi.edge_ids); w.bbox(wi.bbox);
}
template <typename T, typename Fn>
void wVec(Writer& w, const std::vector<T>& v, Fn&& fn) {
    if (v.size() > kMaxVectorElems) { w.bad = true; return; }
    w.u32(static_cast<uint32_t>(v.size()));
    for (const auto& e : v) fn(w, e);
}

} // namespace

// 写出顺序必须与 readModel 中 rVec 的解析顺序完全一致:
// units/faces/edges/wires。
// 新增字段或调整顺序时须同步两处,并在 ModelIO.h 中 +1 版本号。
bool writeModel(const std::string& path, const Model& m) {
    try {
        std::ofstream os(utf8Path(path), std::ios::binary | std::ios::trunc);
        if (!os) return false;
        Writer w(os);
        w.raw(kMagic.data(), kMagic.size());
        w.u32(kModelIOFormatVersion);
        wVec(w, m.units,          wUnit);
        wVec(w, m.faces,          wFace);
        wVec(w, m.edges,          wEdge);
        wVec(w, m.wires,          wWire);
        w.flush();
        os.flush();
        return !w.bad && static_cast<bool>(os);
    } catch (...) {
        return false;
    }
}

namespace {

// ---- 流式读入原语。任何越界、截断或 I/O 失败都置 bad 标志。 ----
struct Reader {
    std::istream& is;
    uint64_t bytesRemaining = 0;
    std::vector<char> buffer;
    size_t bufferOffset = 0;
    size_t bufferSize = 0;
    bool bad = false;
    Reader(std::istream& input, uint64_t size)
        : is(input), bytesRemaining(size), buffer(kWriterBufferBytes) {}
    bool ok() const noexcept { return !bad; }
    uint64_t remaining() const noexcept { return bytesRemaining; }
    // 从游标拷出 n 字节到 dst;越界则置 bad 并清零。
    void take(void* dst, size_t n) {
        if (bad || remaining() < n ||
            n > static_cast<size_t>(std::numeric_limits<std::streamsize>::max())) {
            bad = true;
            std::memset(dst, 0, n);
            return;
        }
        char* output = static_cast<char*>(dst);
        size_t needed = n;
        while (needed > 0) {
            if (bufferOffset == bufferSize) {
                const size_t request = static_cast<size_t>(std::min<uint64_t>(
                    buffer.size(), bytesRemaining));
                is.read(buffer.data(), static_cast<std::streamsize>(request));
                if (is.gcount() != static_cast<std::streamsize>(request)) {
                    bad = true;
                    std::memset(output, 0, needed);
                    return;
                }
                bufferOffset = 0;
                bufferSize = request;
            }
            const size_t copied = std::min(needed, bufferSize - bufferOffset);
            std::memcpy(output, buffer.data() + bufferOffset, copied);
            output += copied;
            needed -= copied;
            bufferOffset += copied;
            bytesRemaining -= copied;
        }
    }
    uint8_t  u8()  { uint8_t v = 0; take(&v, 1); return v; }
    uint32_t u32() { uint32_t v = 0; take(&v, 4); return v; }
    int32_t  i32() { int32_t v = 0; take(&v, 4); return v; }
    double   dbl() { double v = 0; take(&v, 8); return v; }
    bool     boolean() { return u8() != 0; }
    std::string str() {
        uint32_t n = u32();
        if (bad || n > kMaxStringBytes || remaining() < n) { bad = true; return {}; }
        std::string s(n, '\0');
        if (n > 0) take(s.data(), n);
        return s;
    }
    std::vector<std::string> vstr() {
        uint32_t n = u32();
        std::vector<std::string> v;
        if (bad || n > kMaxVectorElems || n > remaining() / sizeof(uint32_t)) {
            bad = true;
            return v;
        }
        v.reserve(n);
        for (uint32_t i = 0; i < n && !bad; ++i) v.push_back(str());
        return v;
    }
    Bbox bbox() {
        Bbox b; b.xmin = dbl(); b.ymin = dbl(); b.zmin = dbl();
        b.xmax = dbl(); b.ymax = dbl(); b.zmax = dbl(); return b;
    }
    std::array<double,3> arr3() { return {dbl(), dbl(), dbl()}; }
};

Unit rUnit(Reader& r) {
    Unit u;
    u.unit_id = r.str(); u.unit_name = r.str(); u.decoded_name = r.str();
    u.name_prefix = r.str(); u.name_family = r.str(); u.unit_kind = r.str();
    u.unit_geometry_class = r.str(); u.unit_role_hint = r.str(); u.role_hint_source = r.str();
    u.role_hint_confidence = r.dbl(); u.role_hint_diagnostics = r.vstr();
    u.bbox = r.bbox(); u.face_ids = r.vstr();
    return u;
}
Face rFace(Reader& r) {
    Face f;
    f.face_id = r.str(); f.unit_id = r.str(); f.surface_type = r.str();
    f.area = r.dbl(); f.bbox = r.bbox();
    f.has_center = r.boolean(); f.center = r.arr3();
    f.has_normal = r.boolean(); f.normal = r.arr3();
    f.radius = r.dbl(); f.edge_ids = r.vstr(); f.wire_ids = r.vstr();
    return f;
}
Edge rEdge(Reader& r) {
    Edge e;
    e.edge_id = r.str(); e.unit_id = r.str(); e.curve_type = r.str();
    e.length = r.dbl(); e.bbox = r.bbox(); e.face_ids = r.vstr(); e.wire_ids = r.vstr();
    e.has_start = r.boolean(); e.start_point = r.arr3();
    e.has_end = r.boolean(); e.end_point = r.arr3();
    return e;
}
Wire rWire(Reader& r) {
    Wire w;
    w.wire_id = r.str(); w.unit_id = r.str(); w.face_id = r.str(); w.wire_role = r.str();
    w.is_closed = r.boolean(); w.edge_ids = r.vstr(); w.bbox = r.bbox();
    return w;
}
// 读取 vector:minimumBytesPerElement 为单元素的最小字节数下限,用于分配前
// 预检剩余字节是否足够,避免损坏/伪造的缓存触发超大内存分配。
template <typename T, typename Fn>
void rVec(Reader& r, std::vector<T>& v, uint64_t minimumBytesPerElement,
          Fn&& fn) {
    uint32_t n = r.u32();
    if (r.bad || n > kMaxVectorElems || minimumBytesPerElement == 0 ||
        n > r.remaining() / minimumBytesPerElement) {
        r.bad = true;
        return;
    }
    v.clear(); v.reserve(n);
    for (uint32_t i = 0; i < n && !r.bad; ++i) v.push_back(fn(r));
}

} // namespace

// 读取前先校验文件大小(下限=魔数+版本,上限=kMaxModelFileBytes);
// 随后逐字段解析,任一越界/截断即返回 false,out 不被写回。
bool readModel(const std::string& path, Model& out) {
    try {
        std::ifstream is(utf8Path(path), std::ios::binary | std::ios::ate);
        if (!is) return false;
        const std::streamoff size = is.tellg();
        if (size < static_cast<std::streamoff>(kMagic.size() + sizeof(uint32_t)) ||
            static_cast<uint64_t>(size) > kMaxModelFileBytes)
            return false;
        is.seekg(0);

        Reader r(is, static_cast<uint64_t>(size));
        std::array<char, kMagic.size()> magic{};
        r.take(magic.data(), magic.size());
        if (r.bad || magic != kMagic || r.u32() != kModelIOFormatVersion)
            return false;

        Model parsed;
        rVec(r, parsed.units,          96, rUnit);
        rVec(r, parsed.faces,         134, rFace);
        rVec(r, parsed.edges,         126, rEdge);
        rVec(r, parsed.wires,          69, rWire);
        if (!r.ok() || r.remaining() != 0)
            return false;
        out = std::move(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace oap
