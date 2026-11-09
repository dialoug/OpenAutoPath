// Private cell storage: keep the common single interval beside its metadata.
// Heap storage is owned exclusively by this cell; copies never share mutable data.
#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "toolpath/pocket/simulation/DexelDisplayMesh.h"

namespace oap::toolpath::multi_dexel_detail {

class DexelCellIntervals {
public:
    DexelCellIntervals() noexcept = default;

    explicit DexelCellIntervals(std::span<const DexelInterval> values)
    {
        assign(values);
    }

    DexelCellIntervals(const DexelCellIntervals& other)
        : DexelCellIntervals(std::span<const DexelInterval>(other.data(), other.size()))
    {
    }

    DexelCellIntervals(DexelCellIntervals&& other) noexcept
    {
        take(other);
    }

    DexelCellIntervals& operator=(const DexelCellIntervals& other)
    {
        if (this != &other) {
            DexelCellIntervals copy(other);
            *this = std::move(copy);
        }
        return *this;
    }

    DexelCellIntervals& operator=(DexelCellIntervals&& other) noexcept
    {
        if (this != &other) {
            clear();
            take(other);
        }
        return *this;
    }

    ~DexelCellIntervals() { clear(); }

    std::size_t size() const noexcept { return m_size & ~heapFlag; }
    bool empty() const noexcept { return size() == 0; }
    bool usesHeap() const noexcept { return (m_size & heapFlag) != 0; }
    std::size_t capacity() const noexcept
    {
        return usesHeap() ? m_storage.heap.capacity : 1;
    }

    DexelInterval* data() noexcept
    {
        return usesHeap() ? m_storage.heap.data : &m_storage.single;
    }
    const DexelInterval* data() const noexcept
    {
        return usesHeap() ? m_storage.heap.data : &m_storage.single;
    }
    DexelInterval* begin() noexcept { return data(); }
    DexelInterval* end() noexcept { return data() + size(); }
    const DexelInterval* begin() const noexcept { return data(); }
    const DexelInterval* end() const noexcept { return data() + size(); }
    DexelInterval& front() noexcept { return *data(); }
    const DexelInterval& front() const noexcept { return *data(); }

    void clear() noexcept
    {
        if (usesHeap()) {
            Allocator{}.deallocate(m_storage.heap.data, m_storage.heap.capacity);
            std::construct_at(&m_storage.single);
        }
        m_size = 0;
    }

    void reserve(std::size_t requested)
    {
        if (requested <= capacity())
            return;
        if (requested > std::numeric_limits<std::size_t>::max() / sizeof(DexelInterval))
            throw std::length_error("too many cell intervals");
        const auto count = size();
        auto* allocation = Allocator{}.allocate(requested);
        std::uninitialized_copy_n(data(), count, allocation);
        if (usesHeap())
            Allocator{}.deallocate(m_storage.heap.data, m_storage.heap.capacity);
        std::construct_at(&m_storage.heap, Heap{allocation, requested});
        m_size = heapFlag | count;
    }

    void push_back(const DexelInterval& value)
    {
        // A caller may append an interval already in this cell.
        const DexelInterval copy = value;
        const auto count = size();
        if (count == capacity())
            reserve(count + 1);
        if (usesHeap())
            std::construct_at(data() + count, copy);
        else
            m_storage.single = copy;
        m_size = (m_size & heapFlag) | (count + 1);
    }

    void assign(std::span<const DexelInterval> values)
    {
        // Build first so assigning a view into this cell is safe, including on failure.
        DexelCellIntervals next;
        next.reserve(values.size());
        for (const auto& value : values)
            next.push_back(value);
        *this = std::move(next);
    }

    void compact() noexcept
    {
        if (usesHeap() && size() <= 1) {
            const auto count = size();
            const DexelInterval value = count ? front() : DexelInterval{};
            clear();
            m_storage.single = value;
            m_size = count;
        }
    }

private:
    using Allocator = std::allocator<DexelInterval>;
    static_assert(std::is_trivially_copyable_v<DexelInterval>);
    static_assert(std::is_trivially_destructible_v<DexelInterval>);
    static constexpr std::size_t heapFlag =
        std::size_t{1} << (std::numeric_limits<std::size_t>::digits - 1);
    struct Heap {
        DexelInterval* data;
        std::size_t capacity;
    };
    union Storage {
        DexelInterval single;
        Heap heap;
        Storage() noexcept : single{} {}
        ~Storage() {}
    } m_storage;
    std::size_t m_size = 0;

    void take(DexelCellIntervals& other) noexcept
    {
        if (other.usesHeap()) {
            std::construct_at(&m_storage.heap, other.m_storage.heap);
            m_size = other.m_size;
            std::construct_at(&other.m_storage.single);
        } else {
            m_storage.single = other.m_storage.single;
            m_size = other.m_size;
        }
        other.m_size = 0;
    }
};

// Keep the metadata density of the original release vector on 64-bit builds.
static_assert(sizeof(DexelCellIntervals) <= 3 * sizeof(std::size_t));

// Uses the same subtraction implementation as the public vector API.
[[nodiscard]] bool subtractDexelInterval(
    DexelCellIntervals& material,
    const DexelInterval& removal,
    double tolerance);

} // namespace oap::toolpath::multi_dexel_detail
