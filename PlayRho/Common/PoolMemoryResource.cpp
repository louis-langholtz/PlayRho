/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <algorithm> // for std::find_if
#include <cassert> // for assert
#include <sstream> // for std::ostringstream
#include <utility> // for std::exchange

#include <PlayRho/Common/DynamicMemory.hpp>
#include <PlayRho/Common/Math.hpp> // for ToSigned
#include <PlayRho/Common/PoolMemoryResource.hpp>

namespace playrho::pmr {

static_assert(PoolMemoryResource::Options{}.reserveBuffers == 0u);
static_assert(PoolMemoryResource::Options{}.reserveBytes == 0u);
static_assert(PoolMemoryResource::Options{}.limitBuffers == static_cast<std::size_t>(-1));

namespace {

PoolMemoryResource::Options Validate(const PoolMemoryResource::Options& options)
{
    if (options.reserveBuffers > options.limitBuffers) {
        throw std::length_error{"pre-allocation would exceed buffers limit"};
    }

    if (options.reserveBytes > PoolMemoryResource::GetMaxNumBytes()) {
        throw std::bad_array_new_length{};
    }
    return options;
}

std::vector<PoolMemoryResource::BufferRecord>
GetBuffers(const PoolMemoryResource::Options& options, memory_resource* upstream)
{
    std::vector<PoolMemoryResource::BufferRecord> buffers;
    buffers.resize(options.reserveBuffers);
    for (auto i = std::size_t{0}; i < options.reserveBuffers; ++i) {
        auto* p = static_cast<void*>(nullptr);
        try {
            p = upstream->allocate(options.reserveBytes, alignof(std::max_align_t)); // could throw!
        }
        catch (...) {
            // Attempt to cleanup by deallocating any memory already allocated...
            for (--i; i < std::size_t(-1); --i) {
                auto& buffer = buffers[i];
                try {
                    upstream->deallocate(buffer.data(), buffer.size(), buffer.alignment());
                }
                catch (...) {
                    std::terminate();
                }
            }
            throw; // rethrow original exception
        }
        buffers[i].assign(p, options.reserveBytes, alignof(std::max_align_t));
    }
    return buffers;
}

}

std::size_t PoolMemoryResource::GetMaxNumBytes() noexcept
{
    return static_cast<std::size_t>(std::numeric_limits<ssize_t>::max());
}

PoolMemoryResource::PoolMemoryResource() noexcept:
    m_options{Validate(Options{})},
    m_upstream{new_delete_resource()},
    m_buffers{GetBuffers(m_options, m_upstream)}
{
    // Intentionally empty
}

PoolMemoryResource::PoolMemoryResource(const Options& options, memory_resource* upstream)
    : m_options{Validate(options)},
      m_upstream{upstream ? upstream : new_delete_resource()},
      m_buffers{GetBuffers(m_options, m_upstream)}
{
    // Intentionally empty
}

PoolMemoryResource::PoolMemoryResource(const PoolMemoryResource& other)
    : m_options{other.m_options},
      m_upstream{other.m_upstream},
      m_buffers{GetBuffers(m_options, m_upstream)}
{
    // Intentionally empty
}

PoolMemoryResource::PoolMemoryResource(PoolMemoryResource&& other) noexcept:
    m_options(std::exchange(other.m_options, Options())),
    m_upstream(std::exchange(other.m_upstream, new_delete_resource())),
    m_buffers(std::exchange(other.m_buffers, {}))
{
}

PoolMemoryResource::~PoolMemoryResource() noexcept
{
    for (auto&& buffer: m_buffers) {
        // Deallocate should not throw in this context of having previously allocated
        // this memory. If it does, fail fast! It signifies a significant logic error.
        // In which case, this code and that of the upstream resource needs to be
        // inspected and likely needs to be updated.
        m_upstream->deallocate(buffer.data(), buffer.size(), buffer.alignment());
        buffer = BufferRecord{};
    }
}

PoolMemoryResource& PoolMemoryResource::operator=(PoolMemoryResource&& other) noexcept
{
    if (this != &other) {
        m_options = std::exchange(other.m_options, Options());
        m_upstream = std::exchange(other.m_upstream, new_delete_resource());
        m_buffers = std::exchange(other.m_buffers, {});
    }
    return *this;
}

PoolMemoryResource::Stats PoolMemoryResource::GetStats() const noexcept
{
    Stats stats;
    stats.numBuffers = m_buffers.size();
    for (const auto& buffer: m_buffers) {
        const auto bytes = buffer.size();
        stats.maxBytes = std::max(stats.maxBytes, bytes);
        stats.totalBytes += bytes;
        if (buffer.is_allocated()) {
            ++stats.allocatedBuffers;
        }
    }
    return stats;
}

void *PoolMemoryResource::do_allocate(std::size_t num_bytes, std::size_t alignment)
{
    if (num_bytes > GetMaxNumBytes()) {
        throw std::bad_array_new_length{};
    }
    for (auto&& buffer: m_buffers) {
        if (!buffer.is_allocated()) {
            const auto fit = (num_bytes <= buffer.size()) && (alignment <= buffer.alignment());
            if (!fit && !m_options.releasable) {
                continue;
            }
            if (!fit) {
                m_upstream->deallocate(buffer.data(), buffer.size(), buffer.alignment());
                buffer = BufferRecord{};
                auto* p = m_upstream->allocate(num_bytes, alignment); // could throw!
                buffer.assign(p, num_bytes, alignment);
            }
            buffer.allocate();
            return buffer.data();
        }
    }
    if (m_buffers.size() >= m_options.limitBuffers) {
        std::ostringstream os;
        os << "allocate ";
        os << num_bytes;
        os << "b, aligned to ";
        os << alignment;
        os << "b, would exceed buffer count limit, stats=";
        os << GetStats();
        throw std::length_error{os.str()};
    }
    auto& buffer = m_buffers.emplace_back(); // could throw!
    auto* p = static_cast<void*>(nullptr);
    try {
        p = m_upstream->allocate(num_bytes, alignment); // could throw!
    }
    catch (...) {
        m_buffers.pop_back();
        throw;
    }
    buffer.assign(p, num_bytes, alignment);
    buffer.allocate();
    return buffer.data();
}

void PoolMemoryResource::do_deallocate(void *p, std::size_t num_bytes, std::size_t alignment)
{
    const auto it = std::find_if(begin(m_buffers), end(m_buffers), [p](const auto& buffer){
        return p == buffer.data();
    });
    if (it == end(m_buffers)) {
        throw std::logic_error{"called to deallocate block not known by this allocator"};
    }
    if (num_bytes > it->size()) {
        throw std::logic_error{"deallocation size greater-than size originally allocated"};
    }
    if (alignment > it->alignment()) {
        throw std::logic_error{"deallocation alignment greater-than alignment originally allocated"};
    }
    if (it->is_allocated()) {
        it->deallocate();
    }
}

bool PoolMemoryResource::do_is_equal(const playrho::pmr::memory_resource &other) const noexcept
{
    return &other == this;
}

std::ostream& operator<<(std::ostream& os, const PoolMemoryResource::Stats& stats)
{
    os << "{";
    os << "total-bytes=" << stats.totalBytes;
    os << ", num-buffers=" << stats.numBuffers;
    os << ", allocated-bufs=" << stats.allocatedBuffers;
    os << "}";
    return os;
}

}
