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

#include "UnitTests.hpp"

#include <exception> // for std::set_terminate
#include <stdexcept> // for std::logic_error

#include <PlayRho/Common/PoolMemoryResource.hpp>

using namespace playrho::pmr;

namespace {

struct TestAllocateArgs
{
    std::size_t bytes;
    std::size_t alignment;
};

struct TestDeallocateArgs
{
    void* pointer;
    std::size_t bytes;
    std::size_t alignment;
};

enum class TestMemoryResourceBehavior {
    ReturnNullptr,
    ThrowArgs,
};

class TestMemoryResource: public memory_resource
{
public:
    TestMemoryResource(std::function<void*(std::size_t, std::size_t)> alloc,
                       std::function<void(void*, std::size_t, std::size_t)> dealloc):
    on_allocate(std::move(alloc)), on_deallocate(std::move(dealloc)) {}

    void *do_allocate(std::size_t bytes, std::size_t alignment) override
    {
        return on_allocate(bytes, alignment);
    }

    void do_deallocate(void *p, std::size_t bytes, std::size_t alignment) override
    {
        on_deallocate(p, bytes, alignment);
    }

    bool do_is_equal(const playrho::pmr::memory_resource &other) const noexcept override
    {
        return &other == this;
    }

private:
    std::function<void*(std::size_t, std::size_t)> on_allocate;
    std::function<void(void*, std::size_t, std::size_t)> on_deallocate;
};

}

TEST(PoolMemoryResource_DeathTest, DestructorTerminatesOnDeallocThrow)
{
    auto CauseDestructorFailure = [](){
        static constexpr auto byte_size = 4u;
        static constexpr auto align_size = 4u;
        auto upstream = TestMemoryResource{[](std::size_t, std::size_t){
            return static_cast<void*>(nullptr);
        }, [](void *p, std::size_t bytes, std::size_t alignment){
            throw TestDeallocateArgs{p, bytes, alignment};
        }};
        auto opts = PoolMemoryResource::Options{};
        PoolMemoryResource object{opts, &upstream};
        (void) object.do_allocate(byte_size, align_size);
    };
    constexpr auto exit_value = 42;
    constexpr auto exit_message = "caught TestDeallocateArgs\n";
    EXPECT_EXIT({
        std::set_terminate([](){
            const auto ex = std::current_exception();
            try {
                if (ex) {
                    std::rethrow_exception(ex);
                }
                std::cerr << "terminate called without exception\n";
            }
            catch (const TestDeallocateArgs& ex) {
                std::cerr << exit_message;
            }
            catch (...) {
                std::cerr << "Unhandled unexpected exception\n";
            }
            std::cerr << std::flush;
            _exit(exit_value);
        });
        CauseDestructorFailure();
        std::exit(0);
    }, testing::ExitedWithCode(exit_value), exit_message);
}

TEST(PoolMemoryResource_DeathTest, ConstructionTerminatesOnDeallocThrow)
{
    auto CauseConstructionTermination = [](){
        auto nalloc = 0;
        auto upstream = TestMemoryResource{[&nalloc](std::size_t s, std::size_t a){
            if (nalloc < 1) {
                ++nalloc;
                return new_delete_resource()->allocate(s, a);
            }
            throw TestAllocateArgs{s, a};
        }, [](void *p, std::size_t bytes, std::size_t alignment){
            throw TestDeallocateArgs{p, bytes, alignment};
        }};
        auto opts = PoolMemoryResource::Options{};
        opts.reserveBuffers = 2u;
        PoolMemoryResource object{opts, &upstream};
    };
    constexpr auto exit_value = 42;
    constexpr auto exit_message = "caught TestDeallocateArgs\n";
    EXPECT_EXIT({
        std::set_terminate([](){
            const auto ex = std::current_exception();
            try {
                if (ex) {
                    std::rethrow_exception(ex);
                }
                std::cerr << "terminate called without exception\n";
            }
            catch (const TestDeallocateArgs& ex) {
                std::cerr << exit_message;
            }
            catch (...) {
                std::cerr << "Unhandled unexpected exception\n";
            }
            std::cerr << std::flush;
            std::exit(exit_value);
        });
        CauseConstructionTermination();
        std::exit(0);
    }, testing::ExitedWithCode(exit_value), exit_message);
}

TEST(PoolMemoryResource_Options, DefaultConstruction)
{
    const PoolMemoryResource::Options object;
    EXPECT_EQ(object.reserveBuffers, 0u);
    EXPECT_EQ(object.reserveBytes, 0u);
    EXPECT_EQ(object.limitBuffers, static_cast<std::size_t>(-1));
}

TEST(PoolMemoryResource_Options, Equality)
{
    EXPECT_TRUE(PoolMemoryResource::Options() == PoolMemoryResource::Options());
    constexpr auto reserveBuffers = 11u;
    constexpr auto reserveBytes = 42u;
    constexpr auto limitBuffers = 12u;
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers, reserveBytes, limitBuffers};
        EXPECT_TRUE(a == a);
        EXPECT_TRUE(a == b);
        EXPECT_TRUE(b == a);
        EXPECT_TRUE(b == b);
    }
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers + 1u, reserveBytes, limitBuffers};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers, reserveBytes + 1u, limitBuffers};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers, reserveBytes, limitBuffers + 1u};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
}

TEST(PoolMemoryResource_Options, Inequality)
{
    EXPECT_FALSE(PoolMemoryResource::Options() != PoolMemoryResource::Options());
    constexpr auto reserveBuffers = 11u;
    constexpr auto reserveBytes = 42u;
    constexpr auto limitBuffers = 12u;
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers, reserveBytes, limitBuffers};
        EXPECT_FALSE(a != a);
        EXPECT_FALSE(a != b);
        EXPECT_FALSE(b != a);
        EXPECT_FALSE(b != b);
    }
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers + 1u, reserveBytes, limitBuffers};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers, reserveBytes + 1u, limitBuffers};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
    {
        const PoolMemoryResource::Options a{reserveBuffers, reserveBytes, limitBuffers};
        const PoolMemoryResource::Options b{reserveBuffers, reserveBytes, limitBuffers + 1u};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
}

TEST(PoolMemoryResource_Stats, DefaultConstruction)
{
    const PoolMemoryResource::Stats object;
    EXPECT_EQ(object.numBuffers, 0u);
    EXPECT_EQ(object.maxBytes, 0u);
    EXPECT_EQ(object.totalBytes, 0u);
    EXPECT_EQ(object.allocatedBuffers, 0u);
}

TEST(PoolMemoryResource_Stats, Equality)
{
    EXPECT_TRUE(PoolMemoryResource::Stats() == PoolMemoryResource::Stats());
    constexpr auto numBuffers = 11u;
    constexpr auto maxBytes = 42u;
    constexpr auto totalBytes = 12u;
    constexpr auto allocatedBuffers = 8u;
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        EXPECT_TRUE(a == a);
        EXPECT_TRUE(a == b);
        EXPECT_TRUE(b == a);
        EXPECT_TRUE(b == b);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers + 1u, maxBytes, totalBytes, allocatedBuffers};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes + 1u, totalBytes, allocatedBuffers};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes, totalBytes + 1u, allocatedBuffers};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes, totalBytes, allocatedBuffers + 1u};
        EXPECT_FALSE(a == b);
        EXPECT_FALSE(b == a);
    }
}

TEST(PoolMemoryResource_Stats, Inequality)
{
    EXPECT_FALSE(PoolMemoryResource::Stats() != PoolMemoryResource::Stats());
    constexpr auto numBuffers = 11u;
    constexpr auto maxBytes = 42u;
    constexpr auto totalBytes = 12u;
    constexpr auto allocatedBuffers = 8u;
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        EXPECT_FALSE(a != a);
        EXPECT_FALSE(a != b);
        EXPECT_FALSE(b != a);
        EXPECT_FALSE(b != b);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers + 1u, maxBytes, totalBytes, allocatedBuffers};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes + 1u, totalBytes, allocatedBuffers};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes, totalBytes + 1u, allocatedBuffers};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
    {
        const PoolMemoryResource::Stats a{numBuffers, maxBytes, totalBytes, allocatedBuffers};
        const PoolMemoryResource::Stats b{numBuffers, maxBytes, totalBytes, allocatedBuffers + 1u};
        EXPECT_TRUE(a != b);
        EXPECT_TRUE(b != a);
    }
}

TEST(PoolMemoryResource, DefaultConstruction)
{
    const PoolMemoryResource object;
    EXPECT_EQ(object.GetOptions(), PoolMemoryResource::Options());
    EXPECT_EQ(object.GetStats(), PoolMemoryResource::Stats());
}

TEST(PoolMemoryResource, ConstructWithMoreReserveBuffersThanLimit)
{
    PoolMemoryResource::Options opts;
    opts.limitBuffers = 0u;
    opts.reserveBuffers = opts.limitBuffers + 1u;
    EXPECT_THROW(PoolMemoryResource(opts, new_delete_resource()), std::length_error);
}

TEST(PoolMemoryResource, ConstructWithTooManyReserveBytes)
{
    PoolMemoryResource::Options opts;
    opts.reserveBytes = PoolMemoryResource::GetMaxNumBytes() + 1u;
    EXPECT_THROW(PoolMemoryResource(opts, new_delete_resource()), std::bad_array_new_length);
}

TEST(PoolMemoryResource, ConstructReserveBuffersWithNullResource)
{
    PoolMemoryResource::Options opts;
    opts.reserveBuffers = 2u;
    EXPECT_THROW(PoolMemoryResource(opts, null_memory_resource()), std::bad_alloc);
}

TEST(PoolMemoryResource, do_allocate_throws_bad_array_new_length)
{
    PoolMemoryResource object;
    EXPECT_THROW(object.do_allocate(PoolMemoryResource::GetMaxNumBytes() + 1u, 1u),
                 std::bad_array_new_length);
}

TEST(PoolMemoryResource, do_allocate_throws_length_error)
{
    PoolMemoryResource object{PoolMemoryResource::Options{0u, 0u, 0u}, new_delete_resource()};
    EXPECT_THROW(object.do_allocate(1u, 1u), std::length_error);
}

TEST(PoolMemoryResource, do_allocate_unchanged_on_throw)
{
    auto upstream = TestMemoryResource{
        [](std::size_t bytes, std::size_t alignment) -> void* {
            throw TestAllocateArgs{bytes, alignment};
        },
        [](void *p, std::size_t bytes, std::size_t alignment){
            throw TestDeallocateArgs{p, bytes, alignment};
        }
    };
    PoolMemoryResource object{PoolMemoryResource::Options{0u, 0u, 1u}, &upstream};
    {
        const auto stats = object.GetStats();
        ASSERT_EQ(stats.numBuffers, 0u);
        ASSERT_EQ(stats.maxBytes, 0u);
        ASSERT_EQ(stats.totalBytes, 0u);
        ASSERT_EQ(stats.allocatedBuffers, 0u);
    }
    EXPECT_THROW(object.do_allocate(1u, 1u), TestAllocateArgs);
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 0u);
        EXPECT_EQ(stats.maxBytes, 0u);
        EXPECT_EQ(stats.totalBytes, 0u);
        EXPECT_EQ(stats.allocatedBuffers, 0u);
    }
}

TEST(PoolMemoryResource, do_allocate_deallocate_releasable)
{
    PoolMemoryResource::Options opts;
    opts.releasable = true;
    PoolMemoryResource object{opts, new_delete_resource()};
    void* ptrA = nullptr;
    constexpr auto ptrA_num_bytes = 2u;
    constexpr auto ptrA_align_bytes = 1u;
    EXPECT_NO_THROW(ptrA = object.do_allocate(ptrA_num_bytes, ptrA_align_bytes));
    EXPECT_NE(ptrA, nullptr);
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 1u);
        EXPECT_EQ(stats.maxBytes, ptrA_num_bytes);
        EXPECT_EQ(stats.totalBytes, ptrA_num_bytes);
        EXPECT_EQ(stats.allocatedBuffers, 1u);
    }
    EXPECT_NO_THROW(object.do_deallocate(ptrA, ptrA_num_bytes, ptrA_align_bytes));
    EXPECT_NO_THROW(ptrA = object.do_allocate(ptrA_num_bytes * 2u, ptrA_align_bytes));
    EXPECT_NE(ptrA, nullptr);
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 1u);
        EXPECT_EQ(stats.maxBytes, ptrA_num_bytes * 2u);
        EXPECT_EQ(stats.totalBytes, ptrA_num_bytes * 2u);
        EXPECT_EQ(stats.allocatedBuffers, 1u);
    }
}

TEST(PoolMemoryResource, do_allocate_deallocate_nonreleasable)
{
    PoolMemoryResource::Options opts;
    opts.releasable = false;
    PoolMemoryResource object{opts, new_delete_resource()};
    void* ptrA = nullptr;
    EXPECT_NO_THROW(ptrA = object.do_allocate(2u, 1u));
    EXPECT_NE(ptrA, nullptr);
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 1u);
        EXPECT_EQ(stats.maxBytes, 2u);
        EXPECT_EQ(stats.totalBytes, 2u);
        EXPECT_EQ(stats.allocatedBuffers, 1u);
    }
    void* ptrB = nullptr;
    constexpr auto ptrB_num_bytes = 4u;
    constexpr auto ptrB_align_bytes = 4u;
    EXPECT_NO_THROW(ptrB = object.do_allocate(ptrB_num_bytes, ptrB_align_bytes));
    EXPECT_NE(ptrB, nullptr);
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 2u);
        EXPECT_EQ(stats.maxBytes, 4u);
        EXPECT_EQ(stats.totalBytes, 6u);
        EXPECT_EQ(stats.allocatedBuffers, 2u);
    }
    EXPECT_NO_THROW(object.do_deallocate(ptrA, 2u, 1u));
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 2u);
        EXPECT_EQ(stats.maxBytes, 4u);
        EXPECT_EQ(stats.totalBytes, 6u);
        EXPECT_EQ(stats.allocatedBuffers, 1u);
    }
    void* ptrC = nullptr;
    constexpr auto ptrC_num_bytes = 8u;
    constexpr auto ptrC_align_bytes = 4u;
    EXPECT_NO_THROW(ptrC = object.do_allocate(ptrC_num_bytes, ptrC_align_bytes));
    EXPECT_NE(ptrC, nullptr);
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 3u);
        EXPECT_EQ(stats.maxBytes, ptrC_num_bytes);
        EXPECT_EQ(stats.totalBytes, 14u);
        EXPECT_EQ(stats.allocatedBuffers, 2u);
    }
    EXPECT_THROW(object.do_deallocate(nullptr, 0u, 0u), std::logic_error);
    EXPECT_THROW(object.do_deallocate(ptrC, ptrC_num_bytes + 64u, 0u), std::logic_error);
    EXPECT_THROW(object.do_deallocate(ptrC, ptrC_num_bytes, ptrC_align_bytes + 64u), std::logic_error);
    EXPECT_NO_THROW(object.do_deallocate(ptrC, ptrC_num_bytes, ptrC_align_bytes));
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 3u);
        EXPECT_EQ(stats.maxBytes, ptrC_num_bytes);
        EXPECT_EQ(stats.totalBytes, 14u);
        EXPECT_EQ(stats.allocatedBuffers, 1u);
    }
    EXPECT_NO_THROW(object.do_deallocate(ptrB, ptrB_num_bytes, ptrB_align_bytes));
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 3u);
        EXPECT_EQ(stats.maxBytes, ptrC_num_bytes);
        EXPECT_EQ(stats.totalBytes, 14u);
        EXPECT_EQ(stats.allocatedBuffers, 0u);
    }
    EXPECT_NO_THROW(ptrC = object.do_allocate(ptrC_num_bytes * 2u, ptrC_align_bytes));
    {
        const auto stats = object.GetStats();
        EXPECT_EQ(stats.numBuffers, 4u);
        EXPECT_EQ(stats.maxBytes, 16u);
        EXPECT_EQ(stats.totalBytes, 30u);
        EXPECT_EQ(stats.allocatedBuffers, 1u);
    }
}

TEST(PoolMemoryResource, do_is_equal)
{
    PoolMemoryResource objectA, objectB;
    EXPECT_TRUE(objectA.do_is_equal(objectA));
    EXPECT_TRUE(objectB.do_is_equal(objectB));
    EXPECT_FALSE(objectA.do_is_equal(objectB));
    EXPECT_FALSE(objectB.do_is_equal(objectA));
}
