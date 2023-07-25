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

#include <future> // for std::async

// Macro to avoid ThreadLocalAllocator<T, TestMemoryResource> Windows shared-builds linkage errors.
#define PLAYRHO_EXPORT
#include <PlayRho/Common/ThreadLocalAllocator.hpp>

using namespace playrho;

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

class TestMemoryResource: public pmr::memory_resource
{
    void *do_allocate(std::size_t bytes, std::size_t alignment) override
    {
        throw TestAllocateArgs{bytes, alignment};
    }

    void do_deallocate(void *p, std::size_t bytes, std::size_t alignment) override
    {
        throw TestDeallocateArgs{p, bytes, alignment};
    }

    bool do_is_equal(const playrho::pmr::memory_resource &) const noexcept override
    {
        return false;
    }
};

class NewDeleteResource: public pmr::memory_resource
{
public:
    void *do_allocate(std::size_t bytes, std::size_t alignment) override
    {
        return pmr::new_delete_resource()->allocate(bytes, alignment);
    }

    void do_deallocate(void *p, std::size_t bytes, std::size_t alignment) override
    {
        pmr::new_delete_resource()->deallocate(p, bytes, alignment);
    }

    bool do_is_equal(const playrho::pmr::memory_resource &other) const noexcept override
    {
        return pmr::new_delete_resource()->is_equal(other);
    }
};

}

TEST(ThreadLocalAllocator, max_size)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    EXPECT_EQ(allocator_type::max_size(), (std::numeric_limits<std::size_t>::max() / sizeof(value_type)));
}

TEST(ThreadLocalAllocator, resource)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    auto r = static_cast<pmr::memory_resource*>(nullptr);
    EXPECT_NO_THROW(r = allocator_type().resource());
    ASSERT_NE(nullptr, r);
    const auto promoted = dynamic_cast<TestMemoryResource*>(r);
    EXPECT_NE(nullptr, promoted);
    const auto& resource_type_info = typeid(*r);
    const auto& expected_type_info = typeid(TestMemoryResource);
    EXPECT_TRUE(resource_type_info == expected_type_info);
    auto thread_r = static_cast<pmr::memory_resource*>(nullptr);
    (void) std::async(std::launch::async, [&thread_r](){
        thread_r = allocator_type().resource();
    });
    EXPECT_NE(nullptr, thread_r);
    EXPECT_NE(r, thread_r);
    EXPECT_EQ(r, allocator_type().resource());
}

TEST(ThreadLocalAllocator, allocate_throws_bad_array_new_length)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    allocator_type allocator;
    auto p = static_cast<value_type*>(nullptr);
    EXPECT_THROW(p = allocator.allocate(std::numeric_limits<std::size_t>::max()), std::bad_array_new_length);
    EXPECT_EQ(p, nullptr);
}

TEST(ThreadLocalAllocator, allocate)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    allocator_type allocator;
    constexpr auto count = 42u;
    auto p = static_cast<value_type*>(nullptr);
    try {
        p = allocator.allocate(count);
        FAIL();
    }
    catch (const TestAllocateArgs& args) {
        EXPECT_EQ(args.bytes, count * sizeof(value_type));
        EXPECT_EQ(args.alignment, alignof(value_type));
    }
    EXPECT_EQ(p, nullptr);
}

TEST(ThreadLocalAllocator, deallocate)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    allocator_type allocator;
    constexpr auto count = 42u;
    auto p = reinterpret_cast<value_type*>(0x31);
    try {
        allocator.deallocate(p, count);
        FAIL();
    }
    catch (const TestDeallocateArgs& args) {
        EXPECT_EQ(args.pointer, p);
        EXPECT_EQ(args.bytes, count * sizeof(value_type));
        EXPECT_EQ(args.alignment, alignof(value_type));
    }
}

TEST(ThreadLocalAllocator, allocate_deallocate)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, NewDeleteResource>;
    allocator_type allocator;
    constexpr auto count = 42u;
    auto p = static_cast<value_type*>(nullptr);
    EXPECT_NO_THROW(p = allocator.allocate(count));
    EXPECT_NE(p, nullptr);
    EXPECT_NO_THROW(allocator.deallocate(p, count));
}

TEST(ThreadLocalAllocator, Equals)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    allocator_type allocator_a;
    allocator_type allocator_b;
    EXPECT_TRUE(allocator_a == allocator_a);
    EXPECT_TRUE(allocator_a == allocator_b);
}

TEST(ThreadLocalAllocator, NotEquals)
{
    using value_type = int;
    using allocator_type = ThreadLocalAllocator<value_type, TestMemoryResource>;
    allocator_type allocator_a;
    allocator_type allocator_b;
    EXPECT_FALSE(allocator_a != allocator_a);
    EXPECT_FALSE(allocator_a != allocator_b);
}
