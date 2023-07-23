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

#include <cstdlib> // for std::exit
#include <iostream> // for std::cerr
#include <memory> // for std::unique_ptr
#include <vector>

#include <PlayRho/Common/MemoryResource.hpp>

using namespace playrho;

namespace playrho::pmr {
namespace {

struct free_deleter
{
    void operator()(void* p) const
    {
        std::free(p);
    }
};

using unique_ptr = std::unique_ptr<void, free_deleter>;

struct do_allocate_record
{
    do_allocate_record() = default;

    do_allocate_record(unique_ptr p, std::size_t b, std::size_t a)
        : pointer{std::move(p)}, bytes{b}, alignment{a} {}

    unique_ptr pointer;
    std::size_t bytes{};
    std::size_t alignment{};
};

struct do_deallocate_record
{
    do_deallocate_record() = default;

    do_deallocate_record(void* p, std::size_t b, std::size_t a)
        : pointer{p}, bytes{b}, alignment{a} {}

    void* pointer{};
    std::size_t bytes{};
    std::size_t alignment{};
};

struct do_is_equal_record
{
    do_is_equal_record() = default;
    do_is_equal_record(const memory_resource* r): resource{r} {}

    const memory_resource* resource{};
};

class test_memory_resource: public memory_resource
{
public:
    std::vector<do_allocate_record> do_allocate_calls;
    std::vector<do_deallocate_record> do_deallocate_calls;
    mutable std::vector<do_is_equal_record> do_is_equal_calls;
    
    void *do_allocate(std::size_t bytes, std::size_t alignment) override
    {
        auto pointer = unique_ptr{std::malloc(bytes)};
        auto& back = do_allocate_calls.emplace_back(unique_ptr{}, bytes, alignment);
        back.pointer = std::move(pointer);
        return back.pointer.get();
    }
    
    void do_deallocate(void *p, std::size_t bytes, std::size_t alignment) override
    {
        do_deallocate_calls.emplace_back(p, bytes, alignment);
    }
    
    bool do_is_equal(const memory_resource &other) const noexcept override
    {
        do_is_equal_calls.emplace_back(&other);
        return this == &other;
    }
};

static_assert(!std::is_abstract_v<test_memory_resource>);

}
}

TEST(memory_resource, allocate)
{
    pmr::test_memory_resource object;
    ASSERT_TRUE(object.do_allocate_calls.empty());
    constexpr auto bytes = 42u;
    constexpr auto alignment = 16u;
    void* p = nullptr;
    EXPECT_NO_THROW(p = object.allocate(bytes, alignment));
    ASSERT_EQ(object.do_allocate_calls.size(), 1u);
    EXPECT_EQ(object.do_allocate_calls[0].pointer.get(), p);
    EXPECT_EQ(object.do_allocate_calls[0].bytes, bytes);
    EXPECT_EQ(object.do_allocate_calls[0].alignment, alignment);
    EXPECT_NO_THROW(p = object.allocate(bytes + 1u, alignment * 2));
    ASSERT_EQ(object.do_allocate_calls.size(), 2u);
    EXPECT_EQ(object.do_allocate_calls[1].pointer.get(), p);
    EXPECT_EQ(object.do_allocate_calls[1].bytes, bytes + 1u);
    EXPECT_EQ(object.do_allocate_calls[1].alignment, alignment * 2);
    EXPECT_EQ(object.do_deallocate_calls.size(), 0u);
    EXPECT_EQ(object.do_is_equal_calls.size(), 0u);
}

TEST(memory_resource, deallocate)
{
    pmr::test_memory_resource object;
    ASSERT_TRUE(object.do_deallocate_calls.empty());
    constexpr auto bytes = 42u;
    auto pointerA = std::unique_ptr<void, pmr::free_deleter>{std::malloc(bytes)};
    auto pointerB = std::unique_ptr<void, pmr::free_deleter>{std::malloc(bytes * 2u)};
    constexpr auto alignment = 16u;
    EXPECT_NO_THROW(object.deallocate(pointerA.get(), bytes, alignment));
    ASSERT_EQ(object.do_deallocate_calls.size(), 1u);
    EXPECT_EQ(object.do_deallocate_calls[0].pointer, pointerA.get());
    EXPECT_EQ(object.do_deallocate_calls[0].bytes, bytes);
    EXPECT_EQ(object.do_deallocate_calls[0].alignment, alignment);
    EXPECT_NO_THROW(object.deallocate(pointerB.get(), bytes * 2u, alignment * 2u));
    ASSERT_EQ(object.do_deallocate_calls.size(), 2u);
    EXPECT_EQ(object.do_deallocate_calls[1].pointer, pointerB.get());
    EXPECT_EQ(object.do_deallocate_calls[1].bytes, bytes * 2u);
    EXPECT_EQ(object.do_deallocate_calls[1].alignment, alignment * 2u);
    EXPECT_EQ(object.do_allocate_calls.size(), 0u);
    EXPECT_EQ(object.do_is_equal_calls.size(), 0u);
}

TEST(memory_resource, is_equal)
{
    pmr::test_memory_resource objectA;
    pmr::test_memory_resource objectB;
    auto result = false;
    EXPECT_NO_THROW(result = objectA.is_equal(objectA));
    EXPECT_TRUE(result);
    ASSERT_EQ(objectA.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectA.do_is_equal_calls[0].resource, &objectA);
    EXPECT_EQ(objectA.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectA.do_deallocate_calls.size(), 0u);
    EXPECT_NO_THROW(result = objectB.is_equal(objectA));
    EXPECT_FALSE(result);
    ASSERT_EQ(objectA.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectA.do_is_equal_calls[0].resource, &objectA);
    EXPECT_EQ(objectA.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectA.do_deallocate_calls.size(), 0u);
    ASSERT_EQ(objectB.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectB.do_is_equal_calls[0].resource, &objectA);
    EXPECT_EQ(objectB.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectB.do_deallocate_calls.size(), 0u);
}

TEST(memory_resource, EqualityOperator)
{
    pmr::test_memory_resource objectA;
    pmr::test_memory_resource objectB;
    EXPECT_TRUE(objectA == objectA);
    EXPECT_TRUE(objectB == objectB);
    EXPECT_FALSE(objectA == objectB);
    EXPECT_FALSE(objectB == objectA);
    ASSERT_EQ(objectA.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectA.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectA.do_deallocate_calls.size(), 0u);
    ASSERT_EQ(objectB.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectB.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectB.do_deallocate_calls.size(), 0u);
}

TEST(memory_resource, InequalityOperator)
{
    pmr::test_memory_resource objectA;
    pmr::test_memory_resource objectB;
    EXPECT_FALSE(objectA != objectA);
    EXPECT_FALSE(objectB != objectB);
    EXPECT_TRUE(objectA != objectB);
    EXPECT_TRUE(objectB != objectA);
    ASSERT_EQ(objectA.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectA.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectA.do_deallocate_calls.size(), 0u);
    ASSERT_EQ(objectB.do_is_equal_calls.size(), 1u);
    EXPECT_EQ(objectB.do_allocate_calls.size(), 0u);
    EXPECT_EQ(objectB.do_deallocate_calls.size(), 0u);
}

TEST(memory_resource, new_delete_resource)
{
    using namespace playrho::pmr;
    EXPECT_EQ(new_delete_resource(), new_delete_resource());
    EXPECT_NE(new_delete_resource(), null_memory_resource());
    ASSERT_NE(new_delete_resource(), nullptr);
    EXPECT_TRUE(new_delete_resource()->is_equal(*new_delete_resource()));
    EXPECT_FALSE(new_delete_resource()->is_equal(*null_memory_resource()));
    void* p = nullptr;
    const auto bytes = std::size_t(1u<<2);
    const auto alignment = std::size_t(1u<<3);
#if !defined(__has_include) || !__has_include(<memory_resource>)
    EXPECT_NO_THROW(p = new_delete_resource()->allocate(bytes, alignment));
#else
    try {
        p = new_delete_resource()->allocate(bytes, alignment);
    }
    catch (const std::bad_alloc&) {
        // okay
    }
    catch (...) {
        FAIL() << "expected bad_alloc";
    }
#endif
    if (p) {
        EXPECT_NO_THROW(new_delete_resource()->deallocate(p, bytes, alignment));
    }
    EXPECT_NO_THROW(p = new_delete_resource()->allocate(1u, 1u));
    EXPECT_NE(p, nullptr);
    EXPECT_NO_THROW(new_delete_resource()->deallocate(p, 1u, 1u));
}

TEST(memory_resource, null_memory_resource)
{
    using namespace playrho::pmr;
    EXPECT_EQ(null_memory_resource(), null_memory_resource());
    EXPECT_NE(null_memory_resource(), new_delete_resource());
    ASSERT_NE(null_memory_resource(), nullptr);
    void* p = nullptr;
    ASSERT_THROW(p = null_memory_resource()->allocate(1u, 1u), std::bad_alloc);
    EXPECT_EQ(p, nullptr);
    EXPECT_NO_THROW(null_memory_resource()->deallocate(p, 1u, 1u));
    EXPECT_TRUE(null_memory_resource()->is_equal(*null_memory_resource()));
    ASSERT_NE(new_delete_resource(), nullptr);
    EXPECT_FALSE(null_memory_resource()->is_equal(*new_delete_resource()));
}

TEST(memory_resource, get_default_resource)
{
    using namespace playrho::pmr;
    EXPECT_EQ(get_default_resource(), new_delete_resource());
}

TEST(memory_resource_DeathTest, set_default_resource)
{
    using namespace playrho::pmr;
    EXPECT_EXIT({
        auto old = set_default_resource(null_memory_resource());
        if (old != new_delete_resource()) {
            std::cerr << "unexpected old resource after set\n";
        }
        if (get_default_resource() != null_memory_resource()) {
            std::cerr << "unexpected get_default_resource() return value\n";
        }
        old = set_default_resource(nullptr);
        if (old != null_memory_resource()) {
            std::cerr << "unexpected old resource after reset\n";
        }
        std::exit(get_default_resource() != new_delete_resource());
    }, testing::ExitedWithCode(0), "");
}
