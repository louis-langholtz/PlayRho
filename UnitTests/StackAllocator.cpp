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

#include <PlayRho/StackAllocator.hpp>

#include <chrono>

using namespace playrho;

TEST(StackAllocator, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
#if defined(_WIN32) && !defined(_WIN64)
    EXPECT_EQ(sizeof(StackAllocator), std::size_t(32));
#else
    EXPECT_EQ(sizeof(StackAllocator), std::size_t(64));
#endif
}

TEST(StackAllocator, DefaultConstruction)
{
    const auto config = StackAllocator::GetDefaultConf();
    ASSERT_EQ(config.preallocation_size, StackAllocator::Conf::DefaultPreallocationSize);
    ASSERT_EQ(config.allocation_records, StackAllocator::Conf::DefaultAllocationRecords);

    StackAllocator foo;
    EXPECT_EQ(foo.GetPreallocatedSize(), config.preallocation_size);
    EXPECT_EQ(foo.GetMaxEntries(), config.allocation_records);
    EXPECT_EQ(foo.GetIndex(), decltype(foo.GetIndex()){0});
    EXPECT_EQ(foo.GetAllocation(), decltype(foo.GetAllocation()){0});
}

static inline bool is_aligned(void* ptr, std::size_t siz)
{
    return reinterpret_cast<std::uintptr_t>(ptr) % siz == 0;
}

TEST(StackAllocator, aligns_data)
{
    StackAllocator foo;

    const auto p_char1 = static_cast<char*>(foo.Allocate(sizeof(char)));
    
    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){1});
    EXPECT_EQ(foo.GetIndex(), sizeof(char));
    EXPECT_EQ(foo.GetAllocation(), sizeof(char));

    const auto p_char2 = static_cast<char*>(foo.Allocate(sizeof(char)));

    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){2});
    EXPECT_EQ(foo.GetIndex(), sizeof(char) + sizeof(char));
    EXPECT_EQ(foo.GetAllocation(), sizeof(char) + sizeof(char));

    const auto p_int = static_cast<int*>(foo.Allocate(sizeof(int)));

    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){3});
    EXPECT_EQ(foo.GetIndex(), foo.GetAllocation());
    EXPECT_EQ(foo.GetIndex(), sizeof(int) + sizeof(int));
    EXPECT_EQ(foo.GetAllocation(), sizeof(int) + sizeof(int));

    EXPECT_TRUE(is_aligned(p_char1, alignof(char)));
    EXPECT_TRUE(is_aligned(p_char2, alignof(char)));
    EXPECT_TRUE(is_aligned(p_int, alignof(int)));

    *p_char1 = 'W';
    *p_int = 5;
    
    EXPECT_EQ(*p_char1, 'W');
    EXPECT_EQ(*p_int, 5);
    
    foo.Free(p_int);
    foo.Free(p_char2);
    foo.Free(p_char1);
}

TEST(StackAllocator, uses_malloc_when_full)
{
    StackAllocator foo;    
    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){0});
    
    const auto preallocated_size = foo.GetPreallocatedSize();
    const auto p = foo.Allocate(preallocated_size);
    
    EXPECT_TRUE(p != nullptr);
    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){1});
    EXPECT_EQ(foo.GetIndex(), preallocated_size);
    EXPECT_EQ(foo.GetAllocation(), preallocated_size);
    
    const auto q = foo.Allocate(sizeof(double));
    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){2});
    EXPECT_EQ(foo.GetIndex(), preallocated_size);
    EXPECT_GT(foo.GetAllocation(), preallocated_size);

    foo.Free(q);
    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){1});

    foo.Free(p);
    EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){0});
}

TEST(StackAllocator, ZeroConfig)
{
    constexpr auto allocationRecords = 0u;
    constexpr auto allocationSize = 0u;
    StackAllocator foo{StackAllocator::Conf{allocationSize, allocationRecords}};
    EXPECT_EQ(foo.GetIndex(), 0u);
    EXPECT_EQ(foo.GetPreallocatedSize(), allocationSize);
    EXPECT_EQ(foo.GetMaxEntries(), allocationRecords);
    EXPECT_EQ(foo.Allocate(0u), nullptr);
    EXPECT_NO_THROW(foo.Free(nullptr));
}
