/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "gtest/gtest.h"
#include <Box2D/Common/BlockAllocator.hpp>

using namespace box2d;

TEST(BlockAllocator, ByteSizeIs136)
{
    EXPECT_EQ(sizeof(BlockAllocator), std::size_t(136));
}

TEST(BlockAllocator, Equals)
{
    BlockAllocator a;
    BlockAllocator b;
    
    EXPECT_TRUE(a == a);
    EXPECT_TRUE(b == b);
    EXPECT_FALSE(a == b);
}

TEST(BlockAllocator, NotEquals)
{
    BlockAllocator a;
    BlockAllocator b;
    
    EXPECT_FALSE(a != a);
    EXPECT_FALSE(b != b);
    EXPECT_TRUE(a != b);
}

TEST(BlockAllocator, Allocate_and_Clear)
{
    BlockAllocator allocator;
    ASSERT_EQ(allocator.GetChunkCount(), decltype(allocator.GetChunkCount()){0});

    auto ptr = allocator.Allocate(1);
    EXPECT_EQ(allocator.GetChunkCount(), decltype(allocator.GetChunkCount()){1});
    EXPECT_NE(ptr, nullptr);
    
    *static_cast<char*>(ptr) = 'B';
    
    EXPECT_EQ(*static_cast<char*>(ptr), 'B');
    
    allocator.Clear();
    
    EXPECT_EQ(allocator.GetChunkCount(), decltype(allocator.GetChunkCount()){0});
}

static inline bool is_aligned(void* ptr, std::size_t siz)
{
    return reinterpret_cast<std::uintptr_t>(static_cast<void*>(ptr)) % siz == 0;
}

TEST(BlockAllocator, aligns_data)
{
    BlockAllocator foo;
    
    const auto p_char1 = static_cast<char*>(foo.Allocate(sizeof(char)));
    const auto p_int = static_cast<int*>(foo.Allocate(sizeof(int)));
    const auto p_char2 = static_cast<char*>(foo.Allocate(sizeof(char)));
    
    EXPECT_TRUE(is_aligned(p_char1, alignof(char)));
    EXPECT_TRUE(is_aligned(p_char2, alignof(char)));
    EXPECT_TRUE(is_aligned(p_int, alignof(int)));
    
    *p_char1 = 'W';
    *p_int = 5;
    
    EXPECT_EQ(*p_char1, 'W');
    EXPECT_EQ(*p_int, 5);
    
    foo.Free(p_int, sizeof(int));
    foo.Free(p_char2, sizeof(char));
    foo.Free(p_char1, sizeof(char));
}
