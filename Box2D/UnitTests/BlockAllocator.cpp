/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
	EXPECT_EQ(sizeof(BlockAllocator), size_t(136));
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
