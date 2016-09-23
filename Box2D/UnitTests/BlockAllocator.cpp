//
//  BlockAllocator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/21/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/BlockAllocator.h>

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