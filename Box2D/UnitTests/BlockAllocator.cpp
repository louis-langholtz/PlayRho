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

TEST(BlockAllocator, DefaultInit)
{
	BlockAllocator allocator;
	
	EXPECT_LE(sizeof(allocator), size_t(136));
}