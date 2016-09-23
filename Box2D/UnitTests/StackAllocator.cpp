//
//  StackAllocator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/23/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/StackAllocator.h>

using namespace box2d;

TEST(StackAllocator, ByteSizeIs103200)
{
	EXPECT_EQ(sizeof(StackAllocator), size_t(103200));
}
