//
//  Shape.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/23/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Shapes/Shape.h>

using namespace box2d;

TEST(Shape, ByteSizeIs4)
{
	EXPECT_EQ(sizeof(Shape), size_t(4));
}