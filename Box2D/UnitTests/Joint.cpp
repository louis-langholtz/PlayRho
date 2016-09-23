//
//  Joint.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/23/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Joints/Joint.h>

using namespace box2d;

TEST(Joint, ByteSizeIs136)
{
	EXPECT_EQ(sizeof(Joint), size_t(136));
}