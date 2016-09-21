//
//  Fixture.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/21/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Fixture.h>

using namespace box2d;

TEST(Fixture, ByteSizeIs64)
{
	EXPECT_EQ(sizeof(Fixture), size_t(64));
}