//
//  Vec2.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/3/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/Math.h>

using namespace box2d;

TEST(Vec2, max_size) {
	Vec2 vector;
	EXPECT_EQ(Vec2::size_type{2}, vector.max_size());
}

TEST(Vec2, Constructor) {
	Vec2 vector{float_t{5}, float_t{-3}};
	EXPECT_EQ(float_t{5}, vector.x);
	EXPECT_EQ(float_t{-3}, vector.y);
}

TEST(Vec2, Indexing) {
	Vec2 vector{float_t{5}, float_t{-3}};
	EXPECT_EQ(float_t{5}, vector[0]);
	EXPECT_EQ(float_t{-3}, vector[1]);
	vector[0] = float_t{4};
	EXPECT_EQ(float_t{4}, vector[0]);
	vector[1] = float_t{-2};
	EXPECT_EQ(float_t{-2}, vector[1]);
}
