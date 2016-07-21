//
//  IndexPair.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/8/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/IndexPair.hpp>

using namespace box2d;

TEST(IndexPair, Init)
{
	IndexPair ip{1, 2};
	EXPECT_EQ(ip.a, 1);
	EXPECT_EQ(ip.b, 2);
}

TEST(IndexPair, Equality)
{
	IndexPair ip1{2, 3};
	IndexPair ip2{2, 3};
	EXPECT_EQ(ip1, ip1);
	EXPECT_EQ(ip1, ip2);
	EXPECT_EQ(ip2, ip1);
	EXPECT_EQ(ip2, ip2);
}

TEST(IndexPair, Inequality)
{
	IndexPair ip1{2, 3};
	IndexPair ip2{1, 0};
	EXPECT_NE(ip1, ip2);
	EXPECT_NE(ip2, ip1);
}

TEST(IndexPair, InvalidIndex)
{
	const auto invalid_index = IndexPair::InvalidIndex;
	IndexPair ip{invalid_index, 2};	
	EXPECT_EQ(invalid_index, ip.a);
	EXPECT_NE(invalid_index, ip.b);
}