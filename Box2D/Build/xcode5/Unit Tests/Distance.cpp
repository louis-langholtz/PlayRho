//
//  Distance.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/8/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Distance.h>

using namespace box2d;

TEST(Distance, OverlappedCircles)
{
	SimplexCache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{2, 2};
	const auto pos2 = Vec2{2, 2};
	DistanceProxy dp1{1, pos1};
	DistanceProxy dp2{1, pos2};

	const auto output = Distance(cache, dp1, xf1, dp2, xf2);
	
	EXPECT_EQ(output.witnessPoints.a, pos1);
	EXPECT_EQ(output.witnessPoints.b, pos1);
	EXPECT_EQ(decltype(output.iterations){0}, output.iterations);
	
	EXPECT_EQ(cache.GetCount(), decltype(cache.GetCount()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
}