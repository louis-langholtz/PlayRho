//
//  SimplexCache.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/8/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Distance.h>
#include <Box2D/Collision/SimplexCache.hpp>

using namespace box2d;

TEST(SimplexCache, Init)
{
	SimplexCache foo;
	EXPECT_EQ(decltype(foo.GetCount()){0}, foo.GetCount());
	EXPECT_EQ(false, foo.IsMetricSet());
}

TEST(SimplexCache, SetGetMetric)
{
	SimplexCache foo;
	EXPECT_EQ(false, foo.IsMetricSet());
	const auto metric = float_t(6.12);
	foo.SetMetric(metric);
	EXPECT_EQ(true, foo.IsMetricSet());
	EXPECT_EQ(metric, foo.GetMetric());
}

TEST(SimplexCache, AddIndex)
{
	SimplexCache foo;
	EXPECT_EQ(decltype(foo.GetCount()){0}, foo.GetCount());

	const IndexPair ip0{3, 1};
	foo.AddIndex(ip0);
	EXPECT_EQ(decltype(foo.GetCount()){1}, foo.GetCount());
	EXPECT_EQ(ip0, foo.GetIndexPair(0));
	
	const IndexPair ip1{IndexPair::InvalidIndex, IndexPair::InvalidIndex};
	foo.AddIndex(ip1);
	EXPECT_EQ(decltype(foo.GetCount()){2}, foo.GetCount());
	EXPECT_EQ(ip0, foo.GetIndexPair(0));
	EXPECT_EQ(ip1, foo.GetIndexPair(1));

	const auto invalid_index = IndexPair::InvalidIndex;
	const IndexPair ip2{2, invalid_index};
	EXPECT_EQ(invalid_index, ip2.b);
	foo.AddIndex(ip2);
	EXPECT_EQ(decltype(foo.GetCount()){3}, foo.GetCount());
	EXPECT_EQ(ip0, foo.GetIndexPair(0));
	EXPECT_EQ(ip1, foo.GetIndexPair(1));
	EXPECT_EQ(ip2, foo.GetIndexPair(2));
}

TEST(SimplexCache, ClearIndices)
{
	SimplexCache foo;
	EXPECT_EQ(decltype(foo.GetCount()){0}, foo.GetCount());
	
	const IndexPair ip0{3, 1};
	foo.AddIndex(ip0);
	EXPECT_EQ(decltype(foo.GetCount()){1}, foo.GetCount());
	EXPECT_EQ(ip0, foo.GetIndexPair(0));

	foo.ClearIndices();
	EXPECT_EQ(decltype(foo.GetCount()){0}, foo.GetCount());
}