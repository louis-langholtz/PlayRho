/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

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