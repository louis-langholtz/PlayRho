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
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/SimplexCache.hpp>

using namespace box2d;

TEST(SimplexCache, ByteSizeIs12)
{
	EXPECT_EQ(sizeof(SimplexCache), size_t(12));
}

TEST(SimplexCache, IndexPairListByteSizeIs7)
{
	EXPECT_EQ(sizeof(IndexPairList), size_t(7));
}

TEST(SimplexCache, DefaultInit)
{
	SimplexCache foo;
	EXPECT_EQ(decltype(foo.GetNumIndices()){0}, foo.GetNumIndices());
	EXPECT_FALSE(foo.IsMetricSet());
}

TEST(SimplexCache, InitializingConstructor)
{
	{
		const auto metric = float_t(.3);
		const auto indices = IndexPairList{};
		SimplexCache foo{metric, indices};
		
		EXPECT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){0});
		EXPECT_TRUE(foo.IsMetricSet());
		EXPECT_EQ(foo.GetMetric(), metric);
	}
	{
		const auto ip0 = IndexPair{0, 0};
		const auto ip1 = IndexPair{1, 0};
		const auto ip2 = IndexPair{4, 3};
		const auto metric = float_t(-1.4);
		SimplexCache foo{metric, IndexPairList{ip0, ip1, ip2}};
		
		EXPECT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){3});
		EXPECT_EQ(foo.GetIndexPair(0), ip0);
		EXPECT_EQ(foo.GetIndexPair(1), ip1);
		EXPECT_EQ(foo.GetIndexPair(2), ip2);
		EXPECT_TRUE(foo.IsMetricSet());
		EXPECT_EQ(foo.GetMetric(), metric);
	}
}

TEST(SimplexCache, Assignment)
{
	const auto metric = float_t(.3);
	const auto indices = IndexPairList{};
	SimplexCache foo{metric, indices};
	
	ASSERT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){0});
	ASSERT_TRUE(foo.IsMetricSet());
	ASSERT_EQ(foo.GetMetric(), metric);
	
	const auto ip0 = IndexPair{0, 0};
	const auto ip1 = IndexPair{1, 0};
	const auto ip2 = IndexPair{4, 3};
	const auto roo_metric = float_t(-1.4);
	SimplexCache roo{roo_metric, IndexPairList{ip0, ip1, ip2}};

	foo = roo;
	
	EXPECT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){3});
	EXPECT_EQ(foo.GetIndexPair(0), ip0);
	EXPECT_EQ(foo.GetIndexPair(1), ip1);
	EXPECT_EQ(foo.GetIndexPair(2), ip2);
	EXPECT_TRUE(foo.IsMetricSet());
	EXPECT_EQ(foo.GetMetric(), roo_metric);
}
