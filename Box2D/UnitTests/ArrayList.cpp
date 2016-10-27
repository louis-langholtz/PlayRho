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
#include <Box2D/Common/ArrayList.hpp>

using namespace box2d;

TEST(ArrayList, DefaultConstruction)
{
	{
		constexpr unsigned max_size = 4;
		ArrayList<int, max_size> list;
		EXPECT_EQ(list.size(), decltype(list.size()){0});
		EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
		EXPECT_TRUE(list.empty());
		EXPECT_EQ(list.begin(), list.end());
	}
}

TEST(ArrayList, ArrayConstruction)
{
	{
		constexpr auto array_size = 2;
		int array[array_size];
		constexpr unsigned max_size = 4;
		ArrayList<int, max_size> list(array);
		EXPECT_EQ(list.size(), decltype(list.size()){array_size});
		EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
		EXPECT_FALSE(list.empty());
		EXPECT_EQ(std::distance(list.begin(), list.end()), array_size);
	}
	{
		constexpr auto array_size = 6;
		float array[array_size];
		constexpr unsigned max_size = 6;
		ArrayList<float, max_size> list(array);
		EXPECT_EQ(list.size(), decltype(list.size()){array_size});
		EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
		EXPECT_FALSE(list.empty());
		EXPECT_EQ(std::distance(list.begin(), list.end()), array_size);
	}
}

TEST(ArrayList, add)
{
	{
		constexpr unsigned max_size = 4;
		ArrayList<int, max_size> list;
		ASSERT_EQ(list.size(), decltype(list.size()){0});
		ASSERT_EQ(list.max_size(), decltype(list.size()){max_size});
		ASSERT_TRUE(list.empty());
		ASSERT_EQ(list.begin(), list.end());
		
		const auto value = 5;
		EXPECT_TRUE(list.add(value));
		EXPECT_EQ(list.size(), decltype(list.size()){1});
		EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
		EXPECT_FALSE(list.empty());
		EXPECT_EQ(std::distance(list.begin(), list.end()), 1);
		EXPECT_EQ(*list.begin(), value);
	}
}

TEST(ArrayList, clear)
{
	{
		constexpr unsigned max_size = 4;
		ArrayList<int, max_size> list;
		ASSERT_EQ(list.size(), decltype(list.size()){0});
		ASSERT_EQ(list.max_size(), decltype(list.size()){max_size});
		ASSERT_TRUE(list.empty());
		ASSERT_EQ(list.begin(), list.end());
		
		const auto value = 5;
		ASSERT_TRUE(list.add(value));
		ASSERT_EQ(list.size(), decltype(list.size()){1});
		ASSERT_EQ(list.max_size(), decltype(list.size()){max_size});
		ASSERT_FALSE(list.empty());
		ASSERT_EQ(std::distance(list.begin(), list.end()), 1);
		ASSERT_EQ(*list.begin(), value);
		
		list.clear();
		
		EXPECT_EQ(list.size(), decltype(list.size()){0});
		EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
		EXPECT_TRUE(list.empty());
		EXPECT_EQ(list.begin(), list.end());
	}
}
