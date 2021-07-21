/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Common/ArrayList.hpp>
#include <PlayRho/Common/Vector.hpp>
//#include <PlayRho/Common/Math.hpp>

using namespace playrho;

TEST(ArrayList, traits)
{
    EXPECT_TRUE((IsIterable<ArrayList<int, 4u>>::value));
    EXPECT_FALSE((IsAddable<ArrayList<int, 4u>>::value));
}

TEST(ArrayList, DefaultConstruction)
{
    {
        constexpr auto max_size = 4u;
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
        constexpr auto max_size = 4u;
        ArrayList<int, max_size> list(array);
        EXPECT_EQ(list.size(), decltype(list.size()){array_size});
        EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
        EXPECT_FALSE(list.empty());
        EXPECT_EQ(std::distance(list.begin(), list.end()), array_size);
    }
    {
        constexpr auto array_size = 6;
        float array[array_size];
        constexpr auto max_size = 6u;
        ArrayList<float, max_size> list(array);
        EXPECT_EQ(list.size(), decltype(list.size()){array_size});
        EXPECT_EQ(list.max_size(), decltype(list.size()){max_size});
        EXPECT_FALSE(list.empty());
        EXPECT_EQ(std::distance(list.begin(), list.end()), array_size);
    }
    {
        constexpr auto maxsize = std::size_t{10};
        ArrayList<int, maxsize> list = { 1, 2, 3 };
        EXPECT_EQ(list.size(), decltype(list.size()){3});
        EXPECT_EQ(list.max_size(), maxsize);
    }
    {
        constexpr auto maxsize = std::size_t{10};
        constexpr auto list = std::array<int, maxsize>{{5, 4, 3}};
        EXPECT_EQ(list.size(), decltype(list.size()){maxsize});
        EXPECT_EQ(list.max_size(), maxsize);
        EXPECT_EQ(list[0], 5);
        EXPECT_EQ(list[1], 4);
        EXPECT_EQ(list[2], 3);
    }
    {
        constexpr auto maxsize = std::size_t{10};
        
        // Note: list cannot be constexpr.
        const auto list = ArrayList<int, maxsize>{1, 2, 3};

        EXPECT_EQ(list.size(), decltype(list.size()){3});
        EXPECT_EQ(list.max_size(), maxsize);
        EXPECT_EQ(list[0], 1);
        EXPECT_EQ(list[1], 2);
        EXPECT_EQ(list[2], 3);
    }
    {
        constexpr auto list = Vector<int, 3>{1, 2, 3};
        EXPECT_EQ(list.size(), decltype(list.size()){3});
        EXPECT_EQ(list[0], 1);
        EXPECT_EQ(list[1], 2);
        EXPECT_EQ(list[2], 3);
    }
}

TEST(ArrayList, add)
{
    {
        constexpr auto max_size = 4u;
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

        EXPECT_TRUE(list.add(2));
        EXPECT_EQ(list.size(), decltype(list.size()){2});
        EXPECT_TRUE(list.add(3));
        EXPECT_EQ(list.size(), decltype(list.size()){3});
        EXPECT_TRUE(list.add(4));
        EXPECT_EQ(list.size(), decltype(list.size()){4});

        EXPECT_FALSE(list.add(5));
        EXPECT_EQ(list.size(), decltype(list.size()){4});
    }
}

TEST(ArrayList, clear)
{
    {
        constexpr auto max_size = 4u;
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
