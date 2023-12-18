/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <playrho/ArrayList.hpp>
#include <playrho/Vector.hpp>
//#include <playrho/Math.hpp>

#include "gtest/gtest.h"

using namespace playrho;

TEST(ArrayList, traits)
{
    EXPECT_TRUE((IsIterableV<ArrayList<int, 4u>>));
    EXPECT_FALSE((IsAddableV<ArrayList<int, 4u>>));
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

TEST(ArrayList, CarrayConstruction)
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
}

TEST(ArrayList, CppArrayConstruction)
{
    constexpr auto maxsize = std::size_t{10};
    constexpr auto array = std::array<int, maxsize>{{5, 4, 3}};
    ArrayList<int, maxsize> list(array);
    EXPECT_EQ(list.size(), decltype(list.size()){maxsize});
    EXPECT_EQ(list.max_size(), maxsize);
    EXPECT_EQ(list[0], 5);
    EXPECT_EQ(list[1], 4);
    EXPECT_EQ(list[2], 3);
}

TEST(ArrayList, InitializerListConstruction)
{
    EXPECT_THROW((ArrayList<int, 1>{1, 2}), LengthError);
    ASSERT_NO_THROW((ArrayList<int, 2>{1, 2}));
    EXPECT_EQ((ArrayList<int, 2>{1, 2}.size()), 2u);
    ASSERT_NO_THROW((ArrayList<int, 3>{1, 2}));
    EXPECT_EQ((ArrayList<int, 3>{1, 2}.size()), 2u);
    {
        const auto il = std::initializer_list<int>{1, 2};
        const ArrayList<int, 3> al(il);
        EXPECT_TRUE(Equal(al.begin(), al.end(), il.begin(), il.end()));
    }
    {
        constexpr auto maxsize = std::size_t{10};
        ArrayList<int, maxsize> list = { 1, 2, 3 };
        EXPECT_EQ(list.size(), decltype(list.size()){3});
        EXPECT_EQ(list.max_size(), maxsize);
    }
    {
        constexpr auto maxsize = std::size_t{10};
        constexpr auto list = ArrayList<int, maxsize>{1, 2, 3};
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

TEST(ArrayList, CopyConstruction)
{
    constexpr auto maxsize = std::size_t{10};
    ArrayList<int, maxsize> list = { 1, 2, 3 };
    ASSERT_EQ(list.size(), 3u);
    ASSERT_EQ(list[0], 1);
    ASSERT_EQ(list[1], 2);
    ASSERT_EQ(list[2], 3);
    const ArrayList<int, maxsize> copy(list);
    EXPECT_EQ(copy.size(), 3u);
    EXPECT_EQ(copy[0], 1);
    EXPECT_EQ(copy[1], 2);
    EXPECT_EQ(copy[2], 3);
}

TEST(ArrayList, ConstructionFromSmaller)
{
    constexpr auto minsize = std::size_t{5};
    constexpr auto maxsize = std::size_t{10};
    ArrayList<int, minsize> list = { 1, 2, 3 };
    ASSERT_EQ(list.size(), 3u);
    ASSERT_EQ(list[0], 1);
    ASSERT_EQ(list[1], 2);
    ASSERT_EQ(list[2], 3);
    const ArrayList<int, maxsize> copy(list);
    EXPECT_EQ(copy.size(), 3u);
    EXPECT_EQ(copy[0], 1);
    EXPECT_EQ(copy[1], 2);
    EXPECT_EQ(copy[2], 3);
}

TEST(ArrayList, ConstructionFromVector)
{
    constexpr auto maxsize = std::size_t{10};
    const auto list = std::vector<int>{ 1, 2, 3 };
    ASSERT_EQ(list.size(), 3u);
    ASSERT_EQ(list[0], 1);
    ASSERT_EQ(list[1], 2);
    ASSERT_EQ(list[2], 3);
    const ArrayList<int, maxsize> copy(list);
    EXPECT_EQ(copy.size(), 3u);
    EXPECT_EQ(copy[0], 1);
    EXPECT_EQ(copy[1], 2);
    EXPECT_EQ(copy[2], 3);
    EXPECT_THROW((ArrayList<int, 2u>(list)), LengthError);
}

TEST(ArrayList, AssignmentFromVector)
{
    {
        const auto list = std::vector<int>{ 1, 2, 3 };
        ASSERT_EQ(list.size(), 3u);
        ArrayList<int, 2u> copy;
        ASSERT_EQ(copy.size(), 0u);
        ASSERT_EQ(copy.max_size(), 2u);
        EXPECT_THROW(copy = list, LengthError);
    }
    {
        constexpr auto maxsize = std::size_t{10};
        const auto list = std::vector<int>{ 1, 2, 3 };
        ASSERT_EQ(list.size(), 3u);
        ASSERT_EQ(list[0], 1);
        ASSERT_EQ(list[1], 2);
        ASSERT_EQ(list[2], 3);
        ArrayList<int, maxsize> copy;
        EXPECT_NO_THROW(copy = list);
        EXPECT_EQ(copy.size(), 3u);
        EXPECT_EQ(copy[0], 1);
        EXPECT_EQ(copy[1], 2);
        EXPECT_EQ(copy[2], 3);
    }
}

TEST(ArrayList, add)
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

TEST(ArrayList, clear)
{
    {
        constexpr auto max_size = 4u;
        ArrayList<int, max_size> list;
        ASSERT_EQ(list.size(), 0u);
        ASSERT_EQ(list.max_size(), max_size);
        ASSERT_TRUE(list.empty());
        ASSERT_EQ(list.begin(), list.end());
        
        const auto value = 5;
        ASSERT_TRUE(list.add(value));
        ASSERT_EQ(list.size(), 1u);
        ASSERT_EQ(list.max_size(), max_size);
        ASSERT_FALSE(list.empty());
        ASSERT_EQ(std::distance(list.begin(), list.end()), 1);
        ASSERT_EQ(*list.begin(), value);
        
        list.clear();
        
        EXPECT_EQ(list.size(), 0u);
        EXPECT_EQ(list.max_size(), max_size);
        EXPECT_TRUE(list.empty());
        EXPECT_EQ(list.begin(), list.end());
    }
}

TEST(ArrayList, OperatorAppendOne)
{
    constexpr auto max_size = 2u;
    ArrayList<int, max_size> list;
    ASSERT_EQ(list.size(), 0u);
    ASSERT_EQ(list.max_size(), max_size);
    EXPECT_NO_THROW(list += 1);
    ASSERT_EQ(list.size(), 1u);
    EXPECT_EQ(list[0], 1);
    EXPECT_NO_THROW(list += 2);
    EXPECT_EQ(list[0], 1);
    EXPECT_EQ(list[1], 2);
    EXPECT_THROW(list += 3, LengthError);
}

TEST(ArrayList, OperatorAppendVector)
{
    constexpr auto max_size = 2u;
    ArrayList<int, max_size> list;
    ASSERT_EQ(list.size(), 0u);
    ASSERT_EQ(list.max_size(), max_size);
    EXPECT_NO_THROW(list += std::vector<int>{});
    ASSERT_EQ(list.size(), 0u);
    EXPECT_NO_THROW(list += (std::vector<int>{1}));
    ASSERT_EQ(list.size(), 1u);
    EXPECT_EQ(list[0], 1);
    list.clear();
    const auto v23 = std::vector<int>{2, 3};
    EXPECT_NO_THROW(list += v23);
    ASSERT_EQ(list.size(), 2u);
    EXPECT_EQ(list[0], 2);
    EXPECT_EQ(list[1], 3);
    EXPECT_THROW(list += v23, LengthError);
}

TEST(ArrayList, Equality)
{
    constexpr auto max_size = 4u;
    ArrayList<int, max_size> listA;
    ArrayList<int, max_size> listB;

    EXPECT_TRUE(listA == listA);
    EXPECT_TRUE(listA == listB);
    EXPECT_TRUE(listB == listA);
    EXPECT_TRUE(listB == listB);

    const auto five = 5;
    ASSERT_TRUE(listA.add(five));
    EXPECT_TRUE(listA == listA);
    EXPECT_FALSE(Equal(listA.begin(), listA.end(), listB.begin(), listB.end()));
    EXPECT_FALSE(listA == listB);
    EXPECT_FALSE(listB == listA);
    ASSERT_TRUE(listB.add(five));
    EXPECT_TRUE(listA == listB);
    EXPECT_TRUE(Equal(listA.begin(), listA.end(), listB.begin(), listB.end()));

    const auto six = 6;
    ASSERT_TRUE(listA.add(six));
    EXPECT_FALSE(listA == listB);
    ASSERT_TRUE(listB.add(five));
    ASSERT_TRUE(listA.size() == listB.size());
    EXPECT_FALSE(listA == listB);
}

TEST(ArrayList, Inequality)
{
    constexpr auto max_size = 4u;
    ArrayList<int, max_size> listA;
    ArrayList<int, max_size> listB;

    EXPECT_FALSE(listA != listA);
    EXPECT_FALSE(listA != listB);
    EXPECT_FALSE(listB != listA);
    EXPECT_FALSE(listB != listB);

    const auto five = 5;
    ASSERT_TRUE(listA.add(five));
    EXPECT_FALSE(listA != listA);
    EXPECT_TRUE(listA != listB);
    EXPECT_TRUE(listB != listA);
    ASSERT_TRUE(listB.add(five));
    EXPECT_FALSE(listA != listB);

    const auto six = 6;
    ASSERT_TRUE(listA.add(six));
    EXPECT_TRUE(listA != listB);
    ASSERT_TRUE(listB.add(five));
    ASSERT_TRUE(listA.size() == listB.size());
    EXPECT_TRUE(listA != listB);
}
