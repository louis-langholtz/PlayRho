/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "gtest/gtest.h"
#include <PlayRho/Common/Vector.hpp>
#include <PlayRho/Common/Math.hpp>
#include <sstream>
#include <type_traits>
#include <algorithm>

using namespace playrho;

TEST(Vector, Equality)
{
    Vector<10, int> a;
    Vector<10, int> b;
    
    std::fill(a.begin(), a.end(), 1);
    std::fill(b.begin(), b.end(), 1);
    EXPECT_TRUE(a == b);

    std::fill(b.begin(), b.end(), 2);
    EXPECT_FALSE(a == b);

    std::fill(a.begin(), a.end(), 2);
    EXPECT_TRUE(a == b);
    
    for (auto& e: a)
    {
        const auto old = e;
        e = 10;
        EXPECT_FALSE(a == b);
        e = old;
    }
    
    EXPECT_TRUE(a == b);
}

TEST(Vector, Inequality)
{
    Vector<10, int> a;
    Vector<10, int> b;
    
    std::fill(a.begin(), a.end(), 1);
    std::fill(b.begin(), b.end(), 1);
    EXPECT_FALSE(a != b);
    
    std::fill(b.begin(), b.end(), 2);
    EXPECT_TRUE(a != b);
    
    std::fill(a.begin(), a.end(), 2);
    EXPECT_FALSE(a != b);
    
    for (auto& e: a)
    {
        const auto old = e;
        e = 10;
        EXPECT_TRUE(a != b);
        e = old;
    }
    
    EXPECT_FALSE(a != b);
}

TEST(Vector, LessThan)
{
    Vector<10, int> a;
    Vector<10, int> b;
    
    std::fill(a.begin(), a.end(), 1);
    std::fill(b.begin(), b.end(), 1);
    EXPECT_FALSE(a < b);
    
    std::fill(b.begin(), b.end(), 2);
    EXPECT_TRUE(a < b);
    
    std::fill(a.begin(), a.end(), 2);
    EXPECT_FALSE(a < b);
    
    for (auto& e: b)
    {
        const auto old = e;
        e = 10;
        EXPECT_TRUE(a < b);
        e = old;
    }
    
    EXPECT_FALSE(a < b);
}

TEST(Vector, GreaterThan)
{
    Vector<10, int> a;
    Vector<10, int> b;
    
    std::fill(a.begin(), a.end(), 1);
    std::fill(b.begin(), b.end(), 1);
    EXPECT_FALSE(a > b);
    
    std::fill(b.begin(), b.end(), 2);
    EXPECT_FALSE(a > b);
    
    std::fill(a.begin(), a.end(), 2);
    EXPECT_FALSE(a > b);
    
    for (auto& e: b)
    {
        const auto old = e;
        e = 10;
        EXPECT_TRUE(b > a);
        e = old;
    }
    
    EXPECT_FALSE(b > a);
}

TEST(Vector, LessThanOrEqualTo)
{
    Vector<10, int> a;
    Vector<10, int> b;
    
    std::fill(a.begin(), a.end(), 1);
    std::fill(b.begin(), b.end(), 1);
    EXPECT_TRUE(a <= b);
    
    std::fill(b.begin(), b.end(), 2);
    EXPECT_TRUE(a <= b);
    
    std::fill(a.begin(), a.end(), 2);
    EXPECT_TRUE(a <= b);
    
    for (auto& e: b)
    {
        const auto old = e;
        e = 10;
        EXPECT_TRUE(a <= b);
        e = old;
    }
    
    EXPECT_TRUE(a <= b);
}

TEST(Vector, GreaterThanOrEqualTo)
{
    Vector<10, int> a;
    Vector<10, int> b;
    
    std::fill(a.begin(), a.end(), 1);
    std::fill(b.begin(), b.end(), 1);
    EXPECT_TRUE(a >= b);
    
    std::fill(b.begin(), b.end(), 2);
    EXPECT_FALSE(a >= b);
    
    std::fill(a.begin(), a.end(), 2);
    EXPECT_TRUE(a >= b);
    
    for (auto& e: b)
    {
        const auto old = e;
        e = 10;
        EXPECT_TRUE(b >= a);
        e = old;
    }
    
    EXPECT_TRUE(b >= a);
}

TEST(Vector, ReverseIterateWith_crbeginend)
{
    Vector<4, int> vector{0, 1, 2, 3};
    
    int n = 0;
    for (auto it = vector.crbegin(); it != vector.crend(); ++it)
    {
        EXPECT_EQ(*it, 3 - n);
        ++n;
    }
}

TEST(Vector, ReverseIterateWith_rbeginend)
{
    {
        Vector<4, int> vector{0, 1, 2, 3};
        int n = 0;
        for (auto it = vector.rbegin(); it != vector.rend(); ++it)
        {
            EXPECT_EQ(*it, 3 - n);
            ++n;
        }
    }
    {
        const Vector<4, int> vector{0, 1, 2, 3};
        int n = 0;
        for (auto it = vector.rbegin(); it != vector.rend(); ++it)
        {
            EXPECT_EQ(*it, 3 - n);
            ++n;
        }
    }
}
