/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Common/Math.hpp>
#include <sstream>
#include <type_traits>

using namespace box2d;

TEST(Vec3, ByteSizeIs_12_24_or_48)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(Vec3), std::size_t(12)); break;
        case  8: EXPECT_EQ(sizeof(Vec3), std::size_t(24)); break;
        case 16: EXPECT_EQ(sizeof(Vec3), std::size_t(48)); break;
        default: FAIL(); break;
    }
}

TEST(Vec3, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<Vec3>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<Vec3>::value);
    EXPECT_TRUE(std::is_trivially_default_constructible<Vec3>::value);
    
    EXPECT_TRUE(std::is_constructible<Vec3>::value);
    EXPECT_TRUE(std::is_nothrow_constructible<Vec3>::value);
    EXPECT_TRUE(std::is_trivially_constructible<Vec3>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<Vec3>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<Vec3>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<Vec3>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<Vec3>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<Vec3>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<Vec3>::value);
    
    EXPECT_TRUE(std::is_destructible<Vec3>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Vec3>::value);
    EXPECT_TRUE(std::is_trivially_destructible<Vec3>::value);
}

TEST(Vec3, Constructor) {
    Vec3 vector{RealNum{5}, RealNum{-3}, RealNum{11}};
    EXPECT_EQ(RealNum{5}, vector.x);
    EXPECT_EQ(RealNum{-3}, vector.y);
    EXPECT_EQ(RealNum{11}, vector.z);
}

TEST(Vec3, Equality)
{
    Vec3 vector{RealNum{5}, RealNum{-3}, RealNum{11}};
    EXPECT_EQ(vector.x, vector.x);
    EXPECT_EQ(vector.y, vector.y);
    EXPECT_EQ(vector, vector);
}

TEST(Vec3, Inequality)
{
    Vec3 vector1{RealNum{5}, RealNum{-3}, RealNum{11}};
    Vec3 vector2{RealNum{-5}, RealNum{+3}, RealNum{-6}};
    EXPECT_NE(vector1.x, vector2.x);
    EXPECT_NE(vector1.y, vector2.y);
    EXPECT_NE(vector1.z, vector2.z);
    EXPECT_NE(vector1, vector2);
}

TEST(Vec3, Negate)
{
    Vec3 v10{1, 0, -31};
    Vec3 n10 = -v10;
    Vec3 v01{0, 1, 2};
    Vec3 n01 = -v01;
    EXPECT_EQ(-v10.x, n10.x);
    EXPECT_EQ(-v10.y, n10.y);
    EXPECT_EQ(-v01.x, n01.x);
    EXPECT_EQ(-v01.y, n01.y);
    EXPECT_EQ(-v01.z, n01.z);
    
    EXPECT_EQ(RealNum{-22}, (-Vec3{22, 0, 0}).x);
    EXPECT_EQ(RealNum{-3}, (-Vec3{0, 3, 0}).y);
    EXPECT_EQ(RealNum{5}, (-Vec3{0, 3, -5}).z);
}

TEST(Vec3, IncrementOperator)
{
    auto a = Vec3{0, 0, 0};
    const auto b = Vec3{0, 0, 0};
    ASSERT_EQ(a, b);
    const auto inc = Vec3{1, 1, 1};
    a += inc;
    EXPECT_EQ(a, inc);
    a += inc;
    EXPECT_EQ(a, inc * 2);
}

TEST(Vec3, Addition)
{
    const auto a = Vec3{1, 2, 3};
    const auto b = Vec3{-10, 4, -6};
    const auto c = Vec3{-9, 6, -3};
    EXPECT_EQ(a + b, c);
}

TEST(Vec3, Subtraction)
{
    const auto a = Vec3{1, 2, 3};
    const auto b = Vec3{-10, 4, -6};
    const auto c = Vec3{11, -2, 9};
    EXPECT_EQ(a - b, c);
}

TEST(Vec3, ScalarMultiplication)
{
    const auto a = Vec3{1, 2, 3};
    const auto b = RealNum(10);
    const auto c = Vec3{10, 20, 30};
    EXPECT_EQ(a * b, c);
    EXPECT_EQ(b * a, c);
}
