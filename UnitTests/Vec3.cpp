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

#include "UnitTests.hpp"
#include <PlayRho/Common/Math.hpp>
#include <sstream>
#include <type_traits>

using namespace playrho;

TEST(Vec3, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Vec3), std::size_t(12)); break;
        case  8: EXPECT_EQ(sizeof(Vec3), std::size_t(24)); break;
        case 16: EXPECT_EQ(sizeof(Vec3), std::size_t(48)); break;
        default: FAIL(); break;
    }
}

TEST(Vec3, Traits)
{
    EXPECT_TRUE((IsAddable<Vec3>::value));
    EXPECT_TRUE((IsAddable<Vec3,Vec3>::value));

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
    Vec3 vector{Real{5}, Real{-3}, Real{11}};
    EXPECT_EQ(Real{5}, GetX(vector));
    EXPECT_EQ(Real{-3}, GetY(vector));
    EXPECT_EQ(Real{11}, GetZ(vector));
}

TEST(Vec3, ZeroInitialization)
{
    // Tests C++11 zero initialization.
    // See: http://en.cppreference.com/w/cpp/language/zero_initialization
    
    {
        Vec3 src{Real{-1.2f}, Real{42.5f}, Real{-91.2f}};
        Vec3* foo = new (&src) Vec3;
        ASSERT_TRUE(foo != nullptr);
        ASSERT_EQ(foo->max_size(), std::size_t(3));
        ASSERT_EQ(foo->size(), std::size_t(3));
        ASSERT_NE(*foo, (Vec3{Real{0}, Real{0}, Real{0}}));
        *foo = Vec3{};
        EXPECT_EQ((*foo)[0], Real(0));
        EXPECT_EQ((*foo)[1], Real(0));
    }
    {
        Vec3 src{Real{-1.2f}, Real{42.5f}, Real{-91.2f}};
        Vec3* foo = new (&src) Vec3;
        ASSERT_TRUE(foo != nullptr);
        ASSERT_EQ(foo->max_size(), std::size_t(3));
        ASSERT_EQ(foo->size(), std::size_t(3));
        ASSERT_NE(*foo, (Vec3{Real{0}, Real{0}, Real{0}}));
        *foo = {};
        EXPECT_EQ((*foo)[0], Real(0));
        EXPECT_EQ((*foo)[1], Real(0));
    }
}

TEST(Vec3, Equality)
{
    Vec3 vector{Real{5}, Real{-3}, Real{11}};
    EXPECT_EQ(GetX(vector), GetX(vector));
    EXPECT_EQ(GetY(vector), GetY(vector));
    EXPECT_EQ(vector, vector);
}

TEST(Vec3, Inequality)
{
    Vec3 vector1{Real{5}, Real{-3}, Real{11}};
    Vec3 vector2{Real{-5}, Real{+3}, Real{-6}};
    EXPECT_NE(GetX(vector1), GetX(vector2));
    EXPECT_NE(GetY(vector1), GetY(vector2));
    EXPECT_NE(GetZ(vector1), GetZ(vector2));
    EXPECT_NE(vector1, vector2);
}

TEST(Vec3, Negate)
{
    Vec3 v10{1, 0, -31};
    Vec3 n10 = -v10;
    Vec3 v01{0, 1, 2};
    Vec3 n01 = -v01;
    EXPECT_EQ(-GetX(v10), GetX(n10));
    EXPECT_EQ(-GetY(v10), GetY(n10));
    EXPECT_EQ(-GetX(v01), GetX(n01));
    EXPECT_EQ(-GetY(v01), GetY(n01));
    EXPECT_EQ(-GetZ(v01), GetZ(n01));
    
    EXPECT_EQ(Real{-22}, GetX(-Vec3{22, 0, 0}));
    EXPECT_EQ(Real{-3}, GetY(-Vec3{0, 3, 0}));
    EXPECT_EQ(Real{5}, GetZ(-Vec3{0, 3, -5}));
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
    const auto b = Real(10);
    const auto c = Vec3{10, 20, 30};
    EXPECT_EQ(a * b, c);
    EXPECT_EQ(b * a, c);
}

TEST(Vec3, std_tuple_size)
{
    EXPECT_EQ(std::tuple_size<Vec3>::value, 3u);
}

TEST(Vec3, std_tuple_element)
{
    ::testing::StaticAssertTypeEq<std::tuple_element<0, Vec3>::type, Real>();
    ::testing::StaticAssertTypeEq<std::tuple_element<1, Vec3>::type, Real>();
    ::testing::StaticAssertTypeEq<std::tuple_element<2, Vec3>::type, Real>();
}
