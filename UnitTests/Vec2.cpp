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

#include <playrho/d2/Math.hpp>

#include <sstream>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(Vec2, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Vec2), std::size_t(8)); break;
        case  8: EXPECT_EQ(sizeof(Vec2), std::size_t(16)); break;
        case 16: EXPECT_EQ(sizeof(Vec2), std::size_t(32)); break;
        default: FAIL(); break;
    }
}

TEST(Vec2, max_size) {
    Vec2 vector;
    EXPECT_EQ(Vec2::size_type{2}, vector.max_size());
}

TEST(Vec2, Traits)
{
    EXPECT_TRUE((IsAddableV<Vec2>));
    EXPECT_TRUE((IsAddableV<Vec2,Vec2>));

    EXPECT_TRUE(std::is_default_constructible_v<Vec2>);
    EXPECT_TRUE(std::is_nothrow_default_constructible_v<Vec2>);
    EXPECT_TRUE(std::is_trivially_default_constructible_v<Vec2>);
    
    EXPECT_TRUE((std::is_constructible_v<Vec2, Real, Real>));
    EXPECT_FALSE((std::is_constructible_v<Vec2, Real>));
    EXPECT_TRUE((std::is_constructible_v<Vec2>));

    EXPECT_TRUE((std::is_nothrow_constructible_v<Vec2, Real, Real>));
    EXPECT_FALSE((std::is_nothrow_constructible_v<Vec2, Real>));
    EXPECT_TRUE((std::is_nothrow_constructible_v<Vec2>));
    
    EXPECT_FALSE((std::is_trivially_constructible_v<Vec2, Real, Real>));
    EXPECT_FALSE((std::is_trivially_constructible_v<Vec2, Real>));
    EXPECT_TRUE((std::is_trivially_constructible_v<Vec2>));
    
    EXPECT_TRUE(std::is_copy_constructible_v<Vec2>);
    EXPECT_TRUE(std::is_nothrow_copy_constructible_v<Vec2>);
    EXPECT_TRUE(std::is_trivially_copy_constructible_v<Vec2>);
    
    EXPECT_TRUE(std::is_copy_assignable_v<Vec2>);
    EXPECT_TRUE(std::is_nothrow_copy_assignable_v<Vec2>);
    EXPECT_TRUE(std::is_trivially_copy_assignable_v<Vec2>);
    
    EXPECT_TRUE(std::is_destructible_v<Vec2>);
    EXPECT_TRUE(std::is_nothrow_destructible_v<Vec2>);
    EXPECT_TRUE(std::is_trivially_destructible_v<Vec2>);
}

TEST(Vec2, ZeroInitialization)
{
    // Tests C++11 zero initialization.
    // See: http://en.cppreference.com/w/cpp/language/zero_initialization
    
    {
        Vec2 src{Real{-1.2f}, Real{42.5f}};
        Vec2* foo = new (&src) Vec2;
        ASSERT_TRUE(foo != nullptr);
        ASSERT_EQ(foo->max_size(), std::size_t(2));
        ASSERT_EQ(foo->size(), std::size_t(2));
        ASSERT_NE(*foo, (Vec2{Real{0}, Real{0}}));
        *foo = Vec2{};
        EXPECT_EQ((*foo)[0], Real(0));
        EXPECT_EQ((*foo)[1], Real(0));
    }
    {
        Vec2 src{Real{-1.2f}, Real{42.5f}};
        Vec2* foo = new (&src) Vec2;
        ASSERT_TRUE(foo != nullptr);
        ASSERT_EQ(foo->max_size(), std::size_t(2));
        ASSERT_EQ(foo->size(), std::size_t(2));
        ASSERT_NE(*foo, (Vec2{Real{0}, Real{0}}));
        *foo = {};
        EXPECT_EQ((*foo)[0], Real(0));
        EXPECT_EQ((*foo)[1], Real(0));
    }
}

TEST(Vec2, Constructor)
{
    Vec2 vector{Real{5}, Real{-3}};
    EXPECT_EQ(Real{5}, GetX(vector));
    EXPECT_EQ(Real{-3}, GetY(vector));
}

TEST(Vec2, OutputOperator)
{
    std::stringstream os;
    const Vec2 value{Real(1.5), Real(-2.3)};
    os << value;
    EXPECT_EQ(os.str(), "{1.5,-2.3}");
}

TEST(Vec2, Indexing) {
    Vec2 vector{Real{5}, Real{-3}};
    EXPECT_EQ(Real{5}, vector[0]);
    EXPECT_EQ(Real{-3}, vector[1]);
    vector[0] = Real{4};
    EXPECT_EQ(Real{4}, vector[0]);
    vector[1] = Real{-2};
    EXPECT_EQ(Real{-2}, vector[1]);
}

TEST(Vec2, At) {
    Vec2 vector{Real{5}, Real{-3}};
    EXPECT_EQ(Real{5}, vector.at(0));
    EXPECT_EQ(Real{-3}, vector.at(1));
    vector[0] = Real{4};
    EXPECT_EQ(Real{4}, vector.at(0));
    vector[1] = Real{-2};
    EXPECT_EQ(Real{-2}, vector.at(1));
}

TEST(Vec2, data)
{
    {
        Vec2 vector{Real{5}, Real{-3}};
        EXPECT_EQ(vector.data(), vector.elements);
        EXPECT_EQ(vector.data()[0], vector[0]);
        EXPECT_EQ(vector.data()[1], vector[1]);
        vector.data()[0] = Real(2);
        EXPECT_EQ(vector.data()[0], Real(2));
    }
    {
        const Vec2 vector{Real{5}, Real{-3}};
        EXPECT_EQ(vector.data(), vector.elements);
        EXPECT_EQ(vector.data()[0], vector[0]);
        EXPECT_EQ(vector.data()[1], vector[1]);
    }
}

TEST(Vec2, Equality)
{    
    Vec2 vector{Real{5}, Real{-3}};
    EXPECT_EQ(GetX(vector), GetX(vector));
    EXPECT_EQ(GetY(vector), GetY(vector));
    EXPECT_EQ(vector, vector);
}

TEST(Vec2, Inequality)
{    
    Vec2 vector1{Real{5}, Real{-3}};
    Vec2 vector2{Real{-5}, Real{+3}};
    EXPECT_NE(GetX(vector1), GetX(vector2));
    EXPECT_NE(GetY(vector1), GetY(vector2));
    EXPECT_NE(vector1, vector2);
}

TEST(Vec2, Negate)
{
    Vec2 v10{1, 0};
    Vec2 n10 = -v10;
    Vec2 v01{0, 1};
    Vec2 n01 = -v01;
    EXPECT_EQ(-GetX(v10), GetX(n10));
    EXPECT_EQ(-GetY(v10), GetY(n10));
    EXPECT_EQ(-GetX(v01), GetX(n01));
    EXPECT_EQ(-GetY(v01), GetY(n01));
    
    EXPECT_EQ(Real{-22}, GetX(-Vec2{22, 0}));
    EXPECT_EQ(Real{-3}, GetY(-Vec2{0, 3}));
}

TEST(Vec2, Rotate)
{
    Vec2 v10{1, 0};
    Vec2 v01{0, 1};

    EXPECT_EQ(RoundOff(v01), RoundOff(Rotate(v10, UnitVec::GetTop())));

    EXPECT_EQ(RoundOff(Vec2{22, 30}), RoundOff(Rotate(Vec2{22, 30}, UnitVec::GetRight())));
    EXPECT_EQ(RoundOff(Vec2{22, 30}, 1000), RoundOff(Rotate(Vec2{22, 30}, UnitVec::Get(360_deg)), 1000));
    EXPECT_EQ(RoundOff(-Vec2{22, 30}, 1000), RoundOff(Rotate(Vec2{22, 30}, UnitVec::GetLeft()), 1000));
}

TEST(Vec2, IncrementOperator)
{
    auto a = Vec2{0, 0};
    ASSERT_EQ(a, Vec2(0, 0));
    const auto inc = Vec2{1, 1};
    
    a += inc;
    
    EXPECT_EQ(a, inc);
    
    a += inc;
    
    EXPECT_EQ(a, inc * 2);
}

TEST(Vec2, InvalidIndex)
{
    const auto a = Vec2{0, 0};
    EXPECT_NO_THROW(a[0]);
    EXPECT_NO_THROW(a[1]);
    EXPECT_NO_THROW(a.at(0));
    EXPECT_NO_THROW(a.at(1));
    EXPECT_THROW(a.at(2), InvalidArgument);
    EXPECT_THROW(a.at(3), InvalidArgument);
    
    auto b = Vec2{Real(2), Real(1)};
    EXPECT_NO_THROW(b[0] = Real(3));
    EXPECT_NO_THROW(b[1] = Real(4));
    EXPECT_NO_THROW(b.at(0) = Real(3));
    EXPECT_NO_THROW(b.at(1) = Real(4));
    EXPECT_EQ(b[0], Real(3));
    EXPECT_EQ(b[1], Real(4));
    EXPECT_THROW(b.at(2), InvalidArgument);
    EXPECT_THROW(b.at(3), InvalidArgument);
    EXPECT_THROW(b.at(2) = Real(5), InvalidArgument);
    EXPECT_THROW(b.at(3) = Real(6), InvalidArgument);
}

TEST(Vec2, std_tuple_size)
{
    EXPECT_EQ(std::tuple_size_v<Vec2>, 2u);
}

TEST(Vec2, std_tuple_element)
{
    ::testing::StaticAssertTypeEq<std::tuple_element_t<0, Vec2>, Real>();
    ::testing::StaticAssertTypeEq<std::tuple_element_t<1, Vec2>, Real>();
}

