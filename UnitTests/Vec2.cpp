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
#include <PlayRho/Common/Math.hpp>
#include <sstream>
#include <type_traits>

using namespace box2d;

TEST(Vec2, ByteSizeIs_8_or_16)
{
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
    EXPECT_TRUE(std::is_default_constructible<Vec2>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<Vec2>::value);
    EXPECT_TRUE(std::is_trivially_default_constructible<Vec2>::value);
    
    EXPECT_TRUE(std::is_constructible<Vec2>::value);
    EXPECT_TRUE(std::is_nothrow_constructible<Vec2>::value);
    EXPECT_TRUE(std::is_trivially_constructible<Vec2>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<Vec2>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<Vec2>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<Vec2>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<Vec2>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<Vec2>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<Vec2>::value);
    
    EXPECT_TRUE(std::is_destructible<Vec2>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Vec2>::value);
    EXPECT_TRUE(std::is_trivially_destructible<Vec2>::value);
}

TEST(Vec2, Constructor) {
    Vec2 vector{Real{5}, Real{-3}};
    EXPECT_EQ(Real{5}, vector.x);
    EXPECT_EQ(Real{-3}, vector.y);
}

TEST(Vec2, OutputOperator)
{
    std::stringstream os;
    const Vec2 value{Real(1.5), Real(-2.3)};
    os << value;
    EXPECT_EQ(os.str(), "Vec2(1.5,-2.3)");
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

TEST(Vec2, Equality)
{    
    Vec2 vector{Real{5}, Real{-3}};
    EXPECT_EQ(vector.x, vector.x);
    EXPECT_EQ(vector.y, vector.y);
    EXPECT_EQ(vector, vector);
}

TEST(Vec2, Inequality)
{    
    Vec2 vector1{Real{5}, Real{-3}};
    Vec2 vector2{Real{-5}, Real{+3}};
    EXPECT_NE(vector1.x, vector2.x);
    EXPECT_NE(vector1.y, vector2.y);
    EXPECT_NE(vector1, vector2);
}

TEST(Vec2, Negate)
{
    Vec2 v10{1, 0};
    Vec2 n10 = -v10;
    Vec2 v01{0, 1};
    Vec2 n01 = -v01;
    EXPECT_EQ(-v10.x, n10.x);
    EXPECT_EQ(-v10.y, n10.y);
    EXPECT_EQ(-v01.x, n01.x);
    EXPECT_EQ(-v01.y, n01.y);
    
    EXPECT_EQ(Real{-22}, (-Vec2{22, 0}).x);
    EXPECT_EQ(Real{-3}, (-Vec2{0, 3}).y);
}

TEST(Vec2, Rotate)
{
    Vec2 v10{1, 0};
    Vec2 v01{0, 1};

    EXPECT_EQ(round(v01), round(Rotate(v10, UnitVec2{Angle{Real{90.0f} * Degree}})));

    EXPECT_EQ(round(Vec2{22, 30}), round(Rotate(Vec2{22, 30}, UnitVec2{Angle{0}})));
    EXPECT_EQ(round(Vec2{22, 30}, 1000), round(Rotate(Vec2{22, 30}, UnitVec2{Angle{Real{360.0f} * Degree}}), 1000));
    EXPECT_EQ(round(-Vec2{22, 30}, 1000), round(Rotate(Vec2{22, 30}, UnitVec2{Angle{Real{180.0f} * Degree}}), 1000));
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
