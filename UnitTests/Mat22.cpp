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

using namespace playrho;

TEST(Mat22, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Mat22), std::size_t(16)); break;
        case  8: EXPECT_EQ(sizeof(Mat22), std::size_t(32)); break;
        case 16: EXPECT_EQ(sizeof(Mat22), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(Mat22, Init)
{
    const auto v1 = Vec2{1, 1};
    const auto v2 = Vec2{2, 2};
    const auto foo = Mat22{v1, v2};
    EXPECT_EQ(v1, get<0>(foo));
    EXPECT_EQ(v2, get<1>(foo));
}

TEST(Mat22, IsMatrix)
{
    ASSERT_FALSE(IsMatrix<int>::value);
    ASSERT_TRUE((IsMatrix<Matrix<int, 2, 3>>::value));
    EXPECT_TRUE(IsMatrix<Mat22>::value);
}

TEST(Mat22, IsSquareMatrix)
{
    ASSERT_FALSE(IsSquareMatrix<int>::value);
    ASSERT_FALSE((IsSquareMatrix<Matrix<int, 3, 2>>::value));
    EXPECT_TRUE(IsSquareMatrix<Mat22>::value);
}

TEST(Mat22, Invert)
{
    const auto v1 = Vec2{1, 2};
    const auto v2 = Vec2{3, 4};
    const auto foo = Mat22{v1, v2};
    ASSERT_EQ(get<0>(foo), v1);
    ASSERT_EQ(get<1>(foo), v2);

    const auto inverted = Invert(foo);
    const auto cp = Cross(v1, v2);
    ASSERT_EQ(cp, Real(-2));
    const auto det = (cp != 0)? Real{1} / cp : Real{0};
    
    EXPECT_EQ(get<0>(get<0>(inverted)), det * get<1>(get<1>(foo)));
    EXPECT_EQ(get<1>(get<0>(inverted)), -det * get<1>(get<0>(foo)));
    EXPECT_EQ(get<0>(get<1>(inverted)), -det * get<0>(get<1>(foo)));
    EXPECT_EQ(get<1>(get<1>(inverted)), det * get<0>(get<0>(foo)));
    
    EXPECT_EQ(get<0>(get<0>(inverted)), Real(-2));
    EXPECT_EQ(get<1>(get<0>(inverted)), Real(1));
    EXPECT_EQ(get<0>(get<1>(inverted)), Real(1.5));
    EXPECT_EQ(get<1>(get<1>(inverted)), Real(-0.5));
}

TEST(Mat22, InvertInvertedIsOriginal)
{
    const auto v1 = Vec2{1, 2};
    const auto v2 = Vec2{3, 4};
    const auto foo = Mat22{v1, v2};
    const auto inverted = Invert(foo);
    const auto inverted2 = Invert(inverted);
    EXPECT_EQ(get<0>(get<0>(foo)), get<0>(get<0>(inverted2)));
    EXPECT_EQ(get<1>(get<0>(foo)), get<1>(get<0>(inverted2)));
    EXPECT_EQ(get<0>(get<1>(foo)), get<0>(get<1>(inverted2)));
    EXPECT_EQ(get<1>(get<1>(foo)), get<1>(get<1>(inverted2)));
}

TEST(Mat22, Addition)
{
    {
        const auto m1 = Mat22{};
        const auto m2 = Mat22{};
        EXPECT_EQ(m1 + m2, Mat22());
    }
    {
        const auto m1 = Mat22{Vec2{1, 2}, Vec2{3, 4}};
        const auto m2 = Mat22{Vec2{1, 2}, Vec2{3, 4}};
        EXPECT_EQ(m1 + m2, Mat22(Vec2(2, 4), Vec2(6, 8)));
    }
}

TEST(Mat22, Subtraction)
{
    {
        const auto m1 = Mat22{};
        const auto m2 = Mat22{};
        EXPECT_EQ(m1 - m2, Mat22());
    }
    {
        const auto m1 = Mat22{Vec2{1, 2}, Vec2{3, 4}};
        const auto m2 = Mat22{Vec2{1, 2}, Vec2{3, 4}};
        EXPECT_EQ(m1 - m2, Mat22());
    }
}

TEST(Mat22, Multiplication)
{
    {
        const auto m1 = Mat22{};
        const auto m2 = Mat22{};
        EXPECT_EQ(m1 * m2, Mat22());
    }
    {
        const auto m1 = Mat22{Vec2{1, 2}, Vec2{3, 4}};
        const auto m2 = Mat22{Vec2{1, 2}, Vec2{3, 4}};
        //
        // | 1 2 |   | 1 2 |   | 1*1+2*3=7  1*2+2*4=10 |
        // |     | * |     | = |                       |
        // | 3 4 |   | 3 4 |   | 3*1+4*3=15 3*2+4*4=22 |
        //
        EXPECT_EQ(m1 * m2, Mat22(Vec2(7, 10), Vec2(15, 22)));
        
        static_assert(!IsVector<int>::value, "supposed to not be vector but is");
        const auto m3 = Mat33{};
        static_assert(IsVector<Mat33>::value, "supposed to be vector but not");
        const auto m23 = Vector<Vector<Real, 3>, 2>{};
        static_assert(IsVector<Vector<Vector<Real, 3>, 2>>::value, "supposed to be vector but not");
        static_assert(!IsMultipliable<decltype(m3), decltype(m2)>::value, "");
        static_assert(!IsMultipliable<decltype(m2), decltype(m3)>::value, "");
        EXPECT_EQ(m2 * m23, (Matrix<Real, 2, 3>{}));
    }
}
