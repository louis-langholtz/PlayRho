/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Common/Math.hpp>

using namespace playrho;

TEST(Mat22, ByteSizeIs_16_32_or_64)
{
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
    Vec2 c1{1, 1};
    Vec2 c2{2, 2};
    const Mat22 foo{c1, c2};
    EXPECT_EQ(c1, GetX(foo));
    EXPECT_EQ(c2, GetY(foo));
}

TEST(Mat22, Invert)
{
    Vec2 ex{1, 2};
    Vec2 ey{3, 4};
    const Mat22 foo{ex, ey};
    ASSERT_EQ(GetX(foo), ex);
    ASSERT_EQ(GetY(foo), ey);

    const auto inverted = Invert(foo);
    const auto cp = Cross(ex, ey);
    ASSERT_EQ(cp, Real(-2));
    const auto det = (cp != 0)? Real(1)/cp : Real(0);
    
    EXPECT_EQ(GetX(GetX(inverted)), det * GetY(GetY(foo)));
    EXPECT_EQ(GetY(GetX(inverted)), -det * GetY(GetX(foo)));
    EXPECT_EQ(GetX(GetY(inverted)), -det * GetX(GetY(foo)));
    EXPECT_EQ(GetY(GetY(inverted)), det * GetX(GetX(foo)));
    
    EXPECT_EQ(GetX(GetX(inverted)), Real(-2));
    EXPECT_EQ(GetY(GetX(inverted)), Real(1));
    EXPECT_EQ(GetX(GetY(inverted)), Real(1.5));
    EXPECT_EQ(GetY(GetY(inverted)), Real(-0.5));
}

TEST(Mat22, InvertInvertedIsOriginal)
{
    Vec2 c1{1, 2};
    Vec2 c2{3, 4};
    const Mat22 foo{c1, c2};
    const auto inverted = Invert(foo);
    const auto inverted2 = Invert(inverted);
    EXPECT_EQ(GetX(GetX(foo)), GetX(GetX(inverted2)));
    EXPECT_EQ(GetY(GetX(foo)), GetY(GetX(inverted2)));
    EXPECT_EQ(GetX(GetY(foo)), GetX(GetY(inverted2)));
    EXPECT_EQ(GetY(GetY(foo)), GetY(GetY(inverted2)));
}
