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
#include <Box2D/Common/UnitVec2.hpp>

using namespace box2d;

TEST(UnitVec2, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(UnitVec2), std::size_t(8)); break;
        case  8: EXPECT_EQ(sizeof(UnitVec2), std::size_t(16)); break;
        case 16: EXPECT_EQ(sizeof(UnitVec2), std::size_t(32)); break;
        default: FAIL(); break;
    }
}

TEST(UnitVec2, RightIsRevPerpOfBottom)
{
    EXPECT_EQ(UnitVec2::GetRight(), UnitVec2::GetBottom().GetRevPerpendicular());
}

TEST(UnitVec2, TopIsRevPerpOfRight)
{
    EXPECT_EQ(UnitVec2::GetTop(), UnitVec2::GetRight().GetRevPerpendicular());
}

TEST(UnitVec2, LeftIsRevPerpOfTop)
{
    EXPECT_EQ(UnitVec2::GetLeft(), UnitVec2::GetTop().GetRevPerpendicular());
}

TEST(UnitVec2, BottomIsRevPerpOfLeft)
{
    EXPECT_EQ(UnitVec2::GetBottom(), UnitVec2::GetLeft().GetRevPerpendicular());
}


TEST(UnitVec2, RightIsFwdPerpOfTop)
{
    EXPECT_EQ(UnitVec2::GetRight(), UnitVec2::GetTop().GetFwdPerpendicular());
}

TEST(UnitVec2, TopIsFwdPerpOfLeft)
{
    EXPECT_EQ(UnitVec2::GetTop(), UnitVec2::GetLeft().GetFwdPerpendicular());
}

TEST(UnitVec2, LeftIsFwdPerpOfBottom)
{
    EXPECT_EQ(UnitVec2::GetLeft(), UnitVec2::GetBottom().GetFwdPerpendicular());
}

TEST(UnitVec2, BottomIsFwdPerpOfRight)
{
    EXPECT_EQ(UnitVec2::GetBottom(), UnitVec2::GetRight().GetFwdPerpendicular());
}

TEST(UnitVec2, ByAngleInDegreesNearOriented)
{
    EXPECT_NEAR(GetX(UnitVec2(RealNum(0) * Degree)), GetX(UnitVec2::GetRight()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2(RealNum(0) * Degree)), GetY(UnitVec2::GetRight()), 0.0001);
    EXPECT_NEAR(GetX(UnitVec2(RealNum(90) * Degree)), GetX(UnitVec2::GetTop()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2(RealNum(90) * Degree)), GetY(UnitVec2::GetTop()), 0.0001);
    EXPECT_NEAR(GetX(UnitVec2(RealNum(180) * Degree)), GetX(UnitVec2::GetLeft()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2(RealNum(180) * Degree)), GetY(UnitVec2::GetLeft()), 0.0001);
    EXPECT_NEAR(GetX(UnitVec2(RealNum(270) * Degree)), GetX(UnitVec2::GetBottom()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2(RealNum(270) * Degree)), GetY(UnitVec2::GetBottom()), 0.0001);
}

TEST(UnitVec2, ByAngleInRadiansNearOriented)
{
    EXPECT_NEAR(GetX(UnitVec2((Pi * RealNum(0) / RealNum(2)) * Radian)), GetX(UnitVec2::GetRight()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2((Pi * RealNum(0) / RealNum(2)) * Radian)), GetY(UnitVec2::GetRight()), 0.0001);
    EXPECT_NEAR(GetX(UnitVec2((Pi * RealNum(1) / RealNum(2)) * Radian)), GetX(UnitVec2::GetTop()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2((Pi * RealNum(1) / RealNum(2)) * Radian)), GetY(UnitVec2::GetTop()), 0.0001);
    EXPECT_NEAR(GetX(UnitVec2((Pi * RealNum(2) / RealNum(2)) * Radian)), GetX(UnitVec2::GetLeft()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2((Pi * RealNum(2) / RealNum(2)) * Radian)), GetY(UnitVec2::GetLeft()), 0.0001);
    EXPECT_NEAR(GetX(UnitVec2((Pi * RealNum(3) / RealNum(2)) * Radian)), GetX(UnitVec2::GetBottom()), 0.0001);
    EXPECT_NEAR(GetY(UnitVec2((Pi * RealNum(3) / RealNum(2)) * Radian)), GetY(UnitVec2::GetBottom()), 0.0001);
}

TEST(UnitVec2, GetForInvalid)
{
    {
        const auto x = GetInvalid<RealNum>();
        const auto y = GetInvalid<RealNum>();
        auto magnitude = GetInvalid<RealNum>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude)));
        EXPECT_FALSE(IsValid(GetX(UnitVec2::Get(x, y, magnitude))));
        EXPECT_FALSE(IsValid(GetY(UnitVec2::Get(x, y, magnitude))));
    }
    {
        const auto x = GetInvalid<RealNum>();
        const auto y = RealNum(0);
        auto magnitude = GetInvalid<RealNum>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude)));
        EXPECT_FALSE(IsValid(GetX(UnitVec2::Get(x, y, magnitude))));
        EXPECT_FALSE(IsValid(GetY(UnitVec2::Get(x, y, magnitude))));
    }
    {
        const auto x = RealNum(0);
        const auto y = GetInvalid<RealNum>();
        auto magnitude = GetInvalid<RealNum>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude)));
        EXPECT_FALSE(IsValid(GetX(UnitVec2::Get(x, y, magnitude))));
        EXPECT_FALSE(IsValid(GetY(UnitVec2::Get(x, y, magnitude))));
    }
    {
        const auto x = RealNum(0);
        const auto y = RealNum(0);
        auto magnitude = GetInvalid<RealNum>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude, UnitVec2::GetDefaultFallback())));
        EXPECT_EQ(magnitude, RealNum(0));
        EXPECT_FALSE(IsValid(GetX(UnitVec2::Get(x, y, magnitude, UnitVec2::GetDefaultFallback()))));
        EXPECT_FALSE(IsValid(GetY(UnitVec2::Get(x, y, magnitude, UnitVec2::GetDefaultFallback()))));
    }
    {
        const auto x = RealNum(0);
        const auto y = RealNum(0);
        auto magnitude = GetInvalid<RealNum>();
        EXPECT_EQ(UnitVec2::Get(x, y, magnitude, UnitVec2::GetZero()), UnitVec2::GetZero());
        EXPECT_EQ(magnitude, RealNum(0));
        EXPECT_EQ(GetX(UnitVec2::Get(x, y, magnitude, UnitVec2::GetZero())), RealNum(0));
        EXPECT_EQ(GetY(UnitVec2::Get(x, y, magnitude, UnitVec2::GetZero())), RealNum(0));
    }
}

TEST(UnitVec2, Absolute)
{
    EXPECT_EQ(UnitVec2::GetZero().Absolute(), UnitVec2::GetZero());
    EXPECT_EQ(UnitVec2::GetBottom().Absolute(), UnitVec2::GetTop());
    EXPECT_EQ(UnitVec2::GetTop().Absolute(), UnitVec2::GetTop());
    EXPECT_EQ(UnitVec2::GetLeft().Absolute(), UnitVec2::GetRight());
    EXPECT_EQ(UnitVec2::GetRight().Absolute(), UnitVec2::GetRight());

    RealNum magnitude;
    EXPECT_EQ(UnitVec2::Get(RealNum(-1), RealNum(-1), magnitude).Absolute(),
              UnitVec2::Get(RealNum(+1), RealNum(+1), magnitude));
}
