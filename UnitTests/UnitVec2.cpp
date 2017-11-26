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
#include <PlayRho/Common/UnitVec2.hpp>
#include <PlayRho/Common/Math.hpp>
#include <iostream>

using namespace playrho;

TEST(UnitVec2, ByteSize)
{
    switch (sizeof(Real))
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
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::GetRight())),
                static_cast<double>(GetX(UnitVec2::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::GetRight())),
                static_cast<double>(GetY(UnitVec2::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::GetTop())),
                static_cast<double>(GetX(UnitVec2::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::GetTop())),
                static_cast<double>(GetY(UnitVec2::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::GetLeft())),
                static_cast<double>(GetX(UnitVec2::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::GetLeft())),
                static_cast<double>(GetY(UnitVec2::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::GetBottom())),
                static_cast<double>(GetX(UnitVec2::GetBottom())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::GetBottom())),
                static_cast<double>(GetY(UnitVec2::GetBottom())), 0.0001);
}

TEST(UnitVec2, ByAngleInRadiansNearOriented)
{
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::Get((Pi * Real(0) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec2::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::Get((Pi * Real(0) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec2::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::Get((Pi * Real(1) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec2::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::Get((Pi * Real(1) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec2::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::Get((Pi * Real(2) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec2::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::Get((Pi * Real(2) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec2::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec2::Get((Pi * Real(3) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec2::GetBottom())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec2::Get((Pi * Real(3) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec2::GetBottom())), 0.0001);
}

TEST(UnitVec2, GetForInvalid)
{
    {
        const auto x = GetInvalid<Real>();
        const auto y = GetInvalid<Real>();
        auto magnitude = GetInvalid<Real>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude)));
    }
    {
        const auto x = GetInvalid<Real>();
        const auto y = Real(0);
        auto magnitude = GetInvalid<Real>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude)));
    }
    {
        const auto x = Real(0);
        const auto y = GetInvalid<Real>();
        auto magnitude = GetInvalid<Real>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude)));
    }
    {
        const auto x = Real(0);
        const auto y = Real(0);
        auto magnitude = GetInvalid<Real>();
        EXPECT_FALSE(IsValid(UnitVec2::Get(x, y, magnitude, UnitVec2::GetDefaultFallback())));
        EXPECT_EQ(magnitude, Real(0));
    }
    {
        const auto x = Real(0);
        const auto y = Real(0);
        auto magnitude = GetInvalid<Real>();
        EXPECT_EQ(UnitVec2::Get(x, y, magnitude, UnitVec2::GetZero()), UnitVec2::GetZero());
        EXPECT_EQ(magnitude, Real(0));
        EXPECT_EQ(GetX(UnitVec2::Get(x, y, magnitude, UnitVec2::GetZero())), Real(0));
        EXPECT_EQ(GetY(UnitVec2::Get(x, y, magnitude, UnitVec2::GetZero())), Real(0));
    }
}

TEST(UnitVec2, Absolute)
{
    EXPECT_EQ(UnitVec2::GetZero().Absolute(), UnitVec2::GetZero());
    EXPECT_EQ(UnitVec2::GetBottom().Absolute(), UnitVec2::GetTop());
    EXPECT_EQ(UnitVec2::GetTop().Absolute(), UnitVec2::GetTop());
    EXPECT_EQ(UnitVec2::GetLeft().Absolute(), UnitVec2::GetRight());
    EXPECT_EQ(UnitVec2::GetRight().Absolute(), UnitVec2::GetRight());

    Real magnitude;
    EXPECT_EQ(UnitVec2::Get(Real(-1), Real(-1), magnitude).Absolute(),
              UnitVec2::Get(Real(+1), Real(+1), magnitude));
}

TEST(UnitVec2, RotateMethod)
{
    EXPECT_EQ(UnitVec2::GetRight().Rotate(UnitVec2::GetRight()), UnitVec2::GetRight());
    EXPECT_EQ(UnitVec2::GetTop().Rotate(UnitVec2::GetRight()), UnitVec2::GetTop());
    EXPECT_EQ(UnitVec2::GetLeft().Rotate(UnitVec2::GetRight()), UnitVec2::GetLeft());
    EXPECT_EQ(UnitVec2::GetBottom().Rotate(UnitVec2::GetRight()), UnitVec2::GetBottom());

    EXPECT_EQ(UnitVec2::GetRight().Rotate(UnitVec2::GetTop()), UnitVec2::GetTop());
    EXPECT_EQ(UnitVec2::GetTop().Rotate(UnitVec2::GetTop()), UnitVec2::GetLeft());
    EXPECT_EQ(UnitVec2::GetLeft().Rotate(UnitVec2::GetTop()), UnitVec2::GetBottom());
    EXPECT_EQ(UnitVec2::GetBottom().Rotate(UnitVec2::GetTop()), UnitVec2::GetRight());
    
    EXPECT_EQ(UnitVec2::GetRight().Rotate(UnitVec2::GetLeft()), UnitVec2::GetLeft());
    EXPECT_EQ(UnitVec2::GetTop().Rotate(UnitVec2::GetLeft()), UnitVec2::GetBottom());
    EXPECT_EQ(UnitVec2::GetLeft().Rotate(UnitVec2::GetLeft()), UnitVec2::GetRight());
    EXPECT_EQ(UnitVec2::GetBottom().Rotate(UnitVec2::GetLeft()), UnitVec2::GetTop());
}

TEST(UnitVec2, RotateFunction)
{
    EXPECT_EQ(Rotate(UnitVec2::GetRight(), UnitVec2::GetRight()), UnitVec2::GetRight());
    EXPECT_EQ(Rotate(UnitVec2::GetTop(), UnitVec2::GetRight()), UnitVec2::GetTop());
    EXPECT_EQ(Rotate(UnitVec2::GetLeft(), UnitVec2::GetRight()), UnitVec2::GetLeft());
    EXPECT_EQ(Rotate(UnitVec2::GetBottom(), UnitVec2::GetRight()), UnitVec2::GetBottom());
    
    EXPECT_EQ(Rotate(UnitVec2::GetRight(), UnitVec2::GetTop()), UnitVec2::GetTop());
    EXPECT_EQ(Rotate(UnitVec2::GetTop(), UnitVec2::GetTop()), UnitVec2::GetLeft());
    EXPECT_EQ(Rotate(UnitVec2::GetLeft(), UnitVec2::GetTop()), UnitVec2::GetBottom());
    EXPECT_EQ(Rotate(UnitVec2::GetBottom(), UnitVec2::GetTop()), UnitVec2::GetRight());
    
    EXPECT_EQ(Rotate(UnitVec2::GetRight(), UnitVec2::GetLeft()), UnitVec2::GetLeft());
    EXPECT_EQ(Rotate(UnitVec2::GetTop(), UnitVec2::GetLeft()), UnitVec2::GetBottom());
    EXPECT_EQ(Rotate(UnitVec2::GetLeft(), UnitVec2::GetLeft()), UnitVec2::GetRight());
    EXPECT_EQ(Rotate(UnitVec2::GetBottom(), UnitVec2::GetLeft()), UnitVec2::GetTop());
}

TEST(UnitVec2, Copy)
{
    const auto a = UnitVec2{};
    auto b = a;
    auto c = UnitVec2{};
    c = a;
    EXPECT_EQ(a, b);
}

TEST(UnitVec2, StreamOut)
{
    {
        std::ostringstream os;
        os << UnitVec2::GetLeft();
        EXPECT_STREQ(os.str().c_str(), "UnitVec2(-1,0)");
    }
    {
        std::ostringstream os;
        os << UnitVec2::GetTop();
        EXPECT_STREQ(os.str().c_str(), "UnitVec2(0,1)");
    }
    {
        std::ostringstream os;
        os << UnitVec2::GetRight();
        EXPECT_STREQ(os.str().c_str(), "UnitVec2(1,0)");
    }
    {
        std::ostringstream os;
        os << UnitVec2::GetBottom();
        EXPECT_STREQ(os.str().c_str(), "UnitVec2(0,-1)");
    }
}
