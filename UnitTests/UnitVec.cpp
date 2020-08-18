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

#include "UnitTests.hpp"
#include <PlayRho/Common/UnitVec.hpp>
#include <PlayRho/Common/Math.hpp>
#include <iostream>
#include <utility>

using namespace playrho;
using namespace playrho::d2;

TEST(UnitVec, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(UnitVec), std::size_t(8)); break;
        case  8: EXPECT_EQ(sizeof(UnitVec), std::size_t(16)); break;
        case 16: EXPECT_EQ(sizeof(UnitVec), std::size_t(32)); break;
        default: FAIL(); break;
    }
}

TEST(UnitVec, RightIsRevPerpOfBottom)
{
    EXPECT_EQ(UnitVec::GetRight(), UnitVec::GetBottom().GetRevPerpendicular());
}

TEST(UnitVec, TopIsRevPerpOfRight)
{
    EXPECT_EQ(UnitVec::GetTop(), UnitVec::GetRight().GetRevPerpendicular());
}

TEST(UnitVec, LeftIsRevPerpOfTop)
{
    EXPECT_EQ(UnitVec::GetLeft(), UnitVec::GetTop().GetRevPerpendicular());
}

TEST(UnitVec, BottomIsRevPerpOfLeft)
{
    EXPECT_EQ(UnitVec::GetBottom(), UnitVec::GetLeft().GetRevPerpendicular());
}


TEST(UnitVec, RightIsFwdPerpOfTop)
{
    EXPECT_EQ(UnitVec::GetRight(), UnitVec::GetTop().GetFwdPerpendicular());
}

TEST(UnitVec, TopIsFwdPerpOfLeft)
{
    EXPECT_EQ(UnitVec::GetTop(), UnitVec::GetLeft().GetFwdPerpendicular());
}

TEST(UnitVec, LeftIsFwdPerpOfBottom)
{
    EXPECT_EQ(UnitVec::GetLeft(), UnitVec::GetBottom().GetFwdPerpendicular());
}

TEST(UnitVec, BottomIsFwdPerpOfRight)
{
    EXPECT_EQ(UnitVec::GetBottom(), UnitVec::GetRight().GetFwdPerpendicular());
}

TEST(UnitVec, ByAngleInDegreesNearOriented)
{
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::GetRight())),
                static_cast<double>(GetX(UnitVec::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::GetRight())),
                static_cast<double>(GetY(UnitVec::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::GetTop())),
                static_cast<double>(GetX(UnitVec::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::GetTop())),
                static_cast<double>(GetY(UnitVec::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::GetLeft())),
                static_cast<double>(GetX(UnitVec::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::GetLeft())),
                static_cast<double>(GetY(UnitVec::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::GetBottom())),
                static_cast<double>(GetX(UnitVec::GetBottom())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::GetBottom())),
                static_cast<double>(GetY(UnitVec::GetBottom())), 0.0001);
}

TEST(UnitVec, ByAngleInRadiansNearOriented)
{
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::Get((Pi * Real(0) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::Get((Pi * Real(0) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec::GetRight())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::Get((Pi * Real(1) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::Get((Pi * Real(1) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec::GetTop())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::Get((Pi * Real(2) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::Get((Pi * Real(2) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec::GetLeft())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetX(UnitVec::Get((Pi * Real(3) / Real(2)) * 1_rad))),
                static_cast<double>(GetX(UnitVec::GetBottom())), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(UnitVec::Get((Pi * Real(3) / Real(2)) * 1_rad))),
                static_cast<double>(GetY(UnitVec::GetBottom())), 0.0001);
}

TEST(UnitVec, GetForInvalid)
{
    {
        const auto x = GetInvalid<Real>();
        const auto y = GetInvalid<Real>();
        EXPECT_FALSE(IsValid(UnitVec::Get(x, y).first));
    }
    {
        const auto x = GetInvalid<Real>();
        const auto y = Real(0);
        EXPECT_FALSE(IsValid(UnitVec::Get(x, y).first));
    }
    {
        const auto x = Real(0);
        const auto y = GetInvalid<Real>();
        EXPECT_FALSE(IsValid(UnitVec::Get(x, y).first));
    }
    {
        const auto x = Real(0);
        const auto y = Real(0);
        EXPECT_FALSE(IsValid(UnitVec::Get(x, y, UnitVec::GetDefaultFallback()).first));
        EXPECT_EQ(UnitVec::Get(x, y, UnitVec::GetDefaultFallback()).second, Real(0));
    }
    {
        const auto x = Real(0);
        const auto y = Real(0);
        EXPECT_EQ(UnitVec::Get(x, y, UnitVec::GetZero()).first, UnitVec::GetZero());
        EXPECT_EQ(UnitVec::Get(x, y, UnitVec::GetZero()).second, Real(0));
        EXPECT_EQ(GetX(UnitVec::Get(x, y, UnitVec::GetZero()).first), Real(0));
        EXPECT_EQ(GetY(UnitVec::Get(x, y, UnitVec::GetZero()).first), Real(0));
    }
}

TEST(UnitVec, Assumptions)
{
    const auto maxReal = std::numeric_limits<Real>::max();
    const auto maxRealSquared = maxReal * maxReal;
    EXPECT_FALSE(isnormal(maxRealSquared));
    const auto hypotMaxReal = hypot(maxReal, Real{0});
    EXPECT_TRUE(isnormal(hypotMaxReal));
    EXPECT_EQ(maxReal, hypotMaxReal);
    EXPECT_EQ(maxReal / hypotMaxReal, Real{1});
}

TEST(UnitVec, Get)
{
    EXPECT_EQ(UnitVec::Get(Real(+1), Real(0)).first, UnitVec::GetRight());
    EXPECT_EQ(UnitVec::Get(Real(-1), Real(0)).first, UnitVec::GetLeft());
    EXPECT_EQ(UnitVec::Get(Real(0), Real(+1)).first, UnitVec::GetTop());
    EXPECT_EQ(UnitVec::Get(Real(0), Real(-1)).first, UnitVec::GetBottom());
    EXPECT_EQ(UnitVec::Get(+std::numeric_limits<Real>::max(), Real(0)).first, UnitVec::GetRight());
    EXPECT_EQ(UnitVec::Get(-std::numeric_limits<Real>::max(), Real(0)).first, UnitVec::GetLeft());
    EXPECT_EQ(UnitVec::Get(Real(0), +std::numeric_limits<Real>::max()).first, UnitVec::GetTop());
    EXPECT_EQ(UnitVec::Get(Real(0), -std::numeric_limits<Real>::max()).first, UnitVec::GetBottom());
    EXPECT_EQ(UnitVec::Get(+std::numeric_limits<Real>::min(), Real(0)).first, UnitVec::GetRight());
    EXPECT_EQ(UnitVec::Get(-std::numeric_limits<Real>::min(), Real(0)).first, UnitVec::GetLeft());
    EXPECT_EQ(UnitVec::Get(Real(0), +std::numeric_limits<Real>::min()).first, UnitVec::GetTop());
    EXPECT_EQ(UnitVec::Get(Real(0), -std::numeric_limits<Real>::min()).first, UnitVec::GetBottom());
    
    {
        const auto foo = std::get<0>(UnitVec::Get(Real(1), Real(1)));
        const auto boo = UnitVec::GetTopRight();
        EXPECT_NEAR(static_cast<double>(GetX(foo)), 0.70710676908493042, 0.000001);
        EXPECT_NEAR(static_cast<double>(GetY(foo)), 0.70710676908493042, 0.000001);
        EXPECT_NEAR(static_cast<double>(GetX(foo)), static_cast<double>(GetX(boo)), 0.000001);
        EXPECT_NEAR(static_cast<double>(GetY(foo)), static_cast<double>(GetY(boo)), 0.000001);
    }
}

TEST(UnitVec, Absolute)
{
    EXPECT_EQ(UnitVec::GetZero().Absolute(), UnitVec::GetZero());
    EXPECT_EQ(UnitVec::GetBottom().Absolute(), UnitVec::GetTop());
    EXPECT_EQ(UnitVec::GetTop().Absolute(), UnitVec::GetTop());
    EXPECT_EQ(UnitVec::GetLeft().Absolute(), UnitVec::GetRight());
    EXPECT_EQ(UnitVec::GetRight().Absolute(), UnitVec::GetRight());

    EXPECT_EQ(UnitVec::Get(Real(-1), Real(-1)).first.Absolute(),
              UnitVec::Get(Real(+1), Real(+1)).first);
}

TEST(UnitVec, RotateMethod)
{
    EXPECT_EQ(UnitVec::GetRight().Rotate(UnitVec::GetRight()), UnitVec::GetRight());
    EXPECT_EQ(UnitVec::GetTop().Rotate(UnitVec::GetRight()), UnitVec::GetTop());
    EXPECT_EQ(UnitVec::GetLeft().Rotate(UnitVec::GetRight()), UnitVec::GetLeft());
    EXPECT_EQ(UnitVec::GetBottom().Rotate(UnitVec::GetRight()), UnitVec::GetBottom());

    EXPECT_EQ(UnitVec::GetRight().Rotate(UnitVec::GetTop()), UnitVec::GetTop());
    EXPECT_EQ(UnitVec::GetTop().Rotate(UnitVec::GetTop()), UnitVec::GetLeft());
    EXPECT_EQ(UnitVec::GetLeft().Rotate(UnitVec::GetTop()), UnitVec::GetBottom());
    EXPECT_EQ(UnitVec::GetBottom().Rotate(UnitVec::GetTop()), UnitVec::GetRight());
    
    EXPECT_EQ(UnitVec::GetRight().Rotate(UnitVec::GetLeft()), UnitVec::GetLeft());
    EXPECT_EQ(UnitVec::GetTop().Rotate(UnitVec::GetLeft()), UnitVec::GetBottom());
    EXPECT_EQ(UnitVec::GetLeft().Rotate(UnitVec::GetLeft()), UnitVec::GetRight());
    EXPECT_EQ(UnitVec::GetBottom().Rotate(UnitVec::GetLeft()), UnitVec::GetTop());
}

TEST(UnitVec, RotateFunction)
{
    EXPECT_EQ(Rotate(UnitVec::GetRight(), UnitVec::GetRight()), UnitVec::GetRight());
    EXPECT_EQ(Rotate(UnitVec::GetTop(), UnitVec::GetRight()), UnitVec::GetTop());
    EXPECT_EQ(Rotate(UnitVec::GetLeft(), UnitVec::GetRight()), UnitVec::GetLeft());
    EXPECT_EQ(Rotate(UnitVec::GetBottom(), UnitVec::GetRight()), UnitVec::GetBottom());
    
    EXPECT_EQ(Rotate(UnitVec::GetRight(), UnitVec::GetTop()), UnitVec::GetTop());
    EXPECT_EQ(Rotate(UnitVec::GetTop(), UnitVec::GetTop()), UnitVec::GetLeft());
    EXPECT_EQ(Rotate(UnitVec::GetLeft(), UnitVec::GetTop()), UnitVec::GetBottom());
    EXPECT_EQ(Rotate(UnitVec::GetBottom(), UnitVec::GetTop()), UnitVec::GetRight());
    
    EXPECT_EQ(Rotate(UnitVec::GetRight(), UnitVec::GetLeft()), UnitVec::GetLeft());
    EXPECT_EQ(Rotate(UnitVec::GetTop(), UnitVec::GetLeft()), UnitVec::GetBottom());
    EXPECT_EQ(Rotate(UnitVec::GetLeft(), UnitVec::GetLeft()), UnitVec::GetRight());
    EXPECT_EQ(Rotate(UnitVec::GetBottom(), UnitVec::GetLeft()), UnitVec::GetTop());
}

TEST(UnitVec, Copy)
{
    const auto a = UnitVec{};
    auto b = a;
    auto c = UnitVec{};
    c = a;
    EXPECT_EQ(a, b);
}

TEST(UnitVec, StreamOut)
{
    {
        std::ostringstream os;
        os << UnitVec::GetLeft();
        EXPECT_STREQ(os.str().c_str(), "UnitVec(-1,0)");
    }
    {
        std::ostringstream os;
        os << UnitVec::GetTop();
        EXPECT_STREQ(os.str().c_str(), "UnitVec(0,1)");
    }
    {
        std::ostringstream os;
        os << UnitVec::GetRight();
        EXPECT_STREQ(os.str().c_str(), "UnitVec(1,0)");
    }
    {
        std::ostringstream os;
        os << UnitVec::GetBottom();
        EXPECT_STREQ(os.str().c_str(), "UnitVec(0,-1)");
    }
}
