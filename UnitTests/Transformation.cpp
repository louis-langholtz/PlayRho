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

#include "gtest/gtest.h"
#include <PlayRho/Common/Math.hpp>

using namespace playrho;

TEST(Transformation, ByteSizeIs_16_32_or_64)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Transformation), std::size_t(16)); break;
        case  8: EXPECT_EQ(sizeof(Transformation), std::size_t(32)); break;
        case 16: EXPECT_EQ(sizeof(Transformation), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(Transformation, DefaultConstruct)
{
    const Transformation xfm;
    EXPECT_EQ(xfm.p, Length2D{});
    EXPECT_EQ(xfm.q, UnitVec2::GetRight());
}

TEST(Transformation, Initialize)
{
    const auto translation = Length2D{Real(2) * Meter, Real(4) * Meter};
    const auto rotation = UnitVec2::Get(Radian * Real{Pi / 2});
    const Transformation xfm{translation, rotation};
    EXPECT_EQ(translation, xfm.p);
    EXPECT_EQ(rotation, xfm.q);
}

TEST(Transformation, Equality)
{
    const auto translation = Length2D{Real(2) * Meter, Real(4) * Meter};
    const auto rotation = UnitVec2::Get(Radian * Real{Pi / 2});
    const Transformation xfm{translation, rotation};
    EXPECT_EQ(xfm, xfm);
}

TEST(Transformation, Inequality)
{
    const auto translation1 = Length2D{Real(2) * Meter, Real(4) * Meter};
    const auto rotation1 = UnitVec2::Get(Radian * Pi * Real{0.7f});
    const Transformation xfm1{translation1, rotation1};

    const auto translation2 = Length2D{-Real(3) * Meter, Real(37) * Meter};
    const auto rotation2 = UnitVec2::Get(Radian * Pi * Real{0.002f});
    const Transformation xfm2{translation2, rotation2};

    ASSERT_NE(translation1, translation2);
    ASSERT_NE(rotation1, rotation2);
    EXPECT_NE(xfm1, xfm2);
}

TEST(Transformation, Mul)
{
    const auto translation1 = Length2D{Real(2) * Meter, Real(4) * Meter};
    const auto rotation1 = UnitVec2::Get(Radian * Real{Pi / 2});
    const Transformation xfm{translation1, rotation1};

    const auto xfm2 = Mul(xfm, xfm);
    const Vec2 translation2{4, 8};
    const auto rotation2 = UnitVec2::Get(Radian * Pi);

    const auto Ap = xfm.p;
    const auto Bp = xfm.p;
    const auto newP = Ap + Rotate(Bp, xfm.q);
    EXPECT_EQ(GetX(xfm2.p), GetX(newP));
    EXPECT_EQ(GetY(xfm2.p), GetY(newP));
    
    EXPECT_NEAR(double(xfm2.q.cos()), double(rotation2.cos()), 0.0001);
    EXPECT_NEAR(double(xfm2.q.sin()), double(rotation2.sin()), 0.0001);
}

TEST(Transformation, MulSameAsTransformTwice)
{
    const auto translation1 = Length2D{Real(2) * Meter, Real(4) * Meter};
    const auto rotation1 = UnitVec2::Get(Radian * Real{Pi / 2});
    const Transformation xfm{translation1, rotation1};
    const auto xfm2 = Mul(xfm, xfm);

    const auto location = Length2D{Real(-23.4f) * Meter, Real(0.81f) * Meter};
    const auto twice = Transform(Transform(location, xfm), xfm);
    const auto location2 = Transform(location, xfm2);
    EXPECT_NEAR(static_cast<double>(Real{GetX(twice) / Meter}),
                static_cast<double>(Real{GetX(location2) / Meter}), 0.0001);
    EXPECT_NEAR(static_cast<double>(Real{GetY(twice) / Meter}),
                static_cast<double>(Real{GetY(location2) / Meter}), 0.0001);
}
