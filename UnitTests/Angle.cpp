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

TEST(Angle, ByteSizeIs_4_8_or_16)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Angle), std::size_t(4)); break;
        case  8: EXPECT_EQ(sizeof(Angle), std::size_t(8)); break;
        case 16: EXPECT_EQ(sizeof(Angle), std::size_t(16)); break;
        default: FAIL(); break;
    }
}

TEST(Angle, DegreeAndRadian)
{
    EXPECT_NEAR(double(Real{Angle{Real(1) * Degree} / Angle{Real(1) * Radian}}),
                double(Real{((Pi * Radian) / Real{180}) / Radian}), 0.0001);
}

TEST(Angle, GetRevRotationalAngle)
{
    EXPECT_EQ(GetRevRotationalAngle(Angle{0}, Angle{0}), Angle{0});
    EXPECT_EQ(GetRevRotationalAngle(Angle{0}, Angle{Real{10.0f} * Degree}),
              Angle{Real{10.0f} * Degree});
    // GetRevRotationalAngle(100 * Degree, 110 * Degree) almost equals 10 * Degree (but not exactly)
    EXPECT_NEAR(double(Real{GetRevRotationalAngle(Angle{Real{100.0f} * Degree}, Angle{Real{110.0f} * Degree}) / Angle{Real{1} * Degree}}),
                double(10), 0.0001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle(Angle{Real{10.0f} * Degree}, Angle{0}) / Angle{Real(1) * Degree}}),
                double(350), 0.0001);
    EXPECT_EQ(GetRevRotationalAngle(Angle{-Real{10.0f} * Degree}, Angle{0}),
              Angle{Real{10.0f} * Degree});
    EXPECT_NEAR(static_cast<double>(Real{GetRevRotationalAngle(Angle{Real{90.0f} * Degree}, Angle{-Real{90.0f} * Degree})/Radian}),
                static_cast<double>(Real{Angle{Real{180.0f} * Degree}/Radian}),
                0.0001);
}

TEST(Angle, GetNormalized)
{
    EXPECT_EQ(GetNormalized(Angle(0)) / Angle{Real(1) * Degree}, Real(0));
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real(  90.0) * Degree}) / Angle{Real(1) * Degree})),  90.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real( 180.0) * Degree}) / Angle{Real(1) * Degree})), 180.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real( 270.0) * Degree}) / Angle{Real(1) * Degree})), 270.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real( 395.0) * Degree}) / Angle{Real(1) * Degree})),  35.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real( 733.0) * Degree}) / Angle{Real(1) * Degree})),  13.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{-Real(  45.0) * Degree}) / Angle{Real(1) * Degree})), -45.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{-Real(  90.0) * Degree}) / Angle{Real(1) * Degree})), -90.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{-Real(3610.0) * Degree}) / Angle{Real(1) * Degree})), -10.0, 0.01);
    
    // Following test doesn't work when Real=long double, presumably because of rounding issues.
    //EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real{ 360.0} * Degree}) / Angle{Real(1) * Degree})),   0.0, 0.0001);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ 2 * Pi * Radian}) / Angle{Real(1) * Radian})),   0.0, 0.0001);
    // Following test doesn't work when Real=long double, presumably because of rounding issues.
    //EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real( 720.0) * Degree}) / Angle{Real(1) * Degree})),   0.0, 0.0001);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ 4 * Pi * Radian}) / Angle{Real(1) * Radian})),   0.0, 0.0001);
}
