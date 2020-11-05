/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
    EXPECT_NEAR(double(Real{Degree / 1_rad}),
                double(Real{((Pi * 1_rad) / Real{180}) / 1_rad}), 0.0001);
}

TEST(Angle, GetRevRotationalAngle)
{
    EXPECT_EQ(GetRevRotationalAngle(0_deg, 0_deg), 0_deg);
    EXPECT_EQ(GetRevRotationalAngle(0_deg, 10_deg), 10_deg);
    // GetRevRotationalAngle(100 * Degree, 110 * Degree) almost equals 10 * Degree (but not exactly)
    EXPECT_EQ(GetRevRotationalAngle(-10_deg, 0_deg), 10_deg);
    EXPECT_NEAR(static_cast<double>(Real{GetRevRotationalAngle(90_deg, -90_deg)/1_deg}), 180.0, 0.0001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle(100_deg, 110_deg) / 1_deg}),  10.0, 0.0001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle( 10_deg,   0_deg) / 1_deg}), 350.0, 0.0001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle( -2_deg,  +3_deg) / 1_deg}),   5.0, 0.001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle( +2_deg,  -3_deg) / 1_deg}), 355.0, 0.001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle(-13_deg,  -3_deg) / 1_deg}),  10.0, 0.001);
    EXPECT_NEAR(double(Real{GetRevRotationalAngle(-10_deg, -20_deg) / 1_deg}), 350.0, 0.001);
}

TEST(Angle, GetDelta)
{
    EXPECT_EQ(GetDelta(0_deg, 0_deg), 0_deg);
    EXPECT_NEAR(static_cast<double>(Real{GetDelta(0_deg, 10_deg) / Degree}), 10.0, 0.01);
    // GetDelta(100 * Degree, 110 * Degree) almost equals 10 * Degree (but not exactly)
    EXPECT_NEAR(double(Real{GetDelta(100_deg, 110_deg) / Degree}), 10.0, 0.0001);
    EXPECT_NEAR(double(Real{GetDelta(10_deg, 0_deg) / Degree}), -10.0, 0.0001);
    EXPECT_NEAR(double(Real{GetDelta(-10_deg, 0_deg) / Degree}), 10.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real{GetDelta(+90_deg, -90_deg)/1_deg}), -180.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real{GetDelta(+80_deg, -80_deg)/1_deg}), -160.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real{GetDelta(-90_deg, +90_deg)/1_deg}), +180.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real{GetDelta(-80_deg, +80_deg)/1_deg}), +160.0, 0.0001);
    EXPECT_NEAR(double(Real{GetDelta(-Pi * Radian, +Pi * Radian) / Degree}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetDelta(+Pi * Radian, -Pi * Radian) / Degree}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetDelta(-2_deg, +3_deg) / Degree}), 5.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(+2_deg, -3_deg) / Degree}), -5.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(-13_deg, -3_deg) / Degree}), 10.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(-10_deg, -20_deg) / Degree}), -10.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(10_deg, 340_deg) / Degree}), -30.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(400_deg, 440_deg) / Degree}), 40.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(400_deg, 300_deg) / Degree}), -100.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(400_deg, 100_deg) / Degree}), 60.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(800_deg, 100_deg) / Degree}), 20.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(400_deg, -100_deg) / Degree}), -140.0, 0.01);
    EXPECT_NEAR(double(Real{GetDelta(-400_deg, 10_deg) / Degree}), 50.0, 0.01);
}

TEST(Angle, limits)
{
    EXPECT_EQ(Real(+std::numeric_limits<Angle>::infinity()/Radian), +std::numeric_limits<Real>::infinity());
    EXPECT_EQ(Real(+std::numeric_limits<Angle>::infinity()/Degree), +std::numeric_limits<Real>::infinity());
    EXPECT_EQ(Real(-std::numeric_limits<Angle>::infinity()/Radian), -std::numeric_limits<Real>::infinity());
    EXPECT_EQ(Real(-std::numeric_limits<Angle>::infinity()/Degree), -std::numeric_limits<Real>::infinity());
}

TEST(Angle, GetNormalized)
{
    EXPECT_EQ(GetNormalized(0_deg) / Degree, Real(0));
    EXPECT_NEAR(double(Real(GetNormalized(    0.0_deg) / Degree)),    0.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(   21.3_deg) / Degree)),   21.3, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(   90.0_deg) / Degree)),   90.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(   93.2_deg) / Degree)),   93.2, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  180.0_deg) / Degree)),  180.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  190.0_deg) / Degree)), -170.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized( -180.0_deg) / Degree)), -180.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  Pi*Radian) / Degree)),  180.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized( -Pi*Radian) / Degree)), -180.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  270.0_deg) / Degree)),  -90.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  395.0_deg) / Degree)),   35.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  396.4_deg) / Degree)),   36.4, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  733.0_deg) / Degree)),   13.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  734.5_deg) / Degree)),   14.5, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  -45.0_deg) / Degree)),  -45.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(  -90.0_deg) / Degree)),  -90.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(-3610.0_deg) / Degree)),  -10.0, 0.01);
    EXPECT_NEAR(double(Real(GetNormalized(-3611.2_deg) / Degree)),  -11.2, 0.01);
    EXPECT_TRUE(std::isnan(double(Real(GetNormalized(std::numeric_limits<Angle>::infinity()) / Degree))));
    EXPECT_TRUE(std::isnan(float(Real(GetNormalized(std::numeric_limits<Angle>::infinity()) / Degree))));
    EXPECT_TRUE(std::isnan(float(Real(GetNormalized(std::numeric_limits<Angle>::quiet_NaN()) / Degree))));

    // Following test doesn't work when Real=long double, presumably because of rounding issues.
    //EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real{ 360.0} * Degree}) / Degree)),   0.0, 0.0001);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ 2 * Pi * 1_rad}) / 1_rad)),   0.0, 0.0001);
    // Following test doesn't work when Real=long double, presumably because of rounding issues.
    //EXPECT_NEAR(double(Real(GetNormalized(Angle{ Real( 720.0) * Degree}) / Degree)),   0.0, 0.0001);
    EXPECT_NEAR(double(Real(GetNormalized(Angle{ 4 * Pi * 1_rad}) / 1_rad)),   0.0, 0.0001);
}
