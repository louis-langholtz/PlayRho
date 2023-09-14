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

#include <playrho/Units.hpp>

#include <limits> // for std::numeric_limits

using namespace playrho;

TEST(Units, Literals)
{
    ASSERT_EQ(Centi, Real(1e-2));
    ASSERT_EQ(Deci, Real(1e-1));
    ASSERT_EQ(Kilo, Real(1e3));
    ASSERT_EQ(Giga, Real(1e9));
    ASSERT_EQ(Yotta, Real(1e24));

    // Times...
    EXPECT_EQ(1_s,   Second);
    EXPECT_EQ(1_min, Second * 60);
    EXPECT_EQ(1_h,   Second * 60 * 60);
    EXPECT_EQ(1_d,   Second * 60 * 60 * 24);
    
    // Masses...
    EXPECT_EQ(1_g,  Kilogram / Kilo);
    EXPECT_EQ(1_kg, Kilogram);
    EXPECT_EQ(1_Yg, Yotta * (Kilogram / Kilo));
    
    // Lengths...
    EXPECT_EQ(1_m,  Meter);
    EXPECT_EQ(1.0_m,  Meter);
    EXPECT_EQ(1_km, Meter * Kilo);
    EXPECT_EQ(1.0_km, Meter * Kilo);
    EXPECT_EQ(1_Gm, Meter * Giga);
    EXPECT_EQ(1.0_Gm, Meter * Giga);
    EXPECT_EQ(1_dm, Meter * Deci);
    EXPECT_EQ(1.0_dm, Meter * Deci);
    EXPECT_EQ(1_cm, Meter * Centi);
    EXPECT_EQ(1.0_cm, Meter * Centi);

    // Linear velocities...
    EXPECT_EQ(1_mps, MeterPerSecond);
    EXPECT_EQ(1_kps, MeterPerSecond * Kilo);
    
    // Densities...
    EXPECT_EQ(1_kgpm2, KilogramPerSquareMeter);
    EXPECT_EQ(1.0_kgpm2, KilogramPerSquareMeter);
    
    // Torques...
    EXPECT_EQ(1_Nm, NewtonMeter);
    EXPECT_EQ(1.0_Nm, NewtonMeter);
}

TEST(Units, IsArithmeticV)
{
    ASSERT_FALSE(IsArithmeticV<void>);
    
    ASSERT_TRUE(IsArithmeticV<int>);
    ASSERT_TRUE(IsArithmeticV<float>);
    ASSERT_TRUE(IsArithmeticV<double>);
#ifndef _WIN32
    ASSERT_TRUE(IsArithmeticV<Fixed64>);
#endif
    
    EXPECT_TRUE(IsArithmeticV<Length>);
    EXPECT_TRUE(IsArithmeticV<Mass>);
    EXPECT_TRUE(IsArithmeticV<Time>);
    EXPECT_TRUE(IsArithmeticV<Force>);
    EXPECT_TRUE(IsArithmeticV<LinearVelocity>);
}

TEST(Angle, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
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

TEST(Angle, limits)
{
    EXPECT_EQ(Real(+std::numeric_limits<Angle>::infinity()/Radian), +std::numeric_limits<Real>::infinity());
    EXPECT_EQ(Real(+std::numeric_limits<Angle>::infinity()/Degree), +std::numeric_limits<Real>::infinity());
    EXPECT_EQ(Real(-std::numeric_limits<Angle>::infinity()/Radian), -std::numeric_limits<Real>::infinity());
    EXPECT_EQ(Real(-std::numeric_limits<Angle>::infinity()/Degree), -std::numeric_limits<Real>::infinity());
}
