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
#include <PlayRho/Common/Units.hpp>

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

TEST(Units, IsArithmetic)
{
    ASSERT_FALSE(IsArithmetic<void>::value);
    
    ASSERT_TRUE(IsArithmetic<int>::value);
    ASSERT_TRUE(IsArithmetic<float>::value);
    ASSERT_TRUE(IsArithmetic<double>::value);
#ifndef _WIN32
    ASSERT_TRUE(IsArithmetic<Fixed64>::value);
#endif
    
    EXPECT_TRUE(IsArithmetic<Length>::value);
    EXPECT_TRUE(IsArithmetic<Mass>::value);
    EXPECT_TRUE(IsArithmetic<Time>::value);
    EXPECT_TRUE(IsArithmetic<Force>::value);
    EXPECT_TRUE(IsArithmetic<LinearVelocity>::value);
}
