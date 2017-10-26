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
    EXPECT_EQ(1_km, Meter * Kilo);
    EXPECT_EQ(1_Gm, Meter * Giga);
    
    // Linear velocities...
    EXPECT_EQ(1_mps, MeterPerSecond);
    EXPECT_EQ(1_kps, MeterPerSecond * Kilo);
}
