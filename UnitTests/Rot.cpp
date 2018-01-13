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

#if 0
TEST(Rot, ByteSizeIs8)
{
    EXPECT_EQ(sizeof(Rot), std::size_t(8));
}

TEST(Rot, sin)
{
    Rot rot0(0_deg);
    Rot rot90(1_rad * Pi / 2);
    Rot rot180(1_rad * Pi);
    Rot rot270(1_rad * 3 * Pi / 2);
    Rot rot360(1_rad * 2 * Pi);
    
    EXPECT_EQ(Real{0}, Round(rot0.sin()));
    EXPECT_EQ(Real{1}, Round(rot90.sin()));
    EXPECT_EQ(Real{0}, Round(rot180.sin()));
    EXPECT_EQ(Real{-1}, Round(rot270.sin()));
    EXPECT_EQ(Real{0}, Round(rot360.sin()));
    EXPECT_EQ(Round(rot0.sin()), Round(rot360.sin()));
    EXPECT_EQ(0.0f, Round(std::asin(rot360.sin())));
}

TEST(Rot, cos)
{
    Rot rot0(0_deg);
    Rot rot90(1_rad * Pi / 2);
    Rot rot180(1_rad * 1 * Pi);
    Rot rot360(1_rad * 2 * Pi);
    EXPECT_EQ(Round(rot0.cos()), Round(rot360.cos()));
    EXPECT_EQ(Real{1}, Round(rot0.cos()));
    EXPECT_EQ(Real{-1}, Round(rot180.cos()));
    EXPECT_EQ(Real{1}, Round(rot360.cos()));

    EXPECT_EQ(Real{0}, Round(rot90.cos()));
}

TEST(Rot, Add)
{
    Rot rot0(0_deg);
    Rot rot90(1_rad * Pi / 2);
    Rot rot180(1_rad * Pi);
    Rot rot270(1_rad * 3 * Pi / 2);
    Rot rot360(1_rad * 2 * Pi);

    EXPECT_EQ(Round(Real(0)), Round(DegreesToRadians(0)));
    EXPECT_EQ(Round(Pi/2), Round(DegreesToRadians(90)));
    EXPECT_EQ(Round(Pi), Round(DegreesToRadians(180)));
    EXPECT_EQ(Round(3*Pi/2), Round(DegreesToRadians(270)));
    EXPECT_EQ(Round(2*Pi), Round(DegreesToRadians(360)));
    
    EXPECT_EQ(rot0, rot0.Rotate(rot0));    
    EXPECT_EQ(rot90, rot0.Rotate(rot90));
    EXPECT_EQ(rot180, rot90.Rotate(rot90));
    EXPECT_EQ(Round(ToRadians(rot270)), Round(ToRadians(rot180.Rotate(rot90))));
    EXPECT_EQ(Round(ToRadians(Rot(20_deg))), Round(ToRadians(Rot(30_deg).Rotate(Rot(-10_deg)))));
    EXPECT_EQ(Round(ToRadians(Rot(20_deg))), Round(ToRadians(Rot(-10_deg).Rotate(Rot(30_deg)))));
    EXPECT_EQ(Round(ToRadians(Rot(20_deg))), Round(ToRadians(Rot(10_deg).FlipY().Rotate(Rot(30_deg)))));
    EXPECT_EQ(Round(ToRadians(Rot(20_deg))), Round(ToRadians(Rot(30_deg).Rotate(Rot(10_deg).FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(105_deg))), Round(ToRadians(Rot(45_deg).Rotate(Rot(60_deg)))));
    EXPECT_EQ(Round(ToRadians(Rot(290_deg))), Round(ToRadians(Rot(145_deg).Rotate(Rot(145_deg)))));
    EXPECT_EQ(Round(ToRadians(Rot(64_deg))), Round(ToRadians(Rot(30_deg).Rotate(Rot(34_deg)))));
}

TEST(Rot, Negate)
{
    EXPECT_EQ(Round(DegreesToRadians(0)), Round(ToRadians(Rot(0_deg).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(360_deg))), Round(ToRadians(Rot(0_deg).FlipY())));
    EXPECT_EQ(-Round(DegreesToRadians(45)), Round(ToRadians(Rot(45_deg).FlipY())));
    EXPECT_EQ(-Round(DegreesToRadians(10)), Round(ToRadians(Rot(10_deg).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(315_deg))), Round(ToRadians(Rot(45_deg).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(270_deg))), Round(ToRadians(Rot(90_deg).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(260_deg))), Round(ToRadians(Rot(100_deg).FlipY())));
    EXPECT_EQ(-Round(ToRadians(Rot(180_deg))), Round(ToRadians(Rot(180_deg).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(64_deg))), Round(ToRadians((Rot(30_deg).FlipY()).Rotate(Rot(94_deg)))));
    EXPECT_EQ(Round(ToRadians(Rot(-64_deg))), Round(ToRadians(Rot(30_deg).Rotate(Rot(94_deg).FlipY()))));
}

TEST(Rot, Subtract)
{
    Rot rot0(0_deg);
    Rot rot90(1_rad * Pi / 2);
    Rot rot180(1_rad * Pi);
    Rot rot270(1_rad * 3 * Pi / 2);
    Rot rot360(1_rad * 2 * Pi);

    EXPECT_EQ(Round(ToRadians(rot0)), Round(ToRadians(rot0.Rotate(rot0.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot90)), Round(ToRadians(rot90.Rotate(rot0.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot180)), Round(ToRadians(rot180.Rotate(rot0.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot270)), Round(ToRadians(rot270.Rotate(rot0))));
    
    EXPECT_NE(Round(ToRadians(rot90)), Round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot270)), Round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(-90_deg))), Round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(64_deg))), Round(ToRadians(Rot(34_deg).Rotate(Rot(-30_deg).FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(64_deg))), Round(ToRadians(Rot(94_deg).Rotate(Rot(30_deg).FlipY()))));
}
#endif
