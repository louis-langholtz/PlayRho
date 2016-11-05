/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Common/Math.h>

using namespace box2d;

TEST(Rot, ByteSizeIs8)
{
	EXPECT_EQ(sizeof(Rot), size_t(8));
}

TEST(Rot, sin)
{
	Rot rot0(0_deg);
	Rot rot90(1_rad * Pi / 2);
	Rot rot180(1_rad * Pi);
	Rot rot270(1_rad * 3 * Pi / 2);
	Rot rot360(1_rad * 2 * Pi);
	
	EXPECT_EQ(float_t{0}, round(rot0.sin()));
	EXPECT_EQ(float_t{1}, round(rot90.sin()));
	EXPECT_EQ(float_t{0}, round(rot180.sin()));
	EXPECT_EQ(float_t{-1}, round(rot270.sin()));
	EXPECT_EQ(float_t{0}, round(rot360.sin()));
	EXPECT_EQ(round(rot0.sin()), round(rot360.sin()));
	EXPECT_EQ(0.0f, round(std::asin(rot360.sin())));
}

TEST(Rot, cos)
{
	Rot rot0(0_deg);
	Rot rot90(1_rad * Pi / 2);
	Rot rot180(1_rad * 1 * Pi);
	Rot rot360(1_rad * 2 * Pi);
	EXPECT_EQ(round(rot0.cos()), round(rot360.cos()));
	EXPECT_EQ(float_t{1}, round(rot0.cos()));
	EXPECT_EQ(float_t{-1}, round(rot180.cos()));
	EXPECT_EQ(float_t{1}, round(rot360.cos()));

	EXPECT_EQ(float_t{0}, round(rot90.cos()));
}

TEST(Rot, Add)
{
	Rot rot0(0_deg);
	Rot rot90(1_rad * Pi / 2);
	Rot rot180(1_rad * Pi);
	Rot rot270(1_rad * 3 * Pi / 2);
	Rot rot360(1_rad * 2 * Pi);

	EXPECT_EQ(round(float_t(0)), round(DegreesToRadians(0)));
	EXPECT_EQ(round(Pi/2), round(DegreesToRadians(90)));
	EXPECT_EQ(round(Pi), round(DegreesToRadians(180)));
	EXPECT_EQ(round(3*Pi/2), round(DegreesToRadians(270)));
	EXPECT_EQ(round(2*Pi), round(DegreesToRadians(360)));
	
	EXPECT_EQ(rot0, rot0.Rotate(rot0));	
	EXPECT_EQ(rot90, rot0.Rotate(rot90));
	EXPECT_EQ(rot180, rot90.Rotate(rot90));
	EXPECT_EQ(round(ToRadians(rot270)), round(ToRadians(rot180.Rotate(rot90))));
	EXPECT_EQ(round(ToRadians(Rot(20_deg))), round(ToRadians(Rot(30_deg).Rotate(Rot(-10_deg)))));
	EXPECT_EQ(round(ToRadians(Rot(20_deg))), round(ToRadians(Rot(-10_deg).Rotate(Rot(30_deg)))));
	EXPECT_EQ(round(ToRadians(Rot(20_deg))), round(ToRadians(Rot(10_deg).FlipY().Rotate(Rot(30_deg)))));
	EXPECT_EQ(round(ToRadians(Rot(20_deg))), round(ToRadians(Rot(30_deg).Rotate(Rot(10_deg).FlipY()))));
	EXPECT_EQ(round(ToRadians(Rot(105_deg))), round(ToRadians(Rot(45_deg).Rotate(Rot(60_deg)))));
	EXPECT_EQ(round(ToRadians(Rot(290_deg))), round(ToRadians(Rot(145_deg).Rotate(Rot(145_deg)))));
	EXPECT_EQ(round(ToRadians(Rot(64_deg))), round(ToRadians(Rot(30_deg).Rotate(Rot(34_deg)))));
}

TEST(Rot, Negate)
{
	EXPECT_EQ(round(DegreesToRadians(0)), round(ToRadians(Rot(0_deg).FlipY())));
	EXPECT_EQ(round(ToRadians(Rot(360_deg))), round(ToRadians(Rot(0_deg).FlipY())));
	EXPECT_EQ(-round(DegreesToRadians(45)), round(ToRadians(Rot(45_deg).FlipY())));
	EXPECT_EQ(-round(DegreesToRadians(10)), round(ToRadians(Rot(10_deg).FlipY())));
	EXPECT_EQ(round(ToRadians(Rot(315_deg))), round(ToRadians(Rot(45_deg).FlipY())));
	EXPECT_EQ(round(ToRadians(Rot(270_deg))), round(ToRadians(Rot(90_deg).FlipY())));
	EXPECT_EQ(round(ToRadians(Rot(260_deg))), round(ToRadians(Rot(100_deg).FlipY())));
	EXPECT_EQ(-round(ToRadians(Rot(180_deg))), round(ToRadians(Rot(180_deg).FlipY())));
	EXPECT_EQ(round(ToRadians(Rot(64_deg))), round(ToRadians((Rot(30_deg).FlipY()).Rotate(Rot(94_deg)))));
	EXPECT_EQ(round(ToRadians(Rot(-64_deg))), round(ToRadians(Rot(30_deg).Rotate(Rot(94_deg).FlipY()))));	
}

TEST(Rot, Subtract)
{
	Rot rot0(0_deg);
	Rot rot90(1_rad * Pi / 2);
	Rot rot180(1_rad * Pi);
	Rot rot270(1_rad * 3 * Pi / 2);
	Rot rot360(1_rad * 2 * Pi);

	EXPECT_EQ(round(ToRadians(rot0)), round(ToRadians(rot0.Rotate(rot0.FlipY()))));
	EXPECT_EQ(round(ToRadians(rot90)), round(ToRadians(rot90.Rotate(rot0.FlipY()))));
	EXPECT_EQ(round(ToRadians(rot180)), round(ToRadians(rot180.Rotate(rot0.FlipY()))));
	EXPECT_EQ(round(ToRadians(rot270)), round(ToRadians(rot270.Rotate(rot0))));
	
	EXPECT_NE(round(ToRadians(rot90)), round(ToRadians(rot0.Rotate(rot90.FlipY()))));
	EXPECT_EQ(round(ToRadians(rot270)), round(ToRadians(rot0.Rotate(rot90.FlipY()))));
	EXPECT_EQ(round(ToRadians(Rot(-90_deg))), round(ToRadians(rot0.Rotate(rot90.FlipY()))));
	EXPECT_EQ(round(ToRadians(Rot(64_deg))), round(ToRadians(Rot(34_deg).Rotate(Rot(-30_deg).FlipY()))));
	EXPECT_EQ(round(ToRadians(Rot(64_deg))), round(ToRadians(Rot(94_deg).Rotate(Rot(30_deg).FlipY()))));
}
