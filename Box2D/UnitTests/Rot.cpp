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
	Rot rot0(0);
	Rot rot90(Pi / 2);
	Rot rot180(Pi);
	Rot rot270(3 * Pi / 2);
	Rot rot360(2 * Pi);
	
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
	Rot rot0(0);
	Rot rot90(Pi / 2);
	Rot rot180(1 * Pi);
	Rot rot360(2 * Pi);
	EXPECT_EQ(round(rot0.cos()), round(rot360.cos()));
	EXPECT_EQ(float_t{1}, round(rot0.cos()));
	EXPECT_EQ(float_t{-1}, round(rot180.cos()));
	EXPECT_EQ(float_t{1}, round(rot360.cos()));

	EXPECT_EQ(float_t{0}, round(rot90.cos()));
}

TEST(Rot, Add)
{
	Rot rot0(0);
	Rot rot90(Pi / 2);
	Rot rot180(Pi);
	Rot rot270(3 * Pi / 2);
	Rot rot360(2 * Pi);

	EXPECT_EQ(round(float_t(0)), round(DegreesToRadians(0)));
	EXPECT_EQ(round(Pi/2), round(DegreesToRadians(90)));
	EXPECT_EQ(round(Pi), round(DegreesToRadians(180)));
	EXPECT_EQ(round(3*Pi/2), round(DegreesToRadians(270)));
	EXPECT_EQ(round(2*Pi), round(DegreesToRadians(360)));
	
	EXPECT_EQ(rot0, rot0 + rot0);	
	EXPECT_EQ(rot90, rot0 + rot90);
	EXPECT_EQ(rot180, rot90 + rot90);
	EXPECT_EQ(round(GetAngle(rot270)), round(GetAngle(rot180 + rot90)));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(20)))), round(GetAngle(Rot(DegreesToRadians(30)) + Rot(DegreesToRadians(-10)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(20)))), round(GetAngle(Rot(DegreesToRadians(-10)) + Rot(DegreesToRadians(30)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(20)))), round(GetAngle(-Rot(DegreesToRadians(10)) + Rot(DegreesToRadians(30)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(20)))), round(GetAngle(Rot(DegreesToRadians(30)) + -Rot(DegreesToRadians(10)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(105)))), round(GetAngle(Rot(DegreesToRadians(45)) + Rot(DegreesToRadians(60)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(290)))), round(GetAngle(Rot(DegreesToRadians(145)) + Rot(DegreesToRadians(145)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(64)))), round(GetAngle(Rot(DegreesToRadians(30)) + Rot(DegreesToRadians(34)))));
}

TEST(Rot, Negate)
{
	EXPECT_EQ(round(DegreesToRadians(0)), round(GetAngle(-Rot(DegreesToRadians(0)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(360)))), round(GetAngle(-Rot(DegreesToRadians(0)))));
	EXPECT_EQ(-round(DegreesToRadians(45)), round(GetAngle(-Rot(DegreesToRadians(45)))));
	EXPECT_EQ(-round(DegreesToRadians(10)), round(GetAngle(-Rot(DegreesToRadians(10)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(315)))), round(GetAngle(-Rot(DegreesToRadians(45)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(270)))), round(GetAngle(-Rot(DegreesToRadians(90)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(260)))), round(GetAngle(-Rot(DegreesToRadians(100)))));
	EXPECT_EQ(-round(GetAngle(Rot(DegreesToRadians(180)))), round(GetAngle(-Rot(DegreesToRadians(180)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(64)))), round(GetAngle(-Rot(DegreesToRadians(30)) + Rot(DegreesToRadians(94)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(-64)))), round(GetAngle(Rot(DegreesToRadians(30)) + -Rot(DegreesToRadians(94)))));	
}

TEST(Rot, Subtract)
{
	Rot rot0(0);
	Rot rot90(Pi / 2);
	Rot rot180(Pi);
	Rot rot270(3 * Pi / 2);
	Rot rot360(2 * Pi);

	EXPECT_EQ(round(GetAngle(rot0)), round(GetAngle(rot0 - rot0)));
	EXPECT_EQ(round(GetAngle(rot90)), round(GetAngle(rot90 - rot0)));
	EXPECT_EQ(round(GetAngle(rot180)), round(GetAngle(rot180 - rot0)));
	EXPECT_EQ(round(GetAngle(rot270)), round(GetAngle(rot270 - rot0)));
	
	EXPECT_NE(round(GetAngle(rot90)), round(GetAngle(rot0 - rot90)));
	EXPECT_EQ(round(GetAngle(rot270)), round(GetAngle(rot0 - rot90)));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(-90)))), round(GetAngle(rot0 - rot90)));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(64)))), round(GetAngle(Rot(DegreesToRadians(34)) - Rot(DegreesToRadians(-30)))));
	EXPECT_EQ(round(GetAngle(Rot(DegreesToRadians(64)))), round(GetAngle(Rot(DegreesToRadians(94)) - Rot(DegreesToRadians(30)))));
}
