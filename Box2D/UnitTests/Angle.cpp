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
#include <Box2D/Common/Angle.hpp>
#include <Box2D/Common/Math.hpp>

using namespace box2d;

TEST(Angle, ByteSizeIs_4_8_or_16)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(Angle), size_t(4)); break;
		case  8: EXPECT_EQ(sizeof(Angle), size_t(8)); break;
		case 16: EXPECT_EQ(sizeof(Angle), size_t(16)); break;
		default: FAIL(); break;
	}
}

TEST(Angle, GetFromRadiansMatchesToRadians)
{
	EXPECT_EQ(Angle::GetFromRadians(Pi).ToRadians(), RealNum(Pi));
	EXPECT_EQ(Angle::GetFromRadians(-Pi).ToRadians(), -Pi);
	EXPECT_EQ(Angle::GetFromRadians(0.0f).ToRadians(), 0.0f);
	EXPECT_EQ(Angle::GetFromRadians(-101.8f).ToRadians(), -101.8f);
}

TEST(Angle, GetRevRotationalAngle)
{
	EXPECT_EQ(GetRevRotationalAngle(0_deg, 0_deg), 0_deg);
	EXPECT_EQ(GetRevRotationalAngle(0_deg, 10_deg), 10_deg);
	// GetRevRotationalAngle(100_deg, 110_deg) almost equals 10_deg (but not exactly)
	EXPECT_NEAR(double(GetRevRotationalAngle(100_deg, 110_deg) / 1_deg), double(10_deg / 1_deg), 0.0001);
	EXPECT_NEAR(double(GetRevRotationalAngle(10_deg, 0_deg) / 1_deg), double(350_deg / 1_deg), 0.0001);
	EXPECT_EQ(GetRevRotationalAngle(-10_deg, 0_deg), 10_deg);
	EXPECT_EQ(GetRevRotationalAngle(90_deg, -90_deg), 180_deg);
}

TEST(Angle, GetNormalized)
{
	EXPECT_EQ(GetNormalized(0_deg) / 1_deg, 0_deg / 1_deg);
	EXPECT_EQ(GetNormalized(90_deg) / 1_deg, 90_deg / 1_deg);
	EXPECT_EQ(GetNormalized(180_deg) / 1_deg, 180_deg / 1_deg);
	EXPECT_EQ(GetNormalized(270_deg) / 1_deg, 270_deg / 1_deg);
	EXPECT_EQ(GetNormalized(360_deg) / 1_deg, 0_deg / 1_deg);
	EXPECT_EQ(round(GetNormalized(395_deg) / 1_deg, 1000), round(35_deg / 1_deg, 1000));
	EXPECT_EQ(GetNormalized(720_deg) / 1_deg, 0_deg / 1_deg);
	EXPECT_EQ(round(GetNormalized(733_deg) / 1_deg, 1000), round(13_deg / 1_deg, 1000));
	EXPECT_EQ(GetNormalized(-45_deg) / 1_deg, -45_deg / 1_deg);
	EXPECT_EQ(GetNormalized(-90_deg) / 1_deg, -90_deg / 1_deg);
	EXPECT_EQ(round(GetNormalized(-3610_deg) / 1_deg, 1000), round(-10_deg / 1_deg, 1000));
}
