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

using namespace box2d;

TEST(Angle, ByteSizeIs4)
{
	EXPECT_EQ(sizeof(Angle), size_t(4));
}

TEST(Angle, GetFromRadiansMatchesToRadians)
{
	EXPECT_EQ(Angle::GetFromRadians(Pi).ToRadians(), Pi);
	EXPECT_EQ(Angle::GetFromRadians(-Pi).ToRadians(), -Pi);
	EXPECT_EQ(Angle::GetFromRadians(0.0f).ToRadians(), 0.0f);
	EXPECT_EQ(Angle::GetFromRadians(-101.8f).ToRadians(), -101.8f);
}

TEST(Angle, GetRevRotationalAngle)
{
	EXPECT_EQ(GetRevRotationalAngle(0_deg, 0_deg), 0_deg);
	EXPECT_EQ(GetRevRotationalAngle(0_deg, 10_deg), 10_deg);
	// GetRevRotationalAngle(100_deg, 110_deg) almost equals 10_deg (but not exactly)
	EXPECT_FLOAT_EQ(GetRevRotationalAngle(100_deg, 110_deg) / 1_deg, 10_deg / 1_deg);
	EXPECT_EQ(GetRevRotationalAngle(10_deg, 0_deg), 350_deg);
	EXPECT_EQ(GetRevRotationalAngle(-10_deg, 0_deg), 10_deg);
	EXPECT_EQ(GetRevRotationalAngle(90_deg, -90_deg), 180_deg);
}

TEST(Angle, GetNormalized)
{
	EXPECT_FLOAT_EQ(GetNormalized(0_deg).ToRadians(), (0_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(90_deg).ToRadians(), (90_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(180_deg).ToRadians(), (180_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(270_deg).ToRadians(), (270_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(360_deg).ToRadians(), (0_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(395_deg).ToRadians(), (35_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(720_deg).ToRadians(), (0_deg).ToRadians());
	EXPECT_FLOAT_EQ(std::roundf(GetNormalized(733_deg).ToRadians() * 1000)/1000, std::roundf((13_deg).ToRadians() * 1000)/1000);
	EXPECT_FLOAT_EQ(GetNormalized(-45_deg).ToRadians(), (-45_deg).ToRadians());
	EXPECT_FLOAT_EQ(GetNormalized(-90_deg).ToRadians(), (-90_deg).ToRadians());
	EXPECT_FLOAT_EQ(std::roundf(GetNormalized(-3610_deg).ToRadians() * 1000)/1000, std::roundf((-10_deg).ToRadians() * 1000)/1000);
}
