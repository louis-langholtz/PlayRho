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
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/World.hpp>

using namespace box2d;

TEST(StepConf, ByteSizeIs_88_168_or_320)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(StepConf), size_t(88)); break;
		case  8: EXPECT_EQ(sizeof(StepConf), size_t(168)); break;
		case 16: EXPECT_EQ(sizeof(StepConf), size_t(320)); break;
		default: FAIL(); break;
	}
}

TEST(StepConf, maxTranslation)
{
	const auto v = RealNum(1);
	const auto n = std::nextafter(v, RealNum(0));
	const auto inc = v - n;
	ASSERT_GT(inc, RealNum(0));
	ASSERT_LT(inc, RealNum(1));
	const auto max_inc = inc * StepConf{}.maxTranslation;
	EXPECT_GT(max_inc, RealNum(0));
	EXPECT_LT(max_inc, DefaultLinearSlop / 2);
	EXPECT_LT(max_inc, StepConf{}.linearSlop / 2);
	EXPECT_LT(max_inc, StepConf{}.tolerance);
#if 0
	std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	std::cout << " inc=" << inc;
	std::cout << " max_inc=" << max_inc;
	std::cout << " slop=" << DefaultLinearSlop;
	std::cout << std::endl;
#endif
	
	{
		StepConf conf;
		conf.tolerance = RealNum(0.0000001);
		conf.maxTranslation = RealNum(8.0);
		EXPECT_FALSE(IsMaxTranslationWithinTolerance(conf));
	}
}

TEST(StepConf, maxRotation)
{
	const auto v = RealNum(1);
	const auto n = std::nextafter(v, RealNum(0));
	const auto inc = v - n;
	ASSERT_GT(inc, RealNum(0));
	ASSERT_LT(inc, RealNum(1));
	const auto max_inc = inc * StepConf{}.maxRotation / 1_rad;
	EXPECT_GT(max_inc, RealNum(0));
	EXPECT_LT(max_inc, DefaultAngularSlop / 2);
#if 0
	std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	std::cout << " inc=" << inc;
	std::cout << " max_inc=" << max_inc;
	std::cout << " slop=" << DefaultAngularSlop;
	std::cout << std::endl;
#endif
}
