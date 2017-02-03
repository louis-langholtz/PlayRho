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
#include <Box2D/Common/Fixed.hpp>
#include <Box2D/Common/Math.hpp>

using namespace box2d;

TEST(Fixed32, ByteSizeIs4)
{
	EXPECT_EQ(sizeof(Fixed32), size_t(4));
}

TEST(Fixed32, IntConstruction)
{
	EXPECT_EQ(int(Fixed32(-1)), -1);
	EXPECT_EQ(int(Fixed32(+1)), +1);

	const auto range = 30000;
	for (auto i = -range; i < range; ++i)
	{
		EXPECT_EQ(Fixed32(i), i);
	}
}

TEST(Fixed32, FloatConstruction)
{
	EXPECT_EQ(float(Fixed32(-1)), -1.0f);
	EXPECT_EQ(float(Fixed32(+1)), +1.0f);

	const auto range = 30000;
	for (auto i = -range; i < range; ++i)
	{
		EXPECT_EQ(Fixed32(static_cast<float>(i)), i);
	}
}

TEST(Fixed32, Equals)
{
	EXPECT_TRUE(Fixed32(12) == Fixed32(12.0f));
}

TEST(Fixed32, NotEquals)
{
	EXPECT_TRUE(Fixed32(-302) != Fixed32(12.0f));
	EXPECT_FALSE(Fixed32(-302) != Fixed32(-302));
}

TEST(Fixed32, LessThan)
{
	EXPECT_TRUE(Fixed32(-302) < Fixed32(12.0f));
	EXPECT_TRUE(Fixed32(40) < Fixed32(44));
	EXPECT_FALSE(Fixed32(76) < Fixed32(31));
	EXPECT_TRUE(Fixed32(0.001) < Fixed32(0.002));
	EXPECT_TRUE(Fixed32(0.000) < Fixed32(0.001));
}

TEST(Fixed32, GreaterThan)
{
	EXPECT_FALSE(Fixed32(-302) > Fixed32(12.0f));
	EXPECT_FALSE(Fixed32(40) > Fixed32(44));
	EXPECT_TRUE(Fixed32(76) > Fixed32(31));
}

TEST(Fixed32, Addition)
{
	for (auto val = 0; val < 100; ++val)
	{
		Fixed32 a{val};
		Fixed32 b{val};
		EXPECT_EQ(a + b, Fixed32(val * 2));
	}
}

TEST(Fixed32, EqualSubtraction)
{
	for (auto val = 0; val < 100; ++val)
	{
		Fixed32 a{val};
		Fixed32 b{val};
		EXPECT_EQ(a - b, Fixed32(0));
	}
}

TEST(Fixed32, OppositeSubtraction)
{
	for (auto val = 0; val < 100; ++val)
	{
		Fixed32 a{-val};
		Fixed32 b{val};
		EXPECT_EQ(a - b, Fixed32(val * -2));
	}
}

TEST(Fixed32, Multiplication)
{
	for (auto val = 0; val < 100; ++val)
	{
		Fixed32 a{val};
		EXPECT_EQ(a * a, Fixed32(val * val));
	}
	EXPECT_EQ(Fixed32(9) * Fixed32(3), Fixed32(27));
	EXPECT_EQ(Fixed32(-5) * Fixed32(-4), Fixed32(20));
	EXPECT_EQ(Fixed32(0.5) * Fixed32(0.5), Fixed32(0.25));
	EXPECT_EQ(round(Fixed32(-0.05) * Fixed32(0.05), 1000), round(Fixed32(-0.0025), 1000));
	EXPECT_EQ(round(Fixed32(Pi) * 2, 100), round(Fixed32(Pi * 2), 100));
	EXPECT_EQ(Fixed32(181) * Fixed32(181), Fixed32(32761));
}

TEST(Fixed32, Division)
{
	for (auto val = 1; val < 100; ++val)
	{
		Fixed32 a{val};
		EXPECT_EQ(a / a, Fixed32(1));
	}
	EXPECT_EQ(Fixed32(9) / Fixed32(3), Fixed32(3));
	EXPECT_EQ(Fixed32(81) / Fixed32(9), Fixed32(9));
	EXPECT_EQ(Fixed32(-10) / Fixed32(2), Fixed32(-5));
	EXPECT_EQ(Fixed32(1) / Fixed32(2), Fixed32(0.5));
	EXPECT_EQ(Fixed32(7) / Fixed32(3), Fixed32(7.0/3.0));
}

TEST(Fixed32, Sin)
{
	EXPECT_EQ(std::sin(Fixed32(0)), float(0));
	EXPECT_EQ(std::sin(Fixed32(1)), std::sin(1.0f));
	EXPECT_EQ(std::sin(Fixed32(2)), std::sin(2.0f));
	EXPECT_EQ(std::sin(Fixed32(Pi/2)), 1.0f);
}

TEST(Fixed32, Cos)
{
	EXPECT_EQ(std::cos(Fixed32(0)), float(1));
	EXPECT_EQ(std::cos(Fixed32(1)), std::cos(1.0f));
	EXPECT_EQ(std::cos(Fixed32(2)), std::cos(2.0f));
	EXPECT_LT(std::cos(Fixed32(Pi/2)), +0.001f);
	EXPECT_GT(std::cos(Fixed32(Pi/2)), -0.001f);
}

TEST(Fixed32, Max)
{
	const auto max_internal_val = std::numeric_limits<int32_t>::max() - 1;
	const auto max_fixed32 = *reinterpret_cast<const Fixed32*>(&max_internal_val);
	
	EXPECT_EQ(Fixed32::GetMax(), Fixed32::GetMax());
	EXPECT_EQ(Fixed32::GetMax(), max_fixed32);
	//EXPECT_EQ(static_cast<long double>(Fixed32::GetMax()), 131071.99993896484375000000L);
	//std::cout << std::setprecision(22) << static_cast<long double>(Fixed32::GetMax()) << std::endl;
	EXPECT_EQ(static_cast<long double>(Fixed32::GetMax()), 131071.9998779296875L);

	EXPECT_GT(Fixed32::GetMax(), Fixed32(0));
	EXPECT_GT(Fixed32::GetMax(), Fixed32::GetMin());
	EXPECT_GT(Fixed32::GetMax(), Fixed32::GetLowest());
	EXPECT_GT(Fixed32::GetMax(), Fixed32((1 << (31u - Fixed32::FractionBits)) - 1));
}

TEST(Fixed32, Min)
{
	EXPECT_EQ(Fixed32::GetMin(), Fixed32::GetMin());
	EXPECT_EQ(Fixed32::GetMin(), Fixed32(0, 1));
	EXPECT_EQ(static_cast<long double>(Fixed32::GetMin()), 0.00006103515625000000L);

	EXPECT_LT(Fixed32::GetMin(), Fixed32::GetMax());
	
	EXPECT_GT(Fixed32::GetMin(), Fixed32(0));
	EXPECT_GT(Fixed32::GetMin(), Fixed32::GetLowest());
}

TEST(Fixed32, Lowest)
{
	const auto lowest_internal_val = std::numeric_limits<int32_t>::min() + 2;
	const auto lowest_fixed32 = *reinterpret_cast<const Fixed32*>(&lowest_internal_val);

	EXPECT_EQ(Fixed32::GetLowest(), Fixed32::GetLowest());
	EXPECT_EQ(Fixed32::GetLowest(), lowest_fixed32);
	//EXPECT_EQ(static_cast<long double>(Fixed32::GetLowest()), -131072.00000000000000000000L);
	//std::cout << std::setprecision(22) << static_cast<long double>(Fixed32::GetLowest()) << std::endl;
	EXPECT_EQ(static_cast<long double>(Fixed32::GetLowest()), -131071.9998779296875L);

	EXPECT_LT(Fixed32::GetLowest(), Fixed32(0));
	EXPECT_LT(Fixed32::GetLowest(), Fixed32(-((1 << (31u - Fixed32::FractionBits)) - 1), 0u));
	EXPECT_LT(Fixed32::GetLowest(), Fixed32(-((1 << (31u - Fixed32::FractionBits)) - 1), (1u << Fixed32::FractionBits) - 1u));
	EXPECT_EQ(Fixed32::GetLowest(), -Fixed32::GetMax());
}

TEST(Fixed32, SubtractingFromLowestGetsNegativeInfinity)
{
	EXPECT_EQ(Fixed32::GetLowest() - Fixed32::GetMin(), Fixed32::GetNegativeInfinity());
	EXPECT_EQ(Fixed32::GetLowest() - 1, Fixed32::GetNegativeInfinity());
}

TEST(Fixed32, AddingToMaxGetsInfinity)
{
	EXPECT_EQ(Fixed32::GetMax() + Fixed32::GetMin(), Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetMax() + 1, Fixed32::GetInfinity());
}

TEST(Fixed32, MinusInfinityEqualsNegativeInfinity)
{
	EXPECT_EQ(-Fixed32::GetInfinity(), Fixed32::GetNegativeInfinity());
}

TEST(Fixed32, InfinityEqualsMinusNegativeInfinity)
{
	EXPECT_EQ(Fixed32::GetInfinity(), -Fixed32::GetNegativeInfinity());
}

TEST(Fixed32, InifnityTimesPositiveIsInfinity)
{
	EXPECT_EQ(Fixed32::GetInfinity() * 1, Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() * 2, Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() * 0.5, Fixed32::GetInfinity());
}

TEST(Fixed32, InifnityDividedByPositiveIsInfinity)
{
	EXPECT_EQ(Fixed32::GetInfinity() / 1, Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() / 2, Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() / 0.5, Fixed32::GetInfinity());
}

TEST(Fixed32, InifnityTimesNegativeIsNegativeInfinity)
{
	EXPECT_EQ(Fixed32::GetInfinity() * -1, -Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() * -2, -Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() * -0.5, -Fixed32::GetInfinity());
}

TEST(Fixed32, InifnityDividedByNegativeIsNegativeInfinity)
{
	EXPECT_EQ(Fixed32::GetInfinity() / -1, -Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() / -2, -Fixed32::GetInfinity());
	EXPECT_EQ(Fixed32::GetInfinity() / -0.5, -Fixed32::GetInfinity());
}

TEST(Fixed32, NaN)
{
	EXPECT_TRUE(std::isnan(Fixed32::GetNaN()));
	EXPECT_TRUE(std::isnan(Fixed32::GetInfinity() / Fixed32::GetInfinity()));

	EXPECT_FALSE(std::isnan(Fixed32{0}));
	EXPECT_FALSE(std::isnan(Fixed32{10.0f}));
	EXPECT_FALSE(std::isnan(Fixed32{-10.0f}));
	EXPECT_FALSE(std::isnan(Fixed32::GetInfinity()));
	EXPECT_FALSE(std::isnan(Fixed32::GetNegativeInfinity()));
	EXPECT_FALSE(std::isnan(Fixed32::GetMax()));
	EXPECT_FALSE(std::isnan(Fixed32::GetMin()));
	EXPECT_FALSE(std::isnan(Fixed32::GetLowest()));
}

TEST(Fixed32, InfinityTimesZeroIsNaN)
{
	EXPECT_TRUE(std::isnan(Fixed32::GetInfinity() * 0));
}

TEST(Fixed32, Comparators)
{
	EXPECT_FALSE(Fixed32::GetNaN() > 0.0f);
	EXPECT_FALSE(Fixed32::GetNaN() < 0.0f);
	EXPECT_FALSE(Fixed32::GetNaN() == 0.0f);
	EXPECT_TRUE(Fixed32::GetNaN() != 0.0f);
	EXPECT_FALSE(Fixed32::GetNaN() == Fixed32::GetNaN());
}
