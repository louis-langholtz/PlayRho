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
#include <Box2D/Common/Math.hpp>

using namespace box2d;

TEST(Sweep, ByteSizeIs36)
{
	EXPECT_EQ(sizeof(Sweep), size_t(36));
}

TEST(Sweep, ConstructorSetsPos0and1) {
	const auto pos = Position{Vec2{float_t(-0.4), float_t(2.34)}, 3.14_rad};
	Sweep sweep{pos};
	EXPECT_EQ(pos, sweep.pos0);
	EXPECT_EQ(pos, sweep.pos1);
}

TEST(Sweep, ResetSetsAlpha0to0) {
	const auto pos = Position{Vec2{float_t(-0.4), float_t(2.34)}, 3.14_rad};
	Sweep sweep{pos, pos, Vec2_zero, float_t(0.6)};
	EXPECT_NE(float_t{0}, sweep.GetAlpha0());
	sweep.ResetAlpha0();
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());	
}

TEST(Sweep, GetPosition) {
	const auto pos0 = Position{Vec2{float_t(-0.4), float_t(+2.34)}, 3.14_rad};
	const auto pos1 = Position{Vec2{float_t(+0.4), float_t(-2.34)}, -3.14_rad};
	Sweep sweep{pos0, pos1, Vec2_zero, float_t(0.6)};
	EXPECT_EQ(pos0, GetPosition(sweep.pos0, sweep.pos1, 0));
	EXPECT_EQ(pos1, GetPosition(sweep.pos0, sweep.pos1, 1));
}

TEST(Sweep, Advance) {
	const auto pos0 = Position{Vec2{float_t(-0.4), float_t(+2.34)}, 3.14_rad};
	const auto pos1 = Position{Vec2{float_t(+0.4), float_t(-2.34)}, -3.14_rad};
	
	Sweep sweep{pos0, pos1, Vec2_zero, 0};
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());
	
	sweep.Advance0(0);
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());
	EXPECT_EQ(pos0, sweep.pos0);
	EXPECT_EQ(pos1, sweep.pos1);
	
	sweep.Advance0(float_t{1}/float_t{2});
	EXPECT_EQ(float_t{1}/float_t{2}, sweep.GetAlpha0());
	EXPECT_EQ(pos1, sweep.pos1);
	EXPECT_EQ( (Position{ Vec2{0, 0}, 0_rad }), sweep.pos0);

	sweep.Advance0(0);
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());
	EXPECT_EQ(pos0, sweep.pos0);
	EXPECT_EQ(pos1, sweep.pos1);
}
