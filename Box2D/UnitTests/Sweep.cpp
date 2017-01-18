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
	const auto pos = Position{Vec2{realnum(-0.4), realnum(2.34)}, 3.14_rad};
	Sweep sweep{pos};
	EXPECT_EQ(pos, sweep.pos0);
	EXPECT_EQ(pos, sweep.pos1);
}

TEST(Sweep, ResetSetsAlpha0to0) {
	const auto pos = Position{Vec2{realnum(-0.4), realnum(2.34)}, 3.14_rad};
	Sweep sweep{pos, pos, Vec2_zero, realnum(0.6)};
	EXPECT_NE(realnum{0}, sweep.GetAlpha0());
	sweep.ResetAlpha0();
	EXPECT_EQ(realnum{0}, sweep.GetAlpha0());	
}

TEST(Sweep, GetPosition) {
	const auto pos0 = Position{Vec2{realnum(-0.4), realnum(+2.34)}, 3.14_rad};
	const auto pos1 = Position{Vec2{realnum(+0.4), realnum(-2.34)}, -3.14_rad};
	Sweep sweep{pos0, pos1, Vec2_zero, realnum(0.6)};
	EXPECT_EQ(pos0, GetPosition(sweep.pos0, sweep.pos1, 0));
	EXPECT_EQ(pos1, GetPosition(sweep.pos0, sweep.pos1, 1));
}

TEST(Sweep, Advance) {
	const auto pos0 = Position{Vec2{realnum(-0.4), realnum(+2.34)}, 3.14_rad};
	const auto pos1 = Position{Vec2{realnum(+0.4), realnum(-2.34)}, -3.14_rad};
	
	Sweep sweep{pos0, pos1, Vec2_zero, 0};
	EXPECT_EQ(realnum{0}, sweep.GetAlpha0());
	
	sweep.Advance0(0);
	EXPECT_EQ(realnum{0}, sweep.GetAlpha0());
	EXPECT_EQ(pos0, sweep.pos0);
	EXPECT_EQ(pos1, sweep.pos1);
	
	sweep.Advance0(realnum{1}/realnum{2});
	EXPECT_EQ(realnum{1}/realnum{2}, sweep.GetAlpha0());
	EXPECT_EQ(pos1, sweep.pos1);
	EXPECT_EQ( (Position{ Vec2{0, 0}, 0_rad }), sweep.pos0);

	sweep.Advance0(0);
	EXPECT_EQ(realnum{0}, sweep.GetAlpha0());
	EXPECT_EQ(pos0, sweep.pos0);
	EXPECT_EQ(pos1, sweep.pos1);
}

TEST(Sweep, GetAnglesNormalized)
{
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},0_deg}, Position{Vec2{0,0},0_deg}}).pos0.angular, 0_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},0_deg}, Position{Vec2{0,0},0_deg}}).pos1.angular, 0_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},90_deg}, Position{Vec2{0,0},90_deg}}).pos0.angular, 90_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},90_deg}, Position{Vec2{0,0},90_deg}}).pos1.angular, 90_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},180_deg}, Position{Vec2{0,0},180_deg}}).pos0.angular, 180_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},180_deg}, Position{Vec2{0,0},180_deg}}).pos1.angular, 180_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},270_deg}, Position{Vec2{0,0},270_deg}}).pos0.angular, 270_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},270_deg}, Position{Vec2{0,0},270_deg}}).pos1.angular, 270_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},360_deg}, Position{Vec2{0,0},360_deg}}).pos0.angular, 0_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},360_deg}, Position{Vec2{0,0},360_deg}}).pos1.angular, 0_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},720_deg}, Position{Vec2{0,0},720_deg}}).pos0.angular, 0_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},720_deg}, Position{Vec2{0,0},720_deg}}).pos1.angular, 0_deg);
	EXPECT_FLOAT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},720_deg}, Position{Vec2{0,0},90_deg}}).pos1.angular.ToRadians(), (-630_deg).ToRadians());
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},-90_deg}, Position{Vec2{0,0},-90_deg}}).pos0.angular, -90_deg);
	EXPECT_EQ(GetAnglesNormalized(Sweep{Position{Vec2{0,0},-90_deg}, Position{Vec2{0,0},-90_deg}}).pos1.angular, -90_deg);
}
