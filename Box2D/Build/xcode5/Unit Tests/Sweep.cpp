//
//  Sweep.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/3/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/Math.h>

using namespace box2d;

TEST(Sweep, ConstructorSetsPos0and1) {
	const auto pos = Position{Vec2{float_t(-0.4), float_t(2.34)}, float_t(3.14)};
	Sweep sweep{pos};
	EXPECT_EQ(pos, sweep.pos0);
	EXPECT_EQ(pos, sweep.pos1);
}

TEST(Sweep, ResetSetsAlpha0to0) {
	const auto pos = Position{Vec2{float_t(-0.4), float_t(2.34)}, float_t(3.14)};
	Sweep sweep{pos, pos, Vec2_zero, float_t(0.6)};
	EXPECT_NE(float_t{0}, sweep.GetAlpha0());
	sweep.ResetAlpha0();
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());	
}

TEST(Sweep, GetPosition) {
	const auto pos0 = Position{Vec2{float_t(-0.4), float_t(+2.34)}, float_t(+3.14)};
	const auto pos1 = Position{Vec2{float_t(+0.4), float_t(-2.34)}, float_t(-3.14)};
	Sweep sweep{pos0, pos1, Vec2_zero, float_t(0.6)};
	EXPECT_EQ(pos0, GetPosition(sweep.pos0, sweep.pos1, 0));
	EXPECT_EQ(pos1, GetPosition(sweep.pos0, sweep.pos1, 1));
}

TEST(Sweep, Advance) {
	const auto pos0 = Position{Vec2{float_t(-0.4), float_t(+2.34)}, float_t(+3.14)};
	const auto pos1 = Position{Vec2{float_t(+0.4), float_t(-2.34)}, float_t(-3.14)};
	
	Sweep sweep{pos0, pos1, Vec2_zero, 0};
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());
	
	sweep.Advance0(0);
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());
	EXPECT_EQ(pos0, sweep.pos0);
	EXPECT_EQ(pos1, sweep.pos1);
	
	sweep.Advance0(float_t{1}/float_t{2});
	EXPECT_EQ(float_t{1}/float_t{2}, sweep.GetAlpha0());
	EXPECT_EQ(pos1, sweep.pos1);
	EXPECT_EQ( (Position{ Vec2{0, 0}, float_t{0} }), sweep.pos0);

	sweep.Advance0(0);
	EXPECT_EQ(float_t{0}, sweep.GetAlpha0());
	EXPECT_EQ(pos0, sweep.pos0);
	EXPECT_EQ(pos1, sweep.pos1);
}
