/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

TEST(Sweep, ByteSizeIs_36_or_72)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(Sweep), size_t(36)); break;
        case  8: EXPECT_EQ(sizeof(Sweep), size_t(72)); break;
        case 16: EXPECT_EQ(sizeof(Sweep), size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(Sweep, ConstructorSetsPos0and1) {
    const auto pos = Position{Vec2{RealNum(-0.4), RealNum(2.34)} * Meter, RealNum{3.14f} * Radian};
    Sweep sweep{pos};
    EXPECT_EQ(pos, sweep.pos0);
    EXPECT_EQ(pos, sweep.pos1);
}

TEST(Sweep, ResetSetsAlpha0to0) {
    const auto pos = Position{Vec2{RealNum(-0.4), RealNum(2.34)} * Meter, RealNum{3.14f} * Radian};
    Sweep sweep{pos, pos, Vec2_zero * Meter, RealNum(0.6)};
    EXPECT_NE(RealNum{0}, sweep.GetAlpha0());
    sweep.ResetAlpha0();
    EXPECT_EQ(RealNum{0}, sweep.GetAlpha0());    
}

TEST(Sweep, GetPosition) {
    const auto pos0 = Position{Vec2{RealNum(-0.4), RealNum(+2.34)} * Meter, RealNum{3.14f} * Radian};
    const auto pos1 = Position{Vec2{RealNum(+0.4), RealNum(-2.34)} * Meter, -RealNum{3.14f} * Radian};
    Sweep sweep{pos0, pos1, Vec2_zero * Meter, RealNum(0.6)};
    EXPECT_EQ(pos0, GetPosition(sweep.pos0, sweep.pos1, 0));
    EXPECT_EQ(pos1, GetPosition(sweep.pos0, sweep.pos1, 1));
}

TEST(Sweep, Advance) {
    const auto pos0 = Position{Vec2{RealNum(-0.4), RealNum(+2.34)} * Meter, RealNum{3.14f} * Radian};
    const auto pos1 = Position{Vec2{RealNum(+0.4), RealNum(-2.34)} * Meter, -RealNum{3.14f} * Radian};
    
    Sweep sweep{pos0, pos1, Vec2_zero * Meter, 0};
    EXPECT_EQ(RealNum{0}, sweep.GetAlpha0());
    
    sweep.Advance0(0);
    EXPECT_EQ(RealNum{0}, sweep.GetAlpha0());
    EXPECT_EQ(pos0, sweep.pos0);
    EXPECT_EQ(pos1, sweep.pos1);
    
    sweep.Advance0(RealNum{1}/RealNum{2});
    EXPECT_EQ(RealNum{1}/RealNum{2}, sweep.GetAlpha0());
    EXPECT_EQ(pos1, sweep.pos1);
    EXPECT_EQ((Position{Vec2{0, 0} * Meter, Angle{0}}), sweep.pos0);

    sweep.Advance0(0);
    EXPECT_EQ(RealNum{0}, sweep.GetAlpha0());
    EXPECT_EQ(pos0, sweep.pos0);
    EXPECT_EQ(pos1, sweep.pos1);
}

TEST(Sweep, GetAnglesNormalized)
{
    const auto sweep0 = Sweep{
        Position{Vec2{0,0} * Meter, RealNum{0.0f} * Degree},
        Position{Vec2{0,0} * Meter, RealNum{0.0f} * Degree}
    };
    EXPECT_EQ(GetAnglesNormalized(sweep0).pos0.angular, RealNum{0.0f} * Degree);
    EXPECT_EQ(GetAnglesNormalized(sweep0).pos1.angular, RealNum{0.0f} * Degree);

    const auto sweep1 = Sweep{
        Position{Vec2{0,0} * Meter, RealNum{90.0f} * Degree},
        Position{Vec2{0,0} * Meter, RealNum{90.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep1).pos0.angular / Degree}),
                double( 90), 0.03);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep1).pos1.angular / Degree}),
                double( 90), 0.03);

    const auto sweep2 = Sweep{
        Position{Vec2{0,0} * Meter,RealNum{180.0f} * Degree},
        Position{Vec2{0,0} * Meter,RealNum{180.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep2).pos0.angular / Degree}),
                double(180), 0.03);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep2).pos1.angular / Degree}),
                double(180), 0.03);

    const auto sweep3 = Sweep{
        Position{Vec2{0,0} * Meter,RealNum{270.0f} * Degree},
        Position{Vec2{0,0} * Meter,RealNum{270.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep3).pos0.angular / Degree}),
                double(270), 0.03);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep3).pos1.angular / Degree}),
                       double(270), 0.03);

    const auto sweep4 = Sweep{
        Position{Vec2{0,0} * Meter,RealNum{361.0f} * Degree},
        Position{Vec2{0,0} * Meter,RealNum{361.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep4).pos0.angular / Degree}),
                double(1), 0.001);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep4).pos1.angular / Degree}),
                double(1), 0.001);

    const auto sweep5 = Sweep{
        Position{Vec2{0,0} * Meter,RealNum{722.0f} * Degree},
        Position{Vec2{0,0} * Meter,RealNum{722.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep5).pos0.angular / Degree}),
                double(2), 0.002);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep5).pos1.angular / Degree}),
                double(2), 0.002);

    const auto sweep6 = Sweep{
        Position{Vec2{0,0} * Meter,RealNum{726.0f} * Degree},
        Position{Vec2{0,0} * Meter, RealNum{90.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep6).pos0.angular / Degree}),
                double(6), 0.03);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep6).pos1.angular / Degree}),
                double(-630), 0.03);
    
    const auto sweep7 = Sweep{
        Position{Vec2{0,0} * Meter,-RealNum{90.0f} * Degree},
        Position{Vec2{0,0} * Meter,-RealNum{90.0f} * Degree}
    };
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep7).pos0.angular / Degree}),
                double( -90), 0.03);
    EXPECT_NEAR(double(RealNum{GetAnglesNormalized(sweep7).pos1.angular / Degree}),
                double( -90), 0.03);
}
