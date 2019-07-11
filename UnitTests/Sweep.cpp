/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Common/Math.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Sweep, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Sweep), std::size_t(36)); break;
        case  8: EXPECT_EQ(sizeof(Sweep), std::size_t(72)); break;
        case 16: EXPECT_EQ(sizeof(Sweep), std::size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(Sweep, ConstructorSetsPos0and1) {
    const auto pos = Position{Length2{-0.4_m, 2.34_m}, 3.14_rad};
    Sweep sweep{pos};
    EXPECT_EQ(pos, sweep.pos0);
    EXPECT_EQ(pos, sweep.pos1);
}

TEST(Sweep, ResetSetsAlpha0to0) {
    const auto pos = Position{Length2{-0.4_m, 2.34_m}, 3.14_rad};
    Sweep sweep{pos, pos, Length2{}, Real(0.6)};
    EXPECT_NE(Real{0}, sweep.GetAlpha0());
    sweep.ResetAlpha0();
    EXPECT_EQ(Real{0}, sweep.GetAlpha0());    
}

TEST(Sweep, GetPosition) {
    const auto pos0 = Position{Length2{-0.4_m, +2.34_m}, 3.14_rad};
    const auto pos1 = Position{Length2{+0.4_m, -2.34_m}, -3.14_rad};
    Sweep sweep{pos0, pos1, Length2{}, Real(0.6)};
    EXPECT_EQ(pos0, GetPosition(sweep.pos0, sweep.pos1, 0));
    EXPECT_EQ(pos1, GetPosition(sweep.pos0, sweep.pos1, 1));
}

TEST(Sweep, GetPositionAngleEdgeCase) {
    const auto pos0 = Position{Length2{-0.4_m, +2.34_m}, 3.13_rad};
    const auto pos1 = Position{Length2{+0.4_m, -2.34_m}, -3.13_rad};
    Sweep sweep{pos0, pos1, Length2{}, Real(0.6)};
    EXPECT_NEAR(3.1358_rad, GetPosition(sweep.pos0, sweep.pos1, 0.25).angular,0.0001);
}

TEST(Sweep, Advance) {
    const auto pos0 = Position{Length2{-0.4_m, +2.34_m}, 3.14_rad};
    const auto pos1 = Position{Length2{+0.4_m, -2.34_m}, -3.14_rad};
    
    Sweep sweep{pos0, pos1, Length2{}, 0};
    EXPECT_EQ(Real{0}, sweep.GetAlpha0());
    
    sweep.Advance0(0);
    EXPECT_EQ(Real{0}, sweep.GetAlpha0());
    EXPECT_EQ(pos0, sweep.pos0);
    EXPECT_EQ(pos1, sweep.pos1);
    
    sweep.Advance0(Real{1}/Real{2});
    EXPECT_EQ(Real{1}/Real{2}, sweep.GetAlpha0());
    EXPECT_EQ(pos1, sweep.pos1);
    EXPECT_EQ((Position{Length2{}, -Pi}), sweep.pos0);

    sweep.Advance0(0);
    EXPECT_EQ(Real{0}, sweep.GetAlpha0());
    EXPECT_EQ(pos0, sweep.pos0);
    EXPECT_EQ(pos1, sweep.pos1);
}

TEST(Sweep, GetNormalized)
{
    const auto sweep0 = Sweep{
        Position{Length2{}, 0_deg},
        Position{Length2{}, 0_deg}
    };
    EXPECT_EQ(GetNormalized(sweep0).pos0.angular, 0_deg);
    EXPECT_EQ(GetNormalized(sweep0).pos1.angular, 0_deg);

    const auto sweep1 = Sweep{Position{Length2{}, 90_deg}, Position{Length2{}, 90_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep1).pos0.angular / Degree}),
                double( 90), 0.03);
    EXPECT_NEAR(double(Real{GetNormalized(sweep1).pos1.angular / Degree}),
                double( 90), 0.03);

    const auto sweep2 = Sweep{Position{Length2{}, 180_deg}, Position{Length2{}, 180_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep2).pos0.angular / Degree}),
                double(180), 0.03);
    EXPECT_NEAR(double(Real{GetNormalized(sweep2).pos1.angular / Degree}),
                double(180), 0.03);

    const auto sweep3 = Sweep{Position{Length2{}, 270_deg}, Position{Length2{}, 270_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep3).pos0.angular / Degree}), -90.0, 0.03);
    EXPECT_NEAR(double(Real{GetNormalized(sweep3).pos1.angular / Degree}), -90.0, 0.03);

    const auto sweep4 = Sweep{Position{Length2{}, 361_deg}, Position{Length2{}, 361_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep4).pos0.angular / Degree}),
                double(1), 0.001);
    EXPECT_NEAR(double(Real{GetNormalized(sweep4).pos1.angular / Degree}),
                double(1), 0.001);

    const auto sweep5 = Sweep{Position{Length2{}, 722_deg}, Position{Length2{}, 722_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep5).pos0.angular / Degree}),
                double(2), 0.002);
    EXPECT_NEAR(double(Real{GetNormalized(sweep5).pos1.angular / Degree}),
                double(2), 0.002);

    const auto sweep6 = Sweep{Position{Length2{}, 726_deg}, Position{Length2{}, 90_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep6).pos0.angular / Degree}),
                double(6), 0.03);
    EXPECT_NEAR(double(Real{GetNormalized(sweep6).pos1.angular / Degree}),
                double(-630), 0.03);
    
    const auto sweep7 = Sweep{Position{Length2{}, -90_deg}, Position{Length2{}, -90_deg}};
    EXPECT_NEAR(double(Real{GetNormalized(sweep7).pos0.angular / Degree}),
                double( -90), 0.03);
    EXPECT_NEAR(double(Real{GetNormalized(sweep7).pos1.angular / Degree}),
                double( -90), 0.03);
}
