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
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/World.hpp>

using namespace box2d;

TEST(StepConf, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(StepConf), std::size_t(100)); break;
        case  8: EXPECT_EQ(sizeof(StepConf), std::size_t(192)); break;
        case 16: EXPECT_EQ(sizeof(StepConf), std::size_t(368)); break;
        default: FAIL(); break;
    }
}

TEST(StepConf, CopyConstruction)
{
    const auto dt = Second * RealNum{10};
    const auto displacementMultiplier = RealNum{3.4f};

    StepConf conf;
    conf.SetTime(dt);
    conf.displaceMultiplier = displacementMultiplier;

    ASSERT_EQ(conf.GetInvTime(), RealNum{1} / dt);
    
    EXPECT_EQ(StepConf{conf}.GetTime(), dt);
    EXPECT_EQ(StepConf{conf}.GetInvTime(), RealNum{1} / dt);
    EXPECT_EQ(StepConf{conf}.displaceMultiplier, displacementMultiplier);
    
    const auto cdt = conf.GetTime() * RealNum{0.8f};
    const auto newConf = StepConf{conf}.SetTime(cdt);
    
    EXPECT_EQ(newConf.GetTime(), cdt);
}

TEST(StepConf, maxTranslation)
{
    const auto v = RealNum(1);
    const auto n = std::nextafter(v, RealNum(0));
    const auto inc = v - n;
    ASSERT_GT(inc, RealNum(0));
    ASSERT_LT(inc, RealNum(1));
    const auto max_inc = inc * StepConf{}.maxTranslation;
    EXPECT_GT(max_inc, RealNum(0) * Meter);
    EXPECT_LT(max_inc, DefaultLinearSlop / RealNum{2});
    EXPECT_LT(max_inc, Length{StepConf{}.linearSlop} / RealNum{2});
    EXPECT_LT(max_inc, Length{StepConf{}.tolerance});
#if 0
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
    std::cout << " inc=" << inc;
    std::cout << " max_inc=" << max_inc;
    std::cout << " slop=" << DefaultLinearSlop;
    std::cout << std::endl;
#endif
    
    {
        StepConf conf;
        conf.tolerance = RealNum(0.0000001) * Meter;
        conf.maxTranslation = RealNum(8.0) * Meter;
        switch (sizeof(RealNum))
        {
            case 4: EXPECT_FALSE(IsMaxTranslationWithinTolerance(conf)); break;
            case 8: EXPECT_TRUE(IsMaxTranslationWithinTolerance(conf)); break;
            default: EXPECT_TRUE(IsMaxTranslationWithinTolerance(conf)); break;
        }
    }
}

TEST(StepConf, maxRotation)
{
    const auto v = RealNum(1);
    const auto n = std::nextafter(v, RealNum(0));
    const auto inc = v - n;
    ASSERT_GT(inc, RealNum(0));
    ASSERT_LT(inc, RealNum(1));
    const auto max_inc = inc * StepConf{}.maxRotation;
    EXPECT_GT(max_inc, Angle(0));
    EXPECT_LT(max_inc, DefaultAngularSlop / RealNum{2});
#if 0
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
    std::cout << " inc=" << inc;
    std::cout << " max_inc=" << max_inc;
    std::cout << " slop=" << DefaultAngularSlop;
    std::cout << std::endl;
#endif
}
