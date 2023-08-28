/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <playrho/StepConf.hpp>
#include <playrho/d2/World.hpp>

using namespace playrho;

TEST(StepConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(StepConf), std::size_t(104)); break;
        case  8: EXPECT_EQ(sizeof(StepConf), std::size_t(200)); break;
        case 16: EXPECT_EQ(sizeof(StepConf), std::size_t(384)); break;
        default: FAIL(); break;
    }
}

TEST(StepConf, DefaultConstuction)
{
    const StepConf conf;
    EXPECT_EQ(conf.deltaTime, StepConf::DefaultStepTime);
    EXPECT_EQ(conf.dtRatio, StepConf::DefaultDtRatio);
    EXPECT_EQ(conf.minStillTimeToSleep, StepConf::DefaultMinStillTimeToSleep);
    EXPECT_EQ(conf.linearSlop, StepConf::DefaultLinearSlop);
    EXPECT_EQ(conf.angularSlop, StepConf::DefaultAngularSlop);
    EXPECT_EQ(conf.regResolutionRate, StepConf::DefaultRegResolutionRate);
    EXPECT_EQ(conf.regMinSeparation, StepConf::DefaultRegMinSeparation);
    EXPECT_EQ(conf.regMinMomentum, StepConf::DefaultRegMinMomentum);
    EXPECT_EQ(conf.toiResolutionRate, StepConf::DefaultToiResolutionRate);
    EXPECT_EQ(conf.toiMinSeparation, StepConf::DefaultToiMinSeparation);
    EXPECT_EQ(conf.toiMinMomentum, StepConf::DefaultToiMinMomentum);
    EXPECT_EQ(conf.targetDepth, StepConf::DefaultTargetDepth);
    EXPECT_EQ(conf.tolerance, StepConf::DefaultTolerance);
    EXPECT_EQ(conf.velocityThreshold, StepConf::DefaultVelocityThreshold);
    EXPECT_EQ(conf.maxTranslation, StepConf::DefaultMaxTranslation);
    EXPECT_EQ(conf.maxRotation, StepConf::DefaultMaxRotation);
    EXPECT_EQ(conf.maxLinearCorrection, StepConf::DefaultMaxLinearCorrection);
    EXPECT_EQ(conf.maxAngularCorrection, StepConf::DefaultMaxAngularCorrection);
    EXPECT_EQ(conf.linearSleepTolerance, StepConf::DefaultLinearSleepTolerance);
    EXPECT_EQ(conf.angularSleepTolerance, StepConf::DefaultAngularSleepTolerance);
    EXPECT_EQ(conf.displaceMultiplier, StepConf::DefaultDistanceMultiplier);
    EXPECT_EQ(conf.aabbExtension, StepConf::DefaultAabbExtension);
    EXPECT_EQ(conf.maxCirclesRatio, StepConf::DefaultCirclesRatio);
    EXPECT_EQ(conf.regVelocityIters, StepConf::DefaultRegVelocityIters);
    EXPECT_EQ(conf.regPositionIters, StepConf::DefaultRegPositionIters);
    EXPECT_EQ(conf.toiVelocityIters, StepConf::DefaultToiVelocityIters);
    EXPECT_EQ(conf.toiPositionIters, StepConf::DefaultToiPositionIters);
    EXPECT_EQ(conf.maxToiRootIters, StepConf::DefaultMaxToiRootIters);
    EXPECT_EQ(conf.maxToiIters, StepConf::DefaultMaxToiIters);
    EXPECT_EQ(conf.maxDistanceIters, StepConf::DefaultMaxDistanceIters);
    EXPECT_EQ(conf.maxSubSteps, StepConf::DefaultMaxSubSteps);
    EXPECT_EQ(conf.doWarmStart, StepConf::DefaultDoWarmStart);
    EXPECT_EQ(conf.doToi, StepConf::DefaultDoToi);
    EXPECT_EQ(conf.doBlocksolve, StepConf::DefaultDoBlocksolve);
}

TEST(StepConf, CopyConstruction)
{
    const auto dt = 10_s;
    const auto displacementMultiplier = Real{3.4f};

    StepConf conf;
    conf.deltaTime = dt;
    conf.displaceMultiplier = displacementMultiplier;

    EXPECT_EQ(StepConf{conf}.deltaTime, dt);
    EXPECT_EQ(StepConf{conf}.displaceMultiplier, displacementMultiplier);
    
    const auto cdt = conf.deltaTime * Real{0.8f};
    auto newConf = StepConf{conf};
    newConf.deltaTime = cdt;
    EXPECT_EQ(newConf.deltaTime, cdt);
}

TEST(StepConf, maxTranslation)
{
    const auto v = Real(1);
    const auto n = nextafter(v, Real(0));
    const auto inc = v - n;
    ASSERT_GT(inc, Real(0));
    ASSERT_LT(inc, Real(1));
    const auto max_inc = inc * StepConf{}.maxTranslation;
    EXPECT_GT(max_inc, 0_m);
    EXPECT_LT(max_inc, DefaultLinearSlop / Real{2});
    EXPECT_LT(max_inc, Length{StepConf{}.linearSlop} / Real{2});
    EXPECT_LT(max_inc, Length{StepConf{}.tolerance});
    {
        StepConf conf;
        conf.tolerance = 0.0000001_m;
        conf.maxTranslation = 8.0_m;
        switch (sizeof(Real))
        {
            case 4: EXPECT_FALSE(IsMaxTranslationWithinTolerance(conf)); break;
            case 8: EXPECT_TRUE(IsMaxTranslationWithinTolerance(conf)); break;
            default: EXPECT_TRUE(IsMaxTranslationWithinTolerance(conf)); break;
        }
    }
}

TEST(StepConf, maxRotation)
{
    const auto v = Real(1);
    const auto n = nextafter(v, Real(0));
    const auto inc = v - n;
    ASSERT_GT(inc, Real(0));
    ASSERT_LT(inc, Real(1));
    const auto max_inc = inc * StepConf{}.maxRotation;
    EXPECT_GT(max_inc, 0_deg);
    EXPECT_LT(max_inc, DefaultAngularSlop / Real{2});
}
