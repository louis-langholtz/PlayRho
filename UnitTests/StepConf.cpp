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
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/World.hpp>

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
