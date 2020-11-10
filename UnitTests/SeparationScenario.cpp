/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Collision/SeparationScenario.hpp>
#include <PlayRho/Collision/Simplex.hpp>
#include <PlayRho/Collision/TimeOfImpact.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(SeparationScenario, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(SeparationScenario), std::size_t(28));
#else
            EXPECT_EQ(sizeof(SeparationScenario), std::size_t(40));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(SeparationScenario), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(SeparationScenario), std::size_t(96)); break;
        default: FAIL(); break;
    }
}

TEST(SeparationScenario, BehavesAsExpected)
{
    const auto shape = PolygonShapeConf{0.5_m, 0.5_m};
    const auto distproxy = GetChild(shape, 0);

    const auto x = Real(100);
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, 0_m}, 0_deg},
        Position{Length2{+x * Meter, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, 0_m}, 0_deg},
        Position{Length2{-x * Meter, 0_m}, 0_deg}
    };
    
    auto t = Real{0}; // Will be set to value of t2
    auto last_s = MaxFloat * Meter;
    auto last_distance = MaxFloat * Meter;
    auto xfA = GetTransformation(sweepA, t);
    auto xfB = GetTransformation(sweepB, t);
    DistanceConf conf;
    auto distanceInfo = Distance(distproxy, xfA, distproxy, xfB, conf);
    conf.cache = Simplex::GetCache(distanceInfo.simplex.GetEdges());
    const auto fcn = GetSeparationScenario(conf.cache.indices, distproxy, xfA, distproxy, xfB);
    EXPECT_EQ(fcn.type, SeparationScenario::e_faceA);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(fcn.axis))), 1.0, 0.000001);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(fcn.axis))), 0.0, 0.000001);
    EXPECT_EQ(fcn.localPoint, Length2(0.5_m, 0_m));

    auto last_min_sep = MaxFloat * Meter;
    for (auto i = 0u; i < 500; ++i)
    {
        // Prepare input for distance query.
        const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);
        const auto distance = GetMagnitude(std::get<0>(witnessPoints) - std::get<1>(witnessPoints));

        const auto minSeparation = FindMinSeparation(fcn, xfA, xfB);

        EXPECT_EQ(minSeparation.indices, (IndexPair{InvalidVertex, 2}));
        EXPECT_LT(minSeparation.distance, last_s);
        if (minSeparation.distance > 0_m)
        {
            EXPECT_LT(distance, last_distance);
            EXPECT_NEAR(static_cast<double>(Real{minSeparation.distance / Meter}),
                        static_cast<double>(Real{distance / Meter}),
                        0.0001);
        }
        else if (minSeparation.distance < 0_m)
        {
            if (last_min_sep < 0_m && distance != 0_m)
            {
                EXPECT_GT(distance, last_distance);
            }
        }
        last_min_sep = minSeparation.distance;
        
        const auto s = Evaluate(fcn, xfA, xfB, minSeparation.indices);
        EXPECT_EQ(s, minSeparation.distance);
        if (s >= 0_m)
        {
            EXPECT_NEAR(double(Real{s / Meter}), double(Real{distance / Meter}), 0.0001);
        }
        else
        {
            EXPECT_LE(double(Real{s / Meter}), double(Real{distance / Meter}));
        }
        EXPECT_LT(s, last_s);
        
        //t = nextafter(t, 1.0f);
        t += Real(.001);
        last_distance = distance;
        last_s = s;
        xfA = GetTransformation(sweepA, t);
        xfB = GetTransformation(sweepB, t);
        distanceInfo = Distance(distproxy, xfA, distproxy, xfB, conf);
        conf.cache = Simplex::GetCache(distanceInfo.simplex.GetEdges());
    }
}
