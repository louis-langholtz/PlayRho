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

#include "gtest/gtest.h"
#include <PlayRho/Collision/SeparationFinder.hpp>
#include <PlayRho/Collision/Simplex.hpp>
#include <PlayRho/Collision/TimeOfImpact.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>

using namespace playrho;

TEST(SeparationFinder, ByteSizeIs_40_56_or_96)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(SeparationFinder), std::size_t(40)); break;
        case  8: EXPECT_EQ(sizeof(SeparationFinder), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(SeparationFinder), std::size_t(96)); break;
        default: FAIL(); break;
    }
}

TEST(SeparationFinder, BehavesAsExpected)
{
    const auto shape = PolygonShape{Real{0.5f} * Meter, Real{0.5f} * Meter};
    const auto distproxy = shape.GetChild(0);

    const auto x = Real(100);
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2{+x * Meter, Real(0) * Meter}, Angle{0}}
    };
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2{-x * Meter, Real(0) * Meter}, Angle{0}}
    };
    
    auto t = Real{0}; // Will be set to value of t2
    auto last_s = MaxFloat * Meter;
    auto last_distance = MaxFloat * Meter;
    auto xfA = GetTransformation(sweepA, t);
    auto xfB = GetTransformation(sweepB, t);
    DistanceConf conf;
    auto distanceInfo = Distance(distproxy, xfA, distproxy, xfB, conf);
    conf.cache = Simplex::GetCache(distanceInfo.simplex.GetEdges());
    const auto fcn = SeparationFinder::Get(conf.cache.GetIndices(), distproxy, xfA, distproxy, xfB);
    EXPECT_EQ(fcn.GetType(), SeparationFinder::e_faceA);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(fcn.GetAxis()))), 1.0, 0.000001);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(fcn.GetAxis()))), 0.0, 0.000001);
    EXPECT_EQ(fcn.GetLocalPoint(), Length2(Real(0.5f) * Meter, Real(0) * Meter));

    auto last_min_sep = MaxFloat * Meter;
    for (auto i = 0u; i < 500; ++i)
    {
        // Prepare input for distance query.
        const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);
        const auto distance = Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b));

        const auto minSeparation = fcn.FindMinSeparation(xfA, xfB);

        EXPECT_EQ(minSeparation.indexPair, (IndexPair{IndexPair::InvalidIndex, 2}));
        EXPECT_LT(minSeparation.distance, last_s);
        if (minSeparation.distance > Length{0})
        {
            EXPECT_LT(distance, last_distance);
            EXPECT_NEAR(static_cast<double>(Real{minSeparation.distance / Meter}),
                        static_cast<double>(Real{distance / Meter}),
                        0.0001);
        }
        else if (minSeparation.distance < Length{0})
        {
            if (last_min_sep < Length{0} && distance != Length{0})
            {
                EXPECT_GT(distance, last_distance);
            }
        }
        last_min_sep = minSeparation.distance;
        
        const auto s = fcn.Evaluate(minSeparation.indexPair, xfA, xfB);
        EXPECT_EQ(s, minSeparation.distance);
        if (s >= Length{0})
        {
            EXPECT_NEAR(double(Real{s / Meter}), double(Real{distance / Meter}), 0.0001);
        }
        else
        {
            EXPECT_LE(double(Real{s / Meter}), double(Real{distance / Meter}));
        }
        EXPECT_LT(s, last_s);
        
        //t = std::nextafter(t, 1.0f);
        t += Real(.001);
        last_distance = distance;
        last_s = s;
        xfA = GetTransformation(sweepA, t);
        xfB = GetTransformation(sweepB, t);
        distanceInfo = Distance(distproxy, xfA, distproxy, xfB, conf);
        conf.cache = Simplex::GetCache(distanceInfo.simplex.GetEdges());
    }
}
