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
#include <PlayRho/Collision/TimeOfImpact.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <set>

using namespace playrho;
using namespace playrho::d2;

TEST(TOIConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(ToiConf), std::size_t(16)); break;
        case  8: EXPECT_EQ(sizeof(ToiConf), std::size_t(32)); break;
        case 16: EXPECT_EQ(sizeof(ToiConf), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(TOIConf, DefaultConstruction)
{
    EXPECT_EQ(ToiConf{}.tMax, Real(1));
    EXPECT_EQ(ToiConf{}.maxRootIters, DefaultMaxToiRootIters);
    EXPECT_EQ(ToiConf{}.maxToiIters, DefaultMaxToiIters);
    EXPECT_EQ(ToiConf{}.targetDepth, DefaultLinearSlop * Real{3});
    EXPECT_EQ(ToiConf{}.tolerance, DefaultLinearSlop / Real{4});
}

TEST(TOIOutput, StatsByteSize)
{
    EXPECT_EQ(sizeof(TOIOutput::Statistics), std::size_t(10));
}

TEST(TOIOutput, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(TOIOutput), std::size_t(16)); break;
        case  8: EXPECT_EQ(sizeof(TOIOutput), std::size_t(24)); break;
        case 16: EXPECT_EQ(sizeof(TOIOutput), std::size_t(32)); break;
        default: FAIL(); break;
    }
}

TEST(TOIOutput, StatisticsTraits)
{
    EXPECT_TRUE(std::is_default_constructible<TOIOutput::Statistics>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<TOIOutput::Statistics>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<TOIOutput::Statistics>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<TOIOutput::Statistics>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<TOIOutput::Statistics>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<TOIOutput::Statistics>::value);
}

TEST(TOIOutput, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<TOIOutput>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<TOIOutput>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<TOIOutput>::value);
    
    EXPECT_TRUE((std::is_constructible<TOIOutput, Real, TOIOutput::Statistics, TOIOutput::State>::value));
    EXPECT_TRUE((std::is_constructible<TOIOutput>::value));
    
    EXPECT_TRUE((std::is_nothrow_constructible<TOIOutput, Real, TOIOutput::Statistics, TOIOutput::State>::value));
    EXPECT_TRUE((std::is_nothrow_constructible<TOIOutput>::value));
    
    EXPECT_FALSE((std::is_trivially_constructible<TOIOutput, Real, TOIOutput::Statistics, TOIOutput::State>::value));
    EXPECT_FALSE((std::is_trivially_constructible<TOIOutput>::value));
    
    EXPECT_TRUE(std::is_copy_constructible<TOIOutput>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<TOIOutput>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<TOIOutput>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<TOIOutput>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<TOIOutput>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<TOIOutput>::value);
    
    EXPECT_TRUE(std::is_destructible<TOIOutput>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<TOIOutput>::value);
    EXPECT_TRUE(std::is_trivially_destructible<TOIOutput>::value);
}

TEST(TOIOutput, Types)
{
    EXPECT_GT(sizeof(TOIOutput::Statistics::dist_sum_type), sizeof(TOIOutput::Statistics::dist_iter_type));
    EXPECT_GT(sizeof(TOIOutput::Statistics::root_sum_type), sizeof(TOIOutput::Statistics::root_iter_type));
}

TEST(TOIOutput, DefaultConstruction)
{
    TOIOutput foo;
    EXPECT_EQ(foo.state, TOIOutput::e_unknown);
}

TEST(TOIOutput, InitConstruction)
{
    const auto state = TOIOutput::e_separated;
    const auto time = Real(0.6);
    
    TOIOutput::Statistics stats;
    stats.toi_iters = 3;
    stats.max_dist_iters = 11;
    stats.max_root_iters = 4;
    stats.sum_finder_iters = 0;
    stats.sum_dist_iters = 5;
    stats.sum_root_iters = 10;

    TOIOutput foo{time, stats, state};

    EXPECT_EQ(foo.state, state);
    EXPECT_EQ(foo.time, time);
    
    EXPECT_EQ(foo.stats.toi_iters, 3);
    EXPECT_EQ(foo.stats.max_dist_iters, 11);
    EXPECT_EQ(foo.stats.max_root_iters, 4);
    //EXPECT_EQ(foo.get_sum_finder_iters(), 0);
    EXPECT_EQ(foo.stats.sum_dist_iters, 5);
    EXPECT_EQ(foo.stats.sum_root_iters, 10);
}

TEST(TOIOutput, GetName)
{
    std::set<std::string> names;
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_unknown)).second);
    ASSERT_FALSE(names.insert(GetName(TOIOutput::e_unknown)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_overlapped)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_touching)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_separated)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_maxRootIters)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_nextAfter)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_maxToiIters)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_belowMinTarget)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_maxDistIters)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_targetDepthExceedsTotalRadius)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_minTargetSquaredOverflow)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_maxTargetSquaredOverflow)).second);
    EXPECT_TRUE(names.insert(GetName(TOIOutput::e_notFinite)).second);
}

TEST(TimeOfImpact, Overlapped)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth((slop * 3) * Meter).UseTolerance((slop / 4) * Meter);

    const auto radius = 1_m;
    const auto pA = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2{}, 0_deg}};
    const auto pB = Length2{};
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2{}, 0_deg}};
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    EXPECT_EQ(output.state, TOIOutput::e_overlapped);
    EXPECT_EQ(output.time, Real(0));
    EXPECT_EQ(output.stats.toi_iters, 1);
}

TEST(TimeOfImpact, Touching)
{
    const auto slop = 0.001_m;
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3).UseTolerance(slop / 4);

    const auto radius = 1.0_m + 1.5f * slop;

    const auto pA = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2{}, 0_deg}};
    
    const auto pB = Length2{};
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2{2_m, 0_m}, 0_deg}};

    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_touching);
    EXPECT_EQ(output.time, Real(0));
    EXPECT_EQ(output.stats.toi_iters, 1);
}

TEST(TimeOfImpact, Separated)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3_m).UseTolerance((slop / 4) * Meter);
    const auto radius = 1_m;
    
    const auto pA = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2{}, 0_deg}};
    
    const auto pB = Length2{};
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2{4_m, 0_m}, 0_deg}};
    
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_separated);
    EXPECT_EQ(output.time, Real(1));
    EXPECT_EQ(output.stats.toi_iters, 1);
}

TEST(TimeOfImpact, CollideCirclesHorizontally)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3_m).UseTolerance((slop / 4) * Meter);

    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = 1_m;
    const auto x = Real(2);
    const auto pA = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2{-x * Meter, 0_m}, 0_deg}, Position{Length2{}, 0_deg}};
    const auto pB = Length2{};
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2{+x * Meter, 0_m}, 0_deg}, Position{Length2{}, 0_deg}};
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    const auto approx_time_of_collision = ((x * Meter - radius) + limits.targetDepth / Real{2}) / (x * Meter);

    if (std::is_same<Real, Fixed32>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_TRUE(AlmostEqual(output.time, Real{approx_time_of_collision}));
        EXPECT_EQ(output.stats.toi_iters, 1);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_TRUE(AlmostEqual(output.time, Real{approx_time_of_collision}));
        EXPECT_EQ(output.stats.toi_iters, 2);
    }
}

TEST(TimeOfImpact, CollideEdgeCircleTouchingly)
{
    const auto limits = ToiConf{}
        .UseTimeMax(1)
        .UseTargetDepth(0.0149999997_m)
        .UseTolerance(0_m)
        .UseMaxRootIters(30u)
        .UseMaxToiIters(20u)
        .UseMaxDistIters(20u);
    
    const Length2 edgeVertices[] = {Length2{-40_m, 0_m}, Length2{+40_m, 0_m}};
    const UnitVec edgeNormals[] = {UnitVec::GetLeft(), UnitVec::GetRight()};
    const auto proxyA = DistanceProxy{0.00999999977_m, 2, edgeVertices, edgeNormals};
    const auto sweepA = Sweep{Position{Length2{0_m, 0_m}, 0_deg}, Position{Length2{0_m, 0_m}, 0_deg}};

    const auto pB = Length2{};
    const auto proxyB = DistanceProxy{0.5_m, 1, &pB, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{-0.490861088_m,  0.49556452_m}, 1.0125972_rad},
        Position{Length2{-0.486600876_m, 0.494585991_m}, 1.00495279_rad}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    if (std::is_same<Real, float>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(static_cast<double>(output.time), 0.576901972, 0.000000001);
    }
    else if (std::is_same<Real, double>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(static_cast<double>(output.time), 0.57690669361866653, 0.000000001);
    }
    EXPECT_EQ(output.stats.toi_iters, 2u);
    EXPECT_EQ(output.stats.sum_root_iters, 2u);
    EXPECT_EQ(output.stats.sum_finder_iters, 1u);
    EXPECT_EQ(output.stats.sum_dist_iters, 3u);
}

TEST(TimeOfImpact, CollideCirclesVertically)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3_m).UseTolerance((slop / 4) * Meter);
    const auto radius = 1_m;
    const auto y = Real(20);

    const auto pos = Length2{};

    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2{0_m, -y * Meter}, 0_deg},
        Position{Length2{0_m, +y * Meter}, 0_deg}
    };
    
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{0_m, +y * Meter}, 0_deg},
        Position{Length2{0_m, -y * Meter}, 0_deg}
    };
    
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    if (std::is_same<Real, Fixed32>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(double(output.time), 0.474609375, 0.000001);
        EXPECT_EQ(output.stats.toi_iters, 1);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(double(output.time), 0.4750375, 0.000001);
        EXPECT_EQ(output.stats.toi_iters, 2);
    }
}

TEST(TimeOfImpact, CirclesPassingParSepPathsDontCollide)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3_m).UseTolerance((slop / 4) * Meter);
    
    const auto pos = Length2{};

    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = 1_m;
    const auto x = Real(3);
    const auto y = Real(1);
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, +y * Meter}, 0_deg},
        Position{Length2{+x * Meter, +y * Meter}, 0_deg}
    };
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, -y * Meter}, 0_deg},
        Position{Length2{-x * Meter, -y * Meter}, 0_deg}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    if (std::is_same<Real, Fixed<std::int32_t,9>>::value) // Code for Fixed32 basically
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(static_cast<double>(output.time), 0.37890625, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_separated);
        EXPECT_NEAR(static_cast<double>(output.time), 1.0, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 7);
    }
}

TEST(TimeOfImpact, CirclesExactlyTouchingAtStart)
{
    const auto radius = 1_m;
    
    const Length2 circleVertices[] = {Length2{}};
    const UnitVec circleNormals[] = {UnitVec{}};
    const auto circle = DistanceProxy{radius, 1, circleVertices, circleNormals};
    
    const auto circleSweep0 = Sweep{
        Position{Length2{-1_m, 0_m}, 0_deg}
    };
    const auto circleSweep1 = Sweep{
        Position{Length2{+1_m, 0_m}, 0_deg}
    };
    
    const auto slop = 0_m;
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3).UseTolerance(slop / 4);
    const auto output = GetToiViaSat(circle, circleSweep0, circle, circleSweep1, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_touching);
    EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.001);
    EXPECT_EQ(output.stats.toi_iters, 1);
}

TEST(TimeOfImpact, EdgeCircleExactlyTouchingAtStart)
{
    const auto radius = 1_m;
    
    const auto v0 = Length2{-1_m, 0_m};
    const auto v1 = Length2{+1_m, 0_m};
    const Length2 edgeVertices[] = {v0, v1};
    const auto n0 = GetUnitVector(v1 - v0);
    const auto n1 = -n0;
    const UnitVec edgeNormals[] = {n0, n1};
    const auto edge = DistanceProxy{radius, 2, edgeVertices, edgeNormals};
    
    const Length2 circleVertices[] = {Length2{}};
    const UnitVec circleNormals[] = {UnitVec{}};
    const auto circle = DistanceProxy{radius, 1, circleVertices, circleNormals};
    
    const auto edgeSweep = Sweep{
        Position{Length2{-1_m, 0_m}, 0_deg}
    };
    const auto circleSweep = Sweep{
        Position{Length2{radius * 2, 0_m}, 0_deg}
    };
    
    const auto slop = 0_m;
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3).UseTolerance(slop / 4);
    const auto output = GetToiViaSat(edge, edgeSweep, circle, circleSweep, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_touching);
    EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.001);
    EXPECT_EQ(output.stats.toi_iters, 1);
}

TEST(TimeOfImpact, EdgeCircleTolerantTouchingAsRounded)
{
    // This test will fail if the TOI code isn't using rounded radius of the edge end
    const auto radius = 1_m;
    
    const auto v0 = Length2{-1_m, 0_m};
    const auto v1 = Length2{+1_m, 0_m};
    const Length2 edgeVertices[] = {v0, v1};
    const auto n0 = GetUnitVector(v1 - v0);
    const auto n1 = -n0;
    const UnitVec edgeNormals[] = {n0, n1};
    const auto edge = DistanceProxy{radius, 2, edgeVertices, edgeNormals};
    
    const Length2 circleVertices[] = {Length2{}};
    const UnitVec circleNormals[] = {UnitVec{}};
    const auto circle = DistanceProxy{radius, 1, circleVertices, circleNormals};
    
    const auto edgeSweep = Sweep{
        Position{Length2{-1_m, 0_m}, 0_deg}
    };
    const auto circleSweep = Sweep{
        Position{Length2{2_m, 1_m}, 0_deg},
        Position{Length2{1.5_m, 1_m}, 0_deg}
    };
    
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0_m).UseTolerance(0.001_m);
    const auto output = GetToiViaSat(edge, edgeSweep, circle, circleSweep, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_touching);
    EXPECT_NEAR(static_cast<double>(output.time), 0.53589820861816406, 0.53589820861816406 / 1000);
    EXPECT_EQ(output.stats.toi_iters, 3);
}

TEST(TimeOfImpact, EdgeEdgeTolerantTouchingAsRounded)
{
    // This test will fail if the TOI code isn't using rounded radius of the edge end
    const auto radius = 1_m;
    
    const auto v0 = Length2{-1_m, 0_m};
    const auto v1 = Length2{+1_m, 0_m};
    const Length2 edgeVertices[] = {v0, v1};
    const auto n0 = GetUnitVector(v1 - v0);
    const auto n1 = -n0;
    const UnitVec edgeNormals[] = {n0, n1};
    const auto edge = DistanceProxy{radius, 2, edgeVertices, edgeNormals};
    
    const auto edgeSweep0 = Sweep{
        Position{Length2{-1_m, 0_m}, 0_deg}
    };
    const auto edgeSweep1 = Sweep{
        Position{Length2{3_m, 1_m}, 0_deg},
        Position{Length2{2.5_m, 1_m}, 0_deg}
    };
    
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0_m).UseTolerance(0.001_m);
    const auto output = GetToiViaSat(edge, edgeSweep0, edge, edgeSweep1, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_touching);
    EXPECT_NEAR(static_cast<double>(output.time), 0.53589820861816406, 0.53589820861816406 / 1000);
    EXPECT_EQ(output.stats.toi_iters, 3);
}

TEST(TimeOfImpact, EdgeCircleSeparated)
{
    // This test will fail if the TOI code isn't using rounded radius of the line end
    const auto radius = 1_m;

    const auto v0 = Length2{-1_m, 0_m};
    const auto v1 = Length2{+1_m, 0_m};
    const Length2 edgeVertices[] = {v0, v1};
    const auto n0 = GetUnitVector(v1 - v0);
    const auto n1 = -n0;
    const UnitVec edgeNormals[] = {n0, n1};
    const auto edge = DistanceProxy{radius, 2, edgeVertices, edgeNormals};
    
    const Length2 circleVertices[] = {Length2{}};
    const UnitVec circleNormals[] = {UnitVec{}};
    const auto circle = DistanceProxy{radius, 1, circleVertices, circleNormals};
    
    const auto edgeSweep = Sweep{
        Position{Length2{-1_m, 0_m}, 0_deg}
    };
    const auto circleSweep = Sweep{
        Position{Length2{radius * 2, 0.5_m}, 0_deg}
    };
    
    const auto slop = 0_m;
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3).UseTolerance(slop / 4);
    const auto output = GetToiViaSat(edge, edgeSweep, circle, circleSweep, limits);
    
    EXPECT_EQ(output.state, TOIOutput::e_separated);
    EXPECT_NEAR(static_cast<double>(output.time), 1.0, 0.001);
    EXPECT_EQ(output.stats.toi_iters, 1);
}

TEST(TimeOfImpact, RodCircleMissAt360)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3_m).UseTolerance((slop / 4) * Meter);
    
    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = 1_m;
    const auto x = Real(40);
    const auto vA0 = Length2{-4_m, 0_m};
    const auto vA1 = Length2{4_m, 0_m};
    const Length2 vertices[] = {vA0, vA1};
    const auto nA0 = GetUnitVector(vA1 - vA0);
    const UnitVec normals[] = {nA0, -nA0};
    const auto proxyA = DistanceProxy{radius, 2, vertices, normals};
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, 4_m}, 0_deg},
        Position{Length2{+x * Meter, 4_m}, 360_deg}
    };
    const auto pos = Length2{};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, 0_m}, 0_deg},
        Position{Length2{-x * Meter, 0_m}, 0_deg}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    if (std::is_same<Real, Fixed<std::int32_t,9>>::value) // Code for Fixed32 basically
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(static_cast<double>(output.time), 0.51171875, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_separated);
        EXPECT_NEAR(static_cast<double>(output.time), 1.0, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 4);
    }
}

TEST(TimeOfImpact, RodCircleHitAt180)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * 3_m).UseTolerance((slop / 4) * Meter);
    
    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = 1_m;
    const auto x = Real(40);
    const auto vA0 = Length2{-4_m, 0_m};
    const auto vA1 = Length2{4_m, 0_m};
    const Length2 vertices[] = {vA0, vA1};
    const auto nA0 = GetUnitVector(vA1 - vA0);
    const UnitVec normals[] = {nA0, -nA0};
    const auto proxyA = DistanceProxy{radius, 2, vertices, normals};
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, 4_m}, 0_deg},
        Position{Length2{+x * Meter, 4_m}, Angle{Real{180.0f} * 1_deg}}
    };
    const auto pos = Length2{};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, 0_m}, 0_deg},
        Position{Length2{-x * Meter, 0_m}, 0_deg}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    if (std::is_same<Real, Fixed<std::int32_t,9>>::value) // Code for Fixed32 basically
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(static_cast<double>(output.time), 0.490234375, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(double(output.time), 0.4884203672409058, 0.0001);
        EXPECT_EQ(output.stats.toi_iters, 3);
    }
}

TEST(TimeOfImpact, SucceedsWithClosingSpeedOf800_1)
{
    const auto slop = Real{0.001f};
    const auto radius = 1_m;
    const auto x = Real(200);
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, 0_m}, 0_deg},
        Position{Length2{+x * Meter, 0_m}, 0_deg}
    };
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, 0_m}, 0_deg},
        Position{Length2{-x * Meter, 0_m}, 0_deg}
    };
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(200)
        .UseTargetDepth((slop * 3) * Meter)
        .UseTolerance((slop / 4) * Meter);
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    if (std::is_same<Real, Fixed<std::int32_t,9>>::value) // Code for Fixed32 basically
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(static_cast<double>(output.time), 0.49609375, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 3);
        EXPECT_EQ(output.stats.sum_dist_iters, 1);
        EXPECT_EQ(output.stats.sum_root_iters, 3);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(double(output.time), 0.4975037276744843, 0.0002);
        EXPECT_EQ(output.stats.toi_iters, 2);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 2);
        EXPECT_EQ(output.stats.sum_dist_iters, 2);
        EXPECT_EQ(output.stats.sum_root_iters, 2);
    }
}

TEST(TimeOfImpact, SucceedsWithClosingSpeedOf800_2)
{
    const auto slop = Real{0.001f};
    const auto radius = 1_m;
    const auto x = Real(400);
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2{-x * Meter, 0_m}, 0_deg},
        Position{Length2{}, 0_deg}
    };
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2{+x * Meter, 0_m}, 0_deg},
        Position{Length2{}, 0_deg}
    };
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(200)
        .UseTargetDepth((slop * 3) * Meter)
        .UseTolerance((slop / 4) * Meter);
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    if (std::is_same<Real, Fixed<std::int32_t,9>>::value)
    {
        // The results limited to what's possible with this type...
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(double(output.time), 0.9975037574768066, 0.002);
        EXPECT_EQ(output.stats.toi_iters, 1);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 3);
        EXPECT_EQ(output.stats.sum_dist_iters, 1);
        EXPECT_EQ(output.stats.sum_root_iters, 3);
    }
    else
    {
        // what the results should be like...
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(double(output.time), 0.9975037574768066, 0.002);
        EXPECT_EQ(output.stats.toi_iters, 2);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 2);
        EXPECT_EQ(output.stats.sum_dist_iters, 2);
        EXPECT_EQ(output.stats.sum_root_iters, 2);
    }

#if 0
    ASSERT_EQ(output.state, TOIOutput::e_touching);

    auto touching = true;
    auto iterations = 0u;
    for (auto t = output.time; t > 0; t = nextafter(t, 0.0f))
    {
        const auto conf2 = ToiConf{}.UseMaxToiIters(200).UseMaxRootIters(200).UseTimeMax(t);
        const auto output2 = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, conf2);
        
        EXPECT_LE(output2.time, t);
        if (touching)
        {
            if (output2.state != TOIOutput::e_touching)
            {
                std::cout << "lost touch after " << iterations << " iterations at t=" << t << std::endl;
                touching = false;
            }
        }
        else // !touching
        {
            if (output2.state == TOIOutput::e_touching)
            {
                std::cout << "found additional root at t=" << t << std::endl;
                touching = true;
            }
        }
        ++iterations;
    }
#endif
}

TEST(TimeOfImpact, WithClosingSpeedOf1600)
{
    const auto slop = Real{0.001f};
    const auto radius = 1_m;
    const auto x = Real(400);
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{Position{Length2{-x * Meter, 0_m}, 0_deg}, Position{Length2{+x * Meter, 0_m}, 0_deg}};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{Position{Length2{+x * Meter, 0_m}, 0_deg}, Position{Length2{-x * Meter, 0_m}, 0_deg}};
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(200)
        .UseTargetDepth((slop * 3) * Meter)
        .UseTolerance((slop / 4) * Meter);
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    if (std::is_same<Real, Fixed<std::int32_t,9>>::value)
    {
        // The results limited to what's possible with this type...
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        EXPECT_NEAR(double(output.time), 0.4987518787384033, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 2);
        EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
        EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(double(output.time), 0.4987518787384033, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 2);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 2);
        EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
        EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
    }
}

TEST(TimeOfImpact, ForNonCollidingShapesFails)
{
    // The data for shapes and sweeps comes from PlayRho/Testbed/Tests/TimeOfImpact.hpp

    const auto shapeA = PolygonShapeConf{
        PolygonShapeConf{}.UseVertexRadius(Real{0.0001f * 2} * Meter).SetAsBox(25.0_m, 5.0_m)
    };

    const auto shapeB = PolygonShapeConf{
        PolygonShapeConf{}.UseVertexRadius(Real{0.0001f * 2} * Meter).SetAsBox(2.5_m, 2.5_m)
    };

    const auto dpA = GetChild(shapeA, 0);
    const auto dpB = GetChild(shapeB, 0);

    const auto sweepA = Sweep{
        Position{Length2{-11_m, 10_m}, 2.95000005_rad},
        Position{Length2{-11_m, 10_m}, 2.95000005_rad}
    };
    const auto sweepB = Sweep{
        Position{Length2{18.4742737_m, 19.7474861_m}, 513.36676_rad},
        Position{Length2{19.5954781_m, 18.9165268_m}, 513.627808_rad}
    };
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(20)
        .UseMaxRootIters(32)
        .UseTimeMax(1)
        .UseTargetDepth(Real(3.0f / 10000) * Meter)
        .UseTolerance(Real(1.0f / 40000) * Meter);
    const auto output = GetToiViaSat(dpA, sweepA, dpB, sweepB, conf);

    if (std::is_same<Real, float>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        if (output.state == TOIOutput::e_nextAfter)
        {
            EXPECT_NEAR(double(output.time), 0.863826394, 0.0001);
            EXPECT_EQ(output.stats.toi_iters, 1);
            EXPECT_EQ(output.stats.max_dist_iters, 4);
            //EXPECT_TRUE(output.stats.max_root_iters == 23 || output.stats.max_root_iters == 14);
            EXPECT_EQ(output.stats.max_root_iters, 23);
        }
    }
    else if (std::is_same<Real, Fixed<std::int32_t,9>>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
        if (output.state == TOIOutput::e_nextAfter)
        {
            EXPECT_NEAR(double(output.time), 0.865234375, 0.0001);
            EXPECT_EQ(output.stats.toi_iters, 1);
            EXPECT_EQ(output.stats.max_dist_iters, 4);
            EXPECT_EQ(output.stats.max_root_iters, 4);
        }
    }
    else
    {
        EXPECT_EQ(output.state, TOIOutput::e_separated);
        if (output.state == TOIOutput::e_separated)
        {
            EXPECT_EQ(output.time, Real(1));
            EXPECT_EQ(output.stats.toi_iters, 2);
            EXPECT_GE(output.stats.max_dist_iters, 3);
            EXPECT_LE(output.stats.max_dist_iters, 4);
            EXPECT_EQ(output.stats.max_root_iters, 6);
        }
    }

    EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
    EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
}

TEST(TimeOfImpact, ToleranceReachedWithT1Of1)
{
    // This setup causes the TimeOfImpact function to get into the state where
    // separation has reached tolerance but t2 already equals t1.

    const auto sweepA = Sweep{
        Position{Length2{0.0_m, -0.5_m}, 0_deg},
        Position{Length2{0.0_m, -0.5_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{14.3689661_m, 0.500306308_m}, 0.0000139930862_rad},
        Position{Length2{14.3689451_m, 0.500254989_m}, 0.000260060915_rad}
    };

    // Note that these vertices are interpreted by code using the DistanceProxy as
    // being relative to the body's origin. Don't confuse the locations as being in
    // world coordinates.
    const Length2 vertices[] = {
        Vec2{14.5f, -0.5f} * Meter,
        Vec2{14.5f, +0.5f} * Meter,
        Vec2{13.5f, +0.5f} * Meter,
        Vec2{13.5f, -0.5f} * Meter
    };
    const UnitVec normals[] = {
        GetUnitVector(vertices[1] - vertices[0]),
        GetUnitVector(vertices[2] - vertices[1]),
        GetUnitVector(vertices[3] - vertices[2]),
        GetUnitVector(vertices[0] - vertices[3]),
    };
    const auto dpA = DistanceProxy{
        0.000199999995_m, 4, vertices, normals
    };
    
    auto shapeB = PolygonShapeConf{
        PolygonShapeConf{}.UseVertexRadius(Real{0.0001f * 2} * Meter).SetAsBox(0.5_m, 0.5_m)
    };
    const auto dpB = GetChild(shapeB, 0);
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(30)
        .UseTimeMax(1)
        .UseTargetDepth(Real(3.0f / 10000) * Meter)
        .UseTolerance(Real(1.0f / 40000) * Meter);

    const auto output = GetToiViaSat(dpA, sweepA, dpB, sweepB, conf);

    if (std::is_same<Real, float>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_separated);
        EXPECT_NEAR(static_cast<double>(output.time), 1.0, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 2);
        EXPECT_EQ(output.stats.max_dist_iters, 4);
        EXPECT_EQ(output.stats.max_root_iters, 0);
        EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
        EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
    }
    else if (std::is_same<Real, Fixed<std::int32_t,9>>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_overlapped);
        EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
        EXPECT_EQ(output.stats.max_dist_iters, 4);
        EXPECT_EQ(output.stats.max_root_iters, 0);
        EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
        EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
    }
#ifndef _WIN32
    else if (std::is_same<Real, Fixed<std::int64_t,24>>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_separated);
        EXPECT_NEAR(static_cast<double>(output.time), 1.0, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 1);
        EXPECT_EQ(output.stats.max_dist_iters, 4);
        EXPECT_EQ(output.stats.max_root_iters, 0);
        EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
        EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
    }
#endif
    else // if (std::is_same<Real, double>::value)
    {
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(static_cast<double>(output.time), 1.0, 0.001);
        EXPECT_EQ(output.stats.toi_iters, 2);
        EXPECT_EQ(output.stats.max_dist_iters, 4);
        EXPECT_EQ(output.stats.max_root_iters, 0);
        EXPECT_GE(output.stats.sum_dist_iters, output.stats.max_dist_iters);
        EXPECT_GE(output.stats.sum_root_iters, output.stats.max_root_iters);
    }
}

TEST(TimeOfImpact, MaxToiIters)
{
    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};

    const auto sweepA = Sweep{
        Position{Length2{-5_m, 0_m}, 0_deg},
        Position{Length2{-5_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+5_m, 0_m}, 0_deg},
        Position{Length2{+5_m, 0_m}, 0_deg}
    };
    const auto conf = ToiConf{}
        .UseTargetDepth(0_m)
        .UseTolerance(0_m)
        .UseMaxRootIters(0)
        .UseMaxToiIters(0)
        .UseMaxDistIters(0)
        ;
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    EXPECT_EQ(output.state, TOIOutput::e_maxToiIters);
}

TEST(TimeOfImpact, MaxDistIters)
{
    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};

    const auto sweepA = Sweep{
        Position{Length2{-5_m, 0_m}, 0_deg},
        Position{Length2{-5_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+5_m, 0_m}, 0_deg},
        Position{Length2{+5_m, 0_m}, 0_deg}
    };
    const auto conf = ToiConf{}
        .UseTargetDepth(0_m)
        .UseTolerance(0_m)
        .UseMaxRootIters(0)
        .UseMaxToiIters(1)
        .UseMaxDistIters(0)
        ;
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    EXPECT_EQ(output.state, TOIOutput::e_maxDistIters);
}

TEST(TimeOfImpact, MaxRootIters)
{
    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};

    const auto sweepA = Sweep{
        Position{Length2{-5_m, 0_m}, 0_deg},
        Position{Length2{-0_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+5_m, 0_m}, 0_deg},
        Position{Length2{+0_m, 0_m}, 0_deg}
    };
    const auto conf = ToiConf{}
        .UseTargetDepth(0_m)
        .UseTolerance(0_m)
        .UseMaxRootIters(0)
        .UseMaxToiIters(1)
        .UseMaxDistIters(1)
        ;
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    EXPECT_EQ(output.state, TOIOutput::e_maxRootIters);
}

TEST(TimeOfImpact, NextAfter)
{
    if (!std::is_floating_point<Real>::value)
    {
        // Test only designed for fundamental floating point types.
        return;
    }

    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2{-2e18_m, 0_m}, 0_deg},
        Position{Length2{+1e16_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+2e18_m, 0_m}, 0_deg},
        Position{Length2{+0_m, 0_m}, 0_deg}
    };

    // Negative tolerance results in a TOIOutput::e_nextAfter result no matter how
    // many iterations are allowed.
    const auto conf = ToiConf{}
        .UseTargetDepth(0_m)
        .UseTolerance(0_m)
        .UseMaxRootIters(255)
        .UseMaxToiIters(255)
        .UseMaxDistIters(255)
        ;
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    EXPECT_EQ(output.state, TOIOutput::e_nextAfter);
    EXPECT_NEAR(static_cast<double>(output.time), 0.99750623441396513, 0.0001);
    EXPECT_EQ(output.stats.max_dist_iters, 1);
    if (std::is_same<Real, float>::value)
    {
        EXPECT_EQ(output.stats.max_root_iters, 8);
    }
    else if (std::is_same<Real, double>::value)
    {
        EXPECT_EQ(output.stats.max_root_iters, 8);
    }
    else if (std::is_same<Real, long double>::value)
    {
        EXPECT_EQ(output.stats.max_root_iters, 6);
    }
    EXPECT_EQ(output.stats.toi_iters, 1);
    EXPECT_EQ(output.stats.sum_dist_iters, 1);
    EXPECT_EQ(output.stats.sum_finder_iters, 0);
    if (std::is_same<Real, float>::value)
    {
        EXPECT_EQ(output.stats.sum_root_iters, 8);
    }
    else if (std::is_same<Real, double>::value)
    {
        EXPECT_EQ(output.stats.sum_root_iters, 8);
    }
    else if (std::is_same<Real, long double>::value)
    {
        EXPECT_EQ(output.stats.sum_root_iters, 6);
    }
    else
    {
        EXPECT_EQ(output.stats.sum_root_iters, 0);
    }
}

TEST(TimeOfImpact, TargetDepthExceedsTotalRadius)
{
    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    
    const auto conf = ToiConf{}
        .UseTargetDepth(4_m)
        .UseMaxRootIters(0)
        .UseMaxToiIters(0)
        .UseMaxDistIters(0)
        .UseTolerance(0_m)
        ;
    
    const auto sweepA = Sweep{
        Position{Length2{-200_m, 0_m}, 0_deg},
        Position{Length2{+100_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+200_m, 0_m}, 0_deg},
        Position{Length2{-10_m, 0_m}, 0_deg}
    };
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    EXPECT_EQ(output.state, TOIOutput::e_targetDepthExceedsTotalRadius);
    EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.0001);
    EXPECT_EQ(output.stats.max_dist_iters, 0);
    EXPECT_EQ(output.stats.max_root_iters, 0);
    EXPECT_EQ(output.stats.toi_iters, 0);
    EXPECT_EQ(output.stats.sum_dist_iters, 0);
    EXPECT_EQ(output.stats.sum_finder_iters, 0);
    EXPECT_EQ(output.stats.sum_root_iters, 0);
}

TEST(TimeOfImpact, MinTargetSquaredOverflow)
{
    const auto radius = std::numeric_limits<Length>::max() / 4;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    
    const auto conf = ToiConf{}
        .UseTargetDepth(0_m)
        .UseMaxRootIters(0)
        .UseMaxToiIters(0)
        .UseMaxDistIters(0)
        .UseTolerance(0_m)
        ;
    
    const auto sweepA = Sweep{
        Position{Length2{-200_m, 0_m}, 0_deg},
        Position{Length2{+100_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+200_m, 0_m}, 0_deg},
        Position{Length2{-10_m, 0_m}, 0_deg}
    };
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    EXPECT_EQ(output.state, TOIOutput::e_minTargetSquaredOverflow);
    EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.0001);
    EXPECT_EQ(output.stats.max_dist_iters, 0);
    EXPECT_EQ(output.stats.max_root_iters, 0);
    EXPECT_EQ(output.stats.toi_iters, 0);
    EXPECT_EQ(output.stats.sum_dist_iters, 0);
    EXPECT_EQ(output.stats.sum_finder_iters, 0);
    EXPECT_EQ(output.stats.sum_root_iters, 0);
}

TEST(TimeOfImpact, MaxTargetSquaredOverflow)
{
    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};

    const auto conf = ToiConf{}
        .UseTargetDepth(2_m)
        .UseMaxRootIters(0)
        .UseMaxToiIters(0)
        .UseMaxDistIters(0)
        .UseTolerance(NonNegative<Length>{std::numeric_limits<Length>::max()})
        ;

    const auto sweepA = Sweep{
        Position{Length2{-200_m, 0_m}, 0_deg},
        Position{Length2{+100_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+200_m, 0_m}, 0_deg},
        Position{Length2{-10_m, 0_m}, 0_deg}
    };
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    EXPECT_EQ(output.state, TOIOutput::e_maxTargetSquaredOverflow);
    EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.0001);
    EXPECT_EQ(output.stats.max_dist_iters, 0);
    EXPECT_EQ(output.stats.max_root_iters, 0);
    EXPECT_EQ(output.stats.toi_iters, 0);
    EXPECT_EQ(output.stats.sum_dist_iters, 0);
    EXPECT_EQ(output.stats.sum_finder_iters, 0);
    EXPECT_EQ(output.stats.sum_root_iters, 0);
}

#if 0
TEST(TimeOfImpact, BelowMinTarget)
{
    const auto radius = 1_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};

    const auto sweepA = Sweep{
        Position{Length2{-2e18_m, 0_m}, 0_deg},
        Position{Length2{+0_m, 0_m}, 0_deg}
    };
    const auto sweepB = Sweep{
        Position{Length2{+2e18_m, 0_m}, 0_deg},
        Position{Length2{+0_m, 0_m}, 0_deg}
    };
    const auto conf = ToiConf{}
        .UseTargetDepth(0.002_m)
        .UseTolerance(0.0005_m)
        //.UseTolerance(5e20_m)
        .UseMaxRootIters(255)
        .UseMaxToiIters(2)
        .UseMaxDistIters(1)
        ;
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    EXPECT_EQ(output.state, TOIOutput::e_belowMinTarget);
    EXPECT_NEAR(static_cast<double>(output.time), 0.0, 0.0001);
    EXPECT_EQ(output.stats.max_dist_iters, 1);
    EXPECT_EQ(output.stats.max_root_iters, 0);
    EXPECT_EQ(output.stats.toi_iters, 1);
    EXPECT_EQ(output.stats.sum_dist_iters, 1);
    EXPECT_EQ(output.stats.sum_finder_iters, 0);
    EXPECT_EQ(output.stats.sum_root_iters, 0);
}
#endif

TEST(TimeOfImpact, TryOutDifferentConfs)
{
    if (!std::is_floating_point<Real>::value)
    {
        // Test only designed for fundamental floating point types.
        return;
    }

    const auto radius = 1e10_m;
    const auto pos = Length2{};
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    
    {
        const auto sweepA = Sweep{
            Position{Length2{-2e16_m, 0_m}, 0_deg},
            Position{Length2{+1e16_m, 0_m}, 0_deg}
        };
        const auto sweepB = Sweep{
            Position{Length2{+2e16_m, 0_m}, 0_deg},
            Position{Length2{0_m, 0_m}, 0_deg}
        };
        const auto conf = ToiConf{}
            .UseTargetDepth(0.002_m)
            .UseTolerance(2e10_m)
            .UseMaxRootIters(255)
            .UseMaxToiIters(2)
            .UseMaxDistIters(1)
            ;
        const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
        EXPECT_EQ(output.state, TOIOutput::e_touching);
        EXPECT_NEAR(static_cast<double>(output.time), 0.79999959468841553, 0.0001);
        EXPECT_EQ(output.stats.max_dist_iters, 1);
        EXPECT_EQ(output.stats.max_root_iters, 2);
        EXPECT_EQ(output.stats.toi_iters, 2);
        EXPECT_EQ(output.stats.sum_dist_iters, 2);
        EXPECT_EQ(output.stats.sum_finder_iters, 1);
        EXPECT_EQ(output.stats.sum_root_iters, 2);
    }
}

