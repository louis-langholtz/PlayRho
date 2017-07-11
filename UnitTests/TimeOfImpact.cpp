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
#include <PlayRho/Collision/TimeOfImpact.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>

using namespace box2d;

TEST(TOIConf, DefaultConstruction)
{
    EXPECT_EQ(ToiConf{}.tMax, Real(1));
    EXPECT_EQ(ToiConf{}.maxRootIters, DefaultMaxToiRootIters);
    EXPECT_EQ(ToiConf{}.maxToiIters, DefaultMaxToiIters);
    EXPECT_EQ(ToiConf{}.targetDepth, DefaultLinearSlop * Real{3});
    EXPECT_EQ(ToiConf{}.tolerance, DefaultLinearSlop / Real{4});
}

TEST(TOIOutput, Types)
{
    EXPECT_GT(sizeof(TOIOutput::dist_sum_type), sizeof(TOIOutput::dist_iter_type));
    EXPECT_GT(sizeof(TOIOutput::root_sum_type), sizeof(TOIOutput::root_iter_type));
}

TEST(TOIOutput, DefaultConstruction)
{
    TOIOutput foo;
    EXPECT_EQ(foo.get_state(), TOIOutput::e_unknown);
}

TEST(TOIOutput, InitConstruction)
{
    const auto state = TOIOutput::e_failed;
    const auto time = Real(0.6);
    
    TOIOutput::Stats stats;
    stats.toi_iters = 3;
    stats.max_dist_iters = 11;
    stats.max_root_iters = 4;
    stats.sum_finder_iters = 0;
    stats.sum_dist_iters = 5;
    stats.sum_root_iters = 10;

    TOIOutput foo{state, time, stats};

    EXPECT_EQ(foo.get_state(), state);
    EXPECT_EQ(foo.get_t(), time);
    
    EXPECT_EQ(foo.get_toi_iters(), 3);
    EXPECT_EQ(foo.get_max_dist_iters(), 11);
    EXPECT_EQ(foo.get_max_root_iters(), 4);
    //EXPECT_EQ(foo.get_sum_finder_iters(), 0);
    EXPECT_EQ(foo.get_sum_dist_iters(), 5);
    EXPECT_EQ(foo.get_sum_root_iters(), 10);
}

TEST(TimeOfImpact, Overlapped)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth((slop * 3) * Meter).UseTolerance((slop / 4) * Meter);

    const auto radius = Real(1) * Meter;
    const auto pA = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2D(0, 0), Angle{0}}};
    const auto pB = Length2D(0, 0);
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2D(0, 0), Angle{0}}};
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    EXPECT_EQ(output.get_state(), TOIOutput::e_overlapped);
    EXPECT_EQ(output.get_t(), Real(0));
    EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, Touching)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth((slop * 3) * Meter).UseTolerance((slop / 4) * Meter);

    const auto radius = Real(1.1) * Meter;

    const auto pA = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2D(0, 0), Angle{0}}};
    
    const auto pB = Length2D(0, 0);
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2D{Real(2) * Meter, Real(0) * Meter}, Angle{0}}};

    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_EQ(output.get_t(), Real(0));
    EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, Separated)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * Real(3) * Meter).UseTolerance((slop / 4) * Meter);
    const auto radius = Real(1) * Meter;
    
    const auto pA = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2D(0, 0), Angle{0}}};
    
    const auto pB = Length2D(0, 0);
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2D{Real(4) * Meter, Real(0) * Meter}, Angle{0}}};
    
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
    EXPECT_EQ(output.get_t(), Real(1));
    EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, CollideCirclesHorizontally)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * Real(3) * Meter).UseTolerance((slop / 4) * Meter);

    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = Real(1) * Meter;
    const auto x = Real(2);
    const auto pA = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pA, nullptr};
    const auto sweepA = Sweep{Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}}, Position{Length2D(0, 0), Angle{0}}};
    const auto pB = Length2D(0, 0);
    const auto proxyB = DistanceProxy{radius, 1, &pB, nullptr};
    const auto sweepB = Sweep{Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}}, Position{Length2D(0, 0), Angle{0}}};
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    const auto approx_time_of_collision = ((x * Meter - radius) + limits.targetDepth / Real{2}) / (x * Meter);

    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_TRUE(almost_equal(output.get_t(), approx_time_of_collision));
    EXPECT_EQ(output.get_toi_iters(), 2);
}

TEST(TimeOfImpact, CollideCirclesVertically)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * Real(3) * Meter).UseTolerance((slop / 4) * Meter);
    const auto radius = Real(1) * Meter;
    const auto y = Real(20);

    const auto pos = Length2D(0, 0);

    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2D{Real(0) * Meter, -y * Meter}, Angle{0}},
        Position{Length2D{Real(0) * Meter, +y * Meter}, Angle{0}}
    };
    
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2D{Real(0) * Meter, +y * Meter}, Angle{0}},
        Position{Length2D{Real(0) * Meter, -y * Meter}, Angle{0}}
    };
    
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_NEAR(double(output.get_t()), 0.4750375, 0.000001);
    EXPECT_EQ(output.get_toi_iters(), 2);
}

TEST(TimeOfImpact, CirclesPassingParallelSeparatedPathsDontCollide)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * Real(3) * Meter).UseTolerance((slop / 4) * Meter);
    
    const auto pos = Length2D(0, 0);

    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = Real(1) * Meter;
    const auto x = Real(3);
    const auto y = Real(1);
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2D{-x * Meter, +y * Meter}, Angle{0}},
        Position{Length2D{+x * Meter, +y * Meter}, Angle{0}}
    };
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2D{+x * Meter, -y * Meter}, Angle{0}},
        Position{Length2D{-x * Meter, -y * Meter}, Angle{0}}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
    EXPECT_TRUE(almost_equal(output.get_t(), Real(1.0)));
    EXPECT_EQ(output.get_toi_iters(), 7);
}

TEST(TimeOfImpact, RodCircleMissAt360)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * Real(3) * Meter).UseTolerance((slop / 4) * Meter);
    
    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = Real(1) * Meter;
    const auto x = Real(40);
    const auto vA0 = Length2D{-Real(4) * Meter, Real(0) * Meter};
    const auto vA1 = Length2D{Real(4) * Meter, Real(0) * Meter};
    const Length2D vertices[] = {vA0, vA1};
    const auto nA0 = GetUnitVector(vA1 - vA0);
    const UnitVec2 normals[] = {nA0, -nA0};
    const auto proxyA = DistanceProxy{radius, 2, vertices, normals};
    const auto sweepA = Sweep{
        Position{Length2D{-x * Meter, Real(4) * Meter}, Angle{0}},
        Position{Length2D{+x * Meter, Real(4) * Meter}, Angle{Real{360.0f} * Degree}}
    };
    const auto pos = Length2D(0, 0);
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
    EXPECT_TRUE(almost_equal(output.get_t(), Real(1.0)));
    EXPECT_EQ(output.get_toi_iters(), 4);
}

TEST(TimeOfImpact, RodCircleHitAt180)
{
    const auto slop = Real{0.001f};
    const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(slop * Real(3) * Meter).UseTolerance((slop / 4) * Meter);
    
    // Set up for two bodies moving toward each other at same speeds and each colliding
    // with the other after they have moved roughly two-thirds of their sweep.
    const auto radius = Real(1) * Meter;
    const auto x = Real(40);
    const auto vA0 = Length2D{-Real(4) * Meter, Real(0) * Meter};
    const auto vA1 = Length2D{Real(4) * Meter, Real(0) * Meter};
    const Length2D vertices[] = {vA0, vA1};
    const auto nA0 = GetUnitVector(vA1 - vA0);
    const UnitVec2 normals[] = {nA0, -nA0};
    const auto proxyA = DistanceProxy{radius, 2, vertices, normals};
    const auto sweepA = Sweep{
        Position{Length2D{-x * Meter, Real(4) * Meter}, Angle{0}},
        Position{Length2D{+x * Meter, Real(4) * Meter}, Angle{Real{180.0f} * Degree}}
    };
    const auto pos = Length2D(0, 0);
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}}
    };
    
    // Compute the time of impact information now...
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, limits);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_NEAR(double(output.get_t()), 0.4884203672409058, 0.0001);
    EXPECT_EQ(output.get_toi_iters(), 3);
}

TEST(TimeOfImpact, SucceedsWithClosingSpeedOf800_1)
{
    const auto slop = Real{0.001f};
    const auto radius = Real(1) * Meter;
    const auto x = Real(200);
    const auto pos = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}}
    };
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}}
    };
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(200)
        .UseTargetDepth((slop * 3) * Meter)
        .UseTolerance((slop / 4) * Meter);
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_NEAR(double(output.get_t()), 0.4975037276744843, 0.0002);
    EXPECT_EQ(output.get_toi_iters(), 2);
    EXPECT_EQ(output.get_max_dist_iters(), 1);
    EXPECT_EQ(output.get_max_root_iters(), 2);
    EXPECT_EQ(output.get_sum_dist_iters(), 2);
    EXPECT_EQ(output.get_sum_root_iters(), 2);
}

TEST(TimeOfImpact, SucceedsWithClosingSpeedOf800_2)
{
    const auto slop = Real{0.001f};
    const auto radius = Real(1) * Meter;
    const auto x = Real(400);
    const auto pos = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{
        Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2D(0, 0), Angle{0}}
    };
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{
        Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}},
        Position{Length2D(0, 0), Angle{0}}
    };
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(200)
        .UseTargetDepth((slop * 3) * Meter)
        .UseTolerance((slop / 4) * Meter);
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_NEAR(double(output.get_t()), 0.9975037574768066, 0.002);
    EXPECT_EQ(output.get_toi_iters(), 2);
    EXPECT_EQ(output.get_max_dist_iters(), 1);
    EXPECT_EQ(output.get_max_root_iters(), 2);
    EXPECT_EQ(output.get_sum_dist_iters(), 2);
    EXPECT_EQ(output.get_sum_root_iters(), 2);

#if 0
    ASSERT_EQ(output.get_state(), TOIOutput::e_touching);

    auto touching = true;
    auto iterations = 0u;
    for (auto t = output.get_t(); t > 0; t = std::nextafter(t, 0.0f))
    {
        const auto conf2 = ToiConf{}.UseMaxToiIters(200).UseMaxRootIters(200).UseTimeMax(t);
        const auto output2 = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, conf2);
        
        EXPECT_LE(output2.get_t(), t);
        if (touching)
        {
            if (output2.get_state() != TOIOutput::e_touching)
            {
                std::cout << "lost touch after " << iterations << " iterations at t=" << t << std::endl;
                touching = false;
            }
        }
        else // !touching
        {
            if (output2.get_state() == TOIOutput::e_touching)
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
    const auto radius = Real(1) * Meter;
    const auto x = Real(400);
    const auto pos = Length2D(0, 0);
    const auto proxyA = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepA = Sweep{Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}}, Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}}};
    const auto proxyB = DistanceProxy{radius, 1, &pos, nullptr};
    const auto sweepB = Sweep{Position{Length2D{+x * Meter, Real(0) * Meter}, Angle{0}}, Position{Length2D{-x * Meter, Real(0) * Meter}, Angle{0}}};
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(200)
        .UseTargetDepth((slop * 3) * Meter)
        .UseTolerance((slop / 4) * Meter);
    const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, conf);
    
    EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
    EXPECT_NEAR(double(output.get_t()), 0.4987518787384033, 0.001);
    //std::cout << std::setprecision(std::numeric_limits<double>::digits10 + 1) << output.get_t() << std::defaultfloat << std::endl;
    EXPECT_EQ(output.get_toi_iters(), 2);
    EXPECT_EQ(output.get_max_dist_iters(), 1);
    EXPECT_EQ(output.get_max_root_iters(), 2);
    EXPECT_GE(output.get_sum_dist_iters(), output.get_max_dist_iters());
    EXPECT_GE(output.get_sum_root_iters(), output.get_max_root_iters());
}

TEST(TimeOfImpact, ForNonCollidingShapesFailsIn23)
{
    // The data for shapes and sweeps comes from PlayRho/Testbed/Tests/TimeOfImpact.hpp

    auto shapeA = PolygonShape{};
    shapeA.SetVertexRadius(Real{0.0001f * 2} * Meter);
    shapeA.SetAsBox(Real{25.0f} * Meter, Real{5.0f} * Meter);

    auto shapeB = PolygonShape{};
    shapeB.SetVertexRadius(Real{0.0001f * 2} * Meter);
    shapeB.SetAsBox(Real{2.5f} * Meter, Real{2.5f} * Meter);

    const auto dpA = shapeA.GetChild(0);
    const auto dpB = shapeB.GetChild(0);

    const auto sweepA = Sweep{
        Position{Length2D{-Real(11) * Meter, Real(10) * Meter}, Real{2.95000005f} * Radian},
        Position{Length2D{-Real(11) * Meter, Real(10) * Meter}, Real{2.95000005f} * Radian}
    };
    const auto sweepB = Sweep{
        Position{Length2D{18.4742737f * Meter, 19.7474861f * Meter}, Real{513.36676f} * Radian},
        Position{Length2D{19.5954781f * Meter, 18.9165268f * Meter}, Real{513.627808f} * Radian}
    };
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(20)
        .UseMaxRootIters(32)
        .UseTimeMax(1)
        .UseTargetDepth(Real(3.0f / 10000) * Meter)
        .UseTolerance(Real(1.0f / 40000) * Meter);
    const auto output = GetToiViaSat(dpA, sweepA, dpB, sweepB, conf);
    
    EXPECT_TRUE(output.get_state() == TOIOutput::e_failed || output.get_state() == TOIOutput::e_separated);
    switch (output.get_state())
    {
        case TOIOutput::e_failed:
            EXPECT_NEAR(double(output.get_t()), 0.863826394, 0.0001);
            EXPECT_EQ(output.get_toi_iters(), 1);
            EXPECT_EQ(output.get_max_dist_iters(), 4);
            EXPECT_EQ(output.get_max_root_iters(), 23);
            break;
        case TOIOutput::e_separated:
            EXPECT_EQ(output.get_t(), Real(1));
            EXPECT_EQ(output.get_toi_iters(), 2);
            EXPECT_GE(output.get_max_dist_iters(), 3);
            EXPECT_LE(output.get_max_dist_iters(), 4);
            EXPECT_EQ(output.get_max_root_iters(), 6);
            break;
        default:
            break;
    }
    EXPECT_GE(output.get_sum_dist_iters(), output.get_max_dist_iters());
    EXPECT_GE(output.get_sum_root_iters(), output.get_max_root_iters());
}

TEST(TimeOfImpact, ToleranceReachedWithT1Of1)
{
    // This setup causes the TimeOfImpact function to get into the state where
    // separation has reached tolerance but t2 already equals t1.

    const auto sweepA = Sweep{
        Position{Length2D{0.0f * Meter, -0.5f * Meter}, Angle{0}},
        Position{Length2D{0.0f * Meter, -0.5f * Meter}, Angle{0}}
    };
    const auto sweepB = Sweep{
        Position{Length2D{14.3689661f * Meter, 0.500306308f * Meter}, Real{0.0000139930862f} * Radian},
        Position{Length2D{14.3689451f * Meter, 0.500254989f * Meter}, Real{0.000260060915f} * Radian}
    };

    const Length2D vertices[] = {
        Vec2{14.5f, -0.5f} * (Real(1) * Meter),
        Vec2{14.5f, +0.5f} * (Real(1) * Meter),
        Vec2{13.5f, +0.5f} * (Real(1) * Meter),
        Vec2{13.5f, -0.5f} * (Real(1) * Meter)
    };

    const UnitVec2 normals[] = {
        GetUnitVector(vertices[1] - vertices[0]),
        GetUnitVector(vertices[2] - vertices[1]),
        GetUnitVector(vertices[3] - vertices[2]),
        GetUnitVector(vertices[0] - vertices[3]),
    };

    const auto dpA = DistanceProxy{
        Real{0.000199999995f} * Meter, 4, vertices, normals
    };
    
    auto shapeB = PolygonShape{};
    shapeB.SetVertexRadius(Real{0.0001f * 2} * Meter);
    shapeB.SetAsBox(Real{0.5f} * Meter, Real{0.5f} * Meter);
    const auto dpB = shapeB.GetChild(0);
    
    const auto conf = ToiConf{}
        .UseMaxToiIters(200)
        .UseMaxRootIters(30)
        .UseTimeMax(1)
        .UseTargetDepth(Real(3.0f / 10000) * Meter)
        .UseTolerance(Real(1.0f / 40000) * Meter);

    const auto output = GetToiViaSat(dpA, sweepA, dpB, sweepB, conf);

    EXPECT_TRUE(output.get_state() == TOIOutput::e_separated || output.get_state() == TOIOutput::e_touching);
    EXPECT_TRUE(almost_equal(output.get_t(), Real{1.0f}));
    EXPECT_TRUE(output.get_toi_iters() == 1 || output.get_toi_iters() == 2);
    EXPECT_EQ(output.get_max_dist_iters(), 4);
    EXPECT_EQ(output.get_max_root_iters(), 0);
    EXPECT_GE(output.get_sum_dist_iters(), output.get_max_dist_iters());
    EXPECT_GE(output.get_sum_root_iters(), output.get_max_root_iters());
}
