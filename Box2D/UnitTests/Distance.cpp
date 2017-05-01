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
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>

using namespace box2d;

TEST(Distance, MatchingCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{2, 2} * Meter;
    const auto normal = UnitVec2{};
    DistanceProxy dp1{RealNum{1} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{RealNum{1} * Meter, 1, &pos2, &normal};

    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a, pos1);
    EXPECT_EQ(witnessPoints.b, pos1);
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});

    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}

TEST(Distance, OpposingCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{-2, -2} * Meter;
    const auto normal = UnitVec2{};
    DistanceProxy dp1{RealNum{2} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{RealNum{2} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos1.x);
    EXPECT_EQ(witnessPoints.a.y, pos1.y);

    EXPECT_EQ(witnessPoints.b.x, pos2.x);
    EXPECT_EQ(witnessPoints.b.y, pos2.y);

    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}

TEST(Distance, HorTouchingCircles)
{
    DistanceConf conf;
    
    const auto pos1 = Vec2{-2, 2} * Meter;
    const auto pos2 = Vec2{+2, 2} * Meter;
    const auto normal = UnitVec2{};

    const auto output = [&]() {
        Transformation xf1 = Transform_identity;
        Transformation xf2 = Transform_identity;
        DistanceProxy dp1{RealNum{2} * Meter, 1, &pos1, &normal};
        DistanceProxy dp2{RealNum{2} * Meter, 1, &pos2, &normal};
        return Distance(dp1, xf1, dp2, xf2, conf);
    }();
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos1.x);
    EXPECT_EQ(witnessPoints.a.y, pos1.y);
    
    EXPECT_EQ(witnessPoints.b.x, pos2.x);
    EXPECT_EQ(witnessPoints.b.y, pos2.y);
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}

TEST(Distance, OverlappingCirclesPN)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Vec2{1, 1} * Meter;
    const auto pos2 = Vec2{-1, -1} * Meter;
    const auto normal = UnitVec2{};
    DistanceProxy dp1{RealNum{2} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{RealNum{2} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos1.x);
    EXPECT_EQ(witnessPoints.a.y, pos1.y);
    
    EXPECT_EQ(witnessPoints.b.x, pos2.x);
    EXPECT_EQ(witnessPoints.b.y, pos2.y);
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}

TEST(Distance, OverlappingCirclesNP)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Vec2{-1, -1} * Meter;
    const auto pos2 = Vec2{1, 1} * Meter;
    const auto normal = UnitVec2{};
    DistanceProxy dp1{RealNum{2} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{RealNum{2} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos1.x);
    EXPECT_EQ(witnessPoints.a.y, pos1.y);
    
    EXPECT_EQ(witnessPoints.b.x, pos2.x);
    EXPECT_EQ(witnessPoints.b.y, pos2.y);
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}


TEST(Distance, SeparatedCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{-2, -2} * Meter;
    const auto normal = UnitVec2{};
    DistanceProxy dp1{RealNum{1} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{RealNum{1} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos1.x);
    EXPECT_EQ(witnessPoints.a.y, pos1.y);
    
    EXPECT_EQ(witnessPoints.b.x, pos2.x);
    EXPECT_EQ(witnessPoints.b.y, pos2.y);
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}

TEST(Distance, EdgeCircleOverlapping)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;

    const auto pos1 = Vec2{0, 2} * Meter;
    const auto pos2 = Vec2{4, 2} * Meter;
    const Length2D vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec2 normals[] = {normal1, -normal1};
    DistanceProxy dp1{RealNum(0.1) * Meter, 2, vertices, normals};
    
    const auto pos3 = Vec2{2, 2} * Meter;
    const auto normal = UnitVec2{};
    DistanceProxy dp2{RealNum{1} * Meter, 1, &pos3, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos3.x);
    EXPECT_EQ(witnessPoints.a.y, pos3.y);

    EXPECT_EQ(witnessPoints.b.x, pos3.x);
    EXPECT_EQ(witnessPoints.b.y, pos3.y);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});

    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{1});
    EXPECT_EQ(ip1.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{4});
}

TEST(Distance, EdgeCircleOverlapping2)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto pos1 = Vec2{-3, 2} * Meter;
    const auto pos2 = Vec2{7, 2} * Meter;
    const Length2D vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec2 normals[] = {normal1, -normal1};
    DistanceProxy dp1{RealNum(0.1) * Meter, 2, vertices, normals};

    const auto pos3 = Vec2{2, 2} * Meter;
    DistanceProxy dp2{RealNum{1} * Meter, 1, &pos3, nullptr};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, pos3.x);
    EXPECT_EQ(witnessPoints.a.y, pos3.y);
    
    EXPECT_EQ(witnessPoints.b.x, pos3.x);
    EXPECT_EQ(witnessPoints.b.y, pos3.y);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{1});
    EXPECT_EQ(ip1.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{10});
}

TEST(Distance, EdgeCircleTouching)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto pos1 = Vec2{0, 3} * Meter;
    const auto pos2 = Vec2{4, 3} * Meter;
    const Length2D vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec2 normals[] = {normal1, -normal1};
    DistanceProxy dp1{RealNum(1) * Meter, 2, vertices, normals};

    const auto pos3 = Vec2{2, 1} * Meter;
    DistanceProxy dp2{RealNum{1} * Meter, 1, &pos3, normals};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, RealNum{2} * Meter);
    EXPECT_EQ(witnessPoints.a.y, RealNum{3} * Meter);
    
    EXPECT_EQ(witnessPoints.b.x, RealNum{2} * Meter);
    EXPECT_EQ(witnessPoints.b.y, RealNum{1} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{1});
    EXPECT_EQ(ip1.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{4});
}

TEST(Distance, HorEdgeSquareTouching)
{
    const auto pos1 = Vec2{1, 1} * Meter;
    const auto pos2 = Vec2{1, 3} * Meter;
    const auto pos3 = Vec2{3, 3} * Meter;
    const auto pos4 = Vec2{3, 1} * Meter;
    const Length2D square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{RealNum(0.5) * Meter, 4, square, squareNormals};

    const auto pos5 = Vec2{-2, 0} * Meter;
    const auto pos6 = Vec2{6, 0} * Meter;
    const Length2D vertices[] = {pos5, pos6};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const UnitVec2 normals[] = {n5, -n5};
    DistanceProxy dp2{RealNum(0.5) * Meter, 2, vertices, normals};
    
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;

    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, RealNum{1} * Meter);
    EXPECT_EQ(witnessPoints.a.y, RealNum{1} * Meter);
    
    EXPECT_EQ(witnessPoints.b.x, RealNum{1} * Meter);
    EXPECT_EQ(witnessPoints.b.y, RealNum{0} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{0});
    EXPECT_EQ(ip1.b, IndexPair::size_type{1});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{8});
}

TEST(Distance, VerEdgeSquareTouching)
{
    const auto pos1 = Vec2{1, 1} * Meter;
    const auto pos2 = Vec2{1, 3} * Meter;
    const auto pos3 = Vec2{3, 3} * Meter;
    const auto pos4 = Vec2{3, 1} * Meter;
    const Length2D square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{RealNum(0.5) * Meter, 4, square, squareNormals};
    
    const auto pos5 = Vec2{4, -2} * Meter;
    const auto pos6 = Vec2{4, 6} * Meter;
    const Length2D vertices[] = {pos5, pos6};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const UnitVec2 normals[] = {n5, -n5};
    DistanceProxy dp2{RealNum(0.5) * Meter, 2, vertices, normals};
    
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b)), RealNum(1) * Meter);
    EXPECT_EQ(witnessPoints.a.x, RealNum{3} * Meter);
    EXPECT_EQ(witnessPoints.a.y, RealNum{2} * Meter);
    
    EXPECT_EQ(witnessPoints.b.x, RealNum{4} * Meter);
    EXPECT_EQ(witnessPoints.b.y, RealNum{2} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{2});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{3});
    EXPECT_EQ(ip1.b, IndexPair::size_type{1});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{10});
}

TEST(Distance, SquareTwice)
{
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{2, 4} * Meter;
    const auto pos3 = Vec2{4, 4} * Meter;
    const auto pos4 = Vec2{4, 2} * Meter;
    const Length2D square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{RealNum(0.05) * Meter, 4, square, squareNormals};

    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp1, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, RealNum{2} * Meter);
    EXPECT_EQ(witnessPoints.a.y, RealNum{2} * Meter);

    EXPECT_EQ(witnessPoints.b.x, RealNum{2} * Meter);
    EXPECT_EQ(witnessPoints.b.y, RealNum{2} * Meter);

    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}


TEST(Distance, SquareSquareTouchingVertically)
{
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{2, 4} * Meter;
    const auto pos3 = Vec2{4, 4} * Meter;
    const auto pos4 = Vec2{4, 2} * Meter;
    const Length2D square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{RealNum(0.05) * Meter, 4, square, squareNormals};
    
    const auto pos5 = Vec2{4, 2} * Meter;
    const auto pos6 = Vec2{4, 4} * Meter;
    const auto pos7 = Vec2{6, 4} * Meter;
    const auto pos8 = Vec2{6, 2} * Meter;
    const Length2D square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec2 normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{RealNum(0.05) * Meter, 4, square2, normals2};

    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, RealNum{4} * Meter);
    EXPECT_EQ(witnessPoints.a.y, RealNum{3} * Meter);
    
    EXPECT_EQ(witnessPoints.b.x, RealNum{4} * Meter);
    EXPECT_EQ(witnessPoints.b.y, RealNum{3} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){2});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{3});
    EXPECT_EQ(ip.b, IndexPair::size_type{1});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{4});
}

TEST(Distance, SquareSquareDiagonally)
{
    const auto pos1 = Vec2{-3, -3} * Meter;
    const auto pos2 = Vec2{-3, -1} * Meter;
    const auto pos3 = Vec2{-1, -1} * Meter;
    const auto pos4 = Vec2{-1, -3} * Meter;
    const Length2D square1[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 normals1[] = {n1, n2, n3, n4};
    DistanceProxy dp1{RealNum(0.05) * Meter, 4, square1, normals1};
    
    const auto pos5 = Vec2{1, 3} * Meter;
    const auto pos6 = Vec2{3, 3} * Meter;
    const auto pos7 = Vec2{3, 1} * Meter;
    const auto pos8 = Vec2{1, 1} * Meter;
    const Length2D square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec2 normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{RealNum(0.05) * Meter, 4, square2, normals2};
    
    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, RealNum{-1} * Meter);
    EXPECT_EQ(witnessPoints.a.y, RealNum{-1} * Meter);
    
    EXPECT_EQ(witnessPoints.b.x, RealNum{1} * Meter);
    EXPECT_EQ(witnessPoints.b.y, RealNum{1} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{2});
    EXPECT_EQ(ip.b, IndexPair::size_type{3});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{0});
}

TEST(Distance, SquareSquareOverlappingDiagnally)
{
    /*
     *  +-----1-+
     *  |     | |
     * -3-2-1-+-1-2-3-
     *  |     | |
     *  |     1 |
     *  |     | |
     *  |     2 |
     *  |     | |
     *  +-----3-+
     */
    // Go counter-clockwise...
    const auto pos1 = Vec2{-3, 1} * Meter;
    const auto pos2 = Vec2{-3, -3} * Meter;
    const auto pos3 = Vec2{1, -3} * Meter;
    const auto pos4 = Vec2{1, 1} * Meter;
    const Length2D square1[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 normals1[] = {n1, n2, n3, n4};
    DistanceProxy dp1{0, 4, square1, normals1};
    
    /*
     *  +-3-----+
     *  | |     |
     *  | 2     |
     *  | |     |
     *  | 1     |
     *  | |     |
     * -1-+-1-2-3--
     *  | |     |
     *  +-1-----+
     */
    // Go counter-clockwise...
    const auto pos5 = Vec2{3, 3} * Meter;
    const auto pos6 = Vec2{-1, 3} * Meter;
    const auto pos7 = Vec2{-1, -1} * Meter;
    const auto pos8 = Vec2{-1, 3} * Meter;
    const Length2D square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec2 normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{0, 4, square2, normals2};
    
    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a.x, Length{0});
    EXPECT_EQ(witnessPoints.a.y, RealNum{0.5f} * Meter);
    
    EXPECT_EQ(witnessPoints.b.x, Length{0});
    EXPECT_EQ(witnessPoints.b.y, RealNum{0.5f} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(conf.cache.GetNumIndices(), decltype(conf.cache.GetNumIndices()){3});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), RealNum{-64});
}
