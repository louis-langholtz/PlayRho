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
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>

using namespace playrho;

TEST(Distance, MatchingCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{Real(2) * Meter, Real(2) * Meter};
    const auto pos2 = Length2{Real(2) * Meter, Real(2) * Meter};
    const auto normal = UnitVec2{};
    DistanceProxy dp1{Real{1} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{Real{1} * Meter, 1, &pos2, &normal};

    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    const auto edges = output.simplex.GetEdges();
    ASSERT_EQ(edges.size(), std::uint8_t(1));

    const auto ips = Simplex::GetIndexPairs(edges);
    EXPECT_EQ(ips[0], (IndexPair{0, 0}));
    EXPECT_EQ(ips[1], InvalidIndexPair);
    EXPECT_EQ(ips[2], InvalidIndexPair);

    conf.cache = Simplex::GetCache(edges);
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});

    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(witnessPoints.a, pos1);
    EXPECT_EQ(witnessPoints.b, pos1);
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});

    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}

TEST(Distance, OpposingCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{Real(2) * Meter, Real(2) * Meter};
    const auto pos2 = Length2{-Real(2) * Meter, -Real(2) * Meter};
    const auto normal = UnitVec2{};
    DistanceProxy dp1{Real{2} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{Real{2} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos1));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos1));

    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos2));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos2));

    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}

TEST(Distance, HorTouchingCircles)
{
    DistanceConf conf;
    
    const auto pos1 = Length2{-Real(2) * Meter, Real(2) * Meter};
    const auto pos2 = Length2{+Real(2) * Meter, Real(2) * Meter};
    const auto normal = UnitVec2{};

    const auto output = [&]() {
        Transformation xf1 = Transform_identity;
        Transformation xf2 = Transform_identity;
        DistanceProxy dp1{Real{2} * Meter, 1, &pos1, &normal};
        DistanceProxy dp2{Real{2} * Meter, 1, &pos2, &normal};
        return Distance(dp1, xf1, dp2, xf2, conf);
    }();
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos1));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos1));
    
    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos2));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}

TEST(Distance, OverlappingCirclesPN)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{Real(1) * Meter, Real(1) * Meter};
    const auto pos2 = Length2{-Real(1) * Meter, -Real(1) * Meter};
    const auto normal = UnitVec2{};
    DistanceProxy dp1{Real{2} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{Real{2} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos1));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos1));
    
    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos2));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}

TEST(Distance, OverlappingCirclesNP)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{-Real(1) * Meter, -Real(1) * Meter};
    const auto pos2 = Length2{Real(1) * Meter, Real(1) * Meter};
    const auto normal = UnitVec2{};
    DistanceProxy dp1{Real{2} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{Real{2} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos1));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos1));
    
    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos2));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}


TEST(Distance, SeparatedCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{Real(2) * Meter, Real(2) * Meter};
    const auto pos2 = Length2{-Real(2) * Meter, -Real(2) * Meter};
    const auto normal = UnitVec2{};
    DistanceProxy dp1{Real{1} * Meter, 1, &pos1, &normal};
    DistanceProxy dp2{Real{1} * Meter, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos1));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos1));
    
    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos2));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}

TEST(Distance, EdgeCircleOverlapping)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;

    const auto pos1 = Length2{Real(0) * Meter, Real(2) * Meter};
    const auto pos2 = Length2{Real(4) * Meter, Real(2) * Meter};
    const Length2 vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec2 normals[] = {normal1, -normal1};
    DistanceProxy dp1{Real(0.1) * Meter, 2, vertices, normals};
    
    const auto pos3 = Length2{Real(2) * Meter, Real(2) * Meter};
    const auto normal = UnitVec2{};
    DistanceProxy dp2{Real{1} * Meter, 1, &pos3, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos3));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos3));

    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos3));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos3));
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});

    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{1});
    EXPECT_EQ(ip1.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_NEAR(static_cast<double>(conf.cache.GetMetric()), 4.0, 0.000001);
}

TEST(Distance, EdgeCircleOverlapping2)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto pos1 = Length2{-Real(3) * Meter, Real(2) * Meter};
    const auto pos2 = Length2{Real(7) * Meter, Real(2) * Meter};
    const Length2 vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec2 normals[] = {normal1, -normal1};
    DistanceProxy dp1{Real(0.1) * Meter, 2, vertices, normals};

    const auto pos3 = Length2{Real(2) * Meter, Real(2) * Meter};
    DistanceProxy dp2{Real{1} * Meter, 1, &pos3, nullptr};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), GetX(pos3));
    EXPECT_EQ(GetY(witnessPoints.a), GetY(pos3));
    
    EXPECT_EQ(GetX(witnessPoints.b), GetX(pos3));
    EXPECT_EQ(GetY(witnessPoints.b), GetY(pos3));
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{1});
    EXPECT_EQ(ip1.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{10});
}

TEST(Distance, EdgeCircleTouching)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto pos1 = Length2{Real(0) * Meter, Real(3) * Meter};
    const auto pos2 = Length2{Real(4) * Meter, Real(3) * Meter};
    const Length2 vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec2 normals[] = {normal1, -normal1};
    DistanceProxy dp1{Real(1) * Meter, 2, vertices, normals};

    const auto pos3 = Length2{Real(2) * Meter, Real(1) * Meter};
    DistanceProxy dp2{Real{1} * Meter, 1, &pos3, normals};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), Real{2} * Meter);
    EXPECT_EQ(GetY(witnessPoints.a), Real{3} * Meter);
    
    EXPECT_EQ(GetX(witnessPoints.b), Real{2} * Meter);
    EXPECT_EQ(GetY(witnessPoints.b), Real{1} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{1});
    EXPECT_EQ(ip1.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_NEAR(static_cast<double>(conf.cache.GetMetric()), 4.0, 0.000001);
}

TEST(Distance, HorEdgeSquareTouching)
{
    const auto pos1 = Length2{Real(1) * Meter, Real(1) * Meter};
    const auto pos2 = Length2{Real(1) * Meter, Real(3) * Meter};
    const auto pos3 = Length2{Real(3) * Meter, Real(3) * Meter};
    const auto pos4 = Length2{Real(3) * Meter, Real(1) * Meter};
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{Real(0.5) * Meter, 4, square, squareNormals};

    const auto pos5 = Length2{-Real(2) * Meter, Real(0) * Meter};
    const auto pos6 = Length2{Real(6) * Meter, Real(0) * Meter};
    const Length2 vertices[] = {pos5, pos6};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const UnitVec2 normals[] = {n5, -n5};
    DistanceProxy dp2{Real(0.5) * Meter, 2, vertices, normals};
    
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;

    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), Real{1} * Meter);
    EXPECT_EQ(GetY(witnessPoints.a), Real{1} * Meter);
    
    EXPECT_EQ(GetX(witnessPoints.b), Real{1} * Meter);
    EXPECT_EQ(GetY(witnessPoints.b), Real{0} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{0});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{0});
    EXPECT_EQ(ip1.b, IndexPair::size_type{1});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_NEAR(static_cast<double>(conf.cache.GetMetric()), 8.0, 0.000001);
}

TEST(Distance, VerEdgeSquareTouching)
{
    const auto pos1 = Length2{Real(1) * Meter, Real(1) * Meter};
    const auto pos2 = Length2{Real(1) * Meter, Real(3) * Meter};
    const auto pos3 = Length2{Real(3) * Meter, Real(3) * Meter};
    const auto pos4 = Length2{Real(3) * Meter, Real(1) * Meter};
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{Real(0.5) * Meter, 4, square, squareNormals};
    
    const auto pos5 = Length2{Real(4) * Meter, -Real(2) * Meter};
    const auto pos6 = Length2{Real(4) * Meter, Real(6) * Meter};
    const Length2 vertices[] = {pos5, pos6};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const UnitVec2 normals[] = {n5, -n5};
    DistanceProxy dp2{Real(0.5) * Meter, 2, vertices, normals};
    
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_NEAR(static_cast<double>(Real{Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b)) / Meter}),
                1.0, 0.000001);
    EXPECT_EQ(GetX(witnessPoints.a), Real{3} * Meter);
    EXPECT_EQ(GetY(witnessPoints.a), Real{2} * Meter);
    
    EXPECT_EQ(GetX(witnessPoints.b), Real{4} * Meter);
    EXPECT_EQ(GetY(witnessPoints.b), Real{2} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{2});
    
    const auto ip0 = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip0.a, IndexPair::size_type{2});
    EXPECT_EQ(ip0.b, IndexPair::size_type{0});
    
    const auto ip1 = conf.cache.GetIndexPair(1);
    EXPECT_EQ(ip1.a, IndexPair::size_type{3});
    EXPECT_EQ(ip1.b, IndexPair::size_type{1});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{10});
}

TEST(Distance, SquareTwice)
{
    const auto pos1 = Vec2{2, 2} * (Real(1) * Meter);
    const auto pos2 = Vec2{2, 4} * (Real(1) * Meter);
    const auto pos3 = Vec2{4, 4} * (Real(1) * Meter);
    const auto pos4 = Vec2{4, 2} * (Real(1) * Meter);
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{Real(0.05) * Meter, 4, square, squareNormals};

    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp1, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), Real{2} * Meter);
    EXPECT_EQ(GetY(witnessPoints.a), Real{2} * Meter);

    EXPECT_EQ(GetX(witnessPoints.b), Real{2} * Meter);
    EXPECT_EQ(GetY(witnessPoints.b), Real{2} * Meter);

    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
}


TEST(Distance, SquareSquareTouchingVertically)
{
    const auto pos1 = Vec2{2, 2} * (Real(1) * Meter);
    const auto pos2 = Vec2{2, 4} * (Real(1) * Meter);
    const auto pos3 = Vec2{4, 4} * (Real(1) * Meter);
    const auto pos4 = Vec2{4, 2} * (Real(1) * Meter);
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{Real(0.05) * Meter, 4, square, squareNormals};
    
    const auto pos5 = Vec2{4, 2} * (Real(1) * Meter);
    const auto pos6 = Vec2{4, 4} * (Real(1) * Meter);
    const auto pos7 = Vec2{6, 4} * (Real(1) * Meter);
    const auto pos8 = Vec2{6, 2} * (Real(1) * Meter);
    const Length2 square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec2 normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{Real(0.05) * Meter, 4, square2, normals2};

    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), Real{4} * Meter);
    EXPECT_EQ(GetY(witnessPoints.a), Real{3} * Meter);
    
    EXPECT_EQ(GetX(witnessPoints.b), Real{4} * Meter);
    EXPECT_EQ(GetY(witnessPoints.b), Real{3} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{2});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{3});
    EXPECT_EQ(ip.b, IndexPair::size_type{1});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_NEAR(static_cast<double>(conf.cache.GetMetric()), 4.0, 0.000001);
}

TEST(Distance, SquareSquareDiagonally)
{
    const auto pos1 = Vec2{-3, -3} * (Real(1) * Meter);
    const auto pos2 = Vec2{-3, -1} * (Real(1) * Meter);
    const auto pos3 = Vec2{-1, -1} * (Real(1) * Meter);
    const auto pos4 = Vec2{-1, -3} * (Real(1) * Meter);
    const Length2 square1[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 normals1[] = {n1, n2, n3, n4};
    DistanceProxy dp1{Real(0.05) * Meter, 4, square1, normals1};
    
    const auto pos5 = Vec2{1, 3} * (Real(1) * Meter);
    const auto pos6 = Vec2{3, 3} * (Real(1) * Meter);
    const auto pos7 = Vec2{3, 1} * (Real(1) * Meter);
    const auto pos8 = Vec2{1, 1} * (Real(1) * Meter);
    const Length2 square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec2 normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{Real(0.05) * Meter, 4, square2, normals2};
    
    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), Real{-1} * Meter);
    EXPECT_EQ(GetY(witnessPoints.a), Real{-1} * Meter);
    
    EXPECT_EQ(GetX(witnessPoints.b), Real{1} * Meter);
    EXPECT_EQ(GetY(witnessPoints.b), Real{1} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{1});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{2});
    EXPECT_EQ(ip.b, IndexPair::size_type{3});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{0});
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
    const auto pos1 = Vec2{-3, 1} * (Real(1) * Meter);
    const auto pos2 = Vec2{-3, -3} * (Real(1) * Meter);
    const auto pos3 = Vec2{1, -3} * (Real(1) * Meter);
    const auto pos4 = Vec2{1, 1} * (Real(1) * Meter);
    const Length2 square1[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec2 normals1[] = {n1, n2, n3, n4};
    DistanceProxy dp1{NonNegative<Length>{0}, 4, square1, normals1};
    
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
    const auto pos5 = Vec2{3, 3} * (Real(1) * Meter);
    const auto pos6 = Vec2{-1, 3} * (Real(1) * Meter);
    const auto pos7 = Vec2{-1, -1} * (Real(1) * Meter);
    const auto pos8 = Vec2{-1, 3} * (Real(1) * Meter);
    const Length2 square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec2 normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{NonNegative<Length>{0}, 4, square2, normals2};
    
    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(witnessPoints.a), Length{0});
    EXPECT_EQ(GetY(witnessPoints.a), Real{0.5f} * Meter);
    
    EXPECT_EQ(GetX(witnessPoints.b), Length{0});
    EXPECT_EQ(GetY(witnessPoints.b), Real{0.5f} * Meter);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(GetNumIndices(conf.cache.GetIndices()), std::uint8_t{3});
    
    const auto ip = conf.cache.GetIndexPair(0);
    EXPECT_EQ(ip.a, IndexPair::size_type{0});
    EXPECT_EQ(ip.b, IndexPair::size_type{0});
    
    EXPECT_EQ(true, conf.cache.IsMetricSet());
    EXPECT_EQ(conf.cache.GetMetric(), Real{-64});
}
