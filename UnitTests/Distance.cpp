/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Distance, MatchingCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{2_m, 2_m};
    const auto pos2 = Length2{2_m, 2_m};
    const auto normal = UnitVec{};
    DistanceProxy dp1{1_m, 1, &pos1, &normal};
    DistanceProxy dp2{1_m, 1, &pos2, &normal};

    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    const auto edges = output.simplex.GetEdges();
    ASSERT_EQ(edges.size(), std::uint8_t(1));

    const auto ips = GetIndexPairs(edges);
    EXPECT_EQ(ips[0], (IndexPair{0, 0}));
    EXPECT_EQ(ips[1], InvalidIndexPair);
    EXPECT_EQ(ips[2], InvalidIndexPair);

    conf.cache = Simplex::GetCache(edges);
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});

    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(std::get<0>(witnessPoints), pos1);
    EXPECT_EQ(std::get<1>(witnessPoints), pos1);
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});

    EXPECT_EQ(conf.cache.metric, Real{0});
}

TEST(Distance, OpposingCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{2_m, 2_m};
    const auto pos2 = Length2{-2_m, -2_m};
    const auto normal = UnitVec{};
    DistanceProxy dp1{2_m, 1, &pos1, &normal};
    DistanceProxy dp2{2_m, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos1));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos1));

    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos2));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos2));

    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
}

TEST(Distance, HorTouchingCircles)
{
    DistanceConf conf;
    
    const auto pos1 = Length2{-2_m, 2_m};
    const auto pos2 = Length2{+2_m, 2_m};
    const auto normal = UnitVec{};

    const auto output = [&]() {
        Transformation xf1 = Transform_identity;
        Transformation xf2 = Transform_identity;
        DistanceProxy dp1{2_m, 1, &pos1, &normal};
        DistanceProxy dp2{2_m, 1, &pos2, &normal};
        return Distance(dp1, xf1, dp2, xf2, conf);
    }();
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos1));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos1));
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos2));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
}

TEST(Distance, OverlappingCirclesPN)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{1_m, 1_m};
    const auto pos2 = Length2{-1_m, -1_m};
    const auto normal = UnitVec{};
    DistanceProxy dp1{2_m, 1, &pos1, &normal};
    DistanceProxy dp2{2_m, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos1));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos1));
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos2));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
}

TEST(Distance, OverlappingCirclesNP)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{-1_m, -1_m};
    const auto pos2 = Length2{1_m, 1_m};
    const auto normal = UnitVec{};
    DistanceProxy dp1{2_m, 1, &pos1, &normal};
    DistanceProxy dp2{2_m, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos1));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos1));
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos2));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
}


TEST(Distance, SeparatedCircles)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    const auto pos1 = Length2{2_m, 2_m};
    const auto pos2 = Length2{-2_m, -2_m};
    const auto normal = UnitVec{};
    DistanceProxy dp1{1_m, 1, &pos1, &normal};
    DistanceProxy dp2{1_m, 1, &pos2, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos1));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos1));
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos2));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos2));
    
    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
}

TEST(Distance, EdgeCircleOverlapping)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;

    const auto pos1 = Length2{0_m, 2_m};
    const auto pos2 = Length2{4_m, 2_m};
    const Length2 vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec normals[] = {normal1, -normal1};
    DistanceProxy dp1{0.1_m, 2, vertices, normals};
    
    const auto pos3 = Length2{2_m, 2_m};
    const auto normal = UnitVec{};
    DistanceProxy dp2{1_m, 1, &pos3, &normal};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos3));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos3));

    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos3));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos3));
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{2});
    
    const auto ip0 = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip0), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip0), VertexCounter{0});

    const auto ip1 = std::get<1>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip1), VertexCounter{1});
    EXPECT_EQ(std::get<1>(ip1), VertexCounter{0});
    
    EXPECT_NEAR(static_cast<double>(conf.cache.metric), 4.0, 0.000001);
}

TEST(Distance, EdgeCircleOverlapping2)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto pos1 = Length2{-3_m, 2_m};
    const auto pos2 = Length2{7_m, 2_m};
    const Length2 vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec normals[] = {normal1, -normal1};
    DistanceProxy dp1{0.1_m, 2, vertices, normals};

    const auto pos3 = Length2{2_m, 2_m};
    DistanceProxy dp2{1_m, 1, &pos3, nullptr};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), GetX(pos3));
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), GetY(pos3));
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), GetX(pos3));
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), GetY(pos3));
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{2});
    
    const auto ip0 = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip0), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip0), VertexCounter{0});
    
    const auto ip1 = std::get<1>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip1), VertexCounter{1});
    EXPECT_EQ(std::get<1>(ip1), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{10});
}

TEST(Distance, EdgeCircleTouching)
{
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto pos1 = Length2{0_m, 3_m};
    const auto pos2 = Length2{4_m, 3_m};
    const Length2 vertices[] = {pos1, pos2};
    const auto normal1 = GetUnitVector(pos2 - pos1);
    const UnitVec normals[] = {normal1, -normal1};
    DistanceProxy dp1{1_m, 2, vertices, normals};

    const auto pos3 = Length2{2_m, 1_m};
    DistanceProxy dp2{1_m, 1, &pos3, normals};
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), 2_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), 3_m);
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 2_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 1_m);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{2});
    
    const auto ip0 = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip0), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip0), VertexCounter{0});
    
    const auto ip1 = std::get<1>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip1), VertexCounter{1});
    EXPECT_EQ(std::get<1>(ip1), VertexCounter{0});
    
    EXPECT_NEAR(static_cast<double>(conf.cache.metric), 4.0, 0.000001);
}

TEST(Distance, HorEdgeSquareTouching)
{
    const auto pos1 = Length2{1_m, 1_m};
    const auto pos2 = Length2{1_m, 3_m};
    const auto pos3 = Length2{3_m, 3_m};
    const auto pos4 = Length2{3_m, 1_m};
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{0.5_m, 4, square, squareNormals};

    const auto pos5 = Length2{-2_m, 0_m};
    const auto pos6 = Length2{6_m, 0_m};
    const Length2 vertices[] = {pos5, pos6};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const UnitVec normals[] = {n5, -n5};
    DistanceProxy dp2{0.5_m, 2, vertices, normals};
    
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;

    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), 1_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), 1_m);
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 1_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 0_m);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{2});
    
    const auto ip0 = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip0), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip0), VertexCounter{0});
    
    const auto ip1 = std::get<1>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip1), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip1), VertexCounter{1});
    
    EXPECT_NEAR(static_cast<double>(conf.cache.metric), 8.0, 0.000001);
}

TEST(Distance, VerEdgeSquareTouching)
{
    const auto pos1 = Length2{1_m, 1_m};
    const auto pos2 = Length2{1_m, 3_m};
    const auto pos3 = Length2{3_m, 3_m};
    const auto pos4 = Length2{3_m, 1_m};
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{0.5_m, 4, square, squareNormals};
    
    const auto pos5 = Length2{4_m, -2_m};
    const auto pos6 = Length2{4_m, 6_m};
    const Length2 vertices[] = {pos5, pos6};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const UnitVec normals[] = {n5, -n5};
    DistanceProxy dp2{0.5_m, 2, vertices, normals};
    
    DistanceConf conf;
    Transformation xf1 = Transform_identity;
    Transformation xf2 = Transform_identity;
    
    const auto output = Distance(dp1, xf1, dp2, xf2, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_NEAR(static_cast<double>(Real{sqrt(GetMagnitudeSquared(std::get<0>(witnessPoints) - std::get<1>(witnessPoints))) / Meter}),
                1.0, 0.000001);
    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), 3_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), 2_m);
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 4_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 2_m);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{2});
    
    const auto ip0 = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip0), VertexCounter{2});
    EXPECT_EQ(std::get<1>(ip0), VertexCounter{0});
    
    const auto ip1 = std::get<1>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip1), VertexCounter{3});
    EXPECT_EQ(std::get<1>(ip1), VertexCounter{1});
    
    EXPECT_EQ(conf.cache.metric, Real{10});
}

TEST(Distance, SquareTwice)
{
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{2, 4} * Meter;
    const auto pos3 = Vec2{4, 4} * Meter;
    const auto pos4 = Vec2{4, 2} * Meter;
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{0.05_m, 4, square, squareNormals};

    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp1, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), 2_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), 2_m);

    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 2_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 2_m);

    EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
}


TEST(Distance, SquareSquareTouchingVertically)
{
    const auto pos1 = Vec2{2, 2} * Meter;
    const auto pos2 = Vec2{2, 4} * Meter;
    const auto pos3 = Vec2{4, 4} * Meter;
    const auto pos4 = Vec2{4, 2} * Meter;
    const Length2 square[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    DistanceProxy dp1{0.05_m, 4, square, squareNormals};
    
    const auto pos5 = Vec2{4, 2} * Meter;
    const auto pos6 = Vec2{4, 4} * Meter;
    const auto pos7 = Vec2{6, 4} * Meter;
    const auto pos8 = Vec2{6, 2} * Meter;
    const Length2 square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{0.05_m, 4, square2, normals2};

    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), 4_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), 3_m);
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 4_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 3_m);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{2});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{3});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{1});
    
    EXPECT_NEAR(static_cast<double>(conf.cache.metric), 4.0, 0.000001);
}

TEST(Distance, SquareSquareDiagonally)
{
    const auto pos1 = Vec2{-3, -3} * Meter;
    const auto pos2 = Vec2{-3, -1} * Meter;
    const auto pos3 = Vec2{-1, -1} * Meter;
    const auto pos4 = Vec2{-1, -3} * Meter;
    const Length2 square1[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec normals1[] = {n1, n2, n3, n4};
    DistanceProxy dp1{0.05_m, 4, square1, normals1};
    
    const auto pos5 = Vec2{1, 3} * Meter;
    const auto pos6 = Vec2{3, 3} * Meter;
    const auto pos7 = Vec2{3, 1} * Meter;
    const auto pos8 = Vec2{1, 1} * Meter;
    const Length2 square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{0.05_m, 4, square2, normals2};
    
    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), -1_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), -1_m);
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 1_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 1_m);
    
    EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{1});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{2});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{3});
    
    EXPECT_EQ(conf.cache.metric, Real{0});
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
    const Length2 square1[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(pos2 - pos1);
    const auto n2 = GetUnitVector(pos3 - pos2);
    const auto n3 = GetUnitVector(pos4 - pos3);
    const auto n4 = GetUnitVector(pos1 - pos4);
    const UnitVec normals1[] = {n1, n2, n3, n4};
    DistanceProxy dp1{NonNegative<Length>{0_m}, 4, square1, normals1};
    
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
    const Length2 square2[] = {pos5, pos6, pos7, pos8};
    const auto n5 = GetUnitVector(pos6 - pos5);
    const auto n6 = GetUnitVector(pos7 - pos6);
    const auto n7 = GetUnitVector(pos8 - pos7);
    const auto n8 = GetUnitVector(pos5 - pos8);
    const UnitVec normals2[] = {n5, n6, n7, n8};
    DistanceProxy dp2{NonNegative<Length>{0_m}, 4, square2, normals2};
    
    DistanceConf conf;
    Transformation xfm = Transform_identity;
    
    const auto output = Distance(dp1, xfm, dp2, xfm, conf);
    conf.cache = Simplex::GetCache(output.simplex.GetEdges());
    const auto witnessPoints = GetWitnessPoints(output.simplex);

    EXPECT_EQ(GetX(std::get<0>(witnessPoints)), 0_m);
    EXPECT_EQ(GetY(std::get<0>(witnessPoints)), 0.5_m);
    
    EXPECT_EQ(GetX(std::get<1>(witnessPoints)), 0_m);
    EXPECT_EQ(GetY(std::get<1>(witnessPoints)), 0.5_m);
    
    EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
    
    EXPECT_EQ(GetNumValidIndices(conf.cache.indices), std::uint8_t{3});
    
    const auto ip = std::get<0>(conf.cache.indices);
    EXPECT_EQ(std::get<0>(ip), VertexCounter{0});
    EXPECT_EQ(std::get<1>(ip), VertexCounter{0});
    
    EXPECT_EQ(conf.cache.metric, Real{-64});
}
