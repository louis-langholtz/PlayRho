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
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/ShapeSeparation.hpp>
#include <initializer_list>
#include <vector>

using namespace playrho;
using namespace playrho::d2;

TEST(DistanceProxy, ByteSize)
{
    if (sizeof(Real) == 4)
    {
#if defined(_WIN32) && !defined(_WIN64)
        EXPECT_EQ(sizeof(DistanceProxy), std::size_t(16));
#else
        EXPECT_EQ(sizeof(DistanceProxy), std::size_t(24));
#endif
    }
    else if (sizeof(Real) == 8)
    {
        EXPECT_EQ(sizeof(DistanceProxy), std::size_t(32));
    }
    else if (sizeof(Real) == 16)
    {
        EXPECT_EQ(sizeof(DistanceProxy), std::size_t(48));
    }
    else
    {
        FAIL();
    }
}

TEST(DistanceProxy, DefaultInitialization)
{
    const auto defaultDp = DistanceProxy{};
    
    EXPECT_EQ(defaultDp.GetVertexCount(), 0);
    EXPECT_EQ(defaultDp.GetVertexRadius(), (0_m));
}

TEST(DistanceProxy, OneVecInitialization)
{
    const auto radius = 1_m;
    const auto vertex0 = Length2{2_m, -3_m};
    const auto normal0 = UnitVec{};
    const DistanceProxy foo{radius, 1, &vertex0, &normal0};
    EXPECT_EQ(radius, GetVertexRadius(foo));
    EXPECT_EQ(1, foo.GetVertexCount());
    EXPECT_EQ(vertex0, foo.GetVertex(0));
}

TEST(DistanceProxy, OneVecSupportIndex)
{
    const auto radius = 1_m;
    const auto vertex0 = Length2{2_m, -3_m};
    const auto normal0 = UnitVec{};
    const DistanceProxy foo{radius, 1, &vertex0, &normal0};
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(vertex0)));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{})));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{GetY(vertex0), GetX(vertex0)})));
}

TEST(DistanceProxy, TwoVecInitialization)
{
    const auto radius = 1_m;
    const auto vertex0 = Length2{2_m, 3_m};
    const auto vertex1 = Length2{-10_m, -1_m};
    const Length2 vertices[] = {vertex0, vertex1};
    const auto normal0 = GetUnitVector(vertex1 - vertex0);
    const UnitVec normals[] = {normal0, -normal0};
    const DistanceProxy foo{radius, 2, vertices, normals};
    EXPECT_EQ(radius, GetVertexRadius(foo));
    EXPECT_EQ(2, foo.GetVertexCount());
    EXPECT_EQ(vertex0, foo.GetVertex(0));
    EXPECT_EQ(vertex1, foo.GetVertex(1));
}

TEST(DistanceProxy, TwoVecSupportIndex)
{
    const auto radius = 1_m;
    const auto vertex0 = Length2{2_m, 3_m};
    const auto vertex1 = Length2{-10_m, -1_m};
    const Length2 vertices[] = {vertex0, vertex1};
    const auto normal0 = GetUnitVector(vertex1 - vertex0);
    const UnitVec normals[] = {normal0, -normal0};
    const DistanceProxy foo{radius, 2, vertices, normals};
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(vertex0)));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{GetY(vertex0), GetX(vertex0)})));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{0_m, 0_m})));
    EXPECT_EQ(1, GetSupportIndex(foo, GetVec2(vertex1)));
    EXPECT_EQ(1, GetSupportIndex(foo, GetVec2(Length2{GetY(vertex1), GetX(vertex1)})));
}

TEST(DistanceProxy, ThreeVertices)
{
    const auto radius = 33_m;
    const auto count = VertexCounter(3);
    const auto v0 = Length2{1_m, 2_m};
    const auto v1 = Length2{-3_m, -4_m};
    const auto v2 = Length2{-6_m, 5_m};
    const Length2 vertices[] = {v0, v1, v2};
    const auto n0 = GetUnitVector(v1 - v0);
    const auto n1 = GetUnitVector(v2 - v1);
    const auto n2 = GetUnitVector(v0 - v2);
    const UnitVec normals[] = {n0, n1, n2};
    
    const DistanceProxy foo{radius, 3, vertices, normals};
    
    EXPECT_EQ(GetVertexRadius(foo), radius);
    ASSERT_EQ(foo.GetVertexCount(), count);
    EXPECT_EQ(GetX(foo.GetVertex(0)), GetX(v0));
    EXPECT_EQ(GetY(foo.GetVertex(0)), GetY(v0));
    EXPECT_EQ(GetX(foo.GetVertex(1)), GetX(v1));
    EXPECT_EQ(GetY(foo.GetVertex(1)), GetY(v1));
    EXPECT_EQ(GetX(foo.GetVertex(2)), GetX(v2));
    EXPECT_EQ(GetY(foo.GetVertex(2)), GetY(v2));
}

TEST(DistanceProxy, FindLowestRightMostVertex)
{
    const auto vertices = std::vector<Length2>();
    const auto result = FindLowestRightMostVertex(vertices);
    EXPECT_EQ(result, static_cast<std::size_t>(-1));
}

TEST(DistanceProxy, TestPointWithEmptyProxyReturnsFalse)
{
    const auto defaultDp = DistanceProxy{};
    ASSERT_EQ(defaultDp.GetVertexCount(), 0);
    EXPECT_FALSE(TestPoint(defaultDp, Length2{}));
}

TEST(DistanceProxy, TestPoint)
{
    const auto pos1 = Length2{3_m, 1_m};
    const auto pos2 = Length2{3_m, 3_m};
    const auto pos3 = Length2{1_m, 3_m};
    const auto pos4 = Length2{1_m, 1_m};
    const Length2 squareVerts[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    const auto radius = 0.5_m;
    DistanceProxy dp{radius, 4, squareVerts, squareNormals};

    const auto pos0 = Length2{2_m, 2_m};
    
    ASSERT_EQ(dp.GetVertexCount(), 4);
    EXPECT_TRUE(TestPoint(dp, pos0));
    EXPECT_TRUE(TestPoint(dp, pos1));
    EXPECT_TRUE(TestPoint(dp, pos2));
    EXPECT_TRUE(TestPoint(dp, pos3));
    EXPECT_TRUE(TestPoint(dp, pos4));
    EXPECT_TRUE(TestPoint(dp, Length2{3.2_m, 3.2_m}));
    EXPECT_TRUE(TestPoint(dp, pos2 + Length2{radius, radius} / Real(2)));
    EXPECT_FALSE(TestPoint(dp, pos2 + Length2{radius, radius}));
    EXPECT_FALSE(TestPoint(dp, Length2{10_m, 10_m}));
    EXPECT_FALSE(TestPoint(dp, Length2{-10_m, 10_m}));
    EXPECT_FALSE(TestPoint(dp, Length2{10_m, -10_m}));
}

TEST(DistanceProxy, GetMaxSeparationStopping)
{
    const auto pos1 = Length2{-1_m, -1_m};
    const auto pos2 = Length2{+1_m, -1_m};
    const auto pos3 = Length2{+1_m, +1_m};
    const auto pos4 = Length2{-1_m, +1_m};
    const Length2 vertices[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec normals[] = {n1, n2, n3, n4};
    const auto radius = 1_m;
    const auto dp = DistanceProxy{radius, 4, vertices, normals};
    
    const auto xfm1 = Transformation{Length2{-1_m, 0_m}, UnitVec::Get(190_deg)};
    const auto xfm2 = Transformation{Length2{+1_m, 0_m}, UnitVec::Get( 95_deg)};
    const auto resultRegular = GetMaxSeparation(dp, xfm1, dp, xfm2);
    const auto resultStopped = GetMaxSeparation(dp, xfm1, dp, xfm2, -3_m);
    
    EXPECT_NE(resultRegular.distance, resultStopped.distance);
}

TEST(DistanceProxy, GetMaxSeparationFromWorld)
{
    const auto pos1 = Length2{3_m, 1_m};
    const auto pos2 = Length2{3_m, 3_m};
    const auto pos3 = Length2{1_m, 3_m};
    const auto pos4 = Length2{1_m, 1_m};
    const Length2 squareVerts[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    const auto radius = 0.5_m;
    const auto squareDp = DistanceProxy{radius, 4, squareVerts, squareNormals};
    
    const auto pos5 = Length2{-2_m, 2_m};
    const Length2 circleVerts[] = {pos5};
    const auto n5 = UnitVec::GetZero();
    const UnitVec circleNormals[] = {n5};
    const auto circleDp = DistanceProxy{radius, 1, circleVerts, circleNormals};
    
    const auto result1 = GetMaxSeparation(squareDp, circleDp);
    
    EXPECT_NEAR(static_cast<double>(Real(result1.distance / Meter)), 3.0, 0.0001);
    EXPECT_EQ(result1.firstShape, static_cast<VertexCounter>(2));
    EXPECT_EQ(std::get<0>(result1.secondShape), static_cast<VertexCounter>(0));
    
    const auto result2 = GetMaxSeparation(squareDp, circleDp, 0_m);
    
    EXPECT_NEAR(static_cast<double>(Real(result2.distance / Meter)), 3.0, 0.0001);
    EXPECT_EQ(result2.firstShape, static_cast<VertexCounter>(2));
    EXPECT_EQ(std::get<0>(result2.secondShape), static_cast<VertexCounter>(0));
}

TEST(DistanceProxy, Equality)
{
    const auto pos1 = Length2{3_m, 1_m};
    const auto pos2 = Length2{3_m, 3_m};
    const auto pos3 = Length2{1_m, 3_m};
    const auto pos4 = Length2{1_m, 1_m};
    const Length2 verts[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec norms[] = {n1, n2, n3, n4};

    EXPECT_TRUE(DistanceProxy(0.0_m, 4, verts, norms) == DistanceProxy(0.0_m, 4, verts, norms));
    EXPECT_FALSE(DistanceProxy(1.0_m, 4, verts, norms) == DistanceProxy(0.0_m, 4, verts, norms));
}
