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
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/ShapeSeparation.hpp>
#include <initializer_list>
#include <vector>

using namespace playrho;

TEST(DistanceProxy, ByteSize)
{
    if (sizeof(Real) == 4)
    {
        EXPECT_EQ(sizeof(DistanceProxy), std::size_t(24));
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
    EXPECT_EQ(defaultDp.GetVertexRadius(), (Real(0) * Meter));
}

TEST(DistanceProxy, OneVecInitialization)
{
    const auto radius = Real{1} * Meter;
    const auto vertex0 = Length2{Real(2) * Meter, Real(-3) * Meter};
    const auto normal0 = UnitVec2{};
    const DistanceProxy foo{radius, 1, &vertex0, &normal0};
    EXPECT_EQ(radius, foo.GetVertexRadius());
    EXPECT_EQ(1, foo.GetVertexCount());
    EXPECT_EQ(vertex0, foo.GetVertex(0));
}

TEST(DistanceProxy, OneVecSupportIndex)
{
    const auto radius = Real{1} * Meter;
    const auto vertex0 = Length2{Real(2) * Meter, Real(-3) * Meter};
    const auto normal0 = UnitVec2{};
    const DistanceProxy foo{radius, 1, &vertex0, &normal0};
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(vertex0)));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{})));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{GetY(vertex0), GetX(vertex0)})));
}

TEST(DistanceProxy, TwoVecInitialization)
{
    const auto radius = Real{1} * Meter;
    const auto vertex0 = Length2{Real(2) * Meter, Real(3) * Meter};
    const auto vertex1 = Length2{Real(-10) * Meter, Real(-1) * Meter};
    const Length2 vertices[] = {vertex0, vertex1};
    const auto normal0 = GetUnitVector(vertex1 - vertex0);
    const UnitVec2 normals[] = {normal0, -normal0};
    const DistanceProxy foo{radius, 2, vertices, normals};
    EXPECT_EQ(radius, foo.GetVertexRadius());
    EXPECT_EQ(2, foo.GetVertexCount());
    EXPECT_EQ(vertex0, foo.GetVertex(0));
    EXPECT_EQ(vertex1, foo.GetVertex(1));
}

TEST(DistanceProxy, TwoVecSupportIndex)
{
    const auto radius = Real{1} * Meter;
    const auto vertex0 = Length2{Real(2) * Meter, Real(3) * Meter};
    const auto vertex1 = Length2{Real(-10) * Meter, Real(-1) * Meter};
    const Length2 vertices[] = {vertex0, vertex1};
    const auto normal0 = GetUnitVector(vertex1 - vertex0);
    const UnitVec2 normals[] = {normal0, -normal0};
    const DistanceProxy foo{radius, 2, vertices, normals};
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(vertex0)));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{GetY(vertex0), GetX(vertex0)})));
    EXPECT_EQ(0, GetSupportIndex(foo, GetVec2(Length2{Real(0) * Meter, Real(0) * Meter})));
    EXPECT_EQ(1, GetSupportIndex(foo, GetVec2(vertex1)));
    EXPECT_EQ(1, GetSupportIndex(foo, GetVec2(Length2{GetY(vertex1), GetX(vertex1)})));
}

TEST(DistanceProxy, ThreeVertices)
{
    const auto radius = Real(33) * Meter;
    const auto count = DistanceProxy::size_type(3);
    const auto v0 = Length2{Real(1) * Meter, Real(2) * Meter};
    const auto v1 = Length2{Real(-3) * Meter, Real(-4) * Meter};
    const auto v2 = Length2{Real(-6) * Meter, Real(5) * Meter};
    const Length2 vertices[] = {v0, v1, v2};
    const auto n0 = GetUnitVector(v1 - v0);
    const auto n1 = GetUnitVector(v2 - v1);
    const auto n2 = GetUnitVector(v0 - v2);
    const UnitVec2 normals[] = {n0, n1, n2};
    
    const DistanceProxy foo{radius, 3, vertices, normals};
    
    EXPECT_EQ(foo.GetVertexRadius(), radius);
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
    const auto span = Span<const Length2>(vertices.data(), std::size_t{0});
    const auto result = FindLowestRightMostVertex(span);
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
    const auto pos1 = Length2{Real(3) * Meter, Real(1) * Meter};
    const auto pos2 = Length2{Real(3) * Meter, Real(3) * Meter};
    const auto pos3 = Length2{Real(1) * Meter, Real(3) * Meter};
    const auto pos4 = Length2{Real(1) * Meter, Real(1) * Meter};
    const Length2 squareVerts[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    const auto radius = Real(0.5) * Meter;
    DistanceProxy dp{radius, 4, squareVerts, squareNormals};

    const auto pos0 = Length2{Real(2) * Meter, Real(2) * Meter};
    
    ASSERT_EQ(dp.GetVertexCount(), 4);
    EXPECT_TRUE(TestPoint(dp, pos0));
    EXPECT_TRUE(TestPoint(dp, pos1));
    EXPECT_TRUE(TestPoint(dp, pos2));
    EXPECT_TRUE(TestPoint(dp, pos3));
    EXPECT_TRUE(TestPoint(dp, pos4));
    EXPECT_TRUE(TestPoint(dp, Length2{Real(3.2f) * Meter, Real(3.2f) * Meter}));
    EXPECT_TRUE(TestPoint(dp, pos2 + Length2{radius, radius} / Real(2)));
    EXPECT_FALSE(TestPoint(dp, pos2 + Length2{radius, radius}));
    EXPECT_FALSE(TestPoint(dp, Length2{Real(10) * Meter, Real(10) * Meter}));
    EXPECT_FALSE(TestPoint(dp, Length2{-Real(10) * Meter, Real(10) * Meter}));
    EXPECT_FALSE(TestPoint(dp, Length2{Real(10) * Meter, -Real(10) * Meter}));
}

TEST(DistanceProxy, GetMaxSeparationFromWorld)
{
    const auto pos1 = Length2{Real(3) * Meter, Real(1) * Meter};
    const auto pos2 = Length2{Real(3) * Meter, Real(3) * Meter};
    const auto pos3 = Length2{Real(1) * Meter, Real(3) * Meter};
    const auto pos4 = Length2{Real(1) * Meter, Real(1) * Meter};
    const Length2 squareVerts[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec2 squareNormals[] = {n1, n2, n3, n4};
    const auto radius = Real(0.5) * Meter;
    const auto squareDp = DistanceProxy{radius, 4, squareVerts, squareNormals};
    
    const auto pos5 = Length2{Real(-2) * Meter, Real(2) * Meter};
    const Length2 circleVerts[] = {pos5};
    const auto n5 = UnitVec2::GetZero();
    const UnitVec2 circleNormals[] = {n5};
    const auto circleDp = DistanceProxy{radius, 1, circleVerts, circleNormals};
    
    const auto result1 = GetMaxSeparation(squareDp, circleDp);
    
    EXPECT_NEAR(static_cast<double>(Real(result1.separation / Meter)), 3.0, 0.0001);
    EXPECT_EQ(result1.index1, static_cast<decltype(result1.index1)>(2));
    EXPECT_EQ(result1.index2, static_cast<decltype(result1.index2)>(0));
    
    const auto result2 = GetMaxSeparation(squareDp, circleDp, Real(0) * Meter);
    
    EXPECT_NEAR(static_cast<double>(Real(result2.separation / Meter)), 3.0, 0.0001);
    EXPECT_EQ(result2.index1, static_cast<decltype(result2.index1)>(2));
    EXPECT_EQ(result2.index2, static_cast<decltype(result2.index2)>(0));
}
