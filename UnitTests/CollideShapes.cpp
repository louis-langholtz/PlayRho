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
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

using namespace box2d;

TEST(CollideShapes, IdenticalOverlappingCircles)
{
    const auto radius = RealNum(1) * Meter;
    const auto shape = CircleShape{radius};
    const auto position = Vec2{11, -4} * Meter;
    const auto xfm = Transformation{position, UnitVec2{Angle{0}}};
    
    // put shape 1 to left of shape 2
    const auto manifold = CollideShapes(shape.GetChild(0), xfm, shape.GetChild(0), xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetLocalPoint(), shape.GetLocation());
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, shape.GetLocation());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleCircleOrientedHorizontally)
{
    const auto r1 = RealNum(1) * Meter;
    const auto r2 = RealNum(1) * Meter;
    const auto s1 = CircleShape{r1};
    const auto s2 = CircleShape{r2};
    const auto p1 = Vec2{11, -4} * Meter;
    const auto p2 = Vec2{13, -4} * Meter;
    const auto t1 = Transformation{p1, UnitVec2{Angle{0}}};
    const auto t2 = Transformation{p2, UnitVec2{Angle{0}}};
    
    // put shape 1 to left of shape 2
    const auto manifold = CollideShapes(s1.GetChild(0), t1, s2.GetChild(0), t2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetLocalPoint(), s1.GetLocation());
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));

    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, s2.GetLocation());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleCircleOrientedVertically)
{
    const auto r1 = RealNum(1) * Meter;
    const auto r2 = RealNum(1) * Meter;
    const auto s1 = CircleShape{r1};
    const auto s2 = CircleShape{r2};
    const auto p1 = Vec2{7, -2} * Meter;
    const auto p2 = Vec2{7, -1} * Meter;
    
    // Rotations don't matter so long as circle shapes' centers are at (0, 0).
    const auto t1 = Transformation{p1, UnitVec2{RealNum{45.0f} * Degree}};
    const auto t2 = Transformation{p2, UnitVec2{RealNum{-21.0f} * Degree}};
    
    // put shape 1 to left of shape 2
    const auto manifold = CollideShapes(s1.GetChild(0), t1, s2.GetChild(0), t2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTouchingTrianglePointBelow)
{
    const auto circleRadius = RealNum(1) * Meter;
    const auto circle = CircleShape(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto triangleXfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto circleXfm = Transformation{triangleTopPt + UnitVec2{RealNum{90} * Degree} * circleRadius, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(triangle.GetChild(0), triangleXfm, circle.GetChild(0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), triangleTopPt);
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleTopPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTouchingTrianglePointLeft)
{
    const auto circleRadius = RealNum(1) * Meter;
    const auto circle = CircleShape(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{triangleLeftPt + UnitVec2{RealNum{225.0f} * Degree} * circleRadius, UnitVec2{RealNum{0.0f} * Degree}};
    const auto triangleXfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    
    const auto manifold = CollideShapes(triangle.GetChild(0), triangleXfm, circle.GetChild(0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), triangleLeftPt);
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleLeftPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTouchingTrianglePointRight)
{
    const auto circleRadius = RealNum(1) * Meter;
    const auto circle = CircleShape(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{triangleRightPt + UnitVec2{-RealNum{45.0f} * Degree} * circleRadius, UnitVec2{RealNum{0.0f} * Degree}};
    const auto triangleXfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    
    const auto manifold = CollideShapes(triangle.GetChild(0), triangleXfm, circle.GetChild(0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), triangleRightPt);
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleRightPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleJustPastTrianglePointRightDoesntCollide)
{
    const auto circleRadius = RealNum(1) * Meter;
    const auto circle = CircleShape(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    auto triangle = PolygonShape{};
    triangle.SetVertexRadius(RealNum{0.0001f * 2} * Meter);
    triangle.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{triangleRightPt + UnitVec2{-RealNum{45.0f} * Degree} * circleRadius * RealNum(1.001), UnitVec2{RealNum{0.0f} * Degree}};
    const auto triangleXfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    
    const auto manifold = CollideShapes(triangle.GetChild(0), triangleXfm, circle.GetChild(0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
    EXPECT_FALSE(IsValid(manifold.GetLocalPoint()));
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(0));
}

TEST(CollideShapes, CircleOverRightFaceOfTriangle)
{
    const auto circleRadius = RealNum(1) * Meter;
    const auto circle = CircleShape(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{Vec2{1, 1} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    const auto triangleXfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    
    const auto manifold = CollideShapes(triangle.GetChild(0), triangleXfm, circle.GetChild(0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (triangleTopPt + triangleRightPt) / RealNum{2});
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetX()), 0.894427  , 0.0001);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetY()), 0.44721359, 0.0001);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, circle.GetLocation());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(triangle.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);

    EXPECT_EQ(triangle.GetVertex(0), Vec2(+1, -1) * Meter);
}

TEST(CollideShapes, CircleOverLeftFaceOfTriangle)
{
    const auto circleRadius = RealNum(1) * Meter;
    const auto circle = CircleShape(circleRadius);
    const auto triangle = PolygonShape({Vec2{-1, -1} * Meter, Vec2{+1, -1} * Meter, Vec2{0, +1} * Meter});
    const auto circleXfm = Transformation{Vec2{-1, 1} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    const auto triangleXfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}};
    
    const auto manifold = CollideShapes(triangle.GetChild(0), triangleXfm, circle.GetChild(0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(-0.5, 0) * Meter);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetX()), -0.894427,   0.0001);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetY()),  0.44721359, 0.0001);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(triangle.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
    
    EXPECT_EQ(triangle.GetVertex(0), Vec2(+1, -1) * Meter);
}

TEST(CollideShapes, TallRectangleLeftCircleRight)
{
    const auto r2 = RealNum(1) * Meter;
    const auto hx = RealNum(2.2);
    const auto hy = RealNum(4.8);

    const auto s1 = PolygonShape(hx * Meter, hy * Meter);
    ASSERT_EQ(s1.GetVertex(0), Vec2(+hx, -hy) * Meter); // bottom right
    ASSERT_EQ(s1.GetVertex(1), Vec2(+hx, +hy) * Meter); // top right
    ASSERT_EQ(s1.GetVertex(2), Vec2(-hx, +hy) * Meter); // top left
    ASSERT_EQ(s1.GetVertex(3), Vec2(-hx, -hy) * Meter); // bottom left

    const auto s2 = CircleShape{r2};
    
    const auto p1 = Vec2{-1, 0} * Meter;
    const auto p2 = Vec2{3, 0} * Meter;
    const auto t1 = Transformation{p1, UnitVec2{RealNum{45.0f} * Degree}};
    const auto t2 = Transformation{p2, UnitVec2{RealNum{0.0f} * Degree}};
    
    // rotate rectangle 45 degrees and put it on the left of the circle
    const auto manifold = CollideShapes(s1.GetChild(0), t1, s2.GetChild(0), t2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(hx, 0) * Meter);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(s1.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, IdenticalOverlappingSquaresDim1)
{
    const auto dim = RealNum(1);
    const auto shape = PolygonShape(dim * Meter, dim * Meter);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+dim, -dim) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+dim, +dim) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-dim, +dim) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-dim, -dim) * Meter); // bottom left
    
    const auto xfm = Transformation(Vec2_zero * Meter, UnitVec2{RealNum{0.0f} * Degree});
    const auto manifold = CollideShapes(shape.GetChild(0), xfm, shape.GetChild(0), xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(+1, 0));
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+dim, 0) * Meter);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-dim, +dim) * Meter); // top left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, RealNum(0) * Kilogram * MeterPerSecond);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, RealNum(0)* Kilogram * MeterPerSecond);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-dim, -dim) * Meter); // bottom left
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, RealNum(0)* Kilogram * MeterPerSecond);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, RealNum(0)* Kilogram * MeterPerSecond);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, IdenticalOverlappingSquaresDim2)
{
    const auto dim = RealNum(2);
    const auto shape = PolygonShape(dim * Meter, dim * Meter);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+dim, -dim) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+dim, +dim) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-dim, +dim) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-dim, -dim) * Meter); // bottom left
    
    const auto xfm = Transformation(Vec2_zero * Meter, UnitVec2{RealNum{0.0f} * Degree});
    const auto manifold = CollideShapes(shape.GetChild(0), xfm, shape.GetChild(0), xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(+1, 0));
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+dim, 0) * Meter);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-dim, +dim) * Meter); // top left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-dim, -dim) * Meter); // bottom left
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, IdenticalVerticalTouchingSquares)
{
    const auto dim = RealNum(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left

    const auto xfm0 = Transformation(Vec2{0, -1} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // bottom
    const auto xfm1 = Transformation(Vec2{0, +1} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // top
    const auto manifold = CollideShapes(shape.GetChild(0), xfm0, shape.GetChild(0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0,+2) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0,+1));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, -2) * Meter); // bottom left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, -2) * Meter); // bottom right
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(CollideShapes, IdenticalHorizontalTouchingSquares)
{
    const auto dim = RealNum(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left

    const auto xfm0 = Transformation(Vec2{-2, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // left
    const auto xfm1 = Transformation(Vec2{+2, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // right
    const auto manifold = CollideShapes(shape.GetChild(0), xfm0, shape.GetChild(0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+2, 0) * Meter);    
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(+1, 0));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, +2) * Meter); // top left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, -2) * Meter); // bottom left
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, SquareCornerTouchingSquareFaceAbove)
{
    const auto dim = RealNum(2) * Meter;

    // creates a square
    const auto shape = PolygonShape(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left
    
    const auto rot0 = RealNum{45.0f} * Degree;
    const auto rot1 = RealNum{0.0f} * Degree;
    const auto xfm0 = Transformation(Vec2{0, -2} * Meter, UnitVec2{rot0}); // bottom
    const auto xfm1 = Transformation(Vec2{0, +2} * Meter, UnitVec2{rot1}); // top
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +

    const auto manifold = CollideShapes(shape.GetChild(0), xfm0, shape.GetChild(0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0, -1));
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -2) * Meter);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    
    // localPoint is almost equal to Vec2(2, 2) but it's not exactly equal.
    EXPECT_NEAR(double(RealNum{manifold.GetPoint(0).localPoint.x / Meter}), 2.0, 0.0004); // top right shape A
    EXPECT_NEAR(double(RealNum{manifold.GetPoint(0).localPoint.y / Meter}), 2.0, 0.0004); // top right shape A
    
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1); // Shape A top right vertex
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3); // Shape B bottom edge
    
    // Also check things in terms of world coordinates...
    const auto world_manifold = GetWorldManifold(manifold, xfm0, RealNum(0) * Meter, xfm1, RealNum(0) * Meter);
    EXPECT_EQ(world_manifold.GetPointCount(), manifold.GetPointCount());
    
    EXPECT_EQ(GetVec2(world_manifold.GetNormal()), Vec2(0, +1));
    
    const auto corner_point = Rotate(Length2D{dim, dim}, UnitVec2{rot0}) + xfm0.p;
    EXPECT_NEAR(double(RealNum{corner_point.x / Meter}), 0.0,        0.0002);
    EXPECT_NEAR(double(RealNum{corner_point.y / Meter}), 0.82842684, 0.0002);
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_NEAR(double(RealNum{world_manifold.GetPoint(0).x / Meter}),
                double(RealNum{corner_point.x / (RealNum{2} * Meter)}), 0.0004);
    EXPECT_NEAR(double(RealNum{world_manifold.GetPoint(0).y / Meter}),
                double(RealNum{corner_point.y / (RealNum{2} * Meter)}), 0.0004);
    EXPECT_NEAR(double(RealNum{world_manifold.GetSeparation(0) / Meter}),
                double(RealNum{-corner_point.y / Meter}), 0.0005);
}

TEST(CollideShapes, HorizontalOverlappingRects1)
{
    // Shape A: square
    const auto shape0 = PolygonShape(RealNum{2} * Meter, RealNum{2} * Meter);
    ASSERT_EQ(shape0.GetVertex(0), Vec2(+2,-2) * Meter); // bottom right
    ASSERT_EQ(shape0.GetVertex(1), Vec2(+2,+2) * Meter); // top right
    ASSERT_EQ(shape0.GetVertex(2), Vec2(-2,+2) * Meter); // top left
    ASSERT_EQ(shape0.GetVertex(3), Vec2(-2,-2) * Meter); // bottom left
    
    // Shape B: wide rectangle
    const auto shape1 = PolygonShape(RealNum{3} * Meter, RealNum{1.5f} * Meter);
    ASSERT_EQ(shape1.GetVertex(0), Vec2(RealNum(+3.0), RealNum(-1.5)) * Meter); // bottom right
    ASSERT_EQ(shape1.GetVertex(1), Vec2(RealNum(+3.0), RealNum(+1.5)) * Meter); // top right
    ASSERT_EQ(shape1.GetVertex(2), Vec2(RealNum(-3.0), RealNum(+1.5)) * Meter); // top left
    ASSERT_EQ(shape1.GetVertex(3), Vec2(RealNum(-3.0), RealNum(-1.5)) * Meter); // bottom left

    const auto xfm0 = Transformation(Vec2{-2, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // left
    const auto xfm1 = Transformation(Vec2{+2, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // right
    
    // Put square left, wide rectangle right.
    // In ASCII art terms:
    //
    //   +-------2
    //   |     +-+---------+
    //   |   A | 1   B     |
    //   |     | |         |
    //   4-3-2-1-*-1-2-3-4-5
    //   |     | |         |
    //   |     | 1         |
    //   |     +-+---------+
    //   +-------2
    //

    const auto manifold = CollideShapes(shape0.GetChild(0), xfm0, shape1.GetChild(0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(RealNum(+2), RealNum(0)) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(RealNum(+1), RealNum(0)));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(RealNum(-3.0), RealNum(+1.5)) * Meter); // top left shape B
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(RealNum(-3.0), RealNum(-1.5)) * Meter); // bottom left shape B
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
    
    const auto world_manifold = GetWorldManifold(manifold,
                                                 xfm0, GetVertexRadius(shape0),
                                                 xfm1, GetVertexRadius(shape1));
    EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
    
    EXPECT_TRUE(almost_equal(world_manifold.GetNormal().GetX(), RealNum(1)));
    EXPECT_TRUE(almost_equal(world_manifold.GetNormal().GetY(), RealNum(0)));
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(almost_equal(StripUnit(world_manifold.GetPoint(0).x), RealNum(-0.5)));
    EXPECT_TRUE(almost_equal(StripUnit(world_manifold.GetPoint(0).y), RealNum(+1.5)));

    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(almost_equal(StripUnit(world_manifold.GetPoint(1).x), RealNum(-0.5)));
    EXPECT_TRUE(almost_equal(StripUnit(world_manifold.GetPoint(1).y), RealNum(-1.5)));
}

TEST(CollideShapes, HorizontalOverlappingRects2)
{
    // Shape A: wide rectangle
    const auto shape0 = PolygonShape(RealNum{3} * Meter, RealNum{1.5f} * Meter);
    ASSERT_EQ(shape0.GetVertex(0), Vec2(RealNum(+3.0), RealNum(-1.5)) * Meter); // bottom right
    ASSERT_EQ(shape0.GetVertex(1), Vec2(RealNum(+3.0), RealNum(+1.5)) * Meter); // top right
    ASSERT_EQ(shape0.GetVertex(2), Vec2(RealNum(-3.0), RealNum(+1.5)) * Meter); // top left
    ASSERT_EQ(shape0.GetVertex(3), Vec2(RealNum(-3.0), RealNum(-1.5)) * Meter); // bottom left
    
    // Shape B: square
    const auto shape1 = PolygonShape(RealNum{2} * Meter, RealNum{2} * Meter);
    ASSERT_EQ(shape1.GetVertex(0), Vec2(+2,-2) * Meter); // bottom right
    ASSERT_EQ(shape1.GetVertex(1), Vec2(+2,+2) * Meter); // top right
    ASSERT_EQ(shape1.GetVertex(2), Vec2(-2,+2) * Meter); // top left
    ASSERT_EQ(shape1.GetVertex(3), Vec2(-2,-2) * Meter); // bottom left
    
    const auto xfm0 = Transformation(Vec2{-2, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // left
    const auto xfm1 = Transformation(Vec2{+2, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}); // right

    // put wide rectangle on left, square on right
    const auto manifold = CollideShapes(shape0.GetChild(0), xfm0, shape1.GetChild(0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(RealNum(+3), RealNum(0)) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(RealNum(+1), RealNum(0)));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(almost_equal(manifold.GetPoint(0).localPoint.x / Meter, RealNum(-2.0))); // left
    EXPECT_TRUE(almost_equal(manifold.GetPoint(0).localPoint.y / Meter, RealNum(-1.5))); // top
    EXPECT_TRUE(almost_equal(manifold.GetPoint(0).normalImpulse / (Kilogram * MeterPerSecond), RealNum(0)));
    EXPECT_TRUE(almost_equal(manifold.GetPoint(0).tangentImpulse / (Kilogram * MeterPerSecond), RealNum(0)));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(almost_equal(manifold.GetPoint(1).localPoint.x / Meter, RealNum(-2.0))); // left
    EXPECT_TRUE(almost_equal(manifold.GetPoint(1).localPoint.y / Meter, RealNum(+1.5))); // bottom
    EXPECT_TRUE(almost_equal(manifold.GetPoint(1).normalImpulse / (Kilogram * MeterPerSecond), RealNum(0)));
    EXPECT_TRUE(almost_equal(manifold.GetPoint(1).tangentImpulse / (Kilogram * MeterPerSecond), RealNum(0)));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
    
    const auto world_manifold = GetWorldManifold(manifold,
                                                 xfm0, GetVertexRadius(shape0),
                                                 xfm1, GetVertexRadius(shape1));
    EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
    
    EXPECT_TRUE(almost_equal(world_manifold.GetNormal().GetX(), RealNum(1)));
    EXPECT_TRUE(almost_equal(world_manifold.GetNormal().GetY(), RealNum(0)));
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(almost_equal(world_manifold.GetPoint(0).x / Meter, RealNum(+0.5)));
    EXPECT_TRUE(almost_equal(world_manifold.GetPoint(0).y / Meter, RealNum(-1.5)));
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(almost_equal(world_manifold.GetPoint(1).x / Meter, RealNum(+0.5)));
    EXPECT_TRUE(almost_equal(world_manifold.GetPoint(1).y / Meter, RealNum(+1.5)));
}

TEST(CollideShapes, EdgeBelowPolygon)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, -1} * Meter, UnitVec2{RealNum{0.0f} * Degree}};

    const auto hx = RealNum(1) * Meter;
    const auto hy = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(hx, hy);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0.0f} * Degree}};

    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0, 1));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-1, -1) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(+1, -1) * Meter);
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(CollideShapes, EdgeAbovePolygon)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, +1} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto hx = RealNum(1) * Meter;
    const auto hy = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(hx, hy);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0, -1));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(+1, +1) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 1);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-1, +1) * Meter);
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, Momentum(0));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
}

TEST(CollideShapes, EdgeLeftOfPolygon)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{-1, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto hx = RealNum(1) * Meter;
    const auto hy = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(hx, hy);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(+1, 0));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
}

TEST(CollideShapes, EdgeRightOfPolygon)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{+1, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto hx = RealNum(1) * Meter;
    const auto hy = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(hx, hy);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(-1, 0));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
}

TEST(CollideShapes, EdgeInsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 3));
}

TEST(CollideShapes, EdgeTwiceInsideSquare)
{
    const auto p1 = Vec2(0, -2) * Meter;
    const auto p2 = Vec2(0, +2) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 3));
}

TEST(CollideShapes, EdgeHalfInsideSquare)
{
    const auto p1 = Vec2(0, -0.5) * Meter;
    const auto p2 = Vec2(0, +0.5) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(0, 2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
}

TEST(CollideShapes, EdgeR90InsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2::GetTop()};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2::GetRight()};
    
    // Sets up a collision between a line segment (A) and a square (B) where the line segment is
    // fully inside the square and bisects the square into an upper and low rectangular area like
    // the following (where the numbers represent coordinates on the X or Y axis)...
    //
    //     2
    //
    //   +-1-+
    //   |   |
    // 2 1-0-1 2
    //   |   |
    //   +-1-+
    //
    //     2
    //
    // Note that the square's vertex 0 is at its bottom right corner, and they go
    // counter-clockwise from there.
    //
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    if (sizeof(RealNum) == 4)
    {
        EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
    }
    else if (sizeof(RealNum) == 8)
    {
        EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
    }
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    if (sizeof(RealNum) == 4)
    {
        EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 0));
    }
    else if (sizeof(RealNum) == 8)
    {
        EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 0));
    }
}

TEST(CollideShapes, EdgeR45InsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{45.0f} * Degree}};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    if (sizeof(RealNum) == 4)
    {
        // On some platforms, the contact feature is: <01-00 01-02>.
        // On others, it's: <01-00 01-03>
        // XXX Not sure of why the difference.
        // EXPECT_EQ(cf, GetFaceFaceContactFeature(0, 2));
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(0, 3));
        // EXPECT_TRUE(cf == GetFaceFaceContactFeature(0, 2) || cf == GetFaceFaceContactFeature(0, 3));
    }
    else if (sizeof(RealNum) == 8)
    {
#if defined(__core2__)
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(0, 3));
#elif defined(__k8__)
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
#else
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(0, 3));
#endif
    }
}

TEST(CollideShapes, EdgeR180InsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{180.0f} * Degree}};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    switch (sizeof(RealNum))
    {
        case 4: EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1)); break;
        case 8:
#if defined(__core2__)
         	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#else
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
#endif
            break;
    }
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    switch (sizeof(RealNum))
    {
        case 4: EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 0)); break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 0));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(0, 0));
#else
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 0));
#endif
            break;
    }
    
}

TEST(CollideShapes, EdgeTwiceR180Square)
{
    const auto p1 = Vec2(0, -2) * Meter;
    const auto p2 = Vec2(0, +2) * Meter;
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, 1} * Meter, UnitVec2{RealNum{180.0f} * Degree}};
    const auto s = RealNum(1) * Meter;
    const auto polygon_shape = PolygonShape(s, s);
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    switch (sizeof(RealNum))
    {
        case 4: EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0)); break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
#elif defined(__k8__)
            EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(-1, 0));
#else
            EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(1, 0));
#endif
            break;
    }
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    switch (sizeof(RealNum))
    {
        case 4:
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
            break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(1, 2));
#else
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
#endif
            break;
    }
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    switch (sizeof(RealNum))
    {
        case 4:
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 0));
            break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 0));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
#else
            EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 0));
#endif
            break;
    }
}

TEST(CollideShapes, EdgeFooTriangle)
{
    const auto p1 = Vec2(2, -2) * Meter;
    const auto p2 = Vec2(-2, +2) * Meter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = 0;
    auto edge_shape = EdgeShape(conf);
    edge_shape.Set(p2, p1);
    const auto edge_xfm = Transformation{Vec2(0, 0.5) * Meter, UnitVec2{-RealNum{5.0f} * Degree}};
    auto polygon_shape = PolygonShape{};
    polygon_shape.SetVertexRadius(0);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    polygon_shape.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto polygon_xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, polygon_shape.GetChild(0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0) * Meter);
    EXPECT_NEAR(double(GetX(GetVec2(manifold.GetLocalNormal()))), -0.707107, 0.0001);
    EXPECT_NEAR(double(GetY(GetVec2(manifold.GetLocalNormal()))), -0.707107, 0.0001);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
}

TEST(CollideShapes, EdgePolygonFaceB1)
{
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = 0;
    const auto edge_shape = EdgeShape(Vec2(6, 8) * Meter, Vec2(7, 8) * Meter, conf);
    const auto edge_xfm = Transformation(Vec2(0, 0) * Meter, GetUnitVector(Vec2(RealNum(0.707106769), RealNum(0.707106769))));
    const auto poly_shape = PolygonShape({
        Vec2(0.5, 0) * Meter,
        Vec2(0.249999985f, 0.433012724f) * Meter,
        Vec2(-0.25000003f, 0.433012694f) * Meter,
        Vec2(-0.5f, -0.0000000437113883f) * Meter,
        Vec2(-0.249999955f, -0.433012724f) * Meter,
        Vec2(0.249999955f, -0.433012724f) * Meter
    });
    const auto poly_xfm = Transformation(Vec2(-0.797443091f, 11.0397148f) * Meter, GetUnitVector(Vec2(1, 0)));
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, poly_shape.GetChild(0), poly_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    EXPECT_NEAR(double(StripUnit(GetX(manifold.GetLocalPoint()))), 0.0, 0.0001);
    EXPECT_NEAR(double(StripUnit(GetY(manifold.GetLocalPoint()))), -0.43301272, 0.0001);
    EXPECT_TRUE(almost_equal(GetX(GetVec2(manifold.GetLocalNormal())), RealNum{0.0f}));
    EXPECT_TRUE(almost_equal(GetY(GetVec2(manifold.GetLocalNormal())), RealNum{-1.0f}));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(1, 4));
    EXPECT_NEAR(static_cast<double>(RealNum{GetX(manifold.GetOpposingPoint(0)) / Meter}),
                7.0, 0.00001);
    EXPECT_NEAR(static_cast<double>(RealNum{GetY(manifold.GetOpposingPoint(0)) / Meter}),
                8.0, 0.00001);
}

TEST(CollideShapes, EdgePolygonFaceB2)
{
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = RealNum{0.000199999995f} * Meter;
    const auto edge_shape = EdgeShape(Vec2(-6, 2) * Meter, Vec2(-6, 0) * Meter, conf);
    const auto edge_xfm = Transformation(Vec2(-9.99999904f, 4.0f) * Meter, GetUnitVector(Vec2(RealNum(1), RealNum(0))));
    const auto poly_shape = PolygonShape({
        Vec2(0.5f, -0.5f) * Meter,
        Vec2(0.5f, 0.5f) * Meter,
        Vec2(-0.5f, 0.5f) * Meter,
        Vec2(0.0f, 0.0f) * Meter
    });
    const auto poly_xfm = Transformation(Vec2(-16.0989342f, 3.49960017f) * Meter, GetUnitVector(Vec2(1, 0)));
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), edge_xfm, poly_shape.GetChild(0), poly_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    EXPECT_NEAR(double(StripUnit(GetX(manifold.GetLocalPoint()))), 0.0, 0.0001);
    EXPECT_TRUE(almost_equal(GetY(manifold.GetLocalPoint()) / Meter, RealNum{0.5f}));
    EXPECT_TRUE(almost_equal(GetX(GetVec2(manifold.GetLocalNormal())), RealNum{0.0f}));
    EXPECT_TRUE(almost_equal(GetY(GetVec2(manifold.GetLocalNormal())), RealNum{1.0f}));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(1, 1));
    EXPECT_TRUE(almost_equal(GetX(manifold.GetOpposingPoint(0)) / Meter, RealNum{-6.0f}));
    EXPECT_TRUE(almost_equal(GetY(manifold.GetOpposingPoint(0)) / Meter, RealNum{0.0f}));
}

#if 0
TEST(CollideShapes, EdgeOverlapsItself)
{
    const auto p1 = Vec2(0, -1);
    const auto p2 = Vec2(0, +1);
    const auto edge_shape = EdgeShape(p1, p2);
    const auto edge_xfm = Transformation{Vec2{+1, 0}, UnitVec2{RealNum{0} * Degree}};

    const auto manifold = CollideShapes(edge_shape, edge_xfm, edge_shape, edge_xfm);
    
    ASSERT_NE(manifold.GetType(), Manifold::e_unset);
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
}
#endif

TEST(CollideShapes, R0EdgeCollinearAndTouchingR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = 0;
    auto edge_shape = EdgeShape(conf);
    edge_shape.Set(p1, p2);
    const auto xfm1 = Transformation{Vec2{+1, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto xfm2 = Transformation{Vec2{+3, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), xfm1, edge_shape.GetChild(0), xfm2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_TRUE(IsValid(manifold.GetLocalNormal()));
    if (IsValid(manifold.GetLocalNormal()))
    {
        EXPECT_NEAR(static_cast<double>(StripUnit(GetX(manifold.GetLocalNormal()))),  0.0, 0.001);
        EXPECT_NEAR(static_cast<double>(StripUnit(GetY(manifold.GetLocalNormal()))), -1.0, 0.001);
    }
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(1, 0) * Meter);
}

TEST(CollideShapes, R1EdgeCollinearAndTouchingR1Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = RealNum{1} * Meter;
    auto edge_shape = EdgeShape(conf);
    edge_shape.Set(p1, p2);
    const auto xfm1 = Transformation{Vec2{+1, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto xfm2 = Transformation{Vec2{+5, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), xfm1, edge_shape.GetChild(0), xfm2);

    ASSERT_NE(manifold.GetType(), Manifold::e_unset);

    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_EQ(manifold.GetLocalPoint(), p2);
}

TEST(CollideShapes, R0EdgeCollinearAndSeparateFromR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = 0;
    auto edge_shape = EdgeShape(conf);
    edge_shape.Set(p1, p2);
    const auto xfm1 = Transformation{Vec2{+1, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto xfm2 = Transformation{Vec2{+4, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), xfm1, edge_shape.GetChild(0), xfm2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_FALSE(IsValid(manifold.GetLocalPoint()));
}

TEST(CollideShapes, R0EdgeParallelAndSeparateFromR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = 0;
    auto edge_shape = EdgeShape(conf);
    edge_shape.Set(p1, p2);
    const auto xfm1 = Transformation{Vec2{-4, 1} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto xfm2 = Transformation{Vec2{-4, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), xfm1, edge_shape.GetChild(0), xfm2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
    EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
    EXPECT_FALSE(IsValid(manifold.GetLocalPoint()));
}

TEST(CollideShapes, R0EdgePerpendicularCrossingFromR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = 0;
    auto edge_shape = EdgeShape(conf);
    edge_shape.Set(p1, p2);
    const auto xfm1 = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto xfm2 = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{90.0f} * Degree}};
    
    const auto manifold = CollideShapes(edge_shape.GetChild(0), xfm1, edge_shape.GetChild(0), xfm2);
    
    ASSERT_NE(manifold.GetType(), Manifold::e_unset);
    ASSERT_TRUE(IsValid(manifold.GetLocalNormal()));
    ASSERT_TRUE(IsValid(manifold.GetLocalPoint()));
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0, -1));
    EXPECT_NEAR(static_cast<double>(StripUnit(manifold.GetLocalPoint().x)), 0.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(StripUnit(manifold.GetLocalPoint().y)), 0.0, 0.0001);
    EXPECT_EQ(manifold.GetPointCount(), decltype(manifold.GetPointCount()){1});
}
