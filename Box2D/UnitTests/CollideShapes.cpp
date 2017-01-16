/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Collision/CollideShapes.hpp>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

using namespace box2d;

TEST(CollideShapes, IdenticalOverlappingCircles)
{
	const auto radius = float_t(1);
	const auto shape = CircleShape{radius};
	const auto position = Vec2{11, -4};
	const auto xfm = Transformation{position, UnitVec2{0_rad}};
	
	// put shape 1 to left of shape 2
	const auto manifold = CollideShapes(shape, xfm, shape, xfm);
	
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
	const auto r1 = float_t(1);
	const auto r2 = float_t(1);
	const auto s1 = CircleShape{r1};
	const auto s2 = CircleShape{r2};
	const auto p1 = Vec2{11, -4};
	const auto p2 = Vec2{13, -4};
	const auto t1 = Transformation{p1, UnitVec2{0_rad}};
	const auto t2 = Transformation{p2, UnitVec2{0_rad}};
	
	// put shape 1 to left of shape 2
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
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
	const auto r1 = float_t(1);
	const auto r2 = float_t(1);
	const auto s1 = CircleShape{r1};
	const auto s2 = CircleShape{r2};
	const auto p1 = Vec2{7, -2};
	const auto p2 = Vec2{7, -1};
	
	// Rotations don't matter so long as circle shapes' centers are at (0, 0).
	const auto t1 = Transformation{p1, UnitVec2{45_deg}};
	const auto t2 = Transformation{p2, UnitVec2{-21_deg}};
	
	// put shape 1 to left of shape 2
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTouchingTrianglePointBelow)
{
	const auto circleRadius = float_t(1);
	const auto circle = CircleShape(circleRadius);
	const auto triangleTopPt = Vec2{0, +1};
	const auto triangleLeftPt = Vec2{-1, -1};
	const auto triangleRightPt = Vec2{+1, -1};
	const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
	const auto triangleXfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	const auto circleXfm = Transformation{triangleTopPt + UnitVec2{90_deg} * circleRadius, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(triangle, triangleXfm, circle, circleXfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	EXPECT_EQ(manifold.GetLocalPoint(), triangleTopPt);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleTopPt);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTouchingTrianglePointLeft)
{
	const auto circleRadius = float_t(1);
	const auto circle = CircleShape(circleRadius);
	const auto triangleTopPt = Vec2{0, +1};
	const auto triangleLeftPt = Vec2{-1, -1};
	const auto triangleRightPt = Vec2{+1, -1};
	const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
	const auto circleXfm = Transformation{triangleLeftPt + UnitVec2{225_deg} * circleRadius, UnitVec2{0_deg}};
	const auto triangleXfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(triangle, triangleXfm, circle, circleXfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	EXPECT_EQ(manifold.GetLocalPoint(), triangleLeftPt);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleLeftPt);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTouchingTrianglePointRight)
{
	const auto circleRadius = float_t(1);
	const auto circle = CircleShape(circleRadius);
	const auto triangleTopPt = Vec2{0, +1};
	const auto triangleLeftPt = Vec2{-1, -1};
	const auto triangleRightPt = Vec2{+1, -1};
	const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
	const auto circleXfm = Transformation{triangleRightPt + UnitVec2{-45_deg} * circleRadius, UnitVec2{0_deg}};
	const auto triangleXfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(triangle, triangleXfm, circle, circleXfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	EXPECT_EQ(manifold.GetLocalPoint(), triangleRightPt);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleRightPt);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleJustPastTrianglePointRightDoesntCollide)
{
	const auto circleRadius = float_t(1);
	const auto circle = CircleShape(circleRadius);
	const auto triangleTopPt = Vec2{0, +1};
	const auto triangleLeftPt = Vec2{-1, -1};
	const auto triangleRightPt = Vec2{+1, -1};
	auto triangle = PolygonShape{0.0001f * 2};
	triangle.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
	const auto circleXfm = Transformation{triangleRightPt + UnitVec2{-45_deg} * circleRadius * float_t(1.001), UnitVec2{0_deg}};
	const auto triangleXfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(triangle, triangleXfm, circle, circleXfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
	EXPECT_FALSE(IsValid(manifold.GetLocalPoint()));
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(0));
}

TEST(CollideShapes, CircleOverRightFaceOfTriangle)
{
	const auto circleRadius = float_t(1);
	const auto circle = CircleShape(circleRadius);
	const auto triangleTopPt = Vec2{0, +1};
	const auto triangleLeftPt = Vec2{-1, -1};
	const auto triangleRightPt = Vec2{+1, -1};
	const auto triangle = PolygonShape({triangleLeftPt, triangleRightPt, triangleTopPt});
	const auto circleXfm = Transformation{Vec2{1, 1}, UnitVec2{0_deg}};
	const auto triangleXfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(triangle, triangleXfm, circle, circleXfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), (triangleTopPt + triangleRightPt) / 2);
	EXPECT_FLOAT_EQ(manifold.GetLocalNormal().GetX(), float_t(0.894427));
	EXPECT_FLOAT_EQ(manifold.GetLocalNormal().GetY(), float_t(0.44721359));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, circle.GetLocation());
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(triangle.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);

	EXPECT_EQ(triangle.GetVertex(0), Vec2(+1, -1));
}

TEST(CollideShapes, CircleOverLeftFaceOfTriangle)
{
	const auto circleRadius = float_t(1);
	const auto circle = CircleShape(circleRadius);
	const auto triangle = PolygonShape({Vec2{-1, -1}, Vec2{+1, -1}, Vec2{0, +1}});
	const auto circleXfm = Transformation{Vec2{-1, 1}, UnitVec2{0_deg}};
	const auto triangleXfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(triangle, triangleXfm, circle, circleXfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(-0.5, 0));
	EXPECT_FLOAT_EQ(manifold.GetLocalNormal().GetX(), float_t(-0.894427));
	EXPECT_FLOAT_EQ(manifold.GetLocalNormal().GetY(), float_t(0.44721359));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(triangle.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
	
	EXPECT_EQ(triangle.GetVertex(0), Vec2(+1, -1));
}

TEST(CollideShapes, TallRectangleLeftCircleRight)
{
	const auto r2 = float_t(1);
	const auto hx = float_t(2.2);
	const auto hy = float_t(4.8);

	const auto s1 = PolygonShape(hx, hy);
	ASSERT_EQ(s1.GetVertex(0), Vec2(+hx, -hy)); // bottom right
	ASSERT_EQ(s1.GetVertex(1), Vec2(+hx, +hy)); // top right
	ASSERT_EQ(s1.GetVertex(2), Vec2(-hx, +hy)); // top left
	ASSERT_EQ(s1.GetVertex(3), Vec2(-hx, -hy)); // bottom left

	const auto s2 = CircleShape{r2};
	
	const auto p1 = Vec2{-1, 0};
	const auto p2 = Vec2{3, 0};
	const auto t1 = Transformation{p1, UnitVec2{45_deg}};
	const auto t2 = Transformation{p2, UnitVec2{0_deg}};
	
	// rotate rectangle 45 degrees and put it on the left of the circle
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(hx, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(0, 0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(s1.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, IdenticalOverlappingSquaresDim1)
{
	const auto dim = float_t(1);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0), Vec2(+dim, -dim)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(+dim, +dim)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-dim, +dim)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-dim, -dim)); // bottom left
	
	const auto xfm = Transformation(Vec2_zero, UnitVec2{0_deg});
	const auto manifold = CollideShapes(shape, xfm, shape, xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(+1, 0));
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+dim, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-dim, +dim)); // top left
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-dim, -dim)); // bottom left
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, IdenticalOverlappingSquaresDim2)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0), Vec2(+dim, -dim)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(+dim, +dim)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-dim, +dim)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-dim, -dim)); // bottom left
	
	const auto xfm = Transformation(Vec2_zero, UnitVec2{0_deg});
	const auto manifold = CollideShapes(shape, xfm, shape, xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(+1, 0));
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+dim, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-dim, +dim)); // top left
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-dim, -dim)); // bottom left
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, IdenticalVerticalTouchingSquares)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2)); // bottom left

	const auto xfm0 = Transformation(Vec2{0, -1}, UnitVec2{0_deg}); // bottom
	const auto xfm1 = Transformation(Vec2{0, +1}, UnitVec2{0_deg}); // top
	const auto manifold = CollideShapes(shape, xfm0, shape, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0,+2));	
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(0,+1));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, -2)); // bottom left
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, -2)); // bottom right
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(CollideShapes, IdenticalHorizontalTouchingSquares)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2)); // bottom left

	const auto xfm0 = Transformation(Vec2{-2, 0}, UnitVec2{0_deg}); // left
	const auto xfm1 = Transformation(Vec2{+2, 0}, UnitVec2{0_deg}); // right
	const auto manifold = CollideShapes(shape, xfm0, shape, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+2, 0));	
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(+1, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, +2)); // top left
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, -2)); // bottom left
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, SquareCornerTouchingSquareFaceAbove)
{
	const auto dim = float_t(2);

	// creates a square
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2)); // bottom left
	
	const auto rot0 = 45_deg;
	const auto rot1 = 0_deg;
	const auto xfm0 = Transformation(Vec2{0, -2}, UnitVec2{rot0}); // bottom
	const auto xfm1 = Transformation(Vec2{0, +2}, UnitVec2{rot1}); // top
	
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

	const auto manifold = CollideShapes(shape, xfm0, shape, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
	
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(0, -1));
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -2));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	
	// localPoint is almost equal to Vec2(2, 2) but it's not exactly equal.
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.x, float_t(+2)); // top right shape A
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.y, float_t(+2)); // top right shape A
	
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1); // Shape A top right vertex
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3); // Shape B bottom edge
	
	// Also check things in terms of world coordinates...
	const auto world_manifold = GetWorldManifold(manifold, xfm0, float_t(0), xfm1, float_t(0));
	EXPECT_EQ(world_manifold.GetPointCount(), manifold.GetPointCount());
	
	EXPECT_EQ(Vec2{world_manifold.GetNormal()}, Vec2(0, +1));
	
	const auto corner_point = Rotate(Vec2{dim, dim}, UnitVec2{rot0}) + xfm0.p;
	EXPECT_FLOAT_EQ(corner_point.x, float_t(0));
	EXPECT_FLOAT_EQ(corner_point.y, float_t(0.82842684));
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).x, corner_point.x / 2);
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).y, corner_point.y / 2);
	EXPECT_FLOAT_EQ(world_manifold.GetSeparation(0), -corner_point.y);
}

TEST(CollideShapes, HorizontalOverlappingRects1)
{
	// Shape A: square
	const auto shape0 = PolygonShape(2, 2);
	ASSERT_EQ(shape0.GetVertex(0), Vec2(+2,-2)); // bottom right
	ASSERT_EQ(shape0.GetVertex(1), Vec2(+2,+2)); // top right
	ASSERT_EQ(shape0.GetVertex(2), Vec2(-2,+2)); // top left
	ASSERT_EQ(shape0.GetVertex(3), Vec2(-2,-2)); // bottom left
	
	// Shape B: wide rectangle
	const auto shape1 = PolygonShape(3, 1.5);
	ASSERT_EQ(shape1.GetVertex(0), Vec2(float_t(+3.0), float_t(-1.5))); // bottom right
	ASSERT_EQ(shape1.GetVertex(1), Vec2(float_t(+3.0), float_t(+1.5))); // top right
	ASSERT_EQ(shape1.GetVertex(2), Vec2(float_t(-3.0), float_t(+1.5))); // top left
	ASSERT_EQ(shape1.GetVertex(3), Vec2(float_t(-3.0), float_t(-1.5))); // bottom left

	const auto xfm0 = Transformation(Vec2{-2, 0}, UnitVec2{0_deg}); // left
	const auto xfm1 = Transformation(Vec2{+2, 0}, UnitVec2{0_deg}); // right
	
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

	const auto manifold = CollideShapes(shape0, xfm0, shape1, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(float_t(+2), float_t(0)));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(float_t(+1), float_t(0)));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(float_t(-3.0), float_t(+1.5))); // top left shape B
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(float_t(-3.0), float_t(-1.5))); // bottom left shape B
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
	
	const auto world_manifold = GetWorldManifold(manifold,
												 xfm0, GetVertexRadius(shape0),
												 xfm1, GetVertexRadius(shape1));
	EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
	
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().GetX(), float_t(1));
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().GetY(), float_t(0));
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).x, float_t(-0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).y, float_t(+1.5));

	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).x, float_t(-0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).y, float_t(-1.5));
}

TEST(CollideShapes, HorizontalOverlappingRects2)
{
	// Shape A: wide rectangle
	const auto shape0 = PolygonShape(3, 1.5);
	ASSERT_EQ(shape0.GetVertex(0), Vec2(float_t(+3.0), float_t(-1.5))); // bottom right
	ASSERT_EQ(shape0.GetVertex(1), Vec2(float_t(+3.0), float_t(+1.5))); // top right
	ASSERT_EQ(shape0.GetVertex(2), Vec2(float_t(-3.0), float_t(+1.5))); // top left
	ASSERT_EQ(shape0.GetVertex(3), Vec2(float_t(-3.0), float_t(-1.5))); // bottom left
	
	// Shape B: square
	const auto shape1 = PolygonShape(2, 2);
	ASSERT_EQ(shape1.GetVertex(0), Vec2(+2,-2)); // bottom right
	ASSERT_EQ(shape1.GetVertex(1), Vec2(+2,+2)); // top right
	ASSERT_EQ(shape1.GetVertex(2), Vec2(-2,+2)); // top left
	ASSERT_EQ(shape1.GetVertex(3), Vec2(-2,-2)); // bottom left
	
	const auto xfm0 = Transformation(Vec2{-2, 0}, UnitVec2{0_deg}); // left
	const auto xfm1 = Transformation(Vec2{+2, 0}, UnitVec2{0_deg}); // right

	// put wide rectangle on left, square on right
	const auto manifold = CollideShapes(shape0, xfm0, shape1, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(float_t(+3), float_t(0)));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(float_t(+1), float_t(0)));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.x, float_t(-2.0)); // left
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.y, float_t(-1.5)); // top
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).localPoint.x, float_t(-2.0)); // left
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).localPoint.y, float_t(+1.5)); // bottom
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
	
	const auto world_manifold = GetWorldManifold(manifold,
												 xfm0, GetVertexRadius(shape0),
												 xfm1, GetVertexRadius(shape1));
	EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
	
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().GetX(), float_t(1));
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().GetY(), float_t(0));
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).x, float_t(+0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).y, float_t(-1.5));
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).x, float_t(+0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).y, float_t(+1.5));
}

TEST(CollideShapes, EdgeWithDefaultPolygon)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, -1}, UnitVec2{0_deg}};
	
	const auto polygon_shape = PolygonShape{}; // vertex count is 0!
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
	EXPECT_EQ(manifold.GetPointCount(), 0);
}

TEST(CollideShapes, EdgeBelowPolygon)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, -1}, UnitVec2{0_deg}};

	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto polygon_shape = PolygonShape(hx, hy);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};

	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(1, 0));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(0, 1));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-1, -1));
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(+1, -1));
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(CollideShapes, EdgeAbovePolygon)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, +1}, UnitVec2{0_deg}};
	
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto polygon_shape = PolygonShape(hx, hy);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(-1, 0));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(0, -1));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(+1, +1));
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 1);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-1, +1));
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
}

TEST(CollideShapes, EdgeLeftOfPolygon)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{-1, 0}, UnitVec2{0_deg}};
	
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto polygon_shape = PolygonShape(hx, hy);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -1));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(+1, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
}

TEST(CollideShapes, EdgeRightOfPolygon)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{+1, 0}, UnitVec2{0_deg}};
	
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto polygon_shape = PolygonShape(hx, hy);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, +1));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(-1, 0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
}

TEST(CollideShapes, EdgeInsideSquare)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -1));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 3));
}

TEST(CollideShapes, EdgeTwiceInsideSquare)
{
	const auto p1 = Vec2(0, -2);
	const auto p2 = Vec2(0, +2);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -2));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 3));
}

TEST(CollideShapes, EdgeHalfInsideSquare)
{
	const auto p1 = Vec2(0, -0.5);
	const auto p2 = Vec2(0, +0.5);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -0.5));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceFaceContactFeature(0, 2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceFaceContactFeature(0, 2));
}

TEST(CollideShapes, EdgeR90InsideSquare)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 0}, UnitVec2{90_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -1));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 0));
}

TEST(CollideShapes, EdgeR45InsideSquare)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 0}, UnitVec2{45_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -1));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceFaceContactFeature(0, 2));
}

TEST(CollideShapes, EdgeR180InsideSquare)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 0}, UnitVec2{180_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -1));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceFaceContactFeature(0, 0));
}

TEST(CollideShapes, EdgeTwiceR180Square)
{
	const auto p1 = Vec2(0, -2);
	const auto p2 = Vec2(0, +2);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{0, 1}, UnitVec2{180_deg}};
	const auto s = float_t(1);
	const auto polygon_shape = PolygonShape(s, s);
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -2));
	EXPECT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(1, 0));
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
}

TEST(CollideShapes, EdgeFooTriangle)
{
	const auto p1 = Vec2(2, -2);
	const auto p2 = Vec2(-2, +2);
	auto edge_shape = EdgeShape(0);
	edge_shape.Set(p2, p1);
	const auto edge_xfm = Transformation{Vec2(0, 0.5), UnitVec2{-5_deg}};
	auto polygon_shape = PolygonShape(0);
	const auto triangleTopPt = Vec2{0, +1};
	const auto triangleLeftPt = Vec2{-1, -1};
	const auto triangleRightPt = Vec2{+1, -1};
	polygon_shape.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
	const auto polygon_xfm = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, polygon_shape, polygon_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(-2, 2));
	EXPECT_FLOAT_EQ(GetX(Vec2{manifold.GetLocalNormal()}), -0.707107f);
	EXPECT_FLOAT_EQ(GetY(Vec2{manifold.GetLocalNormal()}), -0.707107f);
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
}

TEST(CollideShapes, EdgePolygonFaceB1)
{
	const auto edge_shape = EdgeShape(Vec2(6, 8), Vec2(7, 8), Vec2(5, 7), Vec2(8, 7), 0);
	const auto edge_xfm = Transformation(Vec2(0, 0), GetUnitVector(Vec2(float_t(0.707106769), float_t(0.707106769))));
	const auto poly_shape = PolygonShape({
		Vec2(0.5, 0),
		Vec2(0.249999985f, 0.433012724f),
		Vec2(-0.25000003f, 0.433012694f),
		Vec2(-0.5f, -0.0000000437113883f),
		Vec2(-0.249999955f, -0.433012724f),
		Vec2(0.249999955f, -0.433012724f)
	});
	const auto poly_xfm = Transformation(Vec2(-0.797443091f, 11.0397148f), GetUnitVector(Vec2(1, 0)));
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, poly_shape, poly_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
	EXPECT_FLOAT_EQ(GetX(manifold.GetLocalPoint()), -0.249999955f);
	EXPECT_FLOAT_EQ(GetY(manifold.GetLocalPoint()), -0.43301272f);
	EXPECT_FLOAT_EQ(GetX(Vec2{manifold.GetLocalNormal()}), 0.0f);
	EXPECT_FLOAT_EQ(GetY(Vec2{manifold.GetLocalNormal()}), -1.0f);
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(1, 4));
	EXPECT_FLOAT_EQ(GetX(manifold.GetOpposingPoint(0)), 7.0f);
	EXPECT_FLOAT_EQ(GetY(manifold.GetOpposingPoint(0)), 8.0f);
}

TEST(CollideShapes, EdgePolygonFaceB2)
{
	const auto edge_shape = EdgeShape(Vec2(-6, 2), Vec2(-6, 0), Vec2(-4, 3), Vec2(0, 0), 0.000199999995f);
	const auto edge_xfm = Transformation(Vec2(-9.99999904f, 4.0f), GetUnitVector(Vec2(float_t(1), float_t(0))));
	const auto poly_shape = PolygonShape({
		Vec2(0.5f, -0.5f),
		Vec2(0.5f, 0.5f),
		Vec2(-0.5f, 0.5f),
		Vec2(0.0f, 0.0f)
	});
	const auto poly_xfm = Transformation(Vec2(-16.0989342f, 3.49960017f), GetUnitVector(Vec2(1, 0)));
	
	const auto manifold = CollideShapes(edge_shape, edge_xfm, poly_shape, poly_xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
	EXPECT_FLOAT_EQ(GetX(manifold.GetLocalPoint()), 0.5f);
	EXPECT_FLOAT_EQ(GetY(manifold.GetLocalPoint()), 0.5f);
	EXPECT_FLOAT_EQ(GetX(Vec2{manifold.GetLocalNormal()}), 0.0f);
	EXPECT_FLOAT_EQ(GetY(Vec2{manifold.GetLocalNormal()}), 1.0f);
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(1, 1));
	EXPECT_FLOAT_EQ(GetX(manifold.GetOpposingPoint(0)), -6.0f);
	EXPECT_FLOAT_EQ(GetY(manifold.GetOpposingPoint(0)), 0.0f);
}

TEST(CollideShapes, EdgeOverlapsItself)
{
	const auto p1 = Vec2(0, -1);
	const auto p2 = Vec2(0, +1);
	const auto edge_shape = EdgeShape(p1, p2);
	const auto edge_xfm = Transformation{Vec2{+1, 0}, UnitVec2{0_deg}};

	const auto manifold = CollideShapes(edge_shape, edge_xfm, edge_shape, edge_xfm);
	
	ASSERT_NE(manifold.GetType(), Manifold::e_unset);
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
}

TEST(CollideShapes, R0EdgeCollinearAndTouchingR0Edge)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	auto edge_shape = EdgeShape(0);
	edge_shape.Set(p1, p2);
	const auto xfm1 = Transformation{Vec2{+1, 0}, UnitVec2{0_deg}};
	const auto xfm2 = Transformation{Vec2{+3, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, xfm1, edge_shape, xfm2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetLocalPoint(), Vec2(1, 0));
}

TEST(CollideShapes, R1EdgeCollinearAndTouchingR1Edge)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	auto edge_shape = EdgeShape(1);
	edge_shape.Set(p1, p2);
	const auto xfm1 = Transformation{Vec2{+1, 0}, UnitVec2{0_deg}};
	const auto xfm2 = Transformation{Vec2{+5, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, xfm1, edge_shape, xfm2);

	ASSERT_NE(manifold.GetType(), Manifold::e_unset);

	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_EQ(manifold.GetLocalPoint(), p2);
}

TEST(CollideShapes, R0EdgeCollinearAndSeparateFromR0Edge)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	auto edge_shape = EdgeShape(0);
	edge_shape.Set(p1, p2);
	const auto xfm1 = Transformation{Vec2{+1, 0}, UnitVec2{0_deg}};
	const auto xfm2 = Transformation{Vec2{+4, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, xfm1, edge_shape, xfm2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_FALSE(IsValid(manifold.GetLocalPoint()));
}

TEST(CollideShapes, R0EdgeParallelAndSeparateFromR0Edge)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	auto edge_shape = EdgeShape(0);
	edge_shape.Set(p1, p2);
	const auto xfm1 = Transformation{Vec2{-4, 1}, UnitVec2{0_deg}};
	const auto xfm2 = Transformation{Vec2{-4, 0}, UnitVec2{0_deg}};
	
	const auto manifold = CollideShapes(edge_shape, xfm1, edge_shape, xfm2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
	EXPECT_FALSE(IsValid(manifold.GetLocalNormal()));
	EXPECT_FALSE(IsValid(manifold.GetLocalPoint()));
}

TEST(CollideShapes, R0EdgePerpendicularCrossingFromR0Edge)
{
	const auto p1 = Vec2(-1, 0);
	const auto p2 = Vec2(+1, 0);
	auto edge_shape = EdgeShape(0);
	edge_shape.Set(p1, p2);
	const auto xfm1 = Transformation{Vec2{0, 0}, UnitVec2{0_deg}};
	const auto xfm2 = Transformation{Vec2{0, 0}, UnitVec2{90_deg}};
	
	const auto manifold = CollideShapes(edge_shape, xfm1, edge_shape, xfm2);
	
	ASSERT_NE(manifold.GetType(), Manifold::e_unset);
	ASSERT_TRUE(IsValid(manifold.GetLocalNormal()));
	ASSERT_TRUE(IsValid(manifold.GetLocalPoint()));
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	EXPECT_EQ(Vec2(manifold.GetLocalNormal()), Vec2(0, 1));
	EXPECT_FLOAT_EQ(round(manifold.GetLocalPoint().x), 0);
	EXPECT_FLOAT_EQ(round(manifold.GetLocalPoint().y), 0);
	EXPECT_EQ(manifold.GetPointCount(), decltype(manifold.GetPointCount()){1});
}
