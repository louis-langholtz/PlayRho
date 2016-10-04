//
//  Collision.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/16/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

using namespace box2d;

TEST(Collision, CircleCircle)
{
	const auto r1 = float_t(1);
	const auto r2 = float_t(1);
	const auto s1 = CircleShape{r1};
	const auto s2 = CircleShape{r2};
	const auto p1 = Vec2{1, 0};
	const auto p2 = Vec2{3, 0};
	const auto t1 = Transformation{p1, Rot_identity};
	const auto t2 = Transformation{p2, Rot_identity};
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	
	//EXPECT_EQ(manifold.GetLocalNormal(), Vec2_zero);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, 0);
	EXPECT_EQ(manifold.GetLocalPoint().y, 0);
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));

	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, 0);
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
}

TEST(Collision, PolygonCircle)
{
	const auto r2 = float_t(1);
	const auto hx = float_t(2.2);
	const auto hy = float_t(4.8);
	const auto s1 = PolygonShape(hx, hy);
	const auto s2 = CircleShape{r2};
	const auto p1 = Vec2{-1, 0};
	const auto p2 = Vec2{3, 0};
	const auto t1 = Transformation{p1, Rot{DegreesToRadians(45)}};
	const auto t2 = Transformation{p2, Rot{0}};
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalNormal().x, 1);
	EXPECT_EQ(manifold.GetLocalNormal().y, 0);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, hx);
	EXPECT_EQ(manifold.GetLocalPoint().y, 0);
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, 0);
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
}

TEST(Collision, IdenticalOverlappingSquares)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(0).y, float_t(-2)); // bottom
	ASSERT_EQ(shape.GetVertex(1).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(1).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(2).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(2).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(3).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(3).y, float_t(-2)); // bottom

	const auto xfm = Transformation(Vec2_zero, Rot{0});
	const auto manifold = CollideShapes(shape, xfm, shape, xfm);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);

	EXPECT_EQ(manifold.GetLocalPoint().x, float_t(+2));
	EXPECT_EQ(manifold.GetLocalPoint().y, float_t(0));
	
	EXPECT_EQ(manifold.GetLocalNormal().x, float_t(+1));
	EXPECT_EQ(manifold.GetLocalNormal().y, float_t(0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));

	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, -2); // left
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, 2); // top
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);

	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint.x, -2); // left
	EXPECT_EQ(manifold.GetPoint(1).localPoint.y, -2); // bottom
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(Collision, IdenticalVerticalTouchingSquares)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(0).y, float_t(-2)); // bottom
	ASSERT_EQ(shape.GetVertex(1).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(1).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(2).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(2).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(3).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(3).y, float_t(-2)); // bottom

	const auto xfm0 = Transformation(Vec2{0, -1}, Rot{0}); // bottom
	const auto xfm1 = Transformation(Vec2{0, +1}, Rot{0}); // top
	const auto manifold = CollideShapes(shape, xfm0, shape, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, float_t(0));
	EXPECT_EQ(manifold.GetLocalPoint().y, float_t(+2));
	
	EXPECT_EQ(manifold.GetLocalNormal().x, float_t(0));
	EXPECT_EQ(manifold.GetLocalNormal().y, float_t(+1));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, -2); // left
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, -2); // bottom
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint.x, +2); // right
	EXPECT_EQ(manifold.GetPoint(1).localPoint.y, -2); // bottom
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(Collision, IdenticalHorizontalTouchingSquares)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(0).y, float_t(-2)); // bottom
	ASSERT_EQ(shape.GetVertex(1).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(1).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(2).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(2).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(3).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(3).y, float_t(-2)); // bottom

	const auto xfm0 = Transformation(Vec2{-2, 0}, Rot{0}); // left
	const auto xfm1 = Transformation(Vec2{+2, 0}, Rot{0}); // right
	const auto manifold = CollideShapes(shape, xfm0, shape, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, float_t(+2));
	EXPECT_EQ(manifold.GetLocalPoint().y, float_t(0));
	
	EXPECT_EQ(manifold.GetLocalNormal().x, float_t(+1));
	EXPECT_EQ(manifold.GetLocalNormal().y, float_t(0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, -2); // left
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, +2); // top
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint.x, -2); // left
	EXPECT_EQ(manifold.GetPoint(1).localPoint.y, -2); // bottom
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(Collision, SquareCornerUnderSquareFace)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	ASSERT_EQ(shape.GetVertex(0).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(0).y, float_t(-2)); // bottom
	ASSERT_EQ(shape.GetVertex(1).x, float_t(+2)); // right
	ASSERT_EQ(shape.GetVertex(1).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(2).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(2).y, float_t(+2)); // top
	ASSERT_EQ(shape.GetVertex(3).x, float_t(-2)); // left
	ASSERT_EQ(shape.GetVertex(3).y, float_t(-2)); // bottom
	
	const auto xfm0 = Transformation(Vec2{0, -1}, Rot{DegreesToRadians(45)}); // bottom
	const auto xfm1 = Transformation(Vec2{0, +1}, Rot{0}); // top
	const auto manifold = CollideShapes(shape, xfm0, shape, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, float_t(0));
	EXPECT_EQ(manifold.GetLocalPoint().y, float_t(-2));
	
	EXPECT_EQ(manifold.GetLocalNormal().x, float_t(0));
	EXPECT_EQ(manifold.GetLocalNormal().y, float_t(-1));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.x, float_t(+2)); // left
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.y, float_t(+2)); // bottom
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).localPoint.x, float_t(+2)); // right
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).localPoint.y, float_t(-0.8289929)); // bottom
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(Collision, HorizontalOverlappingRects1)
{
	// square
	const auto shape0 = PolygonShape(2, 2);
	ASSERT_EQ(shape0.GetVertex(0).x, float_t(+2)); // right
	ASSERT_EQ(shape0.GetVertex(0).y, float_t(-2)); // bottom
	ASSERT_EQ(shape0.GetVertex(1).x, float_t(+2)); // right
	ASSERT_EQ(shape0.GetVertex(1).y, float_t(+2)); // top
	ASSERT_EQ(shape0.GetVertex(2).x, float_t(-2)); // left
	ASSERT_EQ(shape0.GetVertex(2).y, float_t(+2)); // top
	ASSERT_EQ(shape0.GetVertex(3).x, float_t(-2)); // left
	ASSERT_EQ(shape0.GetVertex(3).y, float_t(-2)); // bottom
	
	// wide rectangle
	const auto shape1 = PolygonShape(3, 1.5);
	ASSERT_EQ(shape1.GetVertex(0).x, float_t(+3.0)); // right
	ASSERT_EQ(shape1.GetVertex(0).y, float_t(-1.5)); // bottom
	ASSERT_EQ(shape1.GetVertex(1).x, float_t(+3.0)); // right
	ASSERT_EQ(shape1.GetVertex(1).y, float_t(+1.5)); // top
	ASSERT_EQ(shape1.GetVertex(2).x, float_t(-3.0)); // left
	ASSERT_EQ(shape1.GetVertex(2).y, float_t(+1.5)); // top
	ASSERT_EQ(shape1.GetVertex(3).x, float_t(-3.0)); // left
	ASSERT_EQ(shape1.GetVertex(3).y, float_t(-1.5)); // bottom

	const auto xfm0 = Transformation(Vec2{-2, 0}, Rot{0}); // left
	const auto xfm1 = Transformation(Vec2{+2, 0}, Rot{0}); // right
	
	// square left, wide rectangle right
	const auto manifold = CollideShapes(shape0, xfm0, shape1, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, float_t(+2));
	EXPECT_EQ(manifold.GetLocalPoint().y, float_t(0));
	
	EXPECT_EQ(manifold.GetLocalNormal().x, float_t(+1));
	EXPECT_EQ(manifold.GetLocalNormal().y, float_t(0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, float_t(-3.0)); // left
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, float_t(+1.5)); // top
	EXPECT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_EQ(manifold.GetPoint(1).localPoint.x, float_t(-3.0)); // left
	EXPECT_EQ(manifold.GetPoint(1).localPoint.y, float_t(-1.5)); // bottom
	EXPECT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
	
	const auto world_manifold = GetWorldManifold(manifold, xfm0, GetRadius(shape0), xfm1, GetRadius(shape1));
	EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
	
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().x, float_t(1));
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().y, float_t(0));
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).x, float_t(-0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).y, float_t(+1.5));

	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).x, float_t(-0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).y, float_t(-1.5));
}

TEST(Collision, HorizontalOverlappingRects2)
{
	// wide rectangle
	const auto shape0 = PolygonShape(3, 1.5);
	ASSERT_EQ(shape0.GetVertex(0).x, float_t(+3.0)); // right
	ASSERT_EQ(shape0.GetVertex(0).y, float_t(-1.5)); // bottom
	ASSERT_EQ(shape0.GetVertex(1).x, float_t(+3.0)); // right
	ASSERT_EQ(shape0.GetVertex(1).y, float_t(+1.5)); // top
	ASSERT_EQ(shape0.GetVertex(2).x, float_t(-3.0)); // left
	ASSERT_EQ(shape0.GetVertex(2).y, float_t(+1.5)); // top
	ASSERT_EQ(shape0.GetVertex(3).x, float_t(-3.0)); // left
	ASSERT_EQ(shape0.GetVertex(3).y, float_t(-1.5)); // bottom
	
	// square
	const auto shape1 = PolygonShape(2, 2);
	ASSERT_EQ(shape1.GetVertex(0).x, float_t(+2)); // right
	ASSERT_EQ(shape1.GetVertex(0).y, float_t(-2)); // bottom
	ASSERT_EQ(shape1.GetVertex(1).x, float_t(+2)); // right
	ASSERT_EQ(shape1.GetVertex(1).y, float_t(+2)); // top
	ASSERT_EQ(shape1.GetVertex(2).x, float_t(-2)); // left
	ASSERT_EQ(shape1.GetVertex(2).y, float_t(+2)); // top
	ASSERT_EQ(shape1.GetVertex(3).x, float_t(-2)); // left
	ASSERT_EQ(shape1.GetVertex(3).y, float_t(-2)); // bottom
	
	const auto xfm0 = Transformation(Vec2{-2, 0}, Rot{0}); // left
	const auto xfm1 = Transformation(Vec2{+2, 0}, Rot{0}); // right

	// put wide rectangle on left, square on right
	const auto manifold = CollideShapes(shape0, xfm0, shape1, xfm1);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, float_t(+3));
	EXPECT_EQ(manifold.GetLocalPoint().y, float_t(0));
	
	EXPECT_EQ(manifold.GetLocalNormal().x, float_t(+1));
	EXPECT_EQ(manifold.GetLocalNormal().y, float_t(0));
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	const auto total_radius = GetRadius(shape0) + GetRadius(shape1);

	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.x, float_t(-2.0)); // left
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).localPoint.y, float_t(-1.5) - total_radius); // top
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).localPoint.x, float_t(-2.0)); // left
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).localPoint.y, float_t(+1.5) + total_radius); // bottom
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	EXPECT_FLOAT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_face);
	EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
	
	const auto world_manifold = GetWorldManifold(manifold, xfm0, GetRadius(shape0), xfm1, GetRadius(shape1));
	EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
	
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().x, float_t(1));
	EXPECT_FLOAT_EQ(world_manifold.GetNormal().y, float_t(0));
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).x, float_t(+0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(0).y, float_t(-1.5) - total_radius);
	
	ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).x, float_t(+0.5));
	EXPECT_FLOAT_EQ(world_manifold.GetPoint(1).y, float_t(+1.5) + total_radius);
}