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
#include <Box2D/Dynamics/Contacts/PositionSolverManifold.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/WorldManifold.hpp>

using namespace box2d;

TEST(PositionSolverManifold, ByteSizeIs20)
{
	EXPECT_EQ(sizeof(PositionSolverManifold), size_t(20));
}

TEST(PositionSolverManifold, InitializingConstructor)
{
	const auto normal = Vec2{0, -1};
	const auto point = Vec2{-1, 3};
	const auto separation = float_t(8.12);
	
	const auto psm = PositionSolverManifold{normal, point, separation};
	
	EXPECT_EQ(psm.m_normal, normal);
	EXPECT_EQ(psm.m_point, point);
	EXPECT_EQ(psm.m_separation, separation);
}

TEST(PositionSolverManifold, GetPSM)
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
	
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	ASSERT_EQ(manifold.GetLocalPoint().x, float_t(+3));
	ASSERT_EQ(manifold.GetLocalPoint().y, float_t(0));
	
	ASSERT_EQ(manifold.GetLocalNormal().x, float_t(+1));
	ASSERT_EQ(manifold.GetLocalNormal().y, float_t(0));
	
	ASSERT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
	
	const auto total_radius = GetRadius(shape0) + GetRadius(shape1);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
	ASSERT_FLOAT_EQ(manifold.GetPoint(0).localPoint.x, float_t(-2.0)); // left
	ASSERT_FLOAT_EQ(manifold.GetPoint(0).localPoint.y, float_t(-1.5) - total_radius); // top
	ASSERT_FLOAT_EQ(manifold.GetPoint(0).normalImpulse, float_t(0));
	ASSERT_FLOAT_EQ(manifold.GetPoint(0).tangentImpulse, float_t(0));
	ASSERT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	ASSERT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	ASSERT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
	ASSERT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
	
	ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
	ASSERT_FLOAT_EQ(manifold.GetPoint(1).localPoint.x, float_t(-2.0)); // left
	ASSERT_FLOAT_EQ(manifold.GetPoint(1).localPoint.y, float_t(+1.5) + total_radius); // bottom
	ASSERT_FLOAT_EQ(manifold.GetPoint(1).normalImpulse, float_t(0));
	ASSERT_FLOAT_EQ(manifold.GetPoint(1).tangentImpulse, float_t(0));
	ASSERT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_vertex);
	ASSERT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
	ASSERT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_face);
	ASSERT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
	
	{
		// Get world-based manifold to demonstrate where things are in world coordinates.
		const auto world_manifold = GetWorldManifold(manifold, xfm0, GetRadius(shape0), xfm1, GetRadius(shape1));
		ASSERT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
		
		ASSERT_FLOAT_EQ(world_manifold.GetNormal().x, float_t(1));
		ASSERT_FLOAT_EQ(world_manifold.GetNormal().y, float_t(0));
		
		ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
		ASSERT_FLOAT_EQ(world_manifold.GetPoint(0).x, float_t(+0.5));
		ASSERT_FLOAT_EQ(world_manifold.GetPoint(0).y, float_t(-1.5) - total_radius);
		
		ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
		ASSERT_FLOAT_EQ(world_manifold.GetPoint(1).x, float_t(+0.5));
		ASSERT_FLOAT_EQ(world_manifold.GetPoint(1).y, float_t(+1.5) + total_radius);
	}
	
	{
		const auto psm0 = GetPSM(manifold, 0, xfm0, xfm1);
		EXPECT_EQ(psm0.m_normal.x, float_t(1));
		EXPECT_EQ(psm0.m_normal.y, float_t(0));
		EXPECT_EQ(psm0.m_separation, float_t(-1));
		EXPECT_FLOAT_EQ(psm0.m_point.x, float_t(0));
		EXPECT_FLOAT_EQ(psm0.m_point.y, float_t(-1.5) - total_radius);
	}
	{
		const auto psm1 = GetPSM(manifold, 1, xfm0, xfm1);
		EXPECT_EQ(psm1.m_normal.x, float_t(1));
		EXPECT_EQ(psm1.m_normal.y, float_t(0));
		EXPECT_EQ(psm1.m_separation, float_t(-1));
		EXPECT_FLOAT_EQ(psm1.m_point.x, float_t(0));
		EXPECT_FLOAT_EQ(psm1.m_point.y, float_t(+1.5) + total_radius);
	}
}
