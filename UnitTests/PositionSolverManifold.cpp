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
#include <PlayRho/Dynamics/Contacts/PositionSolverManifold.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/Manifold.hpp>

using namespace playrho;

TEST(PositionSolverManifold, ByteSizeIs_20_40_or_80)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(PositionSolverManifold), std::size_t(20)); break;
        case  8: EXPECT_EQ(sizeof(PositionSolverManifold), std::size_t(40)); break;
        case 16: EXPECT_EQ(sizeof(PositionSolverManifold), std::size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(PositionSolverManifold, InitializingConstructor)
{
    const auto normal = UnitVec2::GetBottom();
    const auto point = Length2D{-Real(1) * Meter, Real(3) * Meter};
    const auto separation = Real(8.12) * Meter;
    
    const auto psm = PositionSolverManifold{normal, point, separation};
    
    EXPECT_EQ(psm.m_normal, normal);
    EXPECT_EQ(psm.m_point, point);
    EXPECT_EQ(psm.m_separation, separation);
}

TEST(PositionSolverManifold, GetPSM)
{
    // wide rectangle
    const auto shape0 = PolygonShape(Real{3} * Meter, Real{1.5f} * Meter);
    ASSERT_EQ(shape0.GetVertex(0).x, Real(+3.0) * Meter); // right
    ASSERT_EQ(shape0.GetVertex(0).y, Real(-1.5) * Meter); // bottom
    ASSERT_EQ(shape0.GetVertex(1).x, Real(+3.0) * Meter); // right
    ASSERT_EQ(shape0.GetVertex(1).y, Real(+1.5) * Meter); // top
    ASSERT_EQ(shape0.GetVertex(2).x, Real(-3.0) * Meter); // left
    ASSERT_EQ(shape0.GetVertex(2).y, Real(+1.5) * Meter); // top
    ASSERT_EQ(shape0.GetVertex(3).x, Real(-3.0) * Meter); // left
    ASSERT_EQ(shape0.GetVertex(3).y, Real(-1.5) * Meter); // bottom
    
    // square
    const auto shape1 = PolygonShape(Real{2} * Meter, Real{2} * Meter);
    ASSERT_EQ(shape1.GetVertex(0).x, Real(+2) * Meter); // right
    ASSERT_EQ(shape1.GetVertex(0).y, Real(-2) * Meter); // bottom
    ASSERT_EQ(shape1.GetVertex(1).x, Real(+2) * Meter); // right
    ASSERT_EQ(shape1.GetVertex(1).y, Real(+2) * Meter); // top
    ASSERT_EQ(shape1.GetVertex(2).x, Real(-2) * Meter); // left
    ASSERT_EQ(shape1.GetVertex(2).y, Real(+2) * Meter); // top
    ASSERT_EQ(shape1.GetVertex(3).x, Real(-2) * Meter); // left
    ASSERT_EQ(shape1.GetVertex(3).y, Real(-2) * Meter); // bottom
    
    const auto xfm0 = Transformation{Length2D{-Real(2) * Meter, Real(0) * Meter}, UnitVec2::GetRight()}; // left
    const auto xfm1 = Transformation{Length2D{+Real(2) * Meter, Real(0) * Meter}, UnitVec2::GetRight()}; // right
    
    // put wide rectangle on left, square on right
    const auto manifold = CollideShapes(shape0.GetChild(0), xfm0, shape1.GetChild(0), xfm1);
    
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    ASSERT_EQ(manifold.GetLocalPoint().x, Real(+3) * Meter);
    ASSERT_EQ(manifold.GetLocalPoint().y, Real(0) * Meter);
    
    ASSERT_EQ(manifold.GetLocalNormal().GetX(), Real(+1));
    ASSERT_EQ(manifold.GetLocalNormal().GetY(), Real(0));
    
    ASSERT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    const auto total_radius = GetVertexRadius(shape0) + GetVertexRadius(shape1);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(almost_equal(manifold.GetPoint(0).localPoint.x / Meter, Real(-2.0))); // left
    EXPECT_TRUE(almost_equal(manifold.GetPoint(0).localPoint.y / Meter, Real(-1.5))); // top
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(almost_equal(manifold.GetPoint(1).localPoint.x / Meter, Real(-2.0))); // left
    EXPECT_TRUE(almost_equal(manifold.GetPoint(1).localPoint.y / Meter, Real(+1.5))); // bottom
    ASSERT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_vertex);
    ASSERT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    ASSERT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_face);
    ASSERT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
    
    {
        // Get world-based manifold to demonstrate where things are in world coordinates.
        const auto world_manifold = GetWorldManifold(manifold,
                                                     xfm0, GetVertexRadius(shape0),
                                                     xfm1, GetVertexRadius(shape1));
        ASSERT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
        
        EXPECT_TRUE(almost_equal(world_manifold.GetNormal().GetX(), Real(1)));
        EXPECT_TRUE(almost_equal(world_manifold.GetNormal().GetY(), Real(0)));
        
        ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
        EXPECT_TRUE(almost_equal(world_manifold.GetPoint(0).x / Meter, Real(+0.5)));
        EXPECT_TRUE(almost_equal(world_manifold.GetPoint(0).y / Meter, Real(-1.5)));
        EXPECT_TRUE(almost_equal(world_manifold.GetSeparation(0) / Meter, Real(-1) - total_radius / Meter));
        
        ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
        EXPECT_TRUE(almost_equal(world_manifold.GetPoint(1).x / Meter, Real(+0.5)));
        EXPECT_TRUE(almost_equal(world_manifold.GetPoint(1).y / Meter, Real(+1.5)));
        EXPECT_TRUE(almost_equal(world_manifold.GetSeparation(1) / Meter, Real(-1) - total_radius / Meter));
    }
    
    {
        const auto psm0 = GetPSM(manifold, 0, xfm0, xfm1);
        EXPECT_EQ(psm0.m_normal.GetX(), Real(1));
        EXPECT_EQ(psm0.m_normal.GetY(), Real(0));
        EXPECT_EQ(psm0.m_separation, Real(-1) * Meter);
        EXPECT_TRUE(almost_equal(psm0.m_point.x / Meter, Real(0)));
        EXPECT_TRUE(almost_equal(psm0.m_point.y / Meter, Real(-1.5)));
    }
    {
        const auto psm1 = GetPSM(manifold, 1, xfm0, xfm1);
        EXPECT_EQ(psm1.m_normal.GetX(), Real(1));
        EXPECT_EQ(psm1.m_normal.GetY(), Real(0));
        EXPECT_EQ(psm1.m_separation, Real(-1) * Meter);
        EXPECT_TRUE(almost_equal(psm1.m_point.x / Meter, Real(0)));
        EXPECT_TRUE(almost_equal(psm1.m_point.y / Meter, Real(+1.5)));
    }
}
