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
    const auto point = Length2{-1_m, 3_m};
    const auto separation = 8.12_m;
    
    const auto psm = PositionSolverManifold{normal, point, separation};
    
    EXPECT_EQ(psm.m_normal, normal);
    EXPECT_EQ(psm.m_point, point);
    EXPECT_EQ(psm.m_separation, separation);
}

TEST(PositionSolverManifold, GetPSM)
{
    // wide rectangle
    const auto shape0 = PolygonShape(3_m, 1.5_m);
    ASSERT_EQ(GetX(shape0.GetVertex(0)), +3.0_m); // right
    ASSERT_EQ(GetY(shape0.GetVertex(0)), -1.5_m); // bottom
    ASSERT_EQ(GetX(shape0.GetVertex(1)), +3.0_m); // right
    ASSERT_EQ(GetY(shape0.GetVertex(1)), +1.5_m); // top
    ASSERT_EQ(GetX(shape0.GetVertex(2)), -3.0_m); // left
    ASSERT_EQ(GetY(shape0.GetVertex(2)), +1.5_m); // top
    ASSERT_EQ(GetX(shape0.GetVertex(3)), -3.0_m); // left
    ASSERT_EQ(GetY(shape0.GetVertex(3)), -1.5_m); // bottom
    
    // square
    const auto shape1 = PolygonShape(2_m, 2_m);
    ASSERT_EQ(GetX(shape1.GetVertex(0)), +2_m); // right
    ASSERT_EQ(GetY(shape1.GetVertex(0)), -2_m); // bottom
    ASSERT_EQ(GetX(shape1.GetVertex(1)), +2_m); // right
    ASSERT_EQ(GetY(shape1.GetVertex(1)), +2_m); // top
    ASSERT_EQ(GetX(shape1.GetVertex(2)), -2_m); // left
    ASSERT_EQ(GetY(shape1.GetVertex(2)), +2_m); // top
    ASSERT_EQ(GetX(shape1.GetVertex(3)), -2_m); // left
    ASSERT_EQ(GetY(shape1.GetVertex(3)), -2_m); // bottom
    
    const auto xfm0 = Transformation{Length2{-2_m, 0_m}, UnitVec2::GetRight()}; // left
    const auto xfm1 = Transformation{Length2{+2_m, 0_m}, UnitVec2::GetRight()}; // right
    
    // put wide rectangle on left, square on right
    const auto manifold = CollideShapes(shape0.GetChild(0), xfm0, shape1.GetChild(0), xfm1);
    
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    ASSERT_EQ(GetX(manifold.GetLocalPoint()), +3_m);
    ASSERT_EQ(GetY(manifold.GetLocalPoint()), 0_m);
    
    ASSERT_NEAR(static_cast<double>(manifold.GetLocalNormal().GetX()), +1.0, 0.00001);
    ASSERT_NEAR(static_cast<double>(manifold.GetLocalNormal().GetY()), +0.0, 0.00001);
    
    ASSERT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    const auto total_radius = GetVertexRadius(shape0) + GetVertexRadius(shape1);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(AlmostEqual(Real{GetX(manifold.GetPoint(0).localPoint) / Meter}, Real(-2.0))); // left
    EXPECT_TRUE(AlmostEqual(Real{GetY(manifold.GetPoint(0).localPoint) / Meter}, Real(-1.5))); // top
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
    ASSERT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(AlmostEqual(Real{GetX(manifold.GetPoint(1).localPoint) / Meter}, Real(-2.0))); // left
    EXPECT_TRUE(AlmostEqual(Real{GetY(manifold.GetPoint(1).localPoint) / Meter}, Real(+1.5))); // bottom
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
        
        EXPECT_TRUE(AlmostEqual(world_manifold.GetNormal().GetX(), Real(1)));
        EXPECT_TRUE(AlmostEqual(world_manifold.GetNormal().GetY(), Real(0)));
        
        ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
        EXPECT_TRUE(AlmostEqual(Real{GetX(world_manifold.GetPoint(0)) / Meter}, Real(+0.5)));
        EXPECT_TRUE(AlmostEqual(Real{GetY(world_manifold.GetPoint(0)) / Meter}, Real(-1.5)));
        EXPECT_TRUE(AlmostEqual(Real{world_manifold.GetSeparation(0) / Meter}, Real(-1) - total_radius / Meter));
        
        ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
        EXPECT_TRUE(AlmostEqual(Real{GetX(world_manifold.GetPoint(1)) / Meter}, Real(+0.5)));
        EXPECT_TRUE(AlmostEqual(Real{GetY(world_manifold.GetPoint(1)) / Meter}, Real(+1.5)));
        EXPECT_TRUE(AlmostEqual(Real{world_manifold.GetSeparation(1) / Meter}, Real(-1) - total_radius / Meter));
    }
    
    {
        const auto psm0 = GetPSM(manifold, 0, xfm0, xfm1);
        EXPECT_NEAR(static_cast<double>(psm0.m_normal.GetX()), 1.0, 0.00001);
        EXPECT_NEAR(static_cast<double>(psm0.m_normal.GetY()), 0.0, 0.00001);
        EXPECT_NEAR(static_cast<double>(Real{psm0.m_separation/Meter}), -1.0, 0.00001);
        EXPECT_TRUE(AlmostEqual(Real{GetX(psm0.m_point) / Meter}, Real(0)));
        EXPECT_TRUE(AlmostEqual(Real{GetY(psm0.m_point) / Meter}, Real(-1.5)));
    }
    {
        const auto psm1 = GetPSM(manifold, 1, xfm0, xfm1);
        EXPECT_NEAR(static_cast<double>(psm1.m_normal.GetX()), 1.0, 0.0001);
        EXPECT_NEAR(static_cast<double>(psm1.m_normal.GetY()), 0.0, 0.0001);
        EXPECT_NEAR(static_cast<double>(Real{psm1.m_separation/Meter}), -1.0, 0.00001);
        EXPECT_TRUE(AlmostEqual(Real{GetX(psm1.m_point) / Meter}, Real(0)));
        EXPECT_TRUE(AlmostEqual(Real{GetY(psm1.m_point) / Meter}, Real(+1.5)));
    }
}
