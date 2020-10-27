/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Collision/WorldManifold.hpp>

#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldFixture.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/FixtureConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldManifold, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(WorldManifold), std::size_t(48)); break;
        case  8: EXPECT_EQ(sizeof(WorldManifold), std::size_t(96)); break;
        case 16: EXPECT_EQ(sizeof(WorldManifold), std::size_t(192)); break;
        default: FAIL(); break;
    }
}

TEST(WorldManifold, PointDataByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(WorldManifold::PointData), std::size_t(20)); break;
        case  8: EXPECT_EQ(sizeof(WorldManifold::PointData), std::size_t(40)); break;
        case 16: EXPECT_EQ(sizeof(WorldManifold::PointData), std::size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(WorldManifold, DefaultConstruction)
{
    const auto wm = WorldManifold{};
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_FALSE(IsValid(wm.GetNormal()));
}

TEST(WorldManifold, UnitVecConstruction)
{
    const auto normal = UnitVec::GetLeft();
    const auto wm = WorldManifold{normal};
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_TRUE(IsValid(wm.GetNormal()));
    EXPECT_EQ(wm.GetNormal(), UnitVec::GetLeft());
}

TEST(WorldManifold, GetWorldManifoldForUnsetManifold)
{
    const auto manifold = Manifold{};
    const auto xfA = Transformation{Length2{(4-1) * Meter, 0_m}, UnitVec::GetRight()};
    const auto xfB = Transformation{Length2{(4+1) * Meter, 0_m}, UnitVec::GetRight()};
    const auto rA = 1_m;
    const auto rB = 1_m;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_FALSE(IsValid(wm.GetNormal()));
}

TEST(WorldManifold, GetForFaceEmptyManifoldA)
{
    const auto m = Manifold::GetForFaceA(UnitVec::GetTop(), Length2{});
    const auto wm = GetWorldManifold(m, Transformation{}, 1_m, Transformation{}, 1_m);
    EXPECT_EQ(wm.GetNormal(), UnitVec::GetTop());
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
}

TEST(WorldManifold, GetForFaceEmptyManifoldB)
{
    const auto m = Manifold::GetForFaceB(UnitVec::GetLeft(), Length2{});
    const auto wm = GetWorldManifold(m, Transformation{}, 1_m, Transformation{}, 1_m);
    EXPECT_EQ(wm.GetNormal(), UnitVec::GetRight());
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
}

TEST(WorldManifold, GetWorldManifoldForCirclesTouchingManifold)
{
    const auto manifold = Manifold::GetForCircles(Length2{}, 0, Length2{}, 0);
    const auto xfA = Transformation{
        Length2{Real(4-1) * Meter, 0_m}, UnitVec::GetRight()
    };
    const auto xfB = Transformation{
        Length2{Real(4+1) * Meter, 0_m}, UnitVec::GetRight()
    };
    const auto rA = 1_m;
    const auto rB = 1_m;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
    EXPECT_TRUE(IsValid(wm.GetNormal()));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(wm.GetNormal()))), 1.0, 0.00001);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(wm.GetNormal()))), 0.0, 0.00001);
    EXPECT_EQ(wm.GetSeparation(0), 0_m);
    EXPECT_EQ(wm.GetPoint(0), Length2(4_m, 0_m));
}

TEST(WorldManifold, GetWorldManifoldForCirclesHalfOverlappingManifold)
{
    const auto manifold = Manifold::GetForCircles(Length2{}, 0, Length2{}, 0);
    const auto xfA = Transformation{
        Length2{Real(7-0.5) * Meter, 0_m}, UnitVec::GetRight()
    };
    const auto xfB = Transformation{
        Length2{Real(7+0.5) * Meter, 0_m}, UnitVec::GetRight()
    };
    const auto rA = 1_m;
    const auto rB = 1_m;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
    EXPECT_TRUE(IsValid(wm.GetNormal()));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(wm.GetNormal()))), 1.0, 0.00001);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(wm.GetNormal()))), 0.0, 0.00001);
    EXPECT_NEAR(static_cast<double>(Real{wm.GetSeparation(0)/Meter}), -1.0, 0.00001);
    EXPECT_EQ(wm.GetPoint(0), Length2(7_m, 0_m));
}

TEST(WorldManifold, GetWorldManifoldForCirclesFullyOverlappingManifold)
{
    const auto manifold = Manifold::GetForCircles(Length2{}, 0, Length2{}, 0);
    const auto xfA = Transformation{Length2{Real(3-0) * Meter, 0_m}, UnitVec::GetRight()};
    const auto xfB = Transformation{Length2{Real(3+0) * Meter, 0_m}, UnitVec::GetRight()};
    const auto rA = 1_m;
    const auto rB = 1_m;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
    EXPECT_EQ(wm.GetSeparation(0), -2_m);
    if (IsValid(wm.GetNormal()))
    {
        EXPECT_EQ(wm.GetPoint(0), Length2(3_m, 0_m));
    }
    else
    {
        EXPECT_FALSE(IsValid(wm.GetPoint(0)));
    }
}

TEST(WorldManifold, GetForContact)
{
    const auto shape = Shape{DiskShapeConf{}};
    auto world = World{};
    const auto bA = CreateBody(world);
    const auto bB = CreateBody(world);
    const auto fA = CreateFixture(world, bA, shape);
    const auto fB = CreateFixture(world, bB, shape);
    const auto c = Contact{bA, fA, 0u, bB, fB, 0u};
    const auto wm = GetWorldManifold(world, c, Manifold{});
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_FALSE(IsValid(wm.GetNormal()));
}
