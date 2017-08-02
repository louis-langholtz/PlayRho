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

#include "gtest/gtest.h"
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/FixtureDef.hpp>

using namespace playrho;

TEST(WorldManifold, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(WorldManifold), std::size_t(52)); break;
        case  8: EXPECT_EQ(sizeof(WorldManifold), std::size_t(104)); break;
        case 16: EXPECT_EQ(sizeof(WorldManifold), std::size_t(208)); break;
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
    const auto normal = UnitVec2::GetLeft();
    const auto wm = WorldManifold{normal};
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_TRUE(IsValid(wm.GetNormal()));
    EXPECT_EQ(wm.GetNormal(), UnitVec2::GetLeft());
}

TEST(WorldManifold, GetWorldManifoldForUnsetManifold)
{
    const auto manifold = Manifold{};
    const auto xfA = Transformation{
        Length2D{Real(4-1) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    const auto xfB = Transformation{
        Length2D{Real(4+1) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    const auto rA = Real(1) * Meter;
    const auto rB = Real(1) * Meter;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_FALSE(IsValid(wm.GetNormal()));
}

TEST(WorldManifold, GetWorldManifoldForCirclesTouchingManifold)
{
    const auto manifold = Manifold::GetForCircles(Length2D{}, 0, Length2D{}, 0);
    const auto xfA = Transformation{
        Length2D{Real(4-1) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    const auto xfB = Transformation{
        Length2D{Real(4+1) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    const auto rA = Real(1) * Meter;
    const auto rB = Real(1) * Meter;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
    EXPECT_TRUE(IsValid(wm.GetNormal()));
    EXPECT_EQ(wm.GetNormal() * Real{1}, Vec2(1, 0));
    EXPECT_EQ(wm.GetSeparation(0), Real{0} * Meter);
    EXPECT_EQ(wm.GetPoint(0), Length2D(Real(4) * Meter, Real(0) * Meter));
}

TEST(WorldManifold, GetWorldManifoldForCirclesHalfOverlappingManifold)
{
    const auto manifold = Manifold::GetForCircles(Length2D{}, 0, Length2D{}, 0);
    const auto xfA = Transformation{
        Length2D{Real(7-0.5) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    const auto xfB = Transformation{
        Length2D{Real(7+0.5) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    const auto rA = Real(1) * Meter;
    const auto rB = Real(1) * Meter;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
    EXPECT_TRUE(IsValid(wm.GetNormal()));
    EXPECT_EQ(wm.GetNormal() * Real{1}, Vec2(1, 0));
    EXPECT_EQ(wm.GetSeparation(0), Real{-1} * Meter);
    EXPECT_EQ(wm.GetPoint(0), Length2D(Real(7) * Meter, Real(0) * Meter));
}

TEST(WorldManifold, GetWorldManifoldForCirclesFullyOverlappingManifold)
{
    const auto manifold = Manifold::GetForCircles(Length2D{}, 0, Length2D{}, 0);
    const auto xfA = Transformation{Length2D{Real(3-0) * Meter, Real(0) * Meter}, UnitVec2::GetRight()};
    const auto xfB = Transformation{Length2D{Real(3+0) * Meter, Real(0) * Meter}, UnitVec2::GetRight()};
    const auto rA = Real(1) * Meter;
    const auto rB = Real(1) * Meter;
    const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
    EXPECT_EQ(wm.GetSeparation(0), Real{-2} * Meter);
    if (IsValid(wm.GetNormal()))
    {
        EXPECT_EQ(wm.GetPoint(0), Length2D(Real(3) * Meter, Real(0) * Meter));
    }
    else
    {
        EXPECT_FALSE(IsValid(wm.GetPoint(0)));
    }
}

TEST(WorldManifold, GetForContact)
{
    const auto shape = std::make_shared<DiskShape>();
    auto bA = Body{BodyDef{}};
    auto bB = Body{BodyDef{}};
    auto fA = Fixture{&bA, FixtureDef{}, shape};
    auto fB = Fixture{&bB, FixtureDef{}, shape};
    const auto c = Contact{&fA, 0, &fB, 0};
    
    const auto wm = GetWorldManifold(c);
    
    EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
    EXPECT_FALSE(IsValid(wm.GetNormal()));
}
