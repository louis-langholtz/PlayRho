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

#include <PlayRho/Collision/AABB.hpp>

#include <PlayRho/Common/Vector2.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

#include <algorithm>
#include <iterator>
#include <limits> // for std::numeric_limits
#include <string>
#include <type_traits> // for std::is_default_constructible etc.
#include <utility>

using namespace playrho;
using namespace playrho::d2;

TEST(AABB, ByteSizeIsTwiceVec2)
{
    EXPECT_EQ(sizeof(AABB), sizeof(Vec2) * 2);
}

TEST(AABB, DefaultConstruction)
{
    const auto infinity = std::numeric_limits<Real>::infinity();
    const auto lb = Vec2{infinity, infinity} * Meter;
    const auto ub = Vec2{-infinity, -infinity} * Meter;
    const auto aabb = AABB{};
    EXPECT_EQ(GetLowerBound(aabb), lb);
    EXPECT_EQ(GetUpperBound(aabb), ub);
}

TEST(AABB, Traits)
{
    EXPECT_FALSE(IsIterable<AABB>::value);
    EXPECT_FALSE(IsAddable<AABB>::value);

    EXPECT_TRUE(std::is_default_constructible<AABB>::value);
    //EXPECT_TRUE(std::is_nothrow_default_constructible<AABB>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<AABB>::value);

    EXPECT_TRUE((std::is_constructible<AABB, Length2>::value));
    //EXPECT_FALSE((std::is_nothrow_constructible<AABB, Length2>::value));
    EXPECT_FALSE((std::is_trivially_constructible<AABB, Length2>::value));
    
    EXPECT_TRUE((std::is_constructible<AABB, Length2, Length2>::value));
    //EXPECT_FALSE((std::is_nothrow_constructible<AABB, Length2, Length2>::value));
    EXPECT_FALSE((std::is_trivially_constructible<AABB, Length2, Length2>::value));
    
    EXPECT_TRUE(std::is_copy_constructible<AABB>::value);
    //EXPECT_TRUE(std::is_nothrow_copy_constructible<AABB>::value);
    //EXPECT_TRUE(std::is_trivially_copy_constructible<AABB>::value);

    EXPECT_TRUE(std::is_move_constructible<AABB>::value);
    //EXPECT_TRUE(std::is_nothrow_move_constructible<AABB>::value);
    //EXPECT_FALSE(std::is_trivially_move_constructible<AABB>::value);

    EXPECT_TRUE(std::is_copy_assignable<AABB>::value);
    //EXPECT_FALSE(std::is_nothrow_copy_assignable<AABB>::value);
    //EXPECT_FALSE(std::is_trivially_copy_assignable<AABB>::value);

    EXPECT_TRUE(std::is_move_assignable<AABB>::value);
    //EXPECT_FALSE(std::is_nothrow_move_assignable<AABB>::value);
    //EXPECT_FALSE(std::is_trivially_move_assignable<AABB>::value);

    EXPECT_TRUE(std::is_destructible<AABB>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<AABB>::value);
    EXPECT_TRUE(std::is_trivially_destructible<AABB>::value);
}

TEST(AABB, DefaultAabbAddsToOther)
{
    const auto default_aabb = AABB{};
    {
        const auto other_aabb = AABB{Length2{}, Length2{}};
        const auto sum_aabb = GetEnclosingAABB(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
    {
        const auto other_aabb = AABB{Length2{}, Length2{}};
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
    {
        const auto other_aabb = AABB{Length2{ -1_m, -2_m}, Length2{+99_m, +3_m}};
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
}

TEST(AABB, DefaultAabbIncrementsToOther)
{
    {
        auto default_aabb = AABB{};
        const auto other_aabb = AABB{Length2{}, Length2{}};
        Include(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(default_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(default_aabb), GetUpperBound(other_aabb));
    }
    {
        auto default_aabb = AABB{};
        const auto other_aabb = AABB{Length2{-1_m, -2_m}, Length2{+99_m, +3_m}};
        Include(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(default_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(default_aabb), GetUpperBound(other_aabb));
    }
}

TEST(AABB, InitializingConstruction)
{
    const auto lower_x = -2_m;
    const auto lower_y = -3_m;
    const auto upper_x = +1.6_m;
    const auto upper_y = +1.9_m;
    
    const auto center_x = (lower_x + upper_x) / Real{2};
    const auto center_y = (lower_y + upper_y) / Real{2};

    const auto v0 = Length2{upper_x, lower_y};
    const auto v1 = Length2{lower_x, upper_y};
    
    {
        AABB foo{v0, v1};
        EXPECT_EQ(GetX(GetCenter(foo)), center_x);
        EXPECT_EQ(GetY(GetCenter(foo)), center_y);
        EXPECT_EQ(GetX(GetLowerBound(foo)), lower_x);
        EXPECT_EQ(GetY(GetLowerBound(foo)), lower_y);
        EXPECT_EQ(GetX(GetUpperBound(foo)), upper_x);
        EXPECT_EQ(GetY(GetUpperBound(foo)), upper_y);
    }
    {
        AABB foo{v1, v0};
        EXPECT_EQ(GetX(GetCenter(foo)), center_x);
        EXPECT_EQ(GetY(GetCenter(foo)), center_y);
        EXPECT_EQ(GetX(GetLowerBound(foo)), lower_x);
        EXPECT_EQ(GetY(GetLowerBound(foo)), lower_y);
        EXPECT_EQ(GetX(GetUpperBound(foo)), upper_x);
        EXPECT_EQ(GetY(GetUpperBound(foo)), upper_y);
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), GetInvalid<Length>()};
        const auto pb = Length2{GetInvalid<Length>(), GetInvalid<Length>()};
        AABB foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), GetInvalid<Length>()};
        const auto pb = Length2{GetInvalid<Length>(), 0_m};
        AABB foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), 0_m};
        const auto pb = Length2{GetInvalid<Length>(), GetInvalid<Length>()};
        AABB foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), 0_m};
        const auto pb = Length2{GetInvalid<Length>(), 0_m};
        AABB foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto rangeX = Interval<Length>{-2_m, +3_m};
        const auto rangeY = Interval<Length>{-8_m, -4_m};
        AABB foo{rangeX, rangeY};
        EXPECT_EQ(foo.ranges[0], rangeX);
        EXPECT_EQ(foo.ranges[1], rangeY);
    }
}

TEST(AABB, Swappable)
{
    auto a = AABB{};
    auto b = AABB{};
    ASSERT_EQ(a, b);
    swap(a, b);
    EXPECT_EQ(a, b);
    const auto aBefore = a;
    Include(a, Length2{2_m, 3_m});
    const auto aAfter = a;
    ASSERT_NE(a, b);
    swap(a, b);
    EXPECT_EQ(a, aBefore);
    EXPECT_EQ(b, aAfter);
}

TEST(AABB, GetPerimeterOfPoint)
{
    EXPECT_EQ(GetPerimeter(AABB{Length2{}}), 0_m);
    EXPECT_EQ(GetPerimeter(AABB{Length2{-1_m, -2_m}}), 0_m);
    EXPECT_EQ(GetPerimeter(AABB{Length2{+99_m, +3_m}}), 0_m);
    EXPECT_TRUE(isnan(StripUnit(GetPerimeter(AABB{
        Length2{
            Real(+std::numeric_limits<Real>::infinity()) * Meter,
            Real(+std::numeric_limits<Real>::infinity()) * Meter
        }
    }))));
}

TEST(AABB, Include)
{
    const auto p1 = Length2{2_m, 3_m};
    const auto p2 = Length2{20_m, 30_m};
    const auto p3 = Length2{-3_m, -4_m};
    const auto p4 = Length2{0_m, 0_m};
    const auto p5 = AABB{};

    auto foo = AABB{};
    
    Include(foo, p1);
    EXPECT_EQ(GetLowerBound(foo), p1);
    EXPECT_EQ(GetUpperBound(foo), p1);
    
    Include(foo, p2);
    EXPECT_EQ(GetLowerBound(foo), p1);
    EXPECT_EQ(GetUpperBound(foo), p2);
    
    Include(foo, p3);
    EXPECT_EQ(GetLowerBound(foo), p3);
    EXPECT_EQ(GetUpperBound(foo), p2);
    
    Include(foo, p4);
    EXPECT_EQ(GetLowerBound(foo), p3);
    EXPECT_EQ(GetUpperBound(foo), p2);
    
    {
        const auto copyOfFoo = foo;
        EXPECT_EQ(Include(foo, p5), copyOfFoo);
    }
    EXPECT_EQ(GetEnclosingAABB(AABB{}, foo), foo);
}

TEST(AABB, Contains)
{
    EXPECT_TRUE(Contains(AABB{}, AABB{}));
    EXPECT_TRUE(Contains(AABB{Length2{}}, AABB{Length2{}}));
    EXPECT_TRUE((Contains(AABB{Length2{}, Length2{}}, AABB{Length2{}})));
    EXPECT_TRUE((Contains(AABB{Length2{}}, AABB{Length2{}, Length2{}})));
    EXPECT_TRUE((Contains(AABB{Length2{1_m, 2_m}}, AABB{})));
    EXPECT_FALSE(Contains(GetInvalid<AABB>(), GetInvalid<AABB>()));
    EXPECT_FALSE(Contains(GetInvalid<AABB>(), AABB{}));
    EXPECT_FALSE(Contains(AABB{}, GetInvalid<AABB>()));
}

TEST(AABB, TestOverlap)
{
    {
        AABB bb1{Length2{-2_m, -3_m}, Length2{-1_m,  0_m}};
        EXPECT_TRUE(TestOverlap(bb1, bb1));
    }
    {
        const auto vec = Length2{-2_m, -3_m};
        AABB bb1{vec, vec};
        EXPECT_TRUE(TestOverlap(bb1, bb1));
    }
    {
        AABB bb1{Length2{-2_m, -3_m}, Length2{-1_m,  0_m}};
        AABB bb2{Length2{-1_m, -1_m}, Length2{ 1_m,  2_m}};
        EXPECT_TRUE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{Length2{-99_m, -3_m}, Length2{-1_m,  0_m}};
        AABB bb2{Length2{ 76_m, -1_m}, Length2{-2_m,  2_m}};
        EXPECT_TRUE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{Length2{-20_m, -3_m}, Length2{-18_m,  0_m}};
        AABB bb2{Length2{ -1_m, -1_m}, Length2{  1_m,  2_m}};
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{Length2{-2_m, -3_m}, Length2{-1_m,  0_m}};
        AABB bb2{Length2{-1_m, +1_m}, Length2{ 1_m,  2_m}};
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{Length2{-2_m, +3_m}, Length2{-1_m,  0_m}};
        AABB bb2{Length2{-1_m, -1_m}, Length2{ 0_m, -2_m}};
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
}

TEST(AABB, ComputeAabbForDefaultDistanceProxy)
{
    const auto defaultAabb = AABB{};
    const auto proxyAabb = ComputeAABB(DistanceProxy{}, Transform_identity);
    
    EXPECT_EQ(defaultAabb, proxyAabb);
}

TEST(AABB, Move)
{
    const auto zeroLoc = Length2{};
    const auto zeroAabb = AABB{zeroLoc};
    {
        auto aabb = AABB{};
        EXPECT_EQ(Move(aabb, zeroLoc), AABB{});
        EXPECT_EQ(Move(aabb, Length2{10_m, -4_m}), AABB{});
    }
    {
        auto aabb = AABB{Length2{}};
        EXPECT_EQ(Move(aabb, Length2{}), zeroAabb);
    }
    {
        const auto aabb1 = AABB{Length2{1_m, 1_m}};
        const auto aabb2 = AABB{Length2{-10_m, 11_m}};
        auto aabb = zeroAabb;
        EXPECT_EQ(Move(aabb, Length2{1_m, 1_m}), aabb1);
        EXPECT_EQ(Move(aabb, Length2{-1_m, -1_m}), zeroAabb);
        EXPECT_EQ(Move(aabb, Length2{-10_m, 11_m}), aabb2);
    }
    {
        const auto lower = Length2{-1_m, -1_m};
        const auto upper = Length2{+3_m, +9_m};
        auto aabb = AABB{lower, upper};
        const auto moveby = Length2{1_m, 1_m};
        EXPECT_EQ(Move(aabb, moveby), AABB(lower + moveby, upper + moveby));
    }
}

TEST(AABB, ComparisonOperators)
{
    EXPECT_TRUE(AABB{} == AABB{});
    EXPECT_FALSE(AABB{} != AABB{});
    EXPECT_TRUE(AABB{} <= AABB{});
    EXPECT_TRUE(AABB{} >= AABB{});
    EXPECT_FALSE(AABB{} < AABB{});
    EXPECT_FALSE(AABB{} > AABB{});
    
    const auto vr0 = Interval<Length>{1_m, 2_m};
    const auto vr1 = Interval<Length>{3_m, 4_m};
    const auto vr2 = Interval<Length>{5_m, 6_m};
    const auto vr3 = Interval<Length>{7_m, 8_m};

    EXPECT_FALSE(AABB(vr0, vr1) == AABB{});
    EXPECT_TRUE(AABB(vr0, vr1) != AABB{});
    EXPECT_TRUE(AABB(vr0, vr1) <= AABB{});
    EXPECT_FALSE(AABB(vr0, vr1) >= AABB{});
    EXPECT_TRUE(AABB(vr0, vr1) < AABB{});
    EXPECT_FALSE(AABB(vr0, vr1) > AABB{});

    EXPECT_FALSE(AABB{} == AABB(vr0, vr1));
    EXPECT_TRUE(AABB{} != AABB(vr0, vr1));
    EXPECT_FALSE(AABB{} <= AABB(vr0, vr1));
    EXPECT_TRUE(AABB{} >= AABB(vr0, vr1));
    EXPECT_FALSE(AABB{} < AABB(vr0, vr1));
    EXPECT_TRUE(AABB{} > AABB(vr0, vr1));

    EXPECT_FALSE(AABB(vr0, vr1) == AABB(vr2, vr3));
    EXPECT_TRUE(AABB(vr0, vr1) != AABB(vr2, vr3));
    EXPECT_TRUE(AABB(vr0, vr1) <= AABB(vr2, vr3));
    EXPECT_FALSE(AABB(vr0, vr1) >= AABB(vr2, vr3));
    EXPECT_TRUE(AABB(vr0, vr1) < AABB(vr2, vr3));
    EXPECT_FALSE(AABB(vr0, vr1) > AABB(vr2, vr3));
}

TEST(AABB, StreamOutputOperator)
{
    const auto rangeX = Interval<Length>{-2_m, +3_m};
    const auto rangeY = Interval<Length>{-8_m, -4_m};
    AABB foo{rangeX, rangeY};
    ASSERT_EQ(foo.ranges[0], rangeX);
    ASSERT_EQ(foo.ranges[1], rangeY);
    
    std::stringstream aabbStream;
    ASSERT_TRUE(aabbStream.str().empty());
    aabbStream << foo;
    ASSERT_FALSE(aabbStream.str().empty());
    
    std::stringstream xRangeStream;
    xRangeStream << foo.ranges[0];
    
    std::stringstream yRangeStream;
    yRangeStream << foo.ranges[1];
    
    std::string comp;
    comp += '{';
    comp += xRangeStream.str();
    comp += ',';
    comp += yRangeStream.str();
    comp += '}';
    EXPECT_STREQ(aabbStream.str().c_str(), comp.c_str());
}

TEST(AABB, ComputeAabbForFixtureAtBodyOrigin)
{
    const auto shape = DiskShapeConf{};
    const auto shapeAabb = ComputeAABB(Shape(shape), Transformation{});

    World world;
    const auto body = world.CreateBody();
    const auto fixture = body->CreateFixture(Shape{shape});
    const auto fixtureAabb = ComputeAABB(*fixture);
    
    ASSERT_NE(shapeAabb, AABB{});
    EXPECT_EQ(shapeAabb, fixtureAabb);
}

TEST(AABB, ComputeAabbForFixtureOffFromBodyOrigin)
{
    const auto shape = DiskShapeConf{};
    const auto shapeAabb = ComputeAABB(Shape{shape}, Transformation{});
    
    const auto bodyLocation = Length2{2_m, 3_m};
    World world;
    const auto body = world.CreateBody(BodyConf{}.UseLocation(bodyLocation));
    const auto fixture = body->CreateFixture(Shape{shape});
    const auto fixtureAabb = ComputeAABB(*fixture);
    
    ASSERT_NE(shapeAabb, AABB{});
    ASSERT_NE(shapeAabb, fixtureAabb);
    EXPECT_EQ(GetMovedAABB(shapeAabb, bodyLocation), fixtureAabb);
}

TEST(AABB, ComputeIntersectingAABBForSameFixture)
{
    const auto shape = DiskShapeConf{};
    const auto shapeAabb = ComputeAABB(Shape{shape}, Transformation{});
    
    World world;
    const auto body = world.CreateBody();
    const auto fixture = body->CreateFixture(Shape{shape});
    const auto fixtureAabb = ComputeAABB(*fixture);
    
    const auto intersectingAabb = ComputeIntersectingAABB(*fixture, 0, *fixture, 0);
    
    ASSERT_NE(shapeAabb, AABB{});
    ASSERT_EQ(shapeAabb, fixtureAabb);
    EXPECT_EQ(fixtureAabb, intersectingAabb);
}

TEST(AABB, ComputeIntersectingAABBForTwoFixtures)
{
    const auto shapeInterval = LengthInterval{-2_m, +2_m};

    const auto shape = DiskShapeConf{}.UseRadius(2_m);
    const auto shapeAabb = ComputeAABB(Shape{shape}, Transformation{});
    ASSERT_EQ(shapeAabb, (AABB{shapeInterval, shapeInterval}));

    const auto bodyLocation0 = Length2{+1_m, 0_m};
    const auto bodyLocation1 = Length2{-1_m, 0_m};

    World world;
    const auto body0 = world.CreateBody(BodyConf{}.UseLocation(bodyLocation0));
    const auto body1 = world.CreateBody(BodyConf{}.UseLocation(bodyLocation1));

    const auto fixture0 = body0->CreateFixture(Shape{shape});
    const auto fixture1 = body1->CreateFixture(Shape{shape});

    const auto fixtureAabb0 = ComputeAABB(*fixture0);
    const auto fixtureAabb1 = ComputeAABB(*fixture1);

    const auto intersectingAabb = ComputeIntersectingAABB(*fixture0, 0, *fixture1, 0);
    const auto intersectInterval = LengthInterval{-1_m, +1_m};

    ASSERT_NE(shapeAabb, fixtureAabb0);
    ASSERT_NE(shapeAabb, fixtureAabb1);
    EXPECT_EQ(intersectingAabb, (AABB{intersectInterval, shapeInterval}));
}

TEST(AABB, ComputeIntersectingAABBForContact)
{
    const auto shapeInterval = LengthInterval{-2_m, +2_m};
    
    const auto shape = DiskShapeConf{}.UseRadius(2_m);
    const auto shapeAabb = ComputeAABB(Shape{shape}, Transformation{});
    ASSERT_EQ(shapeAabb, (AABB{shapeInterval, shapeInterval}));
    
    const auto bodyLocation0 = Length2{+1_m, 0_m};
    const auto bodyLocation1 = Length2{-1_m, 0_m};
    
    World world;
    const auto body0 = world.CreateBody(BodyConf{}.UseLocation(bodyLocation0));
    const auto body1 = world.CreateBody(BodyConf{}.UseLocation(bodyLocation1));
    
    const auto fixture0 = body0->CreateFixture(Shape{shape});
    const auto fixture1 = body1->CreateFixture(Shape{shape});
    
    const auto fixtureAabb0 = ComputeAABB(*fixture0);
    const auto fixtureAabb1 = ComputeAABB(*fixture1);
    
    const auto intersectingAabb = ComputeIntersectingAABB(*fixture0, 0, *fixture1, 0);
    const auto intersectInterval = LengthInterval{-1_m, +1_m};
    
    ASSERT_NE(shapeAabb, fixtureAabb0);
    ASSERT_NE(shapeAabb, fixtureAabb1);
    ASSERT_EQ(intersectingAabb, (AABB{intersectInterval, shapeInterval}));
    
    const auto contact = Contact{fixture0, 0, fixture1, 0};
    const auto contactAabb = ComputeIntersectingAABB(contact);
    
    EXPECT_EQ(contactAabb, intersectingAabb);
}
