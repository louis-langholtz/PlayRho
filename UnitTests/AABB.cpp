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
#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <type_traits>
#include <algorithm>
#include <utility>
#include <string>

using namespace playrho;

TEST(AABB2D, ByteSizeIsTwiceVec2)
{
    EXPECT_EQ(sizeof(AABB2D), sizeof(Vec2) * 2);
}

TEST(AABB2D, DefaultConstruction)
{
    const auto infinity = std::numeric_limits<Real>::infinity();
    const auto lb = Vec2{infinity, infinity} * Meter;
    const auto ub = Vec2{-infinity, -infinity} * Meter;
    const auto aabb = AABB2D{};
    EXPECT_EQ(GetLowerBound(aabb), lb);
    EXPECT_EQ(GetUpperBound(aabb), ub);
}

TEST(AABB2D, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<AABB2D>::value);
    //EXPECT_TRUE(std::is_nothrow_default_constructible<AABB2D>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<AABB2D>::value);

    EXPECT_TRUE((std::is_constructible<AABB2D, Length2>::value));
    //EXPECT_FALSE((std::is_nothrow_constructible<AABB2D, Length2>::value));
    EXPECT_FALSE((std::is_trivially_constructible<AABB2D, Length2>::value));
    
    EXPECT_TRUE((std::is_constructible<AABB2D, Length2, Length2>::value));
    //EXPECT_FALSE((std::is_nothrow_constructible<AABB2D, Length2, Length2>::value));
    EXPECT_FALSE((std::is_trivially_constructible<AABB2D, Length2, Length2>::value));
    
    EXPECT_TRUE(std::is_copy_constructible<AABB2D>::value);
    //EXPECT_TRUE(std::is_nothrow_copy_constructible<AABB2D>::value);
    //EXPECT_TRUE(std::is_trivially_copy_constructible<AABB2D>::value);

    EXPECT_TRUE(std::is_move_constructible<AABB2D>::value);
    //EXPECT_TRUE(std::is_nothrow_move_constructible<AABB2D>::value);
    //EXPECT_FALSE(std::is_trivially_move_constructible<AABB2D>::value);

    EXPECT_TRUE(std::is_copy_assignable<AABB2D>::value);
    //EXPECT_FALSE(std::is_nothrow_copy_assignable<AABB2D>::value);
    //EXPECT_FALSE(std::is_trivially_copy_assignable<AABB2D>::value);

    EXPECT_TRUE(std::is_move_assignable<AABB2D>::value);
    //EXPECT_FALSE(std::is_nothrow_move_assignable<AABB2D>::value);
    //EXPECT_FALSE(std::is_trivially_move_assignable<AABB2D>::value);

    EXPECT_TRUE(std::is_destructible<AABB2D>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<AABB2D>::value);
    EXPECT_TRUE(std::is_trivially_destructible<AABB2D>::value);
}

TEST(AABB2D, DefaultAabbAddsToOther)
{
    const auto default_aabb = AABB2D{};
    {
        const auto other_aabb = AABB2D{Length2{}, Length2{}};
        const auto sum_aabb = GetEnclosingAABB(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
    {
        const auto other_aabb = AABB2D{Length2{}, Length2{}};
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
    {
        const auto other_aabb = AABB2D{Length2{ -1_m, -2_m}, Length2{+99_m, +3_m}};
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
}

TEST(AABB2D, DefaultAabbIncrementsToOther)
{
    {
        auto default_aabb = AABB2D{};
        const auto other_aabb = AABB2D{Length2{}, Length2{}};
        Include(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(default_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(default_aabb), GetUpperBound(other_aabb));
    }
    {
        auto default_aabb = AABB2D{};
        const auto other_aabb = AABB2D{Length2{-1_m, -2_m}, Length2{+99_m, +3_m}};
        Include(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(default_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(default_aabb), GetUpperBound(other_aabb));
    }
}

TEST(AABB2D, InitializingConstruction)
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
        AABB2D foo{v0, v1};
        EXPECT_EQ(GetX(GetCenter(foo)), center_x);
        EXPECT_EQ(GetY(GetCenter(foo)), center_y);
        EXPECT_EQ(GetX(GetLowerBound(foo)), lower_x);
        EXPECT_EQ(GetY(GetLowerBound(foo)), lower_y);
        EXPECT_EQ(GetX(GetUpperBound(foo)), upper_x);
        EXPECT_EQ(GetY(GetUpperBound(foo)), upper_y);
    }
    {
        AABB2D foo{v1, v0};
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
        AABB2D foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), GetInvalid<Length>()};
        const auto pb = Length2{GetInvalid<Length>(), 0_m};
        AABB2D foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), 0_m};
        const auto pb = Length2{GetInvalid<Length>(), GetInvalid<Length>()};
        AABB2D foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2{GetInvalid<Length>(), 0_m};
        const auto pb = Length2{GetInvalid<Length>(), 0_m};
        AABB2D foo{pa, pb};
        EXPECT_TRUE(isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_FALSE(isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto rangeX = Interval<Length>{-2_m, +3_m};
        const auto rangeY = Interval<Length>{-8_m, -4_m};
        AABB2D foo{rangeX, rangeY};
        EXPECT_EQ(foo.ranges[0], rangeX);
        EXPECT_EQ(foo.ranges[1], rangeY);
    }
}

TEST(AABB2D, Swappable)
{
    auto a = AABB2D{};
    auto b = AABB2D{};
    ASSERT_EQ(a, b);
    std::swap(a, b);
    EXPECT_EQ(a, b);
    const auto aBefore = a;
    Include(a, Length2{2_m, 3_m});
    const auto aAfter = a;
    ASSERT_NE(a, b);
    std::swap(a, b);
    EXPECT_EQ(a, aBefore);
    EXPECT_EQ(b, aAfter);
}

TEST(AABB2D, GetPerimeterOfPoint)
{
    EXPECT_EQ(GetPerimeter(AABB2D{Length2{}}), 0_m);
    EXPECT_EQ(GetPerimeter(AABB2D{Length2{-1_m, -2_m}}), 0_m);
    EXPECT_EQ(GetPerimeter(AABB2D{Length2{+99_m, +3_m}}), 0_m);
    EXPECT_TRUE(isnan(StripUnit(GetPerimeter(AABB2D{
        Length2{
            Real(+std::numeric_limits<Real>::infinity()) * Meter,
            Real(+std::numeric_limits<Real>::infinity()) * Meter
        }
    }))));
}

TEST(AABB2D, Include)
{
    const auto p1 = Length2{2_m, 3_m};
    const auto p2 = Length2{20_m, 30_m};
    const auto p3 = Length2{-3_m, -4_m};
    const auto p4 = Length2{0_m, 0_m};
    const auto p5 = AABB2D{};

    auto foo = AABB2D{};
    
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
    EXPECT_EQ(GetEnclosingAABB(AABB2D{}, foo), foo);
}

TEST(AABB2D, Contains)
{
    EXPECT_TRUE(Contains(AABB2D{}, AABB2D{}));
    EXPECT_TRUE(Contains(AABB2D{Length2{}}, AABB2D{Length2{}}));
    EXPECT_TRUE((Contains(AABB2D{Length2{}, Length2{}}, AABB2D{Length2{}})));
    EXPECT_TRUE((Contains(AABB2D{Length2{}}, AABB2D{Length2{}, Length2{}})));
    EXPECT_TRUE((Contains(AABB2D{Length2{1_m, 2_m}}, AABB2D{})));
    EXPECT_FALSE(Contains(GetInvalid<AABB2D>(), GetInvalid<AABB2D>()));
    EXPECT_FALSE(Contains(GetInvalid<AABB2D>(), AABB2D{}));
    EXPECT_FALSE(Contains(AABB2D{}, GetInvalid<AABB2D>()));
}

TEST(AABB2D, TestOverlap)
{
    {
        AABB2D bb1{Length2{-2_m, -3_m}, Length2{-1_m,  0_m}};
        EXPECT_TRUE(TestOverlap(bb1, bb1));
    }
    {
        const auto vec = Length2{-2_m, -3_m};
        AABB2D bb1{vec, vec};
        EXPECT_TRUE(TestOverlap(bb1, bb1));
    }
    {
        AABB2D bb1{Length2{-2_m, -3_m}, Length2{-1_m,  0_m}};
        AABB2D bb2{Length2{-1_m, -1_m}, Length2{ 1_m,  2_m}};
        EXPECT_TRUE(TestOverlap(bb1, bb2));
    }
    {
        AABB2D bb1{Length2{-99_m, -3_m}, Length2{-1_m,  0_m}};
        AABB2D bb2{Length2{ 76_m, -1_m}, Length2{-2_m,  2_m}};
        EXPECT_TRUE(TestOverlap(bb1, bb2));
    }
    {
        AABB2D bb1{Length2{-20_m, -3_m}, Length2{-18_m,  0_m}};
        AABB2D bb2{Length2{ -1_m, -1_m}, Length2{  1_m,  2_m}};
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
    {
        AABB2D bb1{Length2{-2_m, -3_m}, Length2{-1_m,  0_m}};
        AABB2D bb2{Length2{-1_m, +1_m}, Length2{ 1_m,  2_m}};
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
    {
        AABB2D bb1{Length2{-2_m, +3_m}, Length2{-1_m,  0_m}};
        AABB2D bb2{Length2{-1_m, -1_m}, Length2{ 0_m, -2_m}};
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
}

TEST(AABB2D, ComputeAabbForDefaultDistanceProxy)
{
    const auto defaultAabb = AABB2D{};
    const auto proxyAabb = ComputeAABB(DistanceProxy{}, Transform_identity);
    
    EXPECT_EQ(defaultAabb, proxyAabb);
}

TEST(AABB2D, Move)
{
    const auto zeroLoc = Length2{};
    const auto zeroAabb = AABB2D{zeroLoc};
    {
        auto aabb = AABB2D{};
        EXPECT_EQ(Move(aabb, zeroLoc), AABB2D{});
        EXPECT_EQ(Move(aabb, Length2{10_m, -4_m}), AABB2D{});
    }
    {
        auto aabb = AABB2D{Length2{}};
        EXPECT_EQ(Move(aabb, Length2{}), zeroAabb);
    }
    {
        const auto aabb1 = AABB2D{Length2{1_m, 1_m}};
        const auto aabb2 = AABB2D{Length2{-10_m, 11_m}};
        auto aabb = zeroAabb;
        EXPECT_EQ(Move(aabb, Length2{1_m, 1_m}), aabb1);
        EXPECT_EQ(Move(aabb, Length2{-1_m, -1_m}), zeroAabb);
        EXPECT_EQ(Move(aabb, Length2{-10_m, 11_m}), aabb2);
    }
    {
        const auto lower = Length2{-1_m, -1_m};
        const auto upper = Length2{+3_m, +9_m};
        auto aabb = AABB2D{lower, upper};
        const auto moveby = Length2{1_m, 1_m};
        EXPECT_EQ(Move(aabb, moveby), AABB2D(lower + moveby, upper + moveby));
    }
}

TEST(AABB2D, ComparisonOperators)
{
    EXPECT_TRUE(AABB2D{} == AABB2D{});
    EXPECT_FALSE(AABB2D{} != AABB2D{});
    EXPECT_TRUE(AABB2D{} <= AABB2D{});
    EXPECT_TRUE(AABB2D{} >= AABB2D{});
    EXPECT_FALSE(AABB2D{} < AABB2D{});
    EXPECT_FALSE(AABB2D{} > AABB2D{});
    
    const auto vr0 = Interval<Length>{1_m, 2_m};
    const auto vr1 = Interval<Length>{3_m, 4_m};
    const auto vr2 = Interval<Length>{5_m, 6_m};
    const auto vr3 = Interval<Length>{7_m, 8_m};

    EXPECT_FALSE(AABB2D(vr0, vr1) == AABB2D{});
    EXPECT_TRUE(AABB2D(vr0, vr1) != AABB2D{});
    EXPECT_TRUE(AABB2D(vr0, vr1) <= AABB2D{});
    EXPECT_FALSE(AABB2D(vr0, vr1) >= AABB2D{});
    EXPECT_TRUE(AABB2D(vr0, vr1) < AABB2D{});
    EXPECT_FALSE(AABB2D(vr0, vr1) > AABB2D{});

    EXPECT_FALSE(AABB2D{} == AABB2D(vr0, vr1));
    EXPECT_TRUE(AABB2D{} != AABB2D(vr0, vr1));
    EXPECT_FALSE(AABB2D{} <= AABB2D(vr0, vr1));
    EXPECT_TRUE(AABB2D{} >= AABB2D(vr0, vr1));
    EXPECT_FALSE(AABB2D{} < AABB2D(vr0, vr1));
    EXPECT_TRUE(AABB2D{} > AABB2D(vr0, vr1));

    EXPECT_FALSE(AABB2D(vr0, vr1) == AABB2D(vr2, vr3));
    EXPECT_TRUE(AABB2D(vr0, vr1) != AABB2D(vr2, vr3));
    EXPECT_TRUE(AABB2D(vr0, vr1) <= AABB2D(vr2, vr3));
    EXPECT_FALSE(AABB2D(vr0, vr1) >= AABB2D(vr2, vr3));
    EXPECT_TRUE(AABB2D(vr0, vr1) < AABB2D(vr2, vr3));
    EXPECT_FALSE(AABB2D(vr0, vr1) > AABB2D(vr2, vr3));
}

TEST(AABB2D, StreamOutputOperator)
{
    const auto rangeX = Interval<Length>{-2_m, +3_m};
    const auto rangeY = Interval<Length>{-8_m, -4_m};
    AABB2D foo{rangeX, rangeY};
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

TEST(AABB2D, ComputeAabbForFixtureAtBodyOrigin)
{
    const auto shape = std::make_shared<DiskShape>();
    const auto shapeAabb = ComputeAABB(*shape, Transformation{});

    World world;
    const auto body = world.CreateBody();
    const auto fixture = body->CreateFixture(shape);
    const auto fixtureAabb = ComputeAABB(*fixture);
    
    ASSERT_NE(shapeAabb, AABB2D{});
    EXPECT_EQ(shapeAabb, fixtureAabb);
}

TEST(AABB2D, ComputeAabbForFixtureOffFromBodyOrigin)
{
    const auto shape = std::make_shared<DiskShape>();
    const auto shapeAabb = ComputeAABB(*shape, Transformation{});
    
    const auto bodyLocation = Length2{2_m, 3_m};
    World world;
    const auto body = world.CreateBody(BodyDef{}.UseLocation(bodyLocation));
    const auto fixture = body->CreateFixture(shape);
    const auto fixtureAabb = ComputeAABB(*fixture);
    
    ASSERT_NE(shapeAabb, AABB2D{});
    ASSERT_NE(shapeAabb, fixtureAabb);
    EXPECT_EQ(GetMovedAABB(shapeAabb, bodyLocation), fixtureAabb);
}

TEST(AABB2D, ComputeIntersectingAABBForSameFixture)
{
    const auto shape = std::make_shared<DiskShape>();
    const auto shapeAabb = ComputeAABB(*shape, Transformation{});
    
    World world;
    const auto body = world.CreateBody();
    const auto fixture = body->CreateFixture(shape);
    const auto fixtureAabb = ComputeAABB(*fixture);
    
    const auto intersectingAabb = ComputeIntersectingAABB(*fixture, 0, *fixture, 0);
    
    ASSERT_NE(shapeAabb, AABB2D{});
    ASSERT_EQ(shapeAabb, fixtureAabb);
    EXPECT_EQ(fixtureAabb, intersectingAabb);
}

TEST(AABB2D, ComputeIntersectingAABBForTwoFixtures)
{
    const auto shapeInterval = LengthInterval{-2_m, +2_m};

    const auto shape = std::make_shared<DiskShape>(DiskShape::Conf{}.UseVertexRadius(2_m));
    const auto shapeAabb = ComputeAABB(*shape, Transformation{});
    ASSERT_EQ(shapeAabb, (AABB2D{shapeInterval, shapeInterval}));

    const auto bodyLocation0 = Length2{+1_m, 0_m};
    const auto bodyLocation1 = Length2{-1_m, 0_m};

    World world;
    const auto body0 = world.CreateBody(BodyDef{}.UseLocation(bodyLocation0));
    const auto body1 = world.CreateBody(BodyDef{}.UseLocation(bodyLocation1));

    const auto fixture0 = body0->CreateFixture(shape);
    const auto fixture1 = body1->CreateFixture(shape);

    const auto fixtureAabb0 = ComputeAABB(*fixture0);
    const auto fixtureAabb1 = ComputeAABB(*fixture1);

    const auto intersectingAabb = ComputeIntersectingAABB(*fixture0, 0, *fixture1, 0);
    const auto intersectInterval = LengthInterval{-1_m, +1_m};

    ASSERT_NE(shapeAabb, fixtureAabb0);
    ASSERT_NE(shapeAabb, fixtureAabb1);
    EXPECT_EQ(intersectingAabb, (AABB2D{intersectInterval, shapeInterval}));
}

TEST(AABB2D, ComputeIntersectingAABBForContact)
{
    const auto shapeInterval = LengthInterval{-2_m, +2_m};
    
    const auto shape = std::make_shared<DiskShape>(DiskShape::Conf{}.UseVertexRadius(2_m));
    const auto shapeAabb = ComputeAABB(*shape, Transformation{});
    ASSERT_EQ(shapeAabb, (AABB2D{shapeInterval, shapeInterval}));
    
    const auto bodyLocation0 = Length2{+1_m, 0_m};
    const auto bodyLocation1 = Length2{-1_m, 0_m};
    
    World world;
    const auto body0 = world.CreateBody(BodyDef{}.UseLocation(bodyLocation0));
    const auto body1 = world.CreateBody(BodyDef{}.UseLocation(bodyLocation1));
    
    const auto fixture0 = body0->CreateFixture(shape);
    const auto fixture1 = body1->CreateFixture(shape);
    
    const auto fixtureAabb0 = ComputeAABB(*fixture0);
    const auto fixtureAabb1 = ComputeAABB(*fixture1);
    
    const auto intersectingAabb = ComputeIntersectingAABB(*fixture0, 0, *fixture1, 0);
    const auto intersectInterval = LengthInterval{-1_m, +1_m};
    
    ASSERT_NE(shapeAabb, fixtureAabb0);
    ASSERT_NE(shapeAabb, fixtureAabb1);
    ASSERT_EQ(intersectingAabb, (AABB2D{intersectInterval, shapeInterval}));
    
    const auto contact = Contact{fixture0, 0, fixture1, 0};
    const auto contactAabb = ComputeIntersectingAABB(contact);
    
    EXPECT_EQ(contactAabb, intersectingAabb);
}
