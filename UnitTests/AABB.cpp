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

TEST(AABB, ByteSizeIsTwiceVec2)
{
    EXPECT_EQ(sizeof(AABB), sizeof(Vec2) * 2);
}

TEST(AABB, DefaultConstruction)
{
    const auto infinity = std::numeric_limits<Real>::infinity();
    const auto lb = Vec2{infinity, infinity} * (Real(1) * Meter);
    const auto ub = Vec2{-infinity, -infinity} * (Real(1) * Meter);
    const auto aabb = AABB{};
    EXPECT_EQ(GetLowerBound(aabb), lb);
    EXPECT_EQ(GetUpperBound(aabb), ub);
}

TEST(AABB, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<AABB>::value);
    //EXPECT_TRUE(std::is_nothrow_default_constructible<AABB>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<AABB>::value);

    EXPECT_TRUE((std::is_constructible<AABB, Length2D>::value));
    //EXPECT_FALSE((std::is_nothrow_constructible<AABB, Length2D>::value));
    EXPECT_FALSE((std::is_trivially_constructible<AABB, Length2D>::value));
    
    EXPECT_TRUE((std::is_constructible<AABB, Length2D, Length2D>::value));
    //EXPECT_FALSE((std::is_nothrow_constructible<AABB, Length2D, Length2D>::value));
    EXPECT_FALSE((std::is_trivially_constructible<AABB, Length2D, Length2D>::value));
    
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
        const auto other_aabb = AABB{Length2D{}, Length2D{}};
        const auto sum_aabb = GetEnclosingAABB(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
    {
        const auto other_aabb = AABB{Length2D{}, Length2D{}};
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
    {
        const auto other_aabb = AABB{
            Length2D{Real( -1) * Meter, Real(-2) * Meter},
            Length2D{Real(+99) * Meter, Real(+3) * Meter}
        };
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(GetLowerBound(sum_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(sum_aabb), GetUpperBound(other_aabb));
    }
}

TEST(AABB, DefaultAabbIncrementsToOther)
{
    {
        auto default_aabb = AABB{};
        const auto other_aabb = AABB{Length2D{}, Length2D{}};
        Include(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(default_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(default_aabb), GetUpperBound(other_aabb));
    }
    {
        auto default_aabb = AABB{};
        const auto other_aabb = AABB{
            Length2D{Real(-1) * Meter, Real(-2) * Meter},
            Length2D{Real(+99) * Meter, Real(+3) * Meter}
        };
        Include(default_aabb, other_aabb);
        EXPECT_EQ(GetLowerBound(default_aabb), GetLowerBound(other_aabb));
        EXPECT_EQ(GetUpperBound(default_aabb), GetUpperBound(other_aabb));
    }
}

TEST(AABB, InitializingConstruction)
{
    const auto lower_x = Real(-2) * Meter;
    const auto lower_y = Real(-3) * Meter;
    const auto upper_x = Real(+1.6) * Meter;
    const auto upper_y = Real(+1.9) * Meter;
    
    const auto center_x = (lower_x + upper_x) / Real{2};
    const auto center_y = (lower_y + upper_y) / Real{2};

    const auto v0 = Length2D{upper_x, lower_y};
    const auto v1 = Length2D{lower_x, upper_y};
    
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
        const auto pa = Length2D{GetInvalid<Length>(), GetInvalid<Length>()};
        const auto pb = Length2D{GetInvalid<Length>(), GetInvalid<Length>()};
        AABB foo{pa, pb};
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2D{GetInvalid<Length>(), GetInvalid<Length>()};
        const auto pb = Length2D{GetInvalid<Length>(), 0 * Meter};
        AABB foo{pa, pb};
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_FALSE(std::isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2D{GetInvalid<Length>(), 0 * Meter};
        const auto pb = Length2D{GetInvalid<Length>(), GetInvalid<Length>()};
        AABB foo{pa, pb};
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_FALSE(std::isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto pa = Length2D{GetInvalid<Length>(), 0 * Meter};
        const auto pb = Length2D{GetInvalid<Length>(), 0 * Meter};
        AABB foo{pa, pb};
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetLowerBound(foo)))));
        EXPECT_FALSE(std::isnan(StripUnit(GetY(GetLowerBound(foo)))));
        EXPECT_TRUE(std::isnan(StripUnit(GetX(GetUpperBound(foo)))));
        EXPECT_FALSE(std::isnan(StripUnit(GetY(GetUpperBound(foo)))));
    }
    {
        const auto rangeX = Interval<Length>{-2 * Meter, +3 * Meter};
        const auto rangeY = Interval<Length>{-8 * Meter, -4 * Meter};
        AABB foo{rangeX, rangeY};
        EXPECT_EQ(foo.rangeX, rangeX);
        EXPECT_EQ(foo.rangeY, rangeY);
    }
}

TEST(AABB, Swappable)
{
    auto a = AABB{};
    auto b = AABB{};
    ASSERT_EQ(a, b);
    std::swap(a, b);
    EXPECT_EQ(a, b);
    const auto aBefore = a;
    Include(a, Length2D{2 * Meter, 3 * Meter});
    const auto aAfter = a;
    ASSERT_NE(a, b);
    std::swap(a, b);
    EXPECT_EQ(a, aBefore);
    EXPECT_EQ(b, aAfter);
}

TEST(AABB, GetPerimeterOfPoint)
{
    EXPECT_EQ(GetPerimeter(AABB{Length2D{}}), Real(0) * Meter);
    EXPECT_EQ(GetPerimeter(AABB{Length2D{Real(-1) * Meter, Real(-2) * Meter}}), Real(0) * Meter);
    EXPECT_EQ(GetPerimeter(AABB{Length2D{Real(+99) * Meter, Real(+3) * Meter}}), Real(0) * Meter);
    EXPECT_TRUE(std::isnan(StripUnit(GetPerimeter(AABB{
        Length2D{
            Real(+std::numeric_limits<Real>::infinity()) * Meter,
            Real(+std::numeric_limits<Real>::infinity()) * Meter
        }
    }))));
}

TEST(AABB, Include)
{
    const auto p1 = Length2D{Real{2} * Meter, Real{3} * Meter};
    const auto p2 = Length2D{Real{20} * Meter, Real{30} * Meter};
    const auto p3 = Length2D{Real{-3} * Meter, Real{-4} * Meter};
    const auto p4 = Length2D{Real{0} * Meter, Real{0} * Meter};
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
    EXPECT_TRUE(Contains(AABB{Length2D{}}, AABB{Length2D{}}));
    EXPECT_TRUE((Contains(AABB{Length2D{}, Length2D{}}, AABB{Length2D{}})));
    EXPECT_TRUE((Contains(AABB{Length2D{}}, AABB{Length2D{}, Length2D{}})));
    EXPECT_TRUE((Contains(AABB{Length2D{1 * Meter, 2 * Meter}}, AABB{})));
    EXPECT_FALSE(Contains(GetInvalid<AABB>(), GetInvalid<AABB>()));
    EXPECT_FALSE(Contains(GetInvalid<AABB>(), AABB{}));
    EXPECT_FALSE(Contains(AABB{}, GetInvalid<AABB>()));
}

TEST(AABB, TestOverlap)
{
    {
        AABB bb1{
            Length2D{Real(-2) * Meter, Real(-3) * Meter},
            Length2D{Real(-1) * Meter, Real( 0) * Meter}
        };
        EXPECT_TRUE(TestOverlap(bb1, bb1));
    }
    {
        const auto vec = Length2D{Real(-2) * Meter, Real(-3) * Meter};
        AABB bb1{vec, vec};
        EXPECT_TRUE(TestOverlap(bb1, bb1));
    }
    {
        AABB bb1{
            Length2D{Real(-2) * Meter, Real(-3) * Meter},
            Length2D{Real(-1) * Meter, Real( 0) * Meter}
        };
        AABB bb2{
            Length2D{Real(-1) * Meter, Real(-1) * Meter},
            Length2D{Real( 1) * Meter, Real( 2) * Meter}
        };
        EXPECT_TRUE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{
            Length2D{Real(-99) * Meter, Real(-3) * Meter},
            Length2D{Real( -1) * Meter, Real( 0) * Meter}
        };
        AABB bb2{
            Length2D{Real(76) * Meter, Real(-1) * Meter},
            Length2D{Real(-2) * Meter, Real( 2) * Meter}
        };
        EXPECT_TRUE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{
            Length2D{Real(-20) * Meter, Real(-3) * Meter},
            Length2D{Real(-18) * Meter, Real( 0) * Meter}
        };
        AABB bb2{
            Length2D{Real(-1) * Meter, Real(-1) * Meter},
            Length2D{Real( 1) * Meter, Real( 2) * Meter}
        };
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{
            Length2D{Real(-2) * Meter, Real(-3) * Meter},
            Length2D{Real(-1) * Meter, Real( 0) * Meter}
        };
        AABB bb2{
            Length2D{Real(-1) * Meter, Real(+1) * Meter},
            Length2D{Real( 1) * Meter, Real( 2) * Meter}
        };
        EXPECT_FALSE(TestOverlap(bb1, bb2));
    }
    {
        AABB bb1{
            Length2D{Real(-2) * Meter, Real(+3) * Meter},
            Length2D{Real(-1) * Meter, Real( 0) * Meter}
        };
        AABB bb2{
            Length2D{Real(-1) * Meter, Real(-1) * Meter},
            Length2D{Real( 0) * Meter, Real(-2) * Meter}
        };
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
    const auto zeroLoc = Length2D{};
    const auto zeroAabb = AABB{zeroLoc};
    {
        auto aabb = AABB{};
        EXPECT_EQ(Move(aabb, zeroLoc), AABB{});
        EXPECT_EQ(Move(aabb, Length2D{Real(10) * Meter, Real(-4) * Meter}), AABB{});
    }
    {
        auto aabb = AABB{Length2D{}};
        EXPECT_EQ(Move(aabb, Length2D{}), zeroAabb);
    }
    {
        const auto aabb1 = AABB{Length2D{Real(1) * Meter, Real(1) * Meter}};
        const auto aabb2 = AABB{Length2D{Real(-10) * Meter, Real(11) * Meter}};
        auto aabb = zeroAabb;
        EXPECT_EQ(Move(aabb, Length2D{Real(1) * Meter, Real(1) * Meter}), aabb1);
        EXPECT_EQ(Move(aabb, Length2D{Real(-1) * Meter, Real(-1) * Meter}), zeroAabb);
        EXPECT_EQ(Move(aabb, Length2D{Real(-10) * Meter, Real(11) * Meter}), aabb2);
    }
    {
        const auto lower = Length2D{Real(-1) * Meter, Real(-1) * Meter};
        const auto upper = Length2D{Real(+3) * Meter, Real(+9) * Meter};
        auto aabb = AABB{lower, upper};
        const auto moveby = Length2D{Real(1) * Meter, Real(1) * Meter};
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
    
    const auto vr0 = Interval<Length>{1 * Meter, 2 * Meter};
    const auto vr1 = Interval<Length>{3 * Meter, 4 * Meter};
    const auto vr2 = Interval<Length>{5 * Meter, 6 * Meter};
    const auto vr3 = Interval<Length>{7 * Meter, 8 * Meter};

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
    const auto rangeX = Interval<Length>{-2 * Meter, +3 * Meter};
    const auto rangeY = Interval<Length>{-8 * Meter, -4 * Meter};
    AABB foo{rangeX, rangeY};
    ASSERT_EQ(foo.rangeX, rangeX);
    ASSERT_EQ(foo.rangeY, rangeY);
    
    std::stringstream aabbStream;
    ASSERT_TRUE(aabbStream.str().empty());
    aabbStream << foo;
    ASSERT_FALSE(aabbStream.str().empty());
    
    std::stringstream xRangeStream;
    xRangeStream << foo.rangeX;
    
    std::stringstream yRangeStream;
    yRangeStream << foo.rangeY;
    
    std::string comp;
    comp += '{';
    comp += xRangeStream.str();
    comp += ',';
    comp += yRangeStream.str();
    comp += '}';
    EXPECT_STREQ(aabbStream.str().c_str(), comp.c_str());
}
