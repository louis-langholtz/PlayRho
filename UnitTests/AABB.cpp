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
#include <type_traits>

using namespace box2d;

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
    EXPECT_EQ(aabb.GetLowerBound(), lb);
    EXPECT_EQ(aabb.GetUpperBound(), ub);
}

TEST(AABB, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<AABB>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<AABB>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<AABB>::value);

    EXPECT_TRUE(std::is_constructible<AABB>::value);
    EXPECT_TRUE(std::is_nothrow_constructible<AABB>::value);
    EXPECT_FALSE(std::is_trivially_constructible<AABB>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<AABB>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<AABB>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<AABB>::value);

    EXPECT_TRUE(std::is_copy_assignable<AABB>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<AABB>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<AABB>::value);

    EXPECT_TRUE(std::is_destructible<AABB>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<AABB>::value);
    EXPECT_TRUE(std::is_trivially_destructible<AABB>::value);
}

TEST(AABB, DefaultAabbAddsToOther)
{
    const auto default_aabb = AABB{};
    {
        const auto other_aabb = AABB{Length2D(0, 0), Length2D(0, 0)};
        const auto sum_aabb = GetEnclosingAABB(default_aabb, other_aabb);
        EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
        EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
    }
    {
        const auto other_aabb = AABB{Length2D(0, 0), Length2D(0, 0)};
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
        EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
    }
    {
        const auto other_aabb = AABB{
            Length2D{Real( -1) * Meter, Real(-2) * Meter},
            Length2D{Real(+99) * Meter, Real(+3) * Meter}
        };
        const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
        EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
        EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
    }
}

TEST(AABB, DefaultAabbIncrementsToOther)
{
    {
        auto default_aabb = AABB{};
        const auto other_aabb = AABB{Length2D(0, 0), Length2D(0, 0)};
        default_aabb.Include(other_aabb);
        EXPECT_EQ(default_aabb.GetLowerBound(), other_aabb.GetLowerBound());
        EXPECT_EQ(default_aabb.GetUpperBound(), other_aabb.GetUpperBound());
    }
    {
        auto default_aabb = AABB{};
        const auto other_aabb = AABB{
            Length2D{Real(-1) * Meter, Real(-2) * Meter},
            Length2D{Real(+99) * Meter, Real(+3) * Meter}
        };
        default_aabb.Include(other_aabb);
        EXPECT_EQ(default_aabb.GetLowerBound(), other_aabb.GetLowerBound());
        EXPECT_EQ(default_aabb.GetUpperBound(), other_aabb.GetUpperBound());
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
        EXPECT_EQ(GetCenter(foo).x, center_x);
        EXPECT_EQ(GetCenter(foo).y, center_y);
        EXPECT_EQ(foo.GetLowerBound().x, lower_x);
        EXPECT_EQ(foo.GetLowerBound().y, lower_y);
        EXPECT_EQ(foo.GetUpperBound().x, upper_x);
        EXPECT_EQ(foo.GetUpperBound().y, upper_y);
    }
    {
        AABB foo{v1, v0};
        EXPECT_EQ(GetCenter(foo).x, center_x);
        EXPECT_EQ(GetCenter(foo).y, center_y);
        EXPECT_EQ(foo.GetLowerBound().x, lower_x);
        EXPECT_EQ(foo.GetLowerBound().y, lower_y);
        EXPECT_EQ(foo.GetUpperBound().x, upper_x);
        EXPECT_EQ(foo.GetUpperBound().y, upper_y);
    }
}

TEST(AABB, Include)
{
    const auto p1 = Length2D{Real{2} * Meter, Real{3} * Meter};
    const auto p2 = Length2D{Real{20} * Meter, Real{30} * Meter};
    const auto p3 = Length2D{Real{-3} * Meter, Real{-4} * Meter};
    const auto p4 = Length2D{Real{0} * Meter, Real{0} * Meter};
    const auto p5 = AABB{};

    auto foo = AABB{};
    
    foo.Include(p1);
    EXPECT_EQ(foo.GetLowerBound(), p1);
    EXPECT_EQ(foo.GetUpperBound(), p1);
    
    foo.Include(p2);
    EXPECT_EQ(foo.GetLowerBound(), p1);
    EXPECT_EQ(foo.GetUpperBound(), p2);
    
    foo.Include(p3);
    EXPECT_EQ(foo.GetLowerBound(), p3);
    EXPECT_EQ(foo.GetUpperBound(), p2);
    
    foo.Include(p4);
    EXPECT_EQ(foo.GetLowerBound(), p3);
    EXPECT_EQ(foo.GetUpperBound(), p2);
    
    foo.Include(p5);
    EXPECT_EQ(foo.GetLowerBound(), p3);
    EXPECT_EQ(foo.GetUpperBound(), p2);
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
    {
        auto aabb = AABB{};
        EXPECT_EQ(aabb.Move(Length2D(0, 0)), AABB{});
        EXPECT_EQ(aabb.Move(Length2D(Real(10) * Meter, Real(-4) * Meter)), AABB{});
    }
    {
        auto aabb = AABB{Length2D(0, 0)};
        EXPECT_EQ(aabb.Move(Length2D(0, 0)), AABB{Length2D(0, 0)});
    }
    {
        auto aabb = AABB{Length2D(0, 0)};
        EXPECT_EQ(aabb.Move(Length2D(Real(1) * Meter, Real(1) * Meter)),
                  AABB{Length2D(Real(1) * Meter, Real(1) * Meter)});
        EXPECT_EQ(aabb.Move(Length2D(Real(-1) * Meter, Real(-1) * Meter)),
                  AABB{Length2D(0, 0)});
        EXPECT_EQ(aabb.Move(Length2D(Real(-10) * Meter, Real(11) * Meter)),
                  AABB{Length2D(Real(-10) * Meter, Real(11) * Meter)});
    }
    {
        const auto lower = Length2D(Real(-1) * Meter, Real(-1) * Meter);
        const auto upper = Length2D(Real(+3) * Meter, Real(+9) * Meter);
        auto aabb = AABB{lower, upper};
        const auto moveby = Length2D(Real(1) * Meter, Real(1) * Meter);
        EXPECT_EQ(aabb.Move(moveby), AABB(lower + moveby, upper + moveby));
    }
}
