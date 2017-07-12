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
#include <PlayRho/Collision/Manifold.hpp>

using namespace playrho;

TEST(Manifold, ByteSizeIs_60_120_or_240)
{
    switch (sizeof(Real))
    {
        case  4:  EXPECT_EQ(sizeof(Manifold), std::size_t(60)); break;
        case  8: EXPECT_EQ(sizeof(Manifold), std::size_t(120)); break;
        case 16: EXPECT_EQ(sizeof(Manifold), std::size_t(240)); break;
        default: FAIL(); break;
    }
}

TEST(Manifold, DefaultConstruction)
{
    const auto foo = Manifold{};
    EXPECT_EQ(foo.GetType(), Manifold::e_unset);
    EXPECT_EQ(foo.GetPointCount(), 0);
    EXPECT_FALSE(IsValid(foo.GetLocalNormal()));
    EXPECT_FALSE(IsValid(foo.GetLocalPoint()));
}

TEST(Manifold, PointInitializingConstructor)
{
    const auto lp = Length2D{Real(3) * Meter, Real(4) * Meter};
    const auto ni = Real(1.2) * Kilogram * MeterPerSecond;
    const auto ti = Real(2.4) * Kilogram * MeterPerSecond;
    const auto cf = ContactFeature{};
    const auto foo = Manifold::Point{lp, cf, ni, ti};
    EXPECT_EQ(foo.localPoint.x, lp.x);
    EXPECT_EQ(foo.localPoint.y, lp.y);
    EXPECT_EQ(foo.contactFeature, cf);
    EXPECT_EQ(foo.normalImpulse, ni);
    EXPECT_EQ(foo.tangentImpulse, ti);
}

TEST(Manifold, GetForCircles)
{
    const auto ctr = Length2D{Real(99) * Meter, Real(21) * Meter};
    const auto foo = Manifold::GetForCircles(ctr, 0, ctr, 0);
    EXPECT_EQ(foo.GetType(), Manifold::e_circles);
    EXPECT_EQ(foo.GetLocalPoint(), ctr);
    EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(1));
    EXPECT_FALSE(IsValid(foo.GetLocalNormal()));
    EXPECT_TRUE(foo == foo);
    EXPECT_FALSE(foo != foo);
}

TEST(Manifold, GetForFaceA)
{
    const auto ln = UnitVec2::GetLeft();
    const auto lp = Length2D(0, 0);
    {
        Manifold foo = Manifold::GetForFaceA(ln, lp);
        EXPECT_EQ(foo.GetType(), Manifold::e_faceA);
        EXPECT_EQ(foo.GetLocalNormal(), ln);
        EXPECT_EQ(foo.GetLocalPoint(), lp);
        EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(0));
    }
    {
        const auto pl = Length2D{Real(-0.12) * Meter, Real(0.34) * Meter};
        const auto cf = GetFaceFaceContactFeature(0, 0);
        const auto ni = Real(2.9) * Kilogram * MeterPerSecond;
        const auto ti = Real(.7) * Kilogram * MeterPerSecond;
        const auto foo = Manifold::GetForFaceA(ln, lp, Manifold::Point{pl, cf, ni, ti});
        EXPECT_EQ(foo.GetType(), Manifold::e_faceA);
        EXPECT_EQ(foo.GetLocalNormal(), ln);
        EXPECT_EQ(foo.GetLocalPoint(), lp);

        EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(1));
        const auto p0 = foo.GetPoint(0);
        EXPECT_EQ(p0.localPoint, pl);
        EXPECT_EQ(p0.contactFeature, cf);
        EXPECT_EQ(p0.normalImpulse, ni);
        EXPECT_EQ(p0.tangentImpulse, ti);
    }
    {
        const auto pl = Length2D{Real(-0.12) * Meter, Real(0.34) * Meter};
        const auto cf = GetFaceFaceContactFeature(0, 1);
        const auto ni = Real(2.9) * Kilogram * MeterPerSecond;
        const auto ti = Real(.7) * Kilogram * MeterPerSecond;
        const auto foo = Manifold::GetForFaceA(ln, lp, Manifold::Point{pl, cf, ni, ti}, Manifold::Point{-pl, Flip(cf), -ni, -ti});
        EXPECT_EQ(foo.GetType(), Manifold::e_faceA);
        EXPECT_EQ(foo.GetLocalNormal(), ln);
        EXPECT_EQ(foo.GetLocalPoint(), lp);

        EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(2));
        const auto p0 = foo.GetPoint(0);
        EXPECT_EQ(p0.localPoint, pl);
        EXPECT_EQ(p0.contactFeature, cf);
        EXPECT_EQ(p0.normalImpulse, ni);
        EXPECT_EQ(p0.tangentImpulse, ti);
        const auto p1 = foo.GetPoint(1);
        EXPECT_EQ(p1.localPoint, -pl);
        EXPECT_EQ(p1.contactFeature, Flip(cf));
        EXPECT_EQ(p1.normalImpulse, -ni);
        EXPECT_EQ(p1.tangentImpulse, -ti);
    }
}

TEST(Manifold, GetForFaceB)
{
    const auto ln = UnitVec2::GetLeft();
    const auto lp = Length2D(0, 0);
    {
        Manifold foo = Manifold::GetForFaceB(ln, lp);
        EXPECT_EQ(foo.GetType(), Manifold::e_faceB);
        EXPECT_EQ(foo.GetLocalNormal(), ln);
        EXPECT_EQ(foo.GetLocalPoint(), lp);
        EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(0));
    }
    {
        const auto pl = Length2D{Real(-0.12) * Meter, Real(0.34) * Meter};
        const auto cf = GetFaceFaceContactFeature(0, 0);
        const auto ni = Real(2.9) * Kilogram * MeterPerSecond;
        const auto ti = Real(.7) * Kilogram * MeterPerSecond;
        Manifold foo = Manifold::GetForFaceB(ln, lp, Manifold::Point{pl, cf, ni, ti});
        EXPECT_EQ(foo.GetType(), Manifold::e_faceB);
        EXPECT_EQ(foo.GetLocalNormal(), ln);
        EXPECT_EQ(foo.GetLocalPoint(), lp);

        EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(1));
        const auto p0 = foo.GetPoint(0);
        EXPECT_EQ(p0.localPoint, pl);
        EXPECT_EQ(p0.contactFeature, cf);
        EXPECT_EQ(p0.normalImpulse, ni);
        EXPECT_EQ(p0.tangentImpulse, ti);
    }
    {
        const auto pl = Length2D{Real(-0.12) * Meter, Real(0.34) * Meter};
        const auto cf = GetFaceFaceContactFeature(0, 1);
        const auto ni = Real(2.9) * Kilogram * MeterPerSecond;
        const auto ti = Real(.7) * Kilogram * MeterPerSecond;
        Manifold foo = Manifold::GetForFaceB(ln, lp, Manifold::Point{pl, cf, ni, ti}, Manifold::Point{-pl, Flip(cf), -ni, -ti});
        EXPECT_EQ(foo.GetType(), Manifold::e_faceB);
        EXPECT_EQ(foo.GetLocalNormal(), ln);
        EXPECT_EQ(foo.GetLocalPoint(), lp);
        
        EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(2));
        const auto p0 = foo.GetPoint(0);
        EXPECT_EQ(p0.localPoint, pl);
        EXPECT_EQ(p0.contactFeature, cf);
        EXPECT_EQ(p0.normalImpulse, ni);
        EXPECT_EQ(p0.tangentImpulse, ti);
        const auto p1 = foo.GetPoint(1);
        EXPECT_EQ(p1.localPoint, -pl);
        EXPECT_EQ(p1.contactFeature, Flip(cf));
        EXPECT_EQ(p1.normalImpulse, -ni);
        EXPECT_EQ(p1.tangentImpulse, -ti);
    }
}

TEST(Manifold, GetNameFreeFunction)
{
    EXPECT_STREQ(GetName(Manifold::e_unset), "unset");
    EXPECT_STREQ(GetName(Manifold::e_circles), "circles");
    EXPECT_STREQ(GetName(Manifold::e_faceA), "face-a");
    EXPECT_STREQ(GetName(Manifold::e_faceB), "face-b");
}
