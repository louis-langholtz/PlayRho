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

TEST(Manifold, PointEqualsFreeFunction)
{
    const auto localPoint1 = Length2D{Real(1) * Meter, Real(2) * Meter};
    const auto localPoint2 = Length2D{Real(3) * Meter, Real(-1) * Meter};
    const auto cf1 = ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_vertex, 2};
    const auto cf2 = ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 1};
    const auto normalImpulse1 = Momentum{Real(1) * Kilogram * MeterPerSecond};
    const auto normalImpulse2 = Momentum{Real(9) * Kilogram * MeterPerSecond};
    const auto tangentImpulse1 = Momentum{Real(2) * Kilogram * MeterPerSecond};
    const auto tangentImpulse2 = Momentum{Real(1.1) * Kilogram * MeterPerSecond};
    const auto mp1 = Manifold::Point{localPoint1, cf1, normalImpulse1, tangentImpulse1};
    const auto mp2 = Manifold::Point{localPoint1, cf1, normalImpulse1, tangentImpulse1};
    const auto mp3 = Manifold::Point{localPoint2, cf1, normalImpulse1, tangentImpulse1};
    const auto mp4 = Manifold::Point{localPoint1, cf2, normalImpulse1, tangentImpulse1};
    const auto mp5 = Manifold::Point{localPoint1, cf1, normalImpulse2, tangentImpulse1};
    const auto mp6 = Manifold::Point{localPoint1, cf1, normalImpulse1, tangentImpulse2};
    
    EXPECT_TRUE(mp1 == mp1);
    EXPECT_TRUE(mp1 == mp2);
    EXPECT_FALSE(mp1 == mp3);
    EXPECT_FALSE(mp1 == mp4);
    EXPECT_FALSE(mp1 == mp5);
    EXPECT_FALSE(mp1 == mp6);
}

TEST(Manifold, EqualsFreeFunction)
{
    const auto ln0 = UnitVec2::GetLeft();
    const auto ln1 = UnitVec2::GetRight();
    const auto lp0 = Length2D(Real(0) * Meter, Real(0) * Meter);
    const auto lp1 = Length2D(Real(1) * Meter, Real(1) * Meter);
    const auto foo = Manifold::GetForFaceB(ln0, lp0);
    const auto boo = Manifold::GetForFaceA(ln0, lp0);
    const auto poo = Manifold::GetForFaceA(ln0, lp0);
    const auto goo = Manifold::GetForFaceA(ln0, lp1);
    const auto too = Manifold::GetForFaceA(ln1, lp0);
    //const auto nottoo = Manifold::GetForFaceA(UnitVec2{}, lp0);
    const auto localPoint1 = Length2D{Real(1) * Meter, Real(2) * Meter};
    const auto cf1 = ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_vertex, 2};
    const auto cf2 = ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 1};
    const auto normalImpulse1 = Momentum{Real(1) * Kilogram * MeterPerSecond};
    const auto tangentImpulse1 = Momentum{Real(2) * Kilogram * MeterPerSecond};
    const auto mp0 = Manifold::Point{localPoint1, cf1, normalImpulse1, tangentImpulse1};
    const auto mp1 = Manifold::Point{localPoint1, cf2, normalImpulse1, tangentImpulse1};
    const auto faceA000 = Manifold::GetForFaceA(ln0, lp0, mp0);
    const auto faceA0001 = Manifold::GetForFaceA(ln0, lp0, mp0, mp1);
    const auto faceA001 = Manifold::GetForFaceA(ln0, lp0, mp1);
    const auto faceA0010 = Manifold::GetForFaceA(ln0, lp0, mp1, mp0);
    const auto faceA0011 = Manifold::GetForFaceA(ln0, lp0, mp1, mp1);
    EXPECT_TRUE(Manifold{} == Manifold{});
    EXPECT_TRUE(foo == foo);
    EXPECT_TRUE(boo == boo);
    EXPECT_TRUE(boo == poo);
    EXPECT_TRUE(too == too);
    EXPECT_FALSE(foo == Manifold{});
    EXPECT_FALSE(foo == boo);
    EXPECT_FALSE(poo == goo);
    EXPECT_FALSE(poo == too);
    //EXPECT_FALSE(too == nottoo);
    EXPECT_FALSE(faceA000 == foo);
    EXPECT_FALSE(faceA000 == faceA001);
    EXPECT_TRUE(faceA0001 == faceA0010);
    EXPECT_FALSE(faceA0010 == faceA0011);
}

TEST(Manifold, NotEqualsFreeFunction)
{
    const auto ln0 = UnitVec2::GetLeft();
    const auto ln1 = UnitVec2::GetRight();
    const auto lp0 = Length2D(Real(0) * Meter, Real(0) * Meter);
    const auto lp1 = Length2D(Real(1) * Meter, Real(1) * Meter);
    const auto foo = Manifold::GetForFaceB(ln0, lp0);
    const auto boo = Manifold::GetForFaceA(ln0, lp0);
    const auto poo = Manifold::GetForFaceA(ln0, lp0);
    const auto goo = Manifold::GetForFaceA(ln0, lp1);
    const auto too = Manifold::GetForFaceA(ln1, lp0);
    EXPECT_FALSE(Manifold{} != Manifold{});
    EXPECT_FALSE(foo != foo);
    EXPECT_FALSE(boo != boo);
    EXPECT_FALSE(boo != poo);
    EXPECT_FALSE(too != too);
    EXPECT_TRUE(foo != Manifold{});
    EXPECT_TRUE(foo != boo);
    EXPECT_TRUE(poo != goo);
    EXPECT_TRUE(poo != too);
}

TEST(Manifold, GetNameFreeFunction)
{
    EXPECT_STREQ(GetName(Manifold::e_unset), "unset");
    EXPECT_STREQ(GetName(Manifold::e_circles), "circles");
    EXPECT_STREQ(GetName(Manifold::e_faceA), "face-a");
    EXPECT_STREQ(GetName(Manifold::e_faceB), "face-b");
}
