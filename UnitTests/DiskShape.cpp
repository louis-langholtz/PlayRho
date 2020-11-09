/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(DiskShapeConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
            EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(24));
            break;
        case  8: EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(48)); break;
        case 16: EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(96)); break;
        default: FAIL(); break;
    }
}

TEST(DiskShapeConf, DefaultConstruction)
{
    const auto foo = DiskShapeConf{};
    
    EXPECT_EQ(GetTypeID(foo), GetTypeID<DiskShapeConf>());
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(foo.GetRadius(), DiskShapeConf::GetDefaultRadius());
    EXPECT_EQ(GetX(foo.GetLocation()), 0_m);
    EXPECT_EQ(GetY(foo.GetLocation()), 0_m);
}

TEST(DiskShapeConf, InitConstruction)
{
    const auto radius = 1_m;
    const auto position = Length2{-1_m, 1_m};
    auto conf = DiskShapeConf{};
    conf.vertexRadius = radius;
    conf.location = position;
    Shape foo{conf};
    
    EXPECT_EQ(GetTypeID(foo), GetTypeID<Shape>());
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(GetVertexRadius(foo, 0), radius);
    EXPECT_EQ(GetX(conf.GetLocation()), GetX(position));
    EXPECT_EQ(GetY(conf.GetLocation()), GetY(position));
}

TEST(DiskShapeConf, TransformFF)
{
    {
        auto foo = DiskShapeConf{};
        auto tmp = foo;
        Transform(foo, Mat22{});
        EXPECT_EQ(foo, tmp);
    }
    {
        auto foo = DiskShapeConf{};
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>());
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        auto foo = DiskShapeConf{}.UseLocation(v1).UseRadius(1_m);
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>() * 2);
        EXPECT_NE(foo, tmp);
        EXPECT_EQ(foo.GetLocation(), v1 * 2);
    }
}

TEST(DiskShapeConf, GetInvalidChildThrows)
{
    Shape foo{DiskShapeConf{}};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_NO_THROW(GetChild(foo, 0));
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(DiskShapeConf, TypeInfo)
{
    const auto foo = DiskShapeConf{};
    const auto shape = Shape(foo);
    EXPECT_EQ(GetType(shape), GetTypeID<DiskShapeConf>());
    auto copy = DiskShapeConf{};
    EXPECT_TRUE(TypeCast<DiskShapeConf>(&shape) != nullptr);
    EXPECT_NO_THROW(copy = TypeCast<DiskShapeConf>(shape));
    EXPECT_THROW(TypeCast<int>(shape), std::bad_cast);
}

TEST(DiskShapeConf, TestPoint)
{
    const auto radius = 1_m;
    const auto position = Length2{};
    auto conf = DiskShapeConf{};
    conf.vertexRadius = radius;
    conf.location = position;
    Shape foo{conf};
    EXPECT_TRUE(TestPoint(foo, Length2{ 0_m,  0_m}));
    EXPECT_TRUE(TestPoint(foo, Length2{+1_m,  0_m}));
    EXPECT_TRUE(TestPoint(foo, Length2{ 0_m, +1_m}));
    EXPECT_TRUE(TestPoint(foo, Length2{ 0_m, -1_m}));
    EXPECT_TRUE(TestPoint(foo, Length2{-1_m,  0_m}));
    EXPECT_FALSE(TestPoint(foo, Length2{-1_m,  -1_m}));
    EXPECT_FALSE(TestPoint(foo, Length2{+1_m,  +1_m}));
    EXPECT_FALSE(TestPoint(foo, Length2{+0.9_m,  +0.9_m}));
}

TEST(DiskShapeConf, ComputeAABB)
{
    const auto radius = 2.4_m;
    const auto position = Length2{2_m, 1_m};
    const auto conf = DiskShapeConf{}.UseRadius(radius).UseLocation(position);
    const auto shape = Shape{conf};
    ASSERT_EQ(GetChildCount(shape), static_cast<decltype(GetChildCount(shape))>(1));
    const auto aabb = ComputeAABB(shape, Transform_identity);
    EXPECT_EQ(GetX(GetLowerBound(aabb)), GetX(position) - radius);
    EXPECT_EQ(GetY(GetLowerBound(aabb)), GetY(position) - radius);
    EXPECT_EQ(GetX(GetUpperBound(aabb)), GetX(position) + radius);
    EXPECT_EQ(GetY(GetUpperBound(aabb)), GetY(position) + radius);
    EXPECT_NEAR(static_cast<double>(Real{GetX(GetExtents(aabb))/1_m}), static_cast<double>(Real{radius/1_m}), 1.0/1000000);
    EXPECT_NEAR(static_cast<double>(Real{GetY(GetExtents(aabb))/1_m}), static_cast<double>(Real{radius/1_m}), 1.0/1000000);
    EXPECT_TRUE(AlmostEqual(StripUnit(GetX(GetExtents(aabb))), StripUnit(radius)));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetY(GetExtents(aabb))), StripUnit(radius)));
    EXPECT_EQ(GetX(GetCenter(aabb)), GetX(position));
    EXPECT_EQ(GetY(GetCenter(aabb)), GetY(position));
}

TEST(DiskShapeConf, Equality)
{
    EXPECT_TRUE(DiskShapeConf() == DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseRadius(10_m) == DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseRadius(10_m) == DiskShapeConf().UseRadius(10_m));
    EXPECT_FALSE(DiskShapeConf().UseLocation(Length2(1_m, 2_m)) == DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseLocation(Length2(1_m, 2_m)) == DiskShapeConf().UseLocation(Length2(1_m, 2_m)));
    
    EXPECT_FALSE(DiskShapeConf().UseRadius(10_m) == DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseRadius(10_m) == DiskShapeConf().UseRadius(10_m));
    
    EXPECT_FALSE(DiskShapeConf().UseDensity(10_kgpm2) == DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseDensity(10_kgpm2) == DiskShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_FALSE(DiskShapeConf().UseFriction(Real(10)) == DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseFriction(Real(10)) == DiskShapeConf().UseFriction(Real(10)));
    
    EXPECT_FALSE(DiskShapeConf().UseRestitution(Real(10)) == DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseRestitution(Real(10)) == DiskShapeConf().UseRestitution(Real(10)));
}

TEST(DiskShapeConf, Inequality)
{
    EXPECT_FALSE(DiskShapeConf() != DiskShapeConf());
    EXPECT_TRUE(DiskShapeConf().UseRadius(10_m) != DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseRadius(10_m) != DiskShapeConf().UseRadius(10_m));
    EXPECT_TRUE(DiskShapeConf().UseLocation(Length2(1_m, 2_m)) != DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseLocation(Length2(1_m, 2_m)) != DiskShapeConf().UseLocation(Length2(1_m, 2_m)));
    
    EXPECT_TRUE(DiskShapeConf().UseRadius(10_m) != DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseRadius(10_m) != DiskShapeConf().UseRadius(10_m));
    
    EXPECT_TRUE(DiskShapeConf().UseDensity(10_kgpm2) != DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseDensity(10_kgpm2) != DiskShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_TRUE(DiskShapeConf().UseFriction(Real(10)) != DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseFriction(Real(10)) != DiskShapeConf().UseFriction(Real(10)));
    
    EXPECT_TRUE(DiskShapeConf().UseRestitution(Real(10)) != DiskShapeConf());
    EXPECT_FALSE(DiskShapeConf().UseRestitution(Real(10)) != DiskShapeConf().UseRestitution(Real(10)));
}
