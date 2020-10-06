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
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/Manifold.hpp>
#include <chrono>
#include <typeinfo>

using namespace playrho;
using namespace playrho::d2;

TEST(Shape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(Shape), std::size_t(8));
#else
            EXPECT_EQ(sizeof(Shape), std::size_t(16));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(Shape), std::size_t(16)); break;
        case 16: EXPECT_EQ(sizeof(Shape), std::size_t(16)); break;
        default: FAIL(); break;
    }
}

TEST(Shape, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<Shape>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<Shape>::value);
    
    // Construction with any 1 supporting argument should succeed...
    using X = DiskShapeConf;
    EXPECT_TRUE((std::is_constructible<Shape, X>::value));
    EXPECT_FALSE((std::is_nothrow_constructible<Shape, X>::value));
    EXPECT_FALSE((std::is_trivially_constructible<Shape, X>::value));

    // Construction with 2 arguments should fail...
    EXPECT_FALSE((std::is_constructible<Shape, X, X>::value));
    EXPECT_FALSE((std::is_nothrow_constructible<Shape, X, X>::value));
    EXPECT_FALSE((std::is_trivially_constructible<Shape, X, X>::value));
    
    EXPECT_TRUE(std::is_copy_constructible<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<Shape>::value);
    
    EXPECT_TRUE(std::is_move_constructible<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_move_constructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_move_constructible<Shape>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<Shape>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<Shape>::value);
    
    EXPECT_TRUE(std::is_move_assignable<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_move_assignable<Shape>::value);
    EXPECT_FALSE(std::is_trivially_move_assignable<Shape>::value);
    
    EXPECT_TRUE(std::is_destructible<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_destructible<Shape>::value);
}

TEST(Shape, DefaultConstruction)
{
    const auto s = Shape{};
    EXPECT_EQ(GetMassData(s), MassData());
    EXPECT_EQ(GetFriction(s), Real(0));
    EXPECT_EQ(GetRestitution(s), Real(0));
    EXPECT_EQ(GetDensity(s), 0_kgpm2);
    EXPECT_THROW(GetVertexRadius(s, 0), InvalidArgument);
    EXPECT_EQ(GetChildCount(s), ChildCounter(0));
    EXPECT_THROW(GetChild(s, 0), InvalidArgument);
    EXPECT_TRUE(s == s);
    auto t = Shape{};
    EXPECT_TRUE(s == t);
    EXPECT_NO_THROW(Transform(t, Mat22{}));
}

TEST(Shape, types)
{
    EXPECT_EQ(GetTypeID<DiskShapeConf>(), GetTypeID<DiskShapeConf>());

    const auto sc = DiskShapeConf{1_m};
    EXPECT_EQ(GetTypeID(sc), GetTypeID<DiskShapeConf>());
    EXPECT_EQ(GetTypeID<DiskShapeConf>(), GetTypeID(sc));
    EXPECT_EQ(GetTypeID(sc), GetTypeID(sc));
    EXPECT_NE(GetTypeID<DiskShapeConf>(), GetTypeID<EdgeShapeConf>());
    EXPECT_NE(GetTypeID(DiskShapeConf{}), GetTypeID(EdgeShapeConf{}));
    EXPECT_EQ(GetTypeID(DiskShapeConf{}), GetTypeID(DiskShapeConf{}));
    EXPECT_EQ(GetTypeID(EdgeShapeConf{}), GetTypeID(EdgeShapeConf{}));

    const auto s1 = Shape{sc};
    ASSERT_EQ(GetTypeID<Shape>(), GetTypeID(s1));
    const auto& st1 = GetUseTypeInfo(s1);
    ASSERT_NE(st1, GetTypeID<Shape>());
    EXPECT_EQ(st1, GetTypeID(sc));

    const auto s2 = Shape{s1}; // This should copy construct
    const auto& st2 = GetUseTypeInfo(s2);
    EXPECT_EQ(st2, GetTypeID(sc)); // Confirm s2 was a copy construction
}

TEST(Shape, TestOverlapSlowerThanCollideShapesForCircles)
{
    const auto shape = DiskShapeConf{2_m};
    const auto xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto child = GetChild(shape, 0);

    const auto maxloops = 1000000u;

    std::chrono::duration<double> elapsed_test_overlap;
    std::chrono::duration<double> elapsed_collide_shapes;

    for (auto attempt = 0u; attempt < 2u; ++attempt)
    {
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i)
            {
                if (TestOverlap(child, xfm, child, xfm) >= 0_m2)
                {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_test_overlap = end - start;
            ASSERT_EQ(count, maxloops);
        }
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i)
            {
                const auto manifold = CollideShapes(child, xfm, child, xfm);
                if (manifold.GetPointCount() > 0)
                {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_collide_shapes = end - start;
            ASSERT_EQ(count, maxloops);
        }
        
        EXPECT_GT(elapsed_test_overlap.count(), elapsed_collide_shapes.count());
    }
}

TEST(Shape, TestOverlapFasterThanCollideShapesForPolygons)
{
    const auto shape = PolygonShapeConf{2_m, 2_m};
    const auto xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto child = GetChild(shape, 0);

    const auto maxloops = 1000000u;
    
    std::chrono::duration<double> elapsed_test_overlap;
    std::chrono::duration<double> elapsed_collide_shapes;
    
    for (auto attempt = 0u; attempt < 2u; ++attempt)
    {
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i)
            {
                if (TestOverlap(child, xfm, child, xfm) >= 0_m2)
                {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_test_overlap = end - start;
            ASSERT_EQ(count, maxloops);
        }
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i)
            {
                const auto manifold = CollideShapes(child, xfm, child, xfm);
                if (manifold.GetPointCount() > 0)
                {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_collide_shapes = end - start;
            ASSERT_EQ(count, maxloops);
        }
        
        EXPECT_LT(elapsed_test_overlap.count(), elapsed_collide_shapes.count());
    }
}

TEST(Shape, Equality)
{
    EXPECT_TRUE(Shape(EdgeShapeConf()) == Shape(EdgeShapeConf()));

    const auto shapeA = Shape(DiskShapeConf{}.UseRadius(100_m));
    const auto shapeB = Shape(DiskShapeConf{}.UseRadius(100_m));
    EXPECT_TRUE(shapeA == shapeB);
    
    EXPECT_FALSE(Shape(DiskShapeConf()) == Shape(EdgeShapeConf()));
}

TEST(Shape, Inequality)
{
    EXPECT_FALSE(Shape(EdgeShapeConf()) != Shape(EdgeShapeConf()));
    
    const auto shapeA = Shape(DiskShapeConf{}.UseRadius(100_m));
    const auto shapeB = Shape(DiskShapeConf{}.UseRadius(100_m));
    EXPECT_FALSE(shapeA != shapeB);

    EXPECT_TRUE(Shape(DiskShapeConf()) != Shape(EdgeShapeConf()));
}

namespace {

struct X {};

ChildCounter GetChildCount(const X&) noexcept
{
    return ChildCounter{0};
}

DistanceProxy GetChild(const X&, ChildCounter)
{
    return DistanceProxy{};
}

MassData GetMassData(const X&) noexcept
{
    return MassData{};
}

NonNegative<Length> GetVertexRadius(const X&, ChildCounter) noexcept
{
    return 0_m;
}

NonNegative<AreaDensity> GetDensity(const X&) noexcept
{
    return 0_kgpm2;
}

Finite<Real> GetRestitution(const X&) noexcept
{
    return Real{0};
}

void Transform(X&, const Mat22&) noexcept
{
    // Intentionally empty.
}

NonNegative<Real> GetFriction(const X&) noexcept
{
    return Real{0};
}

bool operator== (const X&, const X&) noexcept
{
    return true;
}

#if 0
bool operator!= (const X& lhs, const X& rhs) noexcept
{
    return !(lhs == rhs);
}
#endif

} // anonymous namespace

TEST(Shape, NewConfig)
{
    const auto s = Shape{X{}};
    auto data = UnitTestsVisitorData{};
    EXPECT_FALSE(Visit(s, &data));
    EXPECT_EQ(data.visitedDisk, 0);
    EXPECT_EQ(data.visitedEdge, 0);
    EXPECT_EQ(data.visitedPolygon, 0);
    EXPECT_EQ(data.visitedChain, 0);
    EXPECT_EQ(data.visitedMulti, 0);
}
