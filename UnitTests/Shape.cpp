/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/Manifold.hpp>
#include <chrono>

using namespace box2d;

TEST(Shape, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(Shape), size_t(24)); break;
        case  8: EXPECT_EQ(sizeof(Shape), size_t(40)); break;
        case 16: EXPECT_EQ(sizeof(Shape), size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(Shape, TestOverlapSlowerThanCollideShapesForCircles)
{
    const auto shape = DiskShape{RealNum{2} * Meter};
    const auto xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto child = shape.GetChild(0);

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
                if (TestOverlap(child, xfm, child, xfm) >= Area{0})
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
    const auto shape = PolygonShape{RealNum{2} * Meter, RealNum{2} * Meter};
    const auto xfm = Transformation{Vec2{0, 0} * Meter, UnitVec2{RealNum{0} * Degree}};
    const auto child = shape.GetChild(0);

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
                if (TestOverlap(child, xfm, child, xfm) >= Area{0})
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
