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
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

using namespace playrho;

TEST(DiskShapeConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(28));
#else
            EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(24));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(DiskShapeConf), std::size_t(112)); break;
        default: FAIL(); break;
    }
}

TEST(DiskShapeConf, DefaultConstruction)
{
    const auto foo = DiskShapeConf{};
    
    EXPECT_EQ(typeid(foo), typeid(DiskShapeConf));
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
    
    EXPECT_EQ(typeid(foo), typeid(Shape));
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(GetVertexRadius(foo), radius);
    EXPECT_EQ(GetX(conf.GetLocation()), GetX(position));
    EXPECT_EQ(GetY(conf.GetLocation()), GetY(position));
}

TEST(DiskShapeConf, GetInvalidChildThrows)
{
    Shape foo{DiskShapeConf{}};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_NO_THROW(GetChild(foo, 0));
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(DiskShapeConf, Accept)
{
    auto visited = false;
    auto diskVisited = false;
    Shape foo{DiskShapeConf{}};
    ASSERT_FALSE(visited);
    ASSERT_FALSE(diskVisited);
    Accept(foo, [&](const std::type_info &ti, const void *){
        visited = true;
        if (ti == typeid(DiskShapeConf))
        {
            diskVisited = true;
        }
    });
    EXPECT_TRUE(visited);
    EXPECT_TRUE(diskVisited);
}

#if 0
TEST(DiskShapeConf, BaseVisitorForDiskShape)
{
    const auto shape = DiskShapeConf{}.UseRadius(2_m);
    auto visitor = IsVisitedShapeVisitor{};
    ASSERT_FALSE(visitor.IsVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsVisited());
}
#endif

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
    auto conf = DiskShapeConf{};
    conf.vertexRadius = radius;
    conf.location = position;
    Shape foo{conf};
    const auto aabb = ComputeAABB(foo, Transform_identity);
    EXPECT_EQ(GetX(GetLowerBound(aabb)), GetX(position) - radius);
    EXPECT_EQ(GetY(GetLowerBound(aabb)), GetY(position) - radius);
    EXPECT_EQ(GetX(GetUpperBound(aabb)), GetX(position) + radius);
    EXPECT_EQ(GetY(GetUpperBound(aabb)), GetY(position) + radius);
    EXPECT_TRUE(AlmostEqual(StripUnit(GetX(GetExtents(aabb))), StripUnit(radius)));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetY(GetExtents(aabb))), StripUnit(radius)));
    EXPECT_EQ(GetX(GetCenter(aabb)), GetX(position));
    EXPECT_EQ(GetY(GetCenter(aabb)), GetY(position));
}
