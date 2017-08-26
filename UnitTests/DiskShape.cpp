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
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <PlayRho/Collision/AABB.hpp>

using namespace playrho;

TEST(DiskShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(DiskShape), std::size_t(32)); break;
        case  8: EXPECT_EQ(sizeof(DiskShape), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(DiskShape), std::size_t(112)); break;
        default: FAIL(); break;
    }
}

TEST(DiskShape, DefaultConstruction)
{
    DiskShape foo{};
    
    EXPECT_EQ(typeid(foo), typeid(DiskShape));
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_EQ(foo.GetRadius(), DiskShape::GetDefaultRadius());
    EXPECT_EQ(GetX(foo.GetLocation()), Length{0});
    EXPECT_EQ(GetY(foo.GetLocation()), Length{0});
}

TEST(DiskShape, InitConstruction)
{
    const auto radius = Real(1) * Meter;
    const auto position = Length2D{-Real(1) * Meter, Real(1) * Meter};
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = radius;
    conf.location = position;
    DiskShape foo{conf};
    
    EXPECT_EQ(typeid(foo), typeid(DiskShape));
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_EQ(foo.GetRadius(), radius);
    EXPECT_EQ(GetX(foo.GetLocation()), GetX(position));
    EXPECT_EQ(GetY(foo.GetLocation()), GetY(position));
}

TEST(DiskShape, GetInvalidChildThrows)
{
    DiskShape foo{};
    
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_NO_THROW(foo.GetChild(0));
    EXPECT_THROW(foo.GetChild(1), InvalidArgument);
}

TEST(DiskShape, Accept)
{
    class Visitor: public Shape::Visitor
    {
    public:
        void Visit(const DiskShape&) override
        {
            visited = true;
        }
        bool visited = false;
    };

    DiskShape foo{};
    Visitor v;
    ASSERT_FALSE(v.visited);
    ASSERT_FALSE(v.IsBaseVisited());
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
    EXPECT_FALSE(v.IsBaseVisited());
}

TEST(DiskShape, BaseVisitorForDiskShape)
{
    const auto shape = DiskShape{Real{2} * Meter};
    auto visitor = Shape::Visitor{};
    ASSERT_FALSE(visitor.IsBaseVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsBaseVisited());
}

TEST(DiskShape, TestPoint)
{
    const auto radius = Real(1) * Meter;
    const auto position = Length2D{};
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = radius;
    conf.location = position;
    DiskShape foo{conf};
    EXPECT_TRUE(TestPoint(foo, Length2D{ Real(0) * Meter,  Real(0) * Meter}));
    EXPECT_TRUE(TestPoint(foo, Length2D{+Real(1) * Meter,  Real(0) * Meter}));
    EXPECT_TRUE(TestPoint(foo, Length2D{ Real(0) * Meter, +Real(1) * Meter}));
    EXPECT_TRUE(TestPoint(foo, Length2D{ Real(0) * Meter, -Real(1) * Meter}));
    EXPECT_TRUE(TestPoint(foo, Length2D{-Real(1) * Meter,  Real(0) * Meter}));
    EXPECT_FALSE(TestPoint(foo, Length2D{-Real(1) * Meter,  -Real(1) * Meter}));
    EXPECT_FALSE(TestPoint(foo, Length2D{+Real(1) * Meter,  +Real(1) * Meter}));
    EXPECT_FALSE(TestPoint(foo, Length2D{+Real(0.9) * Meter,  +Real(0.9) * Meter}));
}

TEST(DiskShape, ComputeAABB)
{
    const auto radius = Real(2.4) * Meter;
    const auto position = Length2D{Real(2) * Meter, Real(1) * Meter};
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = radius;
    conf.location = position;
    DiskShape foo{conf};
    const auto aabb = ComputeAABB(foo, Transform_identity);
    EXPECT_EQ(GetX(aabb.GetLowerBound()), GetX(position) - radius);
    EXPECT_EQ(GetY(aabb.GetLowerBound()), GetY(position) - radius);
    EXPECT_EQ(GetX(aabb.GetUpperBound()), GetX(position) + radius);
    EXPECT_EQ(GetY(aabb.GetUpperBound()), GetY(position) + radius);
    EXPECT_TRUE(AlmostEqual(StripUnit(GetX(GetExtents(aabb))), StripUnit(radius)));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetY(GetExtents(aabb))), StripUnit(radius)));
    EXPECT_EQ(GetX(GetCenter(aabb)), GetX(position));
    EXPECT_EQ(GetY(GetCenter(aabb)), GetY(position));
}
