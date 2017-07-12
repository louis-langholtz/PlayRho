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
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <PlayRho/Collision/AABB.hpp>

using namespace playrho;

TEST(DiskShape, ByteSize)
{
    if (sizeof(Real) == 4)
    {
        EXPECT_EQ(sizeof(DiskShape), std::size_t(32));
    }
    else if (sizeof(Real) == 8)
    {
        EXPECT_EQ(sizeof(DiskShape), std::size_t(56));
    }
    else if (sizeof(Real) == 16)
    {
        EXPECT_EQ(sizeof(DiskShape), std::size_t(112));
    }
    else
    {
        FAIL();
    }
}

TEST(DiskShape, DefaultConstruction)
{
    DiskShape foo{};
    
    EXPECT_EQ(typeid(foo), typeid(DiskShape));
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_EQ(foo.GetRadius(), DiskShape::GetDefaultRadius());
    EXPECT_EQ(foo.GetLocation().x, Length{0});
    EXPECT_EQ(foo.GetLocation().y, Length{0});
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
    EXPECT_EQ(foo.GetLocation().x, position.x);
    EXPECT_EQ(foo.GetLocation().y, position.y);
}

TEST(DiskShape, TestPoint)
{
    const auto radius = Real(1) * Meter;
    const auto position = Length2D(0, 0);
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
    EXPECT_EQ(aabb.GetLowerBound().x, position.x - radius);
    EXPECT_EQ(aabb.GetLowerBound().y, position.y - radius);
    EXPECT_EQ(aabb.GetUpperBound().x, position.x + radius);
    EXPECT_EQ(aabb.GetUpperBound().y, position.y + radius);
    EXPECT_TRUE(almost_equal(StripUnit(GetExtents(aabb).x), StripUnit(radius)));
    EXPECT_TRUE(almost_equal(StripUnit(GetExtents(aabb).y), StripUnit(radius)));
    EXPECT_EQ(GetCenter(aabb).x, position.x);
    EXPECT_EQ(GetCenter(aabb).y, position.y);
}
