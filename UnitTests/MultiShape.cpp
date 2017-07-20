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
#include <PlayRho/Collision/Shapes/MultiShape.hpp>
#include <PlayRho/Common/VertexSet.hpp>
#include <array>

using namespace playrho;

TEST(MultiShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(MultiShape), std::size_t(56)); break;
        case  8: EXPECT_EQ(sizeof(MultiShape), std::size_t(80)); break;
        case 16: EXPECT_EQ(sizeof(MultiShape), std::size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(MultiShape, DefaultConstruction)
{
    MultiShape foo{};
    const auto defaultMassData = MassData{};
    const auto defaultConf = MultiShape::Conf{};
    
    EXPECT_EQ(typeid(foo), typeid(MultiShape));
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{0});
    EXPECT_EQ(foo.GetMassData(), defaultMassData);
    
    EXPECT_EQ(foo.GetVertexRadius(), MultiShape::GetDefaultVertexRadius());
    EXPECT_EQ(foo.GetDensity(), defaultConf.density);
    EXPECT_EQ(foo.GetFriction(), defaultConf.friction);
    EXPECT_EQ(foo.GetRestitution(), defaultConf.restitution);
}

TEST(MultiShape, GetInvalidChildThrows)
{
    MultiShape foo{};
    
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{0});
    EXPECT_THROW(foo.GetChild(0), InvalidArgument);
    EXPECT_THROW(foo.GetChild(1), InvalidArgument);
}

TEST(MultiShape, Accept)
{
    class Visitor: public Shape::Visitor
    {
    public:
        void Visit(const MultiShape&) override
        {
            visited = true;
        }
        bool visited = false;
    };
    
    MultiShape foo{};
    Visitor v;
    ASSERT_FALSE(v.visited);
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
}

TEST(MultiShape, AddConvexHullWithOnePointSameAsDisk)
{
    const auto defaultMassData = MassData{};
    const auto center = Length2D(Real(1) * Meter, Real(-4) * Meter);

    auto pointSet = VertexSet{};
    ASSERT_EQ(pointSet.size(), std::size_t(0));
    pointSet.add(center);
    ASSERT_EQ(pointSet.size(), std::size_t(1));

    auto conf = MultiShape::Conf{};
    conf.density = Real(2.3f) * KilogramPerSquareMeter;
    conf.vertexRadius = Real(0.7f) * Meter;

    auto foo = MultiShape{conf};
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{0});
    ASSERT_EQ(foo.GetMassData(), defaultMassData);
    ASSERT_EQ(foo.GetVertexRadius(), conf.vertexRadius);
    ASSERT_EQ(foo.GetDensity(), conf.density);

    foo.AddConvexHull(pointSet);
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});

    const auto child = foo.GetChild(0);
    EXPECT_EQ(child.GetVertexCount(), DistanceProxy::size_type(1));
    
    const auto massData = foo.GetMassData();
    EXPECT_NE(massData, defaultMassData);
    EXPECT_EQ(massData.center, center);
    
    const auto diskMassData = playrho::GetMassData(conf.vertexRadius, conf.density, center);
    EXPECT_EQ(massData, diskMassData);
}
