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
        case  4: EXPECT_EQ(sizeof(MultiShape), std::size_t(48)); break;
        case  8: EXPECT_EQ(sizeof(MultiShape), std::size_t(64)); break;
        case 16: EXPECT_EQ(sizeof(MultiShape), std::size_t(112)); break;
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
    ASSERT_FALSE(v.IsBaseVisited());
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
    EXPECT_FALSE(v.IsBaseVisited());
}

TEST(MultiShape, BaseVisitorForDiskShape)
{
    const auto shape = MultiShape{};
    auto visitor = Shape::Visitor{};
    ASSERT_FALSE(visitor.IsBaseVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsBaseVisited());
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

TEST(MultiShape, AddConvexHullWithTwoPointsSameAsEdge)
{
    const auto defaultMassData = MassData{};
    const auto p0 = Length2D(Real(1) * Meter, Real(-4) * Meter);
    const auto p1 = Length2D(Real(1) * Meter, Real(+4) * Meter);
    
    auto pointSet = VertexSet{};
    ASSERT_EQ(pointSet.size(), std::size_t(0));
    pointSet.add(p0);
    pointSet.add(p1);
    ASSERT_EQ(pointSet.size(), std::size_t(2));
    
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
    EXPECT_EQ(child.GetVertexCount(), DistanceProxy::size_type(2));
    
    const auto massData = foo.GetMassData();
    EXPECT_NE(massData, defaultMassData);
    EXPECT_EQ(massData.center, (p0 + p1) / Real(2));
    
    const auto edgeMassData = playrho::GetMassData(conf.vertexRadius, conf.density, p0, p1);
    EXPECT_EQ(massData, edgeMassData);
}

TEST(MultiShape, AddTwoConvexHullWithOnePoint)
{
    const auto defaultMassData = MassData{};
    const auto p0 = Length2D(Real(1) * Meter, Real(-4) * Meter);
    const auto p1 = Length2D(Real(1) * Meter, Real(+4) * Meter);

    auto pointSet = VertexSet{};
    ASSERT_EQ(pointSet.size(), std::size_t(0));

    auto conf = MultiShape::Conf{};
    conf.density = Real(2.3f) * KilogramPerSquareMeter;
    conf.vertexRadius = Real(0.7f) * Meter;
    
    auto foo = MultiShape{conf};
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{0});
    ASSERT_EQ(foo.GetMassData(), defaultMassData);
    ASSERT_EQ(foo.GetVertexRadius(), conf.vertexRadius);
    ASSERT_EQ(foo.GetDensity(), conf.density);
    
    pointSet.clear();
    ASSERT_EQ(pointSet.size(), std::size_t(0));
    pointSet.add(p0);
    ASSERT_EQ(pointSet.size(), std::size_t(1));

    foo.AddConvexHull(pointSet);
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});

    const auto child0 = foo.GetChild(0);
    EXPECT_EQ(child0.GetVertexCount(), DistanceProxy::size_type(1));
    EXPECT_EQ(child0.GetVertex(0), p0);
    
    pointSet.clear();
    ASSERT_EQ(pointSet.size(), std::size_t(0));
    pointSet.add(p1);
    ASSERT_EQ(pointSet.size(), std::size_t(1));
    
    foo.AddConvexHull(pointSet);
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{2});
    
    const auto child1 = foo.GetChild(1);
    EXPECT_EQ(child1.GetVertexCount(), DistanceProxy::size_type(1));
    EXPECT_EQ(child1.GetVertex(0), p1);

    const auto massData = foo.GetMassData();
    EXPECT_NE(massData, defaultMassData);
    EXPECT_EQ(massData.center, (p0 + p1) / Real(2));
    
    const auto massDataP0 = playrho::GetMassData(conf.vertexRadius, conf.density, p0);
    const auto massDataP1 = playrho::GetMassData(conf.vertexRadius, conf.density, p1);
    EXPECT_EQ(massData.mass, Mass{massDataP0.mass} + Mass{massDataP1.mass});
    EXPECT_EQ(massData.I, RotInertia{massDataP0.I} + RotInertia{massDataP1.I});
}
