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
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <typeinfo>

using namespace playrho;
using namespace playrho::d2;

TEST(EdgeShapeConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(48));
#else
            EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(48));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(96)); break;
        case 16: EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(192)); break;
        default: FAIL(); break;
    }
}

TEST(EdgeShapeConf, GetInvalidChildThrows)
{
    const auto foo = EdgeShapeConf{};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_NO_THROW(GetChild(foo, 0));
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(EdgeShapeConf, TransformFF)
{
    {
        auto foo = EdgeShapeConf{};
        auto tmp = foo;
        Transform(foo, Mat22{});
        EXPECT_EQ(foo, tmp);
    }
    {
        auto foo = EdgeShapeConf{};
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>());
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        auto foo = EdgeShapeConf{v1, v2};
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>());
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        auto foo = EdgeShapeConf{v1, v2};
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>() * 2);
        EXPECT_NE(foo, tmp);
        EXPECT_EQ(foo.GetVertexA(), v1 * 2);
        EXPECT_EQ(foo.GetVertexB(), v2 * 2);
    }
}

TEST(EdgeShapeConf, Visit)
{
    const auto s = Shape{EdgeShapeConf{}};
    auto data = UnitTestsVisitorData{};
    ASSERT_EQ(data.visitedDisk, 0);
    ASSERT_EQ(data.visitedEdge, 0);
    ASSERT_EQ(data.visitedPolygon, 0);
    ASSERT_EQ(data.visitedChain, 0);
    ASSERT_EQ(data.visitedMulti, 0);
    EXPECT_TRUE(Visit(s, &data));
    EXPECT_EQ(data.visitedDisk, 0);
    EXPECT_EQ(data.visitedEdge, 1);
    EXPECT_EQ(data.visitedPolygon, 0);
    EXPECT_EQ(data.visitedChain, 0);
    EXPECT_EQ(data.visitedMulti, 0);
}

TEST(EdgeShapeConf, Accept)
{
#if 0
    auto visited = false;
    auto shapeVisited = false;
    const auto foo = EdgeShapeConf{};
    ASSERT_FALSE(visited);
    ASSERT_FALSE(shapeVisited);
    Accept(Shape(foo), [&](const std::type_info& ti, const void*) {
        visited = true;
        if (ti == typeid(EdgeShapeConf))
        {
            shapeVisited = true;
        }
    });
    EXPECT_TRUE(visited);
    EXPECT_TRUE(shapeVisited);
#endif
}

#if 0
TEST(EdgeShapeConf, BaseVisitorForDiskShape)
{
    const auto shape = EdgeShapeConf{};
    auto visitor = IsVisitedShapeVisitor{};
    ASSERT_FALSE(visitor.IsVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsVisited());
}
#endif

TEST(EdgeShapeConf, Equality)
{
    EXPECT_TRUE(EdgeShapeConf() == EdgeShapeConf());

    EXPECT_FALSE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) == EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)));

    EXPECT_FALSE(EdgeShapeConf().UseVertexRadius(10_m) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseVertexRadius(10_m) == EdgeShapeConf().UseVertexRadius(10_m));
    
    EXPECT_FALSE(EdgeShapeConf().UseDensity(10_kgpm2) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseDensity(10_kgpm2) == EdgeShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_FALSE(EdgeShapeConf().UseFriction(Real(10)) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseFriction(Real(10)) == EdgeShapeConf().UseFriction(Real(10)));
    
    EXPECT_FALSE(EdgeShapeConf().UseRestitution(Real(10)) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseRestitution(Real(10)) == EdgeShapeConf().UseRestitution(Real(10)));
}

TEST(EdgeShapeConf, Inequality)
{
    EXPECT_FALSE(EdgeShapeConf() != EdgeShapeConf());
    
    EXPECT_TRUE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) != EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)));

    EXPECT_TRUE(EdgeShapeConf().UseVertexRadius(10_m) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseVertexRadius(10_m) != EdgeShapeConf().UseVertexRadius(10_m));
    
    EXPECT_TRUE(EdgeShapeConf().UseDensity(10_kgpm2) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseDensity(10_kgpm2) != EdgeShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_TRUE(EdgeShapeConf().UseFriction(Real(10)) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseFriction(Real(10)) != EdgeShapeConf().UseFriction(Real(10)));
    
    EXPECT_TRUE(EdgeShapeConf().UseRestitution(Real(10)) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseRestitution(Real(10)) != EdgeShapeConf().UseRestitution(Real(10)));
}
