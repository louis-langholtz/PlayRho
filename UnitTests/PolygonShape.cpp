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
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>
#include <PlayRho/Collision/Shapes/ShapeVisitor.hpp>

using namespace playrho;

TEST(PolygonShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(PolygonShape), std::size_t(96));
#else
            EXPECT_EQ(sizeof(PolygonShape), std::size_t(80));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(PolygonShape), std::size_t(60));
#else
            EXPECT_EQ(sizeof(PolygonShape), std::size_t(52));
#endif
#else
            EXPECT_EQ(sizeof(PolygonShape), std::size_t(80));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(PolygonShape), std::size_t(104)); break;
        case 16: EXPECT_EQ(sizeof(PolygonShape), std::size_t(160)); break;
        default: FAIL(); break;
    }
}

TEST(PolygonShape, DefaultConstruction)
{
    PolygonShape shape;
    EXPECT_EQ(shape.GetVertexCount(), 0);
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    EXPECT_FALSE(IsValid(shape.GetCentroid()));
}

TEST(PolygonShape, GetInvalidChildThrows)
{
    PolygonShape foo{};
    
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_NO_THROW(foo.GetChild(0));
    EXPECT_THROW(foo.GetChild(1), InvalidArgument);
}

TEST(PolygonShape, Accept)
{
    class Visitor: public IsVisitedShapeVisitor
    {
    public:
        void Visit(const PolygonShape&) override
        {
            visited = true;
        }
        bool visited = false;
    };

    PolygonShape foo{};
    Visitor v;
    ASSERT_FALSE(v.visited);
    ASSERT_FALSE(v.IsVisited());
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
    EXPECT_FALSE(v.IsVisited());
}

TEST(PolygonShape, BaseVisitorForDiskShape)
{
    const auto shape = PolygonShape{};
    auto visitor = IsVisitedShapeVisitor{};
    ASSERT_FALSE(visitor.IsVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsVisited());
}

TEST(PolygonShape, FindLowestRightMostVertex)
{
    Length2 vertices[4];
    
    vertices[0] = Length2{0_m, +1_m};
    vertices[1] = Vec2{-1, -2} * Meter;
    vertices[2] = Vec2{+3, -4} * Meter;
    vertices[3] = Vec2{+2, +2} * Meter;

    const auto index = FindLowestRightMostVertex(vertices);
    
    EXPECT_EQ(index, std::size_t(2));
}

TEST(PolygonShape, BoxConstruction)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto shape = PolygonShape{hx, hy};

    EXPECT_EQ(shape.GetCentroid(), (Length2{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...

    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left

    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
    
    EXPECT_TRUE(Validate(shape));
}

TEST(PolygonShape, Copy)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    
    auto shape = PolygonShape{hx, hy};
    ASSERT_EQ(shape.GetCentroid(), (Length2{}));
    ASSERT_EQ(shape.GetChildCount(), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    ASSERT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    ASSERT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    ASSERT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    ASSERT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    ASSERT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    ASSERT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    ASSERT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));

    const auto copy = shape;
    
    EXPECT_EQ(typeid(copy), typeid(shape));
    EXPECT_EQ(copy.GetCentroid(), (Length2{}));
    EXPECT_EQ(copy.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(copy), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(copy.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(copy.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(copy.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(copy.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(copy.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    EXPECT_EQ(copy.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(copy.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(copy.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(copy.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, Translate)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    
    auto shape = PolygonShape{hx, hy};
    ASSERT_EQ(shape.GetCentroid(), (Length2{}));
    ASSERT_EQ(shape.GetChildCount(), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    ASSERT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    ASSERT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    ASSERT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    ASSERT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    ASSERT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    ASSERT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    ASSERT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
    
    const auto new_ctr = Length2{-3_m, 67_m};
    shape = PolygonShape{
        PolygonShape::Conf{}.SetAsBox(hx, hy).Transform(Transformation{new_ctr, UnitVec2::GetRight()})
    };

    EXPECT_NEAR(static_cast<double>(Real{GetX(shape.GetCentroid())/Meter}),
                static_cast<double>(Real{GetX(new_ctr)/Meter}), 0.001);
    EXPECT_NEAR(static_cast<double>(Real{GetY(shape.GetCentroid())/Meter}),
                static_cast<double>(Real{GetY(new_ctr)/Meter}), 0.001);
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));

    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy) + new_ctr); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy) + new_ctr); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy) + new_ctr); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy) + new_ctr); // bottom left

    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBox)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto shape = PolygonShape{hx, hy};
    EXPECT_EQ(shape.GetCentroid(), (Length2{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsZeroCenteredRotatedBox)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto shape = PolygonShape{PolygonShape::Conf{}.SetAsBox(hx, hy, Length2{}, 0_deg)};
    EXPECT_EQ(shape.GetCentroid(), (Length2{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsCenteredBox)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto x_off = 10.2_m;
    const auto y_off = -5_m;
    const auto shape = PolygonShape{PolygonShape::Conf{}.SetAsBox(hx, hy, Length2(x_off, y_off), 0_deg)};
    EXPECT_EQ(shape.GetCentroid(), Length2(x_off, y_off));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2(hx + x_off, -hy + y_off)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx + x_off, hy + y_off)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx + x_off, hy + y_off)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx + x_off, -hy + y_off)); // bottom left
    
    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBoxAngledDegrees90)
{
    const auto hx = Real(2.3);
    const auto hy = Real(54.1);
    const auto angle = 90_deg;
    const auto shape = PolygonShape{PolygonShape::Conf{}.SetAsBox(hx * Meter, hy * Meter, Length2{}, angle)};

    EXPECT_EQ(GetX(shape.GetCentroid()), 0_m);
    EXPECT_EQ(GetY(shape.GetCentroid()), 0_m);
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise (and normals follow their edges)...
    
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(0)) / Meter}), double( hy), 0.0002); // right
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(0)) / Meter}), double(-hx), 0.0002); // bottom
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(1)) / Meter}), double( hy), 0.0002); // right
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(1)) / Meter}), double( hx), 0.0002); // top
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(2)) / Meter}), double(-hy), 0.0002); // left
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(2)) / Meter}), double( hx), 0.0002); // top
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(3)) / Meter}), double(-hy), 0.0002); // left
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(3)) / Meter}), double(-hx), 0.0002); // bottom
    
    EXPECT_NEAR(double(shape.GetNormal(0).GetX()), +1.0, 0.00001);
    EXPECT_NEAR(double(shape.GetNormal(0).GetY()),  0.0, 0.00001);

    EXPECT_NEAR(double(shape.GetNormal(1).GetX()),  0.0, 0.0001);
    EXPECT_NEAR(double(shape.GetNormal(1).GetY()), +1.0, 0.0001);
    
    EXPECT_NEAR(double(shape.GetNormal(2).GetX()), -1.0, 0.00001);
    EXPECT_NEAR(double(shape.GetNormal(2).GetY()),  0.0, 0.00001);

    EXPECT_NEAR(double(shape.GetNormal(3).GetX()),  0.0, 0.00001);
    EXPECT_NEAR(double(shape.GetNormal(3).GetY()), -1.0, 0.00001);
}

TEST(PolygonShape, SetPoints)
{
    const auto points = Vector<const Length2, 5>{
        Vec2{-1, +2} * Meter,
        Vec2{+3, +3} * Meter,
        Vec2{+2, -1} * Meter,
        Vec2{-1, -2} * Meter,
        Vec2{-4, -1} * Meter
    };
    const auto shape = PolygonShape{PolygonShape::Conf{}.Set(points)};

    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(5));

    // vertices go counter-clockwise from lowest right-most...

    EXPECT_EQ(shape.GetVertex(0), points[1]);
    EXPECT_EQ(shape.GetVertex(1), points[0]);
    EXPECT_EQ(shape.GetVertex(2), points[4]);
    EXPECT_EQ(shape.GetVertex(3), points[3]);
    EXPECT_EQ(shape.GetVertex(4), points[2]);
    
    EXPECT_TRUE(Validate(shape));
}

TEST(PolygonShape, CanSetTwoPoints)
{
    const auto points = Vector<const Length2, 2>{
        Vec2{-1, +0} * Meter,
        Vec2{+1, +0} * Meter
    };
    const auto vertexRadius = 2_m;
    const auto shape = PolygonShape{PolygonShape::Conf{}.SetVertexRadius(vertexRadius).Set(points)};
    EXPECT_EQ(shape.GetVertexCount(), static_cast<VertexCounter>(points.size()));
    EXPECT_EQ(shape.GetVertex(0), points[1]);
    EXPECT_EQ(shape.GetVertex(1), points[0]);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(shape.GetNormal(0)))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(shape.GetNormal(0)))),
                +1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(shape.GetNormal(1)))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(shape.GetNormal(1)))),
                -1.0, 1.0/100000.0);
    EXPECT_EQ(shape.GetCentroid(), Average(Span<const Length2>(points.data(), points.size())));
    EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);

    EXPECT_TRUE(Validate(shape));
}

TEST(PolygonShape, CanSetOnePoint)
{
    const auto points = Vector<const Length2, 1>{Length2{}};
    const auto vertexRadius = 2_m;
    const auto shape = PolygonShape{PolygonShape::Conf{}.SetVertexRadius(vertexRadius).Set(points)};
    EXPECT_EQ(shape.GetVertexCount(), static_cast<VertexCounter>(points.size()));
    EXPECT_EQ(shape.GetVertex(0), points[0]);
    EXPECT_FALSE(IsValid(shape.GetNormal(0)));
    EXPECT_EQ(shape.GetCentroid(), points[0]);
    EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);
}
