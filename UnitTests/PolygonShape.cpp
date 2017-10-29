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
        case  4: EXPECT_EQ(sizeof(PolygonShape), std::size_t(80)); break;
        case  8: EXPECT_EQ(sizeof(PolygonShape), std::size_t(104)); break;
        case 16: EXPECT_EQ(sizeof(PolygonShape), std::size_t(160)); break;
        default: FAIL(); break;
    }
}

TEST(PolygonShape, DefaultConstruction)
{
    PolygonShape shape;
    EXPECT_EQ(shape.GetVertexCount(), 0);
    EXPECT_EQ(shape.GetCentroid(), (Length2D{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
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
    Length2D vertices[4];
    
    vertices[0] = Length2D{Real(0) * Meter, +Real(1) * Meter};
    vertices[1] = Vec2{-1, -2} * (Real(1) * Meter);
    vertices[2] = Vec2{+3, -4} * (Real(1) * Meter);
    vertices[3] = Vec2{+2, +2} * (Real(1) * Meter);

    const auto index = FindLowestRightMostVertex(vertices);
    
    EXPECT_EQ(index, std::size_t(2));
}

TEST(PolygonShape, BoxConstruction)
{
    const auto hx = Real(2.3) * Meter;
    const auto hy = Real(54.1) * Meter;
    const auto shape = PolygonShape{hx, hy};

    EXPECT_EQ(shape.GetCentroid(), (Length2D{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...

    EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left

    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
    
    EXPECT_TRUE(Validate(shape));
}

TEST(PolygonShape, Copy)
{
    const auto hx = Real(2.3) * Meter;
    const auto hy = Real(54.1) * Meter;
    
    auto shape = PolygonShape{hx, hy};
    ASSERT_EQ(shape.GetCentroid(), (Length2D{}));
    ASSERT_EQ(shape.GetChildCount(), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    ASSERT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
    ASSERT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
    ASSERT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
    
    ASSERT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    ASSERT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    ASSERT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    ASSERT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));

    const auto copy = shape;
    
    EXPECT_EQ(typeid(copy), typeid(shape));
    EXPECT_EQ(copy.GetCentroid(), (Length2D{}));
    EXPECT_EQ(copy.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(copy), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(copy.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(copy.GetVertex(0), Length2D(hx, -hy)); // bottom right
    EXPECT_EQ(copy.GetVertex(1), Length2D(hx, hy)); // top right
    EXPECT_EQ(copy.GetVertex(2), Length2D(-hx, hy)); // top left
    EXPECT_EQ(copy.GetVertex(3), Length2D(-hx, -hy)); // bottom left
    
    EXPECT_EQ(copy.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(copy.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(copy.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(copy.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, Translate)
{
    const auto hx = Real(2.3) * Meter;
    const auto hy = Real(54.1) * Meter;
    
    auto shape = PolygonShape{hx, hy};
    ASSERT_EQ(shape.GetCentroid(), (Length2D{}));
    ASSERT_EQ(shape.GetChildCount(), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    ASSERT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
    ASSERT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
    ASSERT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
    
    ASSERT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    ASSERT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    ASSERT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    ASSERT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
    
    const auto new_ctr = Length2D{-Real(3) * Meter, Real(67) * Meter};
    shape.Transform(Transformation{new_ctr, UnitVec2::GetRight()});
    
    EXPECT_EQ(shape.GetCentroid(), new_ctr);
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));

    EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy) + new_ctr); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy) + new_ctr); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy) + new_ctr); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy) + new_ctr); // bottom left

    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBox)
{
    const auto hx = Real(2.3) * Meter;
    const auto hy = Real(54.1) * Meter;
    PolygonShape shape;
    shape.SetAsBox(hx, hy);
    EXPECT_EQ(shape.GetCentroid(), (Length2D{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
    
    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsZeroCenteredRotatedBox)
{
    const auto hx = Real(2.3) * Meter;
    const auto hy = Real(54.1) * Meter;
    PolygonShape shape;
    SetAsBox(shape, hx, hy, Length2D{}, Angle{0});
    EXPECT_EQ(shape.GetCentroid(), (Length2D{}));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
    
    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsCenteredBox)
{
    const auto hx = Real(2.3) * Meter;
    const auto hy = Real(54.1) * Meter;
    PolygonShape shape;
    const auto x_off = Real(10.2) * Meter;
    const auto y_off = Real(-5) * Meter;
    SetAsBox(shape, hx, hy, Length2D(x_off, y_off), Angle{0});
    EXPECT_EQ(shape.GetCentroid(), Length2D(x_off, y_off));
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2D(hx + x_off, -hy + y_off)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2D(hx + x_off, hy + y_off)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2D(-hx + x_off, hy + y_off)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2D(-hx + x_off, -hy + y_off)); // bottom left
    
    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBoxAngledDegrees90)
{
    const auto hx = Real(2.3);
    const auto hy = Real(54.1);
    PolygonShape shape;
    const auto angle = 90_deg;
    SetAsBox(shape, hx * Meter, hy * Meter, Length2D{}, angle);

    EXPECT_EQ(GetX(shape.GetCentroid()), Real(0) * Meter);
    EXPECT_EQ(GetY(shape.GetCentroid()), Real(0) * Meter);
    EXPECT_EQ(shape.GetChildCount(), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(4));
    
    // vertices go counter-clockwise (and normals follow their edges)...
    
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(0)) / Meter}), double( hy), 0.0002); // right
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(0)) / Meter}), double( hx), 0.0002); // top
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(1)) / Meter}), double(-hy), 0.0002); // left
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(1)) / Meter}), double( hx), 0.0002); // top
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(2)) / Meter}), double(-hy), 0.0002); // left
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(2)) / Meter}), double(-hx), 0.0002); // bottom
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(3)) / Meter}), double( hy), 0.0002); // right
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(3)) / Meter}), double(-hx), 0.0002); // bottom
    
    EXPECT_NEAR(double(shape.GetNormal(0).GetX()),  0.0, 0.0001);
    EXPECT_NEAR(double(shape.GetNormal(0).GetY()), +1.0, 0.0001);
    
    EXPECT_NEAR(double(shape.GetNormal(1).GetX()), -1.0, 0.00001);
    EXPECT_NEAR(double(shape.GetNormal(1).GetY()),  0.0, 0.00001);

    EXPECT_NEAR(double(shape.GetNormal(2).GetX()),  0.0, 0.00001);
    EXPECT_NEAR(double(shape.GetNormal(2).GetY()), -1.0, 0.00001);
    
    EXPECT_NEAR(double(shape.GetNormal(3).GetX()), +1.0, 0.00001);
    EXPECT_NEAR(double(shape.GetNormal(3).GetY()),  0.0, 0.00001);
}

TEST(PolygonShape, SetPoints)
{
    PolygonShape shape;
    const auto points = Vector<5, const Length2D>{
        Vec2{-1, +2} * (Real(1) * Meter),
        Vec2{+3, +3} * (Real(1) * Meter),
        Vec2{+2, -1} * (Real(1) * Meter),
        Vec2{-1, -2} * (Real(1) * Meter),
        Vec2{-4, -1} * (Real(1) * Meter)
    };
    shape.Set(points);
    
    ASSERT_EQ(shape.GetVertexCount(), PolygonShape::VertexCounter(5));

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
    const auto points = Vector<2, const Length2D>{
        Vec2{-1, +0} * (Real(1) * Meter),
        Vec2{+1, +0} * (Real(1) * Meter)
    };
    const auto vertexRadius = Real(2) * Meter;
    PolygonShape shape;
    shape.SetVertexRadius(vertexRadius);
    shape.Set(points);
    EXPECT_EQ(shape.GetVertexCount(), static_cast<PolygonShape::VertexCounter>(points.size()));
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
    EXPECT_EQ(shape.GetCentroid(), Average(Span<const Length2D>(points.data(), points.size())));
    EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);

    EXPECT_TRUE(Validate(shape));
}

TEST(PolygonShape, CanSetOnePoint)
{
    const auto points = Vector<1, const Length2D>{Length2D{}};
    const auto vertexRadius = Real(2) * Meter;
    PolygonShape shape;
    shape.SetVertexRadius(vertexRadius);
    shape.Set(points);
    EXPECT_EQ(shape.GetVertexCount(), static_cast<PolygonShape::VertexCounter>(points.size()));
    EXPECT_EQ(shape.GetVertex(0), points[0]);
    EXPECT_FALSE(IsValid(shape.GetNormal(0)));
    EXPECT_EQ(shape.GetCentroid(), points[0]);
    EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);
}
