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
#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/ShapeSeparation.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(CollideShapes, IdenticalOverlappingCircles)
{
    const auto radius = 1_m;
    const auto shape = DiskShapeConf{}.UseRadius(radius);
    const auto position = Vec2{11, -4} * Meter;
    const auto xfm = Transformation{position, UnitVec::GetRight()};
    
    // put shape 1 to left of shape 2
    const auto manifold = CollideShapes(GetChild(shape, 0), xfm, GetChild(shape, 0), xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    
    EXPECT_EQ(manifold.GetLocalPoint(), shape.GetLocation());
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, shape.GetLocation());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleCircleOrientedHorizontally)
{
    const auto r1 = 1_m;
    const auto r2 = 1_m;
    const auto s1 = DiskShapeConf{}.UseRadius(r1);
    const auto s2 = DiskShapeConf{}.UseRadius(r2);
    const auto p1 = Vec2{11, -4} * Meter;
    const auto p2 = Vec2{13, -4} * Meter;
    const auto t1 = Transformation{p1, UnitVec::GetRight()};
    const auto t2 = Transformation{p2, UnitVec::GetRight()};
    
    // put shape 1 to left of shape 2
    const auto manifold = CollideShapes(GetChild(s1, 0), t1, GetChild(s2, 0), t2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    
    EXPECT_EQ(manifold.GetLocalPoint(), s1.GetLocation());
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));

    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, s2.GetLocation());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleCircleOrientedVertically)
{
    const auto r1 = 1_m;
    const auto r2 = 1_m;
    const auto s1 = DiskShapeConf{}.UseRadius(r1);
    const auto s2 = DiskShapeConf{}.UseRadius(r2);
    const auto p1 = Vec2{7, -2} * Meter;
    const auto p2 = Vec2{7, -1} * Meter;
    
    // Rotations don't matter so long as circle shapes' centers are at (0, 0).
    const auto t1 = Transformation{p1, UnitVec::Get(45_deg)};
    const auto t2 = Transformation{p2, UnitVec::Get(-21_deg)};
    
    // put shape 1 to left of shape 2
    const auto manifold = CollideShapes(GetChild(s1, 0), t1, GetChild(s2, 0), t2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), Length2{});
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Length2{});
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleWithinSquareA)
{
    const auto shapeConf = PolygonShapeConf{}.SetAsBox(2_m, 2_m);
    const auto xfm = Transformation{};

    const auto manifold = GetManifold(false, 1_m, GetChild(shapeConf, 0), xfm, Length2{}, xfm);
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetPointCount(), 1u);
    EXPECT_EQ(manifold.GetLocalPoint(), Length2(2_m, 0_m));
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetX()), 1.0, 0.000);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetY()), 0.0, 0.000);
}

TEST(CollideShapes, CircleWithinSquareB)
{
    const auto shapeConf = PolygonShapeConf{}.SetAsBox(2_m, 2_m);
    const auto xfm = Transformation{};
    const auto manifold = GetManifold(true, 1_m, GetChild(shapeConf, 0), xfm, Length2{}, xfm);
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    EXPECT_EQ(manifold.GetPointCount(), 1u);
    EXPECT_EQ(manifold.GetLocalPoint(), Length2(2_m, 0_m));
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetX()), 1.0, 0.000);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetY()), 0.0, 0.000);

    const auto diskConf = DiskShapeConf{1_m};
    const auto manifold2 = CollideShapes(GetChild(diskConf, 0), xfm, GetChild(shapeConf, 0), xfm);
    EXPECT_EQ(manifold, manifold2);
}

TEST(CollideShapes, CircleTrianglePointBelow)
{
    const auto circleRadius = 0.5_m;
    const auto circle = DiskShapeConf{}.UseRadius(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf{}.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto triangleXfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };
    const auto circleXfm = Transformation{
        triangleTopPt + UnitVec::GetTop() * 1_m,
        UnitVec::GetRight()
    };
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(0));
}

TEST(CollideShapes, CircleTouchingTrianglePointBelow)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf{}.UseRadius(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf{}.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto triangleXfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };
    const auto circleXfm = Transformation{
        triangleTopPt + UnitVec::GetTop() * circleRadius,
        UnitVec::GetRight()
    };
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm,
                                        GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), triangleTopPt);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Length2{});
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleTopPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, TriangleTouchingCirclePointBelow)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf{}.UseRadius(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf{}.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto triangleXfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto circlePt = triangleTopPt + UnitVec::GetTop() * circleRadius;
    const auto circleXfm = Transformation{circlePt, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(circle, 0), circleXfm,
                                        GetChild(triangle, 0), triangleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), Length2(0_m, 0_m));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Length2(0_m, 1_m));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleRightPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 1);
}

TEST(CollideShapes, CircleTouchingTrianglePointLeft)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf{}.UseRadius(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf{}.Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{
        triangleLeftPt + UnitVec::Get(225_deg) * circleRadius,
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), triangleLeftPt);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, (Length2{}));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleLeftPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleTrianglePointRight)
{
    const auto circleRadius = 0.5_m;
    const auto circle = DiskShapeConf(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{
        triangleRightPt + UnitVec::Get(-45_deg) * 1_m,
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(0));
}

TEST(CollideShapes, CircleTouchingTrianglePointRight)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{
        triangleRightPt + UnitVec::Get(-45_deg) * circleRadius,
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), triangleRightPt);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, (Length2{}));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleRightPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, TriangleTouchingCirclePointRight)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{
        triangleRightPt + UnitVec::Get(-45_deg) * circleRadius,
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(circle, 0), circleXfm,
                                        GetChild(triangle, 0), triangleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), Length2());
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, triangleRightPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(triangle.GetVertex(manifold.GetPoint(0).contactFeature.indexA), triangleRightPt);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, CircleJustPastTrianglePointRightDoesntCollide)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    auto triangle = PolygonShapeConf{}
            .UseVertexRadius(Real{0.0001f * 2} * Meter)
            .Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{
        triangleRightPt + UnitVec::Get(-45_deg) * circleRadius * Real(1.01),
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(0));
}

TEST(CollideShapes, CircleOverRightFaceOfTriangle)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf(circleRadius);
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto triangle = PolygonShapeConf({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto circleXfm = Transformation{
        Vec2{1, 1} * Meter,
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (triangleTopPt + triangleRightPt) / Real{2});
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetX()), 0.894427  , 0.0002);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetY()), 0.44721359, 0.0002);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, circle.GetLocation());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(triangle.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);

    EXPECT_EQ(triangle.GetVertex(0), (Vec2{+1, -1} * Meter));
}

TEST(CollideShapes, CircleOverLeftFaceOfTriangle)
{
    const auto circleRadius = 1_m;
    const auto circle = DiskShapeConf(circleRadius);
    const auto triangle = PolygonShapeConf({Vec2{-1, -1} * Meter, Vec2{+1, -1} * Meter, Vec2{0, +1} * Meter});
    const auto circleXfm = Transformation{
        Vec2{-1, 1} * Meter,
        UnitVec::GetRight()
    };
    const auto triangleXfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };
    
    const auto manifold = CollideShapes(GetChild(triangle, 0), triangleXfm, GetChild(circle, 0), circleXfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Vec2{-0.5f, 0} * Meter));
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetX()), -0.894427,   0.0002);
    EXPECT_NEAR(double(manifold.GetLocalNormal().GetY()),  0.44721359, 0.0002);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, (Length2{}));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(triangle.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
    
    EXPECT_EQ(triangle.GetVertex(0), (Vec2{+1, -1} * Meter));
}

TEST(CollideShapes, TallRectangleLeftCircleRight)
{
    const auto r2 = 1_m;
    const auto hx = Real(2.2);
    const auto hy = Real(4.8);

    const auto s1 = PolygonShapeConf(hx * Meter, hy * Meter);
    ASSERT_EQ(s1.GetVertex(0), (Vec2{+hx, -hy} * Meter)); // bottom right
    ASSERT_EQ(s1.GetVertex(1), (Vec2{+hx, +hy} * Meter)); // top right
    ASSERT_EQ(s1.GetVertex(2), (Vec2{-hx, +hy} * Meter)); // top left
    ASSERT_EQ(s1.GetVertex(3), (Vec2{-hx, -hy} * Meter)); // bottom left

    const auto s2 = DiskShapeConf{r2};
    
    const auto p1 = Vec2{-1, 0} * Meter;
    const auto p2 = Vec2{3, 0} * Meter;
    const auto t1 = Transformation{p1, UnitVec::Get(45_deg)};
    const auto t2 = Transformation{p2, UnitVec::GetRight()};
    
    // rotate rectangle 45 degrees and put it on the left of the circle
    const auto manifold = CollideShapes(GetChild(s1, 0), t1, GetChild(s2, 0), t2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetLocalPoint(), (Vec2{hx, 0} * Meter));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, (Length2{}));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(s1.GetNormal(manifold.GetPoint(0).contactFeature.indexA), manifold.GetLocalNormal());
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
}

TEST(CollideShapes, IdenticalOverlappingSquaresDim1)
{
    const auto dim = Real(1);
    const auto shape = PolygonShapeConf(dim * Meter, dim * Meter);
    ASSERT_EQ(shape.GetVertex(0), (Vec2{+dim, -dim} * Meter)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), (Vec2{+dim, +dim} * Meter)); // top right
    ASSERT_EQ(shape.GetVertex(2), (Vec2{-dim, +dim} * Meter)); // top left
    ASSERT_EQ(shape.GetVertex(3), (Vec2{-dim, -dim} * Meter)); // bottom left
    
    const auto xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfm, GetChild(shape, 0), xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetLocalPoint(), (Vec2{+dim, 0} * Meter));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, (Vec2{-dim, +dim} * Meter)); // top left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, (Vec2{-dim, -dim} * Meter)); // bottom left
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, IdenticalOverlappingSquaresDim2)
{
    const auto dim = Real(2);
    const auto shape = PolygonShapeConf(dim * Meter, dim * Meter);
    ASSERT_EQ(shape.GetVertex(0), (Vec2{+dim, -dim} * Meter)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), (Vec2{+dim, +dim} * Meter)); // top right
    ASSERT_EQ(shape.GetVertex(2), (Vec2{-dim, +dim} * Meter)); // top left
    ASSERT_EQ(shape.GetVertex(3), (Vec2{-dim, -dim} * Meter)); // bottom left
    
    const auto xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfm, GetChild(shape, 0), xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetLocalPoint(), (Vec2{+dim, 0} * Meter));
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, (Vec2{-dim, +dim} * Meter)); // top left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, (Vec2{-dim, -dim} * Meter)); // bottom left
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, IdenticalVerticalTouchingSquares)
{
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), (Vec2{+2, -2} * Meter)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), (Vec2{+2, +2} * Meter)); // top right
    ASSERT_EQ(shape.GetVertex(2), (Vec2{-2, +2} * Meter)); // top left
    ASSERT_EQ(shape.GetVertex(3), (Vec2{-2, -2} * Meter)); // bottom left

    const auto xfm0 = Transformation{
        Vec2{0, -1} * Meter,
        UnitVec::GetRight()
    }; // bottom
    const auto xfm1 = Transformation{
        Vec2{0, +1} * Meter,
        UnitVec::GetRight()
    }; // top
    const auto manifold = CollideShapes(GetChild(shape, 0), xfm0, GetChild(shape, 0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0,+2) * Meter);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, -2) * Meter); // bottom left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, -2) * Meter); // bottom right
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(CollideShapes, IdenticalHorizontalTouchingSquares)
{
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left

    const auto xfm0 = Transformation{
        Vec2{-2, 0} * Meter,
        UnitVec::GetRight()
    }; // left
    const auto xfm1 = Transformation{
        Vec2{+2, 0} * Meter,
        UnitVec::GetRight()
    }; // right
    const auto manifold = CollideShapes(GetChild(shape, 0), xfm0, GetChild(shape, 0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(+2, 0) * Meter);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, +2) * Meter); // top left
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, -2) * Meter); // bottom left
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
}

TEST(CollideShapes, GetMaxSeparationFreeFunction1)
{
    const auto rot0 = 45_deg;
    const auto xfm0 = Transformation{Vec2{0, -2} * Meter, UnitVec::Get(rot0)}; // bottom
    const auto xfm1 = Transformation{Vec2{0, +2} * Meter, UnitVec::GetRight()}; // top
    
    const auto dim = 2_m;
    const auto shape0 = PolygonShapeConf(dim, dim);
    const auto shape1 = PolygonShapeConf(dim, dim);
    ASSERT_EQ(shape0.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape0.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape0.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape0.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left

    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //          +---4---+ <----- v1 of shape1
    //          |   |   |
    //          | B 3   |
    //          |   |   |
    //          |   2   |
    //          |   |   |
    //          |   1   |
    //          |  /|\  |
    //  v3 ---> 2-1-0-1-2 <----- v0 of shape1
    //           /  |  \
    //          /   1   \
    //         / A  |    \
    //        +     2     + <----- v0 of shape0
    //         \    |    /
    //          \   3   /
    //           \  |  /
    //            \ 4 /
    //             \|/
    //              + <----- v3 of shape0
    
    const auto child0 = GetChild(shape0, 0);
    const auto child1 = GetChild(shape1, 0);
    const auto totalRadius = GetVertexRadius(shape0) + GetVertexRadius(shape1);

    const auto maxSep01_4x4 = GetMaxSeparation4x4(child0, xfm0, child1, xfm1);
    const auto maxSep10_4x4 = GetMaxSeparation4x4(child1, xfm1, child0, xfm0);
    const auto maxSep01_NxN = GetMaxSeparation(child0, xfm0, child1, xfm1, totalRadius);
    const auto maxSep10_NxN = GetMaxSeparation(child1, xfm1, child0, xfm0, totalRadius);
    const auto maxSep01_nos = GetMaxSeparation(child0, xfm0, child1, xfm1);
    const auto maxSep10_nos = GetMaxSeparation(child1, xfm1, child0, xfm0);

    switch (sizeof(Real))
    {
        case 4:
            EXPECT_EQ(GetFirstShapeVertexIdx( maxSep01_4x4), VertexCounter{0}); // v0 of shape0
            EXPECT_EQ(GetFirstShapeVertexIdx( maxSep01_NxN), VertexCounter{0}); // v0 of shape0
            EXPECT_EQ(GetFirstShapeVertexIdx( maxSep01_nos), VertexCounter{0}); // v0 of shape0
            EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_4x4), VertexCounter{3}); // v3 of shape1
            EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_NxN), VertexCounter{3}); // v3 of shape1
            EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_nos), VertexCounter{3}); // v3 of shape1
            break;
        case 8:
            EXPECT_EQ(GetFirstShapeVertexIdx(maxSep01_4x4), VertexCounter{1}); // v1 of shape0
            EXPECT_EQ(GetFirstShapeVertexIdx(maxSep01_NxN), VertexCounter{1}); // v1 of shape0
            EXPECT_EQ(GetFirstShapeVertexIdx(maxSep01_nos), VertexCounter{1}); // v1 of shape0
            EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_4x4), VertexCounter{0}); // v0 of shape1
            EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_NxN), VertexCounter{0}); // v0 of shape1
            EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_nos), VertexCounter{0}); // v0 of shape1
            break;
    }
    
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep10_4x4), VertexCounter{3}); // v3 of shape1
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep10_NxN), VertexCounter{3}); // v3 of shape1
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep10_nos), VertexCounter{3}); // v3 of shape1
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep10_4x4), VertexCounter{1}); // v1 of shape0
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep10_NxN), VertexCounter{1}); // v1 of shape0
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep10_nos), VertexCounter{1}); // v1 of shape0
    
    EXPECT_NEAR(static_cast<double>(Real(maxSep01_4x4.distance / Meter)), -2.0, std::abs(-2.0) / 100);
    EXPECT_NEAR(static_cast<double>(Real(maxSep01_NxN.distance / Meter)), -2.0, std::abs(-2.0) / 100);
    EXPECT_NEAR(static_cast<double>(Real(maxSep01_nos.distance / Meter)), -2.0, std::abs(-2.0) / 100);

    EXPECT_NEAR(static_cast<double>(Real(maxSep10_4x4.distance / Meter)),
                -0.82842707633972168, std::abs(-0.82842707633972168) / 100);
    EXPECT_NEAR(static_cast<double>(Real(maxSep10_NxN.distance / Meter)),
                -0.82842707633972168, std::abs(-0.82842707633972168) / 100);
    EXPECT_NEAR(static_cast<double>(Real(maxSep10_nos.distance / Meter)),
                -0.82842707633972168, std::abs(-0.82842707633972168) / 100);
}

TEST(CollideShapes, GetMaxSeparationFreeFunction2)
{
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left

    const auto xfm0 = Transformation{Vec2{0, -1} * Meter, UnitVec::GetRight()}; // bottom
    const auto xfm1 = Transformation{Vec2{0, +1} * Meter, UnitVec::GetRight()}; // top
    const auto totalRadius = GetVertexRadius(shape) * Real(2);

    const auto child0 = GetChild(shape, 0);
    const auto maxSep01_4x4 = GetMaxSeparation4x4(child0, xfm0, child0, xfm1);
    const auto maxSep10_4x4 = GetMaxSeparation4x4(child0, xfm1, child0, xfm0);
    const auto maxSep01_NxN = GetMaxSeparation(child0, xfm0, child0, xfm1, totalRadius);
    const auto maxSep10_NxN = GetMaxSeparation(child0, xfm1, child0, xfm0, totalRadius);
    
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep01_4x4), VertexCounter{1}); // v0 of shape0
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep01_4x4), GetFirstShapeVertexIdx(maxSep01_NxN));
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_4x4), VertexCounter{0}); // v3 of shape1
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep01_4x4), GetSecondShapeVertexIdx<0>(maxSep01_NxN));
    EXPECT_NEAR(static_cast<double>(Real(maxSep01_4x4.distance / Meter)),
                -2.0, 0.0);
    EXPECT_NEAR(static_cast<double>(Real(maxSep01_4x4.distance / Meter)),
                static_cast<double>(Real(maxSep01_NxN.distance / Meter)),
                std::abs(static_cast<double>(Real(maxSep01_4x4.distance / Meter)) / 1000000.0));
    
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep10_4x4), VertexCounter{3}); // v3 of shape1
    EXPECT_EQ(GetFirstShapeVertexIdx(maxSep10_4x4), GetFirstShapeVertexIdx(maxSep10_NxN));
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep10_4x4), VertexCounter{1}); // v1 of shape0
    EXPECT_EQ(GetSecondShapeVertexIdx<0>(maxSep10_4x4), GetSecondShapeVertexIdx<0>(maxSep10_NxN));
    EXPECT_NEAR(static_cast<double>(Real(maxSep10_4x4.distance / Meter)),
                -2.0, 0.0);
    EXPECT_NEAR(static_cast<double>(Real(maxSep10_4x4.distance / Meter)),
                static_cast<double>(Real(maxSep10_NxN.distance / Meter)),
                std::abs(static_cast<double>(Real(maxSep10_4x4.distance / Meter)) / 1000000.0));
}

TEST(CollideShapes, SquareCornerTouchingSquareFaceAbove)
{
    const auto dim = 2_m;

    // creates a square
    const auto shape = PolygonShapeConf(dim, dim);
    ASSERT_EQ(shape.GetVertex(0), Vec2(+2, -2) * Meter); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Vec2(+2, +2) * Meter); // top right
    ASSERT_EQ(shape.GetVertex(2), Vec2(-2, +2) * Meter); // top left
    ASSERT_EQ(shape.GetVertex(3), Vec2(-2, -2) * Meter); // bottom left
    
    const auto rot0 = 45_deg;
    const auto xfm0 = Transformation{Vec2{0, -2} * Meter, UnitVec::Get(rot0)}; // bottom
    const auto xfm1 = Transformation{Vec2{0, +2} * Meter, UnitVec::GetRight()}; // top
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +

    const auto manifold = CollideShapes(GetChild(shape, 0), xfm0, GetChild(shape, 0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                -1.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(0, -2) * Meter);
    
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    
    // localPoint is almost equal to Vec2(2, 2) but it's not exactly equal.
    EXPECT_NEAR(double(Real{GetX(manifold.GetPoint(0).localPoint) / Meter}), 2.0, 0.006); // top right shape A
    EXPECT_NEAR(double(Real{GetY(manifold.GetPoint(0).localPoint) / Meter}), 2.0, 0.004); // top right shape A
    
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1); // Shape A top right vertex
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3); // Shape B bottom edge
    
    // Also check things in terms of world coordinates...
    const auto world_manifold = GetWorldManifold(manifold, xfm0, 0_m, xfm1, 0_m);
    EXPECT_EQ(world_manifold.GetPointCount(), manifold.GetPointCount());
    
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(world_manifold.GetNormal()))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(world_manifold.GetNormal()))),
                +1.0, 1.0/100000.0);

    const auto corner_point = Rotate(Length2{dim, dim}, UnitVec::Get(rot0)) + xfm0.p;
    EXPECT_NEAR(double(Real{GetX(corner_point) / Meter}), 0.0,        0.02);
    EXPECT_NEAR(double(Real{GetY(corner_point) / Meter}), 0.82842684, 0.02);
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_NEAR(double(Real{GetX(world_manifold.GetPoint(0)) / Meter}),
                double(Real{GetX(corner_point) / (2_m)}), 0.04);
    EXPECT_NEAR(double(Real{GetY(world_manifold.GetPoint(0)) / Meter}),
                double(Real{GetY(corner_point) / (2_m)}), 0.04);
    EXPECT_NEAR(double(Real{world_manifold.GetSeparation(0) / Meter}),
                double(Real{GetY(-corner_point) / Meter}), 0.008);
}

TEST(CollideShapes, HorizontalOverlappingRects1)
{
    // Shape A: square
    const auto shape0 = PolygonShapeConf(2_m, 2_m);
    ASSERT_EQ(shape0.GetVertex(0), Vec2(+2,-2) * Meter); // bottom right
    ASSERT_EQ(shape0.GetVertex(1), Vec2(+2,+2) * Meter); // top right
    ASSERT_EQ(shape0.GetVertex(2), Vec2(-2,+2) * Meter); // top left
    ASSERT_EQ(shape0.GetVertex(3), Vec2(-2,-2) * Meter); // bottom left
    
    // Shape B: wide rectangle
    const auto shape1 = PolygonShapeConf(3_m, 1.5_m);
    ASSERT_EQ(shape1.GetVertex(0), Length2(+3.0_m, -1.5_m)); // bottom right
    ASSERT_EQ(shape1.GetVertex(1), Length2(+3.0_m, +1.5_m)); // top right
    ASSERT_EQ(shape1.GetVertex(2), Length2(-3.0_m, +1.5_m)); // top left
    ASSERT_EQ(shape1.GetVertex(3), Length2(-3.0_m, -1.5_m)); // bottom left

    const auto xfm0 = Transformation{
        Vec2{-2, 0} * Meter,
        UnitVec::GetRight()
    }; // left
    const auto xfm1 = Transformation{
        Vec2{+2, 0} * Meter,
        UnitVec::GetRight()
    }; // right
    
    // Put square left, wide rectangle right.
    // In ASCII art terms:
    //
    //   +-------2
    //   |     +-+---------+
    //   |   A | 1   B     |
    //   |     | |         |
    //   4-3-2-1-*-1-2-3-4-5
    //   |     | |         |
    //   |     | 1         |
    //   |     +-+---------+
    //   +-------2
    //

    const auto manifold = CollideShapes(GetChild(shape0, 0), xfm0, GetChild(shape1, 0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(Real(+2), Real(0)) * Meter);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(Real(-3.0), Real(+1.5)) * Meter); // top left shape B
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(Real(-3.0), Real(-1.5)) * Meter); // bottom left shape B
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 3);
    
    const auto world_manifold = GetWorldManifold(manifold,
                                                 xfm0, GetVertexRadius(shape0),
                                                 xfm1, GetVertexRadius(shape1));
    EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
    
    EXPECT_TRUE(AlmostEqual(world_manifold.GetNormal().GetX(), Real(1)));
    EXPECT_TRUE(AlmostEqual(world_manifold.GetNormal().GetY(), Real(0)));
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetX(world_manifold.GetPoint(0))), Real(-0.5)));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetY(world_manifold.GetPoint(0))), Real(+1.5)));

    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetX(world_manifold.GetPoint(1))), Real(-0.5)));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetY(world_manifold.GetPoint(1))), Real(-1.5)));
}

TEST(CollideShapes, HorizontalOverlappingRects2)
{
    // Shape A: wide rectangle
    const auto shape0 = PolygonShapeConf(3_m, 1.5_m);
    ASSERT_EQ(shape0.GetVertex(0), Length2(+3.0_m, -1.5_m)); // bottom right
    ASSERT_EQ(shape0.GetVertex(1), Length2(+3.0_m, +1.5_m)); // top right
    ASSERT_EQ(shape0.GetVertex(2), Length2(-3.0_m, +1.5_m)); // top left
    ASSERT_EQ(shape0.GetVertex(3), Length2(-3.0_m, -1.5_m)); // bottom left
    
    // Shape B: square
    const auto shape1 = PolygonShapeConf(2_m, 2_m);
    ASSERT_EQ(shape1.GetVertex(0), Length2(+2_m,-2_m)); // bottom right
    ASSERT_EQ(shape1.GetVertex(1), Length2(+2_m,+2_m)); // top right
    ASSERT_EQ(shape1.GetVertex(2), Length2(-2_m,+2_m)); // top left
    ASSERT_EQ(shape1.GetVertex(3), Length2(-2_m,-2_m)); // bottom left
    
    const auto xfm0 = Transformation{
        Vec2{-2, 0} * Meter,
        UnitVec::GetRight()
    }; // left
    const auto xfm1 = Transformation{
        Vec2{+2, 0} * Meter,
        UnitVec::GetRight()
    }; // right

    // put wide rectangle on left, square on right
    const auto manifold = CollideShapes(GetChild(shape0, 0), xfm0, GetChild(shape1, 0), xfm1);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(Real(+3), Real(0)) * Meter);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(AlmostEqual(Real{GetX(manifold.GetPoint(0).localPoint) / Meter}, Real(-2.0))); // left
    EXPECT_TRUE(AlmostEqual(Real{GetY(manifold.GetPoint(0).localPoint) / Meter}, Real(-1.5))); // top
    EXPECT_TRUE(AlmostEqual(Real{manifold.GetPoint(0).normalImpulse / 1_Ns}, Real(0)));
    EXPECT_TRUE(AlmostEqual(Real{manifold.GetPoint(0).tangentImpulse / 1_Ns}, Real(0)));
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 2);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(AlmostEqual(Real{GetX(manifold.GetPoint(1).localPoint) / Meter}, Real(-2.0))); // left
    EXPECT_TRUE(AlmostEqual(Real{GetY(manifold.GetPoint(1).localPoint) / Meter}, Real(+1.5))); // bottom
    EXPECT_TRUE(AlmostEqual(Real{manifold.GetPoint(1).normalImpulse / 1_Ns}, Real(0)));
    EXPECT_TRUE(AlmostEqual(Real{manifold.GetPoint(1).tangentImpulse / 1_Ns}, Real(0)));
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
    
    const auto world_manifold = GetWorldManifold(manifold,
                                                 xfm0, GetVertexRadius(shape0),
                                                 xfm1, GetVertexRadius(shape1));
    EXPECT_EQ(world_manifold.GetPointCount(), Manifold::size_type(2));
    
    EXPECT_TRUE(AlmostEqual(world_manifold.GetNormal().GetX(), Real(1)));
    EXPECT_TRUE(AlmostEqual(world_manifold.GetNormal().GetY(), Real(0)));
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_TRUE(AlmostEqual(Real{GetX(world_manifold.GetPoint(0)) / Meter}, Real(+0.5)));
    EXPECT_TRUE(AlmostEqual(Real{GetY(world_manifold.GetPoint(0)) / Meter}, Real(-1.5)));
    
    ASSERT_GT(world_manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_TRUE(AlmostEqual(Real{GetX(world_manifold.GetPoint(1)) / Meter}, Real(+0.5)));
    EXPECT_TRUE(AlmostEqual(Real{GetY(world_manifold.GetPoint(1)) / Meter}, Real(+1.5)));
}

TEST(CollideShapes, EdgeBelowPolygon)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{
        Vec2{0, -1} * Meter,
        UnitVec::GetRight()
    };

    const auto hx = 1_m;
    const auto hy = 1_m;
    const auto polygon_shape = PolygonShapeConf(hx, hy);
    const auto polygon_xfm = Transformation{
        Length2{},
        UnitVec::GetRight()
    };

    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(-1, -1) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 3);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(+1, -1) * Meter);
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 1);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 0);
}

TEST(CollideShapes, EdgeAbovePolygon)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Vec2{0, +1} * Meter, UnitVec::GetRight()};
    
    const auto hx = 1_m;
    const auto hy = 1_m;
    const auto polygon_shape = PolygonShapeConf(hx, hy);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                -1.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetPoint(0).localPoint, Vec2(+1, +1) * Meter);
    EXPECT_EQ(manifold.GetPoint(0).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 1);
    
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetPoint(1).localPoint, Vec2(-1, +1) * Meter);
    EXPECT_EQ(manifold.GetPoint(1).normalImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).tangentImpulse, 0_Ns);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeA, ContactFeature::e_face);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexA, 0);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.typeB, ContactFeature::e_vertex);
    EXPECT_EQ(manifold.GetPoint(1).contactFeature.indexB, 2);
}

TEST(CollideShapes, EdgeLeftOfPolygon)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Vec2{-1, 0} * Meter, UnitVec::GetRight()};
    
    const auto hx = 1_m;
    const auto hy = 1_m;
    const auto polygon_shape = PolygonShapeConf(hx, hy);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
}

TEST(CollideShapes, EdgeRightOfPolygon)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Vec2{+1, 0} * Meter, UnitVec::GetRight()};
    
    const auto hx = 1_m;
    const auto hy = 1_m;
    const auto polygon_shape = PolygonShapeConf(hx, hy);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                -1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                +0.0, 1.0/100000.0);

    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
}

TEST(CollideShapes, EdgeInsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 3));
}

TEST(CollideShapes, EdgeTwiceInsideSquare)
{
    const auto p1 = Vec2(0, -2) * Meter;
    const auto p2 = Vec2(0, +2) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 3));
}

TEST(CollideShapes, EdgeHalfInsideSquare)
{
    const auto p1 = Vec2(0, -0.5) * Meter;
    const auto p2 = Vec2(0, +0.5) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(0, 2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
}

TEST(CollideShapes, EdgeR90InsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Length2{}, UnitVec::GetTop()};
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    // Sets up a collision between a line segment (A) and a square (B) where the line segment is
    // fully inside the square and bisects the square into an upper and low rectangular area like
    // the following (where the numbers represent coordinates on the X or Y axis)...
    //
    //     2
    //
    //   +-1-+
    //   |   |
    // 2 1-0-1 2
    //   |   |
    //   +-1-+
    //
    //     2
    //
    // Note that the square's vertex 0 is at its bottom right corner, and they go
    // counter-clockwise from there.
    //
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    if (sizeof(Real) == 4)
    {
        EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
    }
    else if (sizeof(Real) == 8)
    {
        EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
    }
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    if (sizeof(Real) == 4)
    {
        EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 0));
    }
    else if (sizeof(Real) == 8)
    {
        EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 0));
    }
}

TEST(CollideShapes, EdgeR45InsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Length2{}, UnitVec::Get(45_deg)};
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 3));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    if (sizeof(Real) == 4)
    {
        // On some platforms, the contact feature is: <01-00 01-02>.
        // On others, it's: <01-00 01-03>
        // XXX Not sure of why the difference.
        // EXPECT_EQ(cf, GetFaceFaceContactFeature(0, 2));
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(0, 3));
        // EXPECT_TRUE(cf == GetFaceFaceContactFeature(0, 2) || cf == GetFaceFaceContactFeature(0, 3));
    }
    else if (sizeof(Real) == 8)
    {
#if defined(__core2__)
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
#elif defined(__k8__)
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
#else
        EXPECT_EQ(manifold.GetContactFeature(1), GetVertexFaceContactFeature(1, 2));
#endif
    }
}

TEST(CollideShapes, EdgeR180InsideSquare)
{
    const auto p1 = Vec2(0, -1) * Meter;
    const auto p2 = Vec2(0, +1) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Length2{}, UnitVec::GetLeft()};
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    switch (sizeof(Real))
    {
        case 4:
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
            break;
        case 8:
#if defined(__core2__)
         	EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#else
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#endif
            break;
        case 16:
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
            break;
    }
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    switch (sizeof(Real))
    {
        case 4:
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
            break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
#else
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
#endif
            break;
        case 16:
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
            break;
    }
    
}

TEST(CollideShapes, EdgeTwiceR180Square)
{
    const auto p1 = Vec2(0, -2) * Meter;
    const auto p2 = Vec2(0, +2) * Meter;
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{
        Vec2{0, 1} * Meter,
        UnitVec::GetLeft()
    };
    const auto s = 1_m;
    const auto polygon_shape = PolygonShapeConf(s, s);
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                0.0, 1.0/100000.0);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    switch (sizeof(Real))
    {
        case 4:
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
            break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#else
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
#endif
            break;
        case 16:
            EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 0));
            break;
    }
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(1));
    switch (sizeof(Real))
    {
        case 4:
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
            break;
        case 8:
#if defined(__core2__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
#elif defined(__k8__)
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
#else
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
#endif
            break;
        case 16:
            EXPECT_EQ(manifold.GetContactFeature(1), GetFaceVertexContactFeature(0, 1));
            break;
    }
}

TEST(CollideShapes, EdgeFooTriangle)
{
    const auto p1 = Vec2(2, -2) * Meter;
    const auto p2 = Vec2(-2, +2) * Meter;
    const auto edge_shape = EdgeShapeConf(p2, p1,
                                      EdgeShapeConf{}.UseVertexRadius(0_m));
    const auto edge_xfm = Transformation{Vec2(0, 0.5) * Meter, UnitVec::Get(-5_deg)};
    const auto triangleTopPt = Vec2{0, +1} * Meter;
    const auto triangleLeftPt = Vec2{-1, -1} * Meter;
    const auto triangleRightPt = Vec2{+1, -1} * Meter;
    const auto polygon_shape = PolygonShapeConf{}.UseVertexRadius(0_m).Set({triangleLeftPt, triangleRightPt, triangleTopPt});
    const auto polygon_xfm = Transformation{Length2{}, UnitVec::GetRight()};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm,
                                        GetChild(polygon_shape, 0), polygon_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_EQ(manifold.GetLocalPoint(), (Length2{}));
    EXPECT_NEAR(double(GetX(GetVec2(manifold.GetLocalNormal()))), -0.707107, 0.0001);
    EXPECT_NEAR(double(GetY(GetVec2(manifold.GetLocalNormal()))), -0.707107, 0.0001);
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetFaceVertexContactFeature(0, 1));
}

TEST(CollideShapes, EdgePolygonFaceB1)
{
    auto conf = EdgeShapeConf{};
    conf.vertexRadius = 0;
    const auto edge_shape = EdgeShapeConf(Vec2(6, 8) * Meter, Vec2(7, 8) * Meter, conf);
    const auto edge_xfm = Transformation{Length2{}, GetUnitVector(Vec2(Real(0.707106769), Real(0.707106769)))};
    const auto poly_shape = PolygonShapeConf({
        Vec2(0.5, 0) * Meter,
        Vec2(0.249999985f, 0.433012724f) * Meter,
        Vec2(-0.25000003f, 0.433012694f) * Meter,
        Vec2(-0.5f, -0.0000000437113883f) * Meter,
        Vec2(-0.249999955f, -0.433012724f) * Meter,
        Vec2(0.249999955f, -0.433012724f) * Meter
    });
    const auto poly_xfm = Transformation{Vec2(-0.797443091f, 11.0397148f) * Meter, GetUnitVector(Vec2(1, 0))};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(poly_shape, 0), poly_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    EXPECT_NEAR(double(StripUnit(GetX(manifold.GetLocalPoint()))), 0.0, 0.0001);
    EXPECT_NEAR(double(StripUnit(GetY(manifold.GetLocalPoint()))), -0.43301272, 0.0001);
    EXPECT_TRUE(AlmostEqual(GetX(GetVec2(manifold.GetLocalNormal())), Real{0.0f}));
    EXPECT_TRUE(AlmostEqual(GetY(GetVec2(manifold.GetLocalNormal())), Real{-1.0f}));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(1, 4));
    EXPECT_NEAR(static_cast<double>(Real{GetX(manifold.GetOpposingPoint(0)) / Meter}),
                7.0, 0.00001);
    EXPECT_NEAR(static_cast<double>(Real{GetY(manifold.GetOpposingPoint(0)) / Meter}),
                8.0, 0.00001);
}

TEST(CollideShapes, EdgePolygonFaceB2)
{
    auto conf = EdgeShapeConf{};
    conf.vertexRadius = 0.000199999995_m;
    const auto edge_shape = EdgeShapeConf(Vec2(-6, 2) * Meter, Vec2(-6, 0) * Meter, conf);
    const auto edge_xfm = Transformation{Vec2(-9.99999904f, 4.0f) * Meter, GetUnitVector(Vec2(Real(1), Real(0)))};
    const auto poly_shape = PolygonShapeConf({
        Vec2(0.5f, -0.5f) * Meter,
        Vec2(0.5f, 0.5f) * Meter,
        Vec2(-0.5f, 0.5f) * Meter,
        Length2{}
    });
    const auto poly_xfm = Transformation{Vec2(-16.0989342f, 3.49960017f) * Meter, GetUnitVector(Vec2(1, 0))};
    
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), edge_xfm, GetChild(poly_shape, 0), poly_xfm);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceB);
    EXPECT_NEAR(double(StripUnit(GetX(manifold.GetLocalPoint()))), 0.0, 0.0001);
    EXPECT_TRUE(AlmostEqual(Real{GetY(manifold.GetLocalPoint()) / Meter}, Real{0.5f}));
    EXPECT_TRUE(AlmostEqual(GetX(GetVec2(manifold.GetLocalNormal())), Real{0.0f}));
    EXPECT_TRUE(AlmostEqual(GetY(GetVec2(manifold.GetLocalNormal())), Real{1.0f}));
    EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
    ASSERT_GT(manifold.GetPointCount(), Manifold::size_type(0));
    EXPECT_EQ(manifold.GetContactFeature(0), GetVertexFaceContactFeature(1, 1));
    EXPECT_TRUE(AlmostEqual(Real{GetX(manifold.GetOpposingPoint(0)) / Meter}, Real{-6.0f}));
    EXPECT_TRUE(AlmostEqual(Real{GetY(manifold.GetOpposingPoint(0)) / Meter}, Real{0.0f}));
}

#if 0
TEST(CollideShapes, EdgeOverlapsItself)
{
    const auto p1 = Vec2(0, -1);
    const auto p2 = Vec2(0, +1);
    const auto edge_shape = EdgeShapeConf(p1, p2);
    const auto edge_xfm = Transformation{Vec2{+1, 0}, UnitVec::GetRight()};

    const auto manifold = CollideShapes(edge_shape, edge_xfm, edge_shape, edge_xfm);
    
    ASSERT_NE(manifold.GetType(), Manifold::e_unset);
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
}
#endif

TEST(CollideShapes, R0EdgeCollinearAndTouchingR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto xfm1 = Transformation{Vec2{+1, 0} * Meter, UnitVec::GetRight()};
    const auto xfm2 = Transformation{Vec2{+3, 0} * Meter, UnitVec::GetRight()};
    const auto edge_shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(0_m).Set(p1, p2));
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), xfm1, GetChild(edge_shape, 0), xfm2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_TRUE(IsValid(manifold.GetLocalNormal()));
    if (IsValid(manifold.GetLocalNormal()))
    {
        EXPECT_NEAR(static_cast<double>(StripUnit(GetX(manifold.GetLocalNormal()))),  0.0, 0.001);
        EXPECT_NEAR(static_cast<double>(StripUnit(GetY(manifold.GetLocalNormal()))), -1.0, 0.001);
    }
    EXPECT_EQ(manifold.GetLocalPoint(), Vec2(1, 0) * Meter);
}

TEST(CollideShapes, R1EdgeCollinearAndTouchingR1Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto edge_shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(1_m).Set(p1, p2));
    const auto xfm1 = Transformation{Vec2{+1, 0} * Meter, UnitVec::GetRight()};
    const auto xfm2 = Transformation{Vec2{+5, 0} * Meter, UnitVec::GetRight()};
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), xfm1, GetChild(edge_shape, 0), xfm2);

    ASSERT_NE(manifold.GetType(), Manifold::e_unset);
    EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
    EXPECT_EQ(manifold.GetLocalPoint(), p2);
}

TEST(CollideShapes, R0EdgeCollinearAndSeparateFromR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto xfm1 = Transformation{Vec2{+1, 0} * Meter, UnitVec::GetRight()};
    const auto xfm2 = Transformation{Vec2{+4, 0} * Meter, UnitVec::GetRight()};
    const auto edge_shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(0_m).Set(p1, p2));
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), xfm1, GetChild(edge_shape, 0), xfm2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
}

TEST(CollideShapes, R0EdgeParallelAndSeparateFromR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto xfm1 = Transformation{Vec2{-4, 1} * Meter, UnitVec::GetRight()};
    const auto xfm2 = Transformation{Vec2{-4, 0} * Meter, UnitVec::GetRight()};
    const auto edge_shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(0_m).Set(p1, p2));
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), xfm1, GetChild(edge_shape, 0), xfm2);
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_unset);
}

TEST(CollideShapes, R0EdgePerpendicularCrossingFromR0Edge)
{
    const auto p1 = Vec2(-1, 0) * Meter;
    const auto p2 = Vec2(+1, 0) * Meter;
    const auto edge_shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(0_m).Set(p1, p2));
    const auto xfm1 = Transformation{Length2{}, UnitVec::GetRight()};
    const auto xfm2 = Transformation{Length2{}, UnitVec::GetTop()};
    const auto manifold = CollideShapes(GetChild(edge_shape, 0), xfm1, GetChild(edge_shape, 0), xfm2);
    
    ASSERT_NE(manifold.GetType(), Manifold::e_unset);
    ASSERT_TRUE(IsValid(manifold.GetLocalNormal()));
    ASSERT_TRUE(IsValid(manifold.GetLocalPoint()));
    
    EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))),
                -1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(StripUnit(GetX(manifold.GetLocalPoint()))), 0.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(StripUnit(GetY(manifold.GetLocalPoint()))), 0.0, 0.0001);
    EXPECT_EQ(manifold.GetPointCount(), decltype(manifold.GetPointCount()){1});
}
