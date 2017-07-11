/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/Simplex.hpp>
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Collision.hpp>
#include <Box2D/Collision/ShapeSeparation.hpp>

#include <array>
#include <bitset>
#include <algorithm>

#define BOX2D_MAGIC(x) (x)

using namespace box2d;

using index_type = IndexPair::size_type;

namespace {

inline index_type GetEdgeIndex(index_type i1, index_type i2, index_type count)
{
    if (GetModuloNext(i1, count) == i2)
    {
        return i1;
    }
    if (GetModuloNext(i2, count) == i1)
    {
        return i2;
    }
    return IndexPair::InvalidIndex;
}

inline ClipList GetClipPoints(IndexSeparation::index_type iv1, Length sideOffset1, UnitVec2 normal1,
                                     IndexSeparation::index_type iv2, Length sideOffset2, UnitVec2 normal2,
                                     const ClipList& incidentEdge)
{
    const auto points = ClipSegmentToLine(incidentEdge, normal1, sideOffset1, iv1);
    return ClipSegmentToLine(points, normal2, sideOffset2, iv2);
}

/// @param shape1 Shape 1. This should be shape A for face-A type manifold or shape B for face-B type manifold.
/// @param xf1 Transform 1. This should be transform A for face-A type manifold or transform B for face-B type manifold.
/// @param idx1 Index 1. This should be the index of the vertex and normal of shape1 that had the maximal
///    separation distance from any vertex in shape2.
/// @param idx2 Index 2. This is the index of the vertex of shape2 that had the maximal separation distance
//     from the edge of shape1 identified by idx1.
Manifold GetFaceManifold(const Manifold::Type type,
                         const DistanceProxy& shape1, const Transformation& xf1,
                         const IndexSeparation::index_type idx1,
                         const DistanceProxy& shape2, const Transformation& xf2,
                         const IndexSeparation::index_type idx2,
                         const Manifold::Conf conf)
{
    assert(type == Manifold::e_faceA || type == Manifold::e_faceB);
    assert(shape1.GetVertexCount() > 1 && shape2.GetVertexCount() > 1);
    
    const auto r1 = shape1.GetVertexRadius();
    const auto r2 = shape2.GetVertexRadius();
    const auto totalRadius = Length{r1 + r2};
    
    const auto idx1Next = GetModuloNext(idx1, shape1.GetVertexCount());
    
    const auto shape1_rel_vertex1 = shape1.GetVertex(idx1);
    const auto shape1_rel_vertex2 = shape1.GetVertex(idx1Next);
    const auto shape1_abs_vertex1 = Transform(shape1_rel_vertex1, xf1);
    const auto shape1_abs_vertex2 = Transform(shape1_rel_vertex2, xf1);
    
    const auto shape1_rel_edge1 = shape1_rel_vertex2 - shape1_rel_vertex1;
    assert(IsValid(shape1_rel_edge1));
    auto shape1_len_edge1 = Length{0};
    const auto shape1_rel_edge1_dir = GetUnitVector(shape1_rel_edge1, shape1_len_edge1, UnitVec2::GetZero());
    assert(IsValid(shape1_rel_edge1_dir));
    const auto shape1_edge1_abs_dir = Rotate(shape1_rel_edge1_dir, xf1.q);
    
    // Clip incident edge against extruded edge1 side edges.
    // Side offsets, extended by polytope skin thickness.
    
    const auto shape1_rel_normal = InverseRotate(Rotate(shape1.GetNormal(idx1), xf1.q), xf2.q);
    const auto shape2_idx0 = GetModuloPrev(idx2, shape2.GetVertexCount());
    const auto shape2_idx1 = idx2;
    const auto shape2_normal0 = shape2.GetNormal(shape2_idx0);
    const auto shape2_normal1 = shape2.GetNormal(shape2_idx1);
    const auto shape2_s0 = Dot(shape1_rel_normal, shape2_normal0);
    const auto shape2_s1 = Dot(shape1_rel_normal, shape2_normal1);
    const auto shape2_i1 = (shape2_s0 < shape2_s1)? shape2_idx0: shape2_idx1;
    const auto shape2_i2 = GetModuloNext(shape2_i1, shape2.GetVertexCount()); /// XXX is this correct?
    const auto clipPoints = [&]()
    {
        const auto incidentEdge = ClipList{
            ClipVertex{Transform(shape2.GetVertex(shape2_i1), xf2), GetFaceVertexContactFeature(idx1, shape2_i1)},
            ClipVertex{Transform(shape2.GetVertex(shape2_i2), xf2), GetFaceVertexContactFeature(idx1, shape2_i2)}
        };
        // Gets the two vertices in world coordinates and their face-vertex contact features
        // of the incident edge of shape2
        //const auto incidentEdge = GetIncidentEdgeClipList(idx1, shape1.GetNormal(idx1), xf1, shape2, xf2, idx2);
        assert(incidentEdge[0].cf.indexB == idx2 || incidentEdge[1].cf.indexB == idx2);
        const auto shape1_dp_v1_e1 = Dot(shape1_edge1_abs_dir, shape1_abs_vertex1);
        const auto shape1_dp_v2_e1 = Dot(shape1_edge1_abs_dir, shape1_abs_vertex2);
        return GetClipPoints(idx1, -shape1_dp_v1_e1, -shape1_edge1_abs_dir,
                             idx1Next, +shape1_dp_v2_e1, shape1_edge1_abs_dir,
                             incidentEdge);
    }();
    if (clipPoints.size() == 2)
    {
        const auto abs_normal = GetFwdPerpendicular(shape1_edge1_abs_dir); // Normal points from 1 to 2
        const auto rel_midpoint = (shape1_rel_vertex1 + shape1_rel_vertex2) / Real{2};
        const auto abs_offset = Dot(abs_normal, shape1_abs_vertex1); ///< Face offset.
        
        auto manifold = Manifold{};
        switch (type)
        {
            case Manifold::e_faceA:
            {
                manifold = Manifold::GetForFaceA(GetFwdPerpendicular(shape1_rel_edge1_dir), rel_midpoint);
                for (auto&& cp: clipPoints)
                {
                    if ((Dot(abs_normal, cp.v) - abs_offset) <= totalRadius)
                    {
                        manifold.AddPoint(Manifold::Point{InverseTransform(cp.v, xf2), cp.cf});
                        //manifold.AddPoint(cp.cf.typeB, cp.cf.indexB, InverseTransform(cp.v, xf2));
                    }
                }
                break;
            }
            case Manifold::e_faceB:
            {
                manifold = Manifold::GetForFaceB(GetFwdPerpendicular(shape1_rel_edge1_dir), rel_midpoint);
                for (auto&& cp: clipPoints)
                {
                    if ((Dot(abs_normal, cp.v) - abs_offset) <= totalRadius)
                    {
                        manifold.AddPoint(Manifold::Point{InverseTransform(cp.v, xf2), Flip(cp.cf)});
                    }
                }
                break;
            }
            default:
                break;
        }
        if (manifold.GetPointCount() > 0)
        {
            return manifold;
        }
    }
    
    // If the shapes are colliding, then they're colliding with each others corners.
    // Using a circles manifold, means these corners will repell each other with a normal
    // that's in the direction between the two vertices.
    // That's problematic though for things like polygons sliding over edges where a face
    // manifold that favors the primary edge can work better.
    // Use a threshold against the ratio of the square of the vertex radius to the square
    // of the length of the primary edge to determine whether to return a circles manifold
    // or a face manifold.
    const auto shape2_rel_vertex1 = shape2.GetVertex(shape2_i1);
    const auto shape2_abs_vertex1 = Transform(shape2_rel_vertex1, xf2);
    const auto shape2_rel_vertex2 = shape2.GetVertex(shape2_i2);
    const auto shape2_abs_vertex2 = Transform(shape2_rel_vertex2, xf2);
    const auto totalRadiusSquared = Square(totalRadius);
    const auto mustUseFaceManifold = shape1_len_edge1 > (conf.maxCirclesRatio * r1);
    if (GetLengthSquared(shape1_abs_vertex1 - shape2_abs_vertex1) <= totalRadiusSquared)
    {
        // shape 1 vertex 1 is colliding with shape 2 vertex 1
        // shape 1 vertex 1 is the vertex at index idx1, or one before idx1Next.
        // shape 2 vertex 1 is the vertex at index shape2_i1, or one before shape2_i2.
        switch (type)
        {
            case Manifold::e_faceA:
                // shape 1 is shape A.
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceA(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1,
                                                 shape1_rel_vertex1, ContactFeature::e_vertex,
                                                 shape2_i1, shape2_rel_vertex1);
                }
                return Manifold::GetForCircles(shape1_rel_vertex1, idx1, shape2_rel_vertex1, shape2_i1);
            case Manifold::e_faceB:
                // shape 2 is shape A.
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceB(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1,
                                                 shape1_rel_vertex1, ContactFeature::e_vertex,
                                                 shape2_i1, shape2_rel_vertex1);
                }
                return Manifold::GetForCircles(shape2_rel_vertex1, shape2_i1, shape1_rel_vertex1, idx1);
            default:
                break;
        }
    }
    else if (GetLengthSquared(shape1_abs_vertex1 - shape2_abs_vertex2) <= totalRadiusSquared)
    {
        // shape 1 vertex 1 is colliding with shape 2 vertex 2
        switch (type)
        {
            case Manifold::e_faceA:
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceA(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1,
                                                 shape1_rel_vertex1, ContactFeature::e_vertex,
                                                 shape2_i2, shape2_rel_vertex2);
                }
                return Manifold::GetForCircles(shape1_rel_vertex1, idx1, shape2_rel_vertex2, shape2_i2);
            case Manifold::e_faceB:
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceB(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1,
                                                 shape1_rel_vertex1, ContactFeature::e_vertex,
                                                 shape2_i2, shape2_rel_vertex2);
                }
                return Manifold::GetForCircles(shape2_rel_vertex2, shape2_i2, shape1_rel_vertex1, idx1);
            default:
                break;
        }
    }
    else if (GetLengthSquared(shape1_abs_vertex2 - shape2_abs_vertex2) <= totalRadiusSquared)
    {
        // shape 1 vertex 2 is colliding with shape 2 vertex 2
        switch (type)
        {
            case Manifold::e_faceA:
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceA(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1Next,
                                                 shape1_rel_vertex2, ContactFeature::e_vertex,
                                                 shape2_i2, shape2_rel_vertex2);
                }
                return Manifold::GetForCircles(shape1_rel_vertex2, idx1Next, shape2_rel_vertex2, shape2_i2);
            case Manifold::e_faceB:
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceB(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1Next,
                                                 shape1_rel_vertex2, ContactFeature::e_vertex,
                                                 shape2_i2, shape2_rel_vertex2);
                }
                return Manifold::GetForCircles(shape2_rel_vertex2, shape2_i2, shape1_rel_vertex2, idx1Next);
            default:
                break;
        }
    }
    else if (GetLengthSquared(shape1_abs_vertex2 - shape2_abs_vertex1) <= totalRadiusSquared)
    {
        // shape 1 vertex 2 is colliding with shape 2 vertex 1
        switch (type)
        {
            case Manifold::e_faceA:
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceA(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1Next,
                                                 shape1_rel_vertex2, ContactFeature::e_vertex,
                                                 shape2_i1, shape2_rel_vertex1);
                }
                return Manifold::GetForCircles(shape1_rel_vertex2, idx1Next, shape2_rel_vertex1, shape2_i1);
            case Manifold::e_faceB:
                if (mustUseFaceManifold)
                {
                    return Manifold::GetForFaceB(GetFwdPerpendicular(shape1_rel_edge1_dir), idx1Next,
                                                 shape1_rel_vertex2, ContactFeature::e_vertex,
                                                 shape2_i1, shape2_rel_vertex1);
                }
                return Manifold::GetForCircles(shape2_rel_vertex1, shape2_i1, shape1_rel_vertex2, idx1Next);
            default:
                break;
        }
    }
    return Manifold{};
}

Manifold CollideShapes(Manifold::Type type,
                       const DistanceProxy& shape, const Transformation& sxf,
                       Length2D point, Length radius, const Transformation& xfm)
{
    // Computes the center of the circle in the frame of the polygon.
    const auto cLocal = InverseTransform(Transform(point, xfm), sxf); ///< Center of circle in frame of polygon.
    
    const auto totalRadius = shape.GetVertexRadius() + radius;
    const auto vertexCount = shape.GetVertexCount();
    
    // Find edge that circle is closest to.
    auto indexOfMax = decltype(vertexCount){0};
    auto maxSeparation = -MaxFloat * Meter;
    {
        for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
        {
            // Get circle's distance from vertex[i] in direction of normal[i].
            const auto s = Dot(shape.GetNormal(i), cLocal - shape.GetVertex(i));
            if (s > totalRadius)
            {
                // Early out - no contact.
                return Manifold{};
            }
            if (maxSeparation < s)
            {
                maxSeparation = s;
                indexOfMax = i;
            }
        }
    }
    const auto indexOfMax2 = GetModuloNext(indexOfMax, vertexCount);
    assert(maxSeparation <= totalRadius);
    
    // Vertices that subtend the incident face.
    const auto v1 = shape.GetVertex(indexOfMax);
    const auto v2 = shape.GetVertex(indexOfMax2);
    
    if (maxSeparation < Length{0})
    {
        const auto faceCenter = (v1 + v2) / Real{2};
        // Circle's center is inside the polygon and closest to edge[indexOfMax].
        switch (type)
        {
            case box2d::Manifold::e_faceA:
                return Manifold::GetForFaceA(shape.GetNormal(indexOfMax), indexOfMax, faceCenter,
                                             ContactFeature::e_vertex, 0, point);
            case box2d::Manifold::e_faceB:
                return Manifold::GetForFaceB(shape.GetNormal(indexOfMax), indexOfMax, faceCenter,
                                             ContactFeature::e_vertex, 0, point);
            default: break;
        }
        return Manifold{};
    }
    
    // Circle's center is outside polygon and closest to edge[indexOfMax].
    // Compute barycentric coordinates.
    
    const auto cLocalV1 = cLocal - v1;
    if (Dot(cLocalV1, v2 - v1) <= Area{0})
    {
        // Circle's center right of v1 (in direction of v1 to v2).
        if (GetLengthSquared(cLocalV1) > Square(totalRadius))
        {
            return Manifold{};
        }
        switch (type)
        {
            case box2d::Manifold::e_faceA:
                return Manifold::GetForCircles(v1, indexOfMax, point, 0);
            case box2d::Manifold::e_faceB:
                return Manifold::GetForCircles(point, 0, v1, indexOfMax);
            default: break;
        }
        return Manifold{};
    }
    
    const auto ClocalV2 = cLocal - v2;
    if (Dot(ClocalV2, v1 - v2) <= Area{0})
    {
        // Circle's center left of v2 (in direction of v2 to v1).
        if (GetLengthSquared(ClocalV2) > Square(totalRadius))
        {
            return Manifold{};
        }
        switch (type)
        {
            case box2d::Manifold::e_faceA:
                return Manifold::GetForCircles(v2, indexOfMax2, point, 0);
            case box2d::Manifold::e_faceB:
                return Manifold::GetForCircles(point, 0, v2, indexOfMax2);
            default: break;
        }
        return Manifold{};
    }
    
    // Circle's center is between v1 and v2.
    const auto faceCenter = (v1 + v2) / Real{2};
    if (Dot(cLocal - faceCenter, shape.GetNormal(indexOfMax)) > totalRadius)
    {
        return Manifold{};
    }
    switch (type)
    {
        case box2d::Manifold::e_faceA:
            return Manifold::GetForFaceA(shape.GetNormal(indexOfMax), indexOfMax, faceCenter,
                                         ContactFeature::e_vertex, 0, point);
        case box2d::Manifold::e_faceB:
            return Manifold::GetForFaceB(shape.GetNormal(indexOfMax), indexOfMax, faceCenter,
                                         ContactFeature::e_vertex, 0, point);
        default: break;
    }
    return Manifold{};
}

Manifold CollideShapes(Length2D locationA, Length radiusA, const Transformation& xfA,
                       Length2D locationB, Length radiusB, const Transformation& xfB)
{
    const auto pA = Transform(locationA, xfA);
    const auto pB = Transform(locationB, xfB);
    const auto totalRadius = radiusA + radiusB;
    return (GetLengthSquared(pB - pA) > Square(totalRadius))?
    Manifold{}: Manifold::GetForCircles(locationA, 0, locationB, 0);
}

} // anonymous namespace

/*
 * Definition of public CollideShapes functions.
 * All CollideShapes functions return a Manifold object.
 */

Manifold box2d::CollideShapes(const DistanceProxy& shapeA, const Transformation& xfA,
                              const DistanceProxy& shapeB, const Transformation& xfB,
                              const Manifold::Conf conf)
{
    // Find edge normal of max separation on A - return if separating axis is found
    // Find edge normal of max separation on B - return if separation axis is found
    // Choose reference edge as min(minA, minB)
    // Find incident edge
    // Clip
    
    const auto vertexCountShapeA = shapeA.GetVertexCount();
    const auto vertexCountShapeB = shapeB.GetVertexCount();
    if (vertexCountShapeA == 1)
    {
        if (vertexCountShapeB > 1)
        {
            return ::CollideShapes(Manifold::e_faceB, shapeB, xfB,
                                   shapeA.GetVertex(0), shapeA.GetVertexRadius(), xfA);
        }
        return ::CollideShapes(shapeA.GetVertex(0), shapeA.GetVertexRadius(), xfA,
                               shapeB.GetVertex(0), shapeB.GetVertexRadius(), xfB);
    }
    if (vertexCountShapeB == 1)
    {
        if (vertexCountShapeA > 1)
        {
            return ::CollideShapes(Manifold::e_faceA, shapeA, xfA,
                                   shapeB.GetVertex(0), shapeB.GetVertexRadius(), xfB);
        }
        return ::CollideShapes(shapeA.GetVertex(0), shapeA.GetVertexRadius(), xfA,
                               shapeB.GetVertex(0), shapeB.GetVertexRadius(), xfB);
    }
    
    const auto totalRadius = shapeA.GetVertexRadius() + shapeB.GetVertexRadius();
    
    const auto edgeSepA = ::GetMaxSeparation(shapeA, xfA, shapeB, xfB, totalRadius);
    if (edgeSepA.separation > totalRadius)
    {
        return Manifold{};
    }
    
    const auto edgeSepB = ::GetMaxSeparation(shapeB, xfB, shapeA, xfA, totalRadius);
    if (edgeSepB.separation > totalRadius)
    {
        return Manifold{};
    }
    
    constexpr auto k_tol = BOX2D_MAGIC(DefaultLinearSlop / Real{10});
    return (edgeSepB.separation > (edgeSepA.separation + k_tol))?
        GetFaceManifold(Manifold::e_faceB,
                        shapeB, xfB, edgeSepB.index1,
                        shapeA, xfA, edgeSepB.index2,
                        conf):
        GetFaceManifold(Manifold::e_faceA,
                        shapeA, xfA, edgeSepA.index1,
                        shapeB, xfB, edgeSepA.index2,
                        conf);
}

#if 0
Manifold box2d::CollideCached(const DistanceProxy& shapeA, const Transformation& xfA,
                              const DistanceProxy& shapeB, const Transformation& xfB,
                              const Manifold::Conf conf)
{
    // Find edge normal of max separation on A - return if separating axis is found
    // Find edge normal of max separation on B - return if separation axis is found
    // Choose reference edge as min(minA, minB)
    // Find incident edge
    // Clip
    
    const auto vertexCountShapeA = shapeA.GetVertexCount();
    const auto vertexCountShapeB = shapeB.GetVertexCount();
    if (vertexCountShapeA == 1)
    {
        if (vertexCountShapeB > 1)
        {
            return ::CollideShapes(Manifold::e_faceB, shapeB, xfB,
                                   shapeA.GetVertex(0), shapeA.GetVertexRadius(), xfA);
        }
        return ::CollideShapes(shapeA.GetVertex(0), shapeA.GetVertexRadius(), xfA,
                               shapeB.GetVertex(0), shapeB.GetVertexRadius(), xfB);
    }
    if (vertexCountShapeB == 1)
    {
        if (vertexCountShapeA > 1)
        {
            return ::CollideShapes(Manifold::e_faceA, shapeA, xfA,
                                   shapeB.GetVertex(0), shapeB.GetVertexRadius(), xfB);
        }
        return ::CollideShapes(shapeA.GetVertex(0), shapeA.GetVertexRadius(), xfA,
                               shapeB.GetVertex(0), shapeB.GetVertexRadius(), xfB);
    }

    const auto totalRadius = shapeA.GetVertexRadius() + shapeB.GetVertexRadius();

    IndexPairSeparation edgeSepA;
    IndexPairSeparation edgeSepB;

    if (vertexCountShapeA == 4 && vertexCountShapeB == 4)
    {
        Length2D verticesA[4];
        Length2D verticesB[4];
        UnitVec2 normalsA[4];
        UnitVec2 normalsB[4];
        
        verticesA[0] = Transform(shapeA.GetVertex(0), xfA);
        verticesA[1] = Transform(shapeA.GetVertex(1), xfA);
        verticesA[2] = Transform(shapeA.GetVertex(2), xfA);
        verticesA[3] = Transform(shapeA.GetVertex(3), xfA);
        
        normalsA[0] = Rotate(shapeA.GetNormal(0), xfA.q);
        normalsA[1] = Rotate(shapeA.GetNormal(1), xfA.q);
        normalsA[2] = Rotate(shapeA.GetNormal(2), xfA.q);
        normalsA[3] = Rotate(shapeA.GetNormal(3), xfA.q);

        verticesB[0] = Transform(shapeB.GetVertex(0), xfB);
        verticesB[1] = Transform(shapeB.GetVertex(1), xfB);
        verticesB[2] = Transform(shapeB.GetVertex(2), xfB);
        verticesB[3] = Transform(shapeB.GetVertex(3), xfB);

        normalsB[0] = Rotate(shapeB.GetNormal(0), xfB.q);
        normalsB[1] = Rotate(shapeB.GetNormal(1), xfB.q);
        normalsB[2] = Rotate(shapeB.GetNormal(2), xfB.q);
        normalsB[3] = Rotate(shapeB.GetNormal(3), xfB.q);
        
        const auto dpA = DistanceProxy{shapeA.GetVertexRadius(), vertexCountShapeA, verticesA, normalsA};
        const auto dpB = DistanceProxy{shapeB.GetVertexRadius(), vertexCountShapeB, verticesB, normalsB};
        edgeSepA = ::GetMaxSeparation(dpA, dpB, totalRadius);
        if (edgeSepA.separation > totalRadius)
        {
            return Manifold{};
        }
        edgeSepB = ::GetMaxSeparation(dpB, dpA, totalRadius);
        if (edgeSepB.separation > totalRadius)
        {
            return Manifold{};
        }
    }
    else
    {
        edgeSepA = ::GetMaxSeparation(shapeA, xfA, shapeB, xfB, totalRadius);
        if (edgeSepA.separation > totalRadius)
        {
            return Manifold{};
        }
        edgeSepB = ::GetMaxSeparation(shapeB, xfB, shapeA, xfA, totalRadius);
        if (edgeSepB.separation > totalRadius)
        {
            return Manifold{};
        }
    }
    
    constexpr auto k_tol = BOX2D_MAGIC(DefaultLinearSlop / Real{10});
    return (edgeSepB.separation > (edgeSepA.separation + k_tol))?
    GetFaceManifold(Manifold::e_faceB,
                    shapeB, xfB, edgeSepB.index1,
                    shapeA, xfA, edgeSepB.index2,
                    conf):
    GetFaceManifold(Manifold::e_faceA,
                    shapeA, xfA, edgeSepA.index1,
                    shapeB, xfB, edgeSepA.index2,
                    conf);
}
#endif

Manifold box2d::GetManifold(const DistanceProxy& proxyA, const Transformation& transformA,
                            const DistanceProxy& proxyB, const Transformation& transformB)
{
    const auto distanceInfo = Distance(proxyA, transformA, proxyB, transformB);
    const auto totalRadius = proxyA.GetVertexRadius() + proxyB.GetVertexRadius();
    const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);

    const auto distance = Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b));
    if (distance > totalRadius)
    {
        // no collision
        return Manifold{};
    }

    const auto a_count = proxyA.GetVertexCount();
    const auto b_count = proxyB.GetVertexCount();

    index_type a_indices_array[Simplex::MaxEdges];
    index_type b_indices_array[Simplex::MaxEdges];
    auto uniqA = std::size_t{0};
    auto uniqB = std::size_t{0};
    {
        std::bitset<MaxShapeVertices> a_indices_set;
        std::bitset<MaxShapeVertices> b_indices_set;
        for (auto&& e: distanceInfo.simplex.GetEdges())
        {
            const auto indexA = e.GetIndexA();
            if (!a_indices_set[indexA])
            {
                a_indices_set[indexA] = true;
                a_indices_array[uniqA] = indexA;
                ++uniqA;
            }
            const auto indexB = e.GetIndexB();
            if (!b_indices_set[indexB])
            {
                b_indices_set[indexB] = true;
                b_indices_array[uniqB] = indexB;
                ++uniqB;
            }
        }
    }

    assert(uniqA > 0 && uniqB > 0);

    std::sort(a_indices_array, a_indices_array + uniqA);
    std::sort(b_indices_array, b_indices_array + uniqB);

    if (uniqA < uniqB)
    {
        switch (uniqA)
        {
            case 1: // uniqB must be 2 or 3
            {
                const auto b_idx0 = GetEdgeIndex(b_indices_array[0], b_indices_array[1], b_count);
                assert(b_idx0 != IndexPair::InvalidIndex);
                const auto b_idx1 = GetModuloNext(b_idx0, b_count);
                const auto b_v0 = proxyB.GetVertex(b_idx0);
                const auto b_v1 = proxyB.GetVertex(b_idx1);
                const auto lp = (b_v0 + b_v1) / Real{2};
                const auto ln = GetFwdPerpendicular(GetUnitVector(b_v1 - b_v0));
                const auto mp0 = Manifold::Point{
                    proxyA.GetVertex(a_indices_array[0]),
                    ContactFeature{
                        ContactFeature::e_vertex,
                        a_indices_array[0],
                        ContactFeature::e_face,
                        b_idx0,
                    }
                };
                return Manifold::GetForFaceB(ln, lp, mp0);
            }
            case 2: // uniqB must be 3
            {
                auto mp0 = Manifold::Point{};
                auto mp1 = Manifold::Point{};
                mp0.contactFeature.typeA = ContactFeature::e_face;
                mp1.contactFeature.typeA = ContactFeature::e_face;
                const auto v0 = proxyA.GetVertex(a_indices_array[0]);
                const auto v1 = proxyA.GetVertex(a_indices_array[1]);
                const auto lp = (v0 + v1) / Real{2};
                const auto count = proxyA.GetVertexCount();
                if ((a_indices_array[1] - a_indices_array[0]) == 1)
                {
                    mp0.contactFeature.indexA = a_indices_array[0];
                    mp1.contactFeature.indexA = a_indices_array[0];
                    const auto ln = GetFwdPerpendicular(GetUnitVector(v1 - v0));
                    return Manifold::GetForFaceA(ln, lp, mp0, mp1);
                }
                else if (GetModuloNext(a_indices_array[1], count) == a_indices_array[0])
                {
                    mp0.contactFeature.indexA = a_indices_array[1];
                    mp1.contactFeature.indexA = a_indices_array[1];
                    const auto ln = GetFwdPerpendicular(GetUnitVector(v0 - v1));
                    return Manifold::GetForFaceA(ln, lp, mp0, mp1);
                }
                else
                {
                    //assert(false);
                }
                return Manifold{};
            }
            default:
                break;
        }
    }
    else if (uniqB < uniqA)
    {
        switch (uniqB)
        {
            case 1: // uniqA must be 2 or 3
            {
                const auto a_idx0 = GetEdgeIndex(a_indices_array[0],a_indices_array[1], a_count);
                assert(a_idx0 != IndexPair::InvalidIndex);
                const auto a_idx1 = GetModuloNext(a_idx0, a_count);
                const auto a_v0 = proxyA.GetVertex(a_idx0);
                const auto a_v1 = proxyA.GetVertex(a_idx1);
                const auto lp = (a_v0 + a_v1) / Real{2};
                const auto ln = GetFwdPerpendicular(GetUnitVector(a_v1 - a_v0));
                const auto mp0 = Manifold::Point{
                    proxyB.GetVertex(b_indices_array[0]),
                    ContactFeature{
                        ContactFeature::e_face,
                        a_idx0,
                        ContactFeature::e_vertex,
                        b_indices_array[0]
                    }
                };
                return Manifold::GetForFaceA(ln, lp, mp0);
            }
            case 2: // uniqA must be 3
            {
                auto mp0 = Manifold::Point{};
                auto mp1 = Manifold::Point{};
                mp0.contactFeature.typeB = ContactFeature::e_face;
                mp1.contactFeature.typeB = ContactFeature::e_face;
                const auto v0 = proxyB.GetVertex(b_indices_array[0]);
                const auto v1 = proxyB.GetVertex(b_indices_array[1]);
                const auto lp = (v0 + v1) / Real{2};
                const auto count = proxyB.GetVertexCount();
                if ((b_indices_array[1] - b_indices_array[0]) == 1)
                {
                    mp0.contactFeature.indexB = b_indices_array[0];
                    mp1.contactFeature.indexB = b_indices_array[0];
                    const auto ln = GetFwdPerpendicular(GetUnitVector(v1 - v0));
                    return Manifold::GetForFaceB(ln, lp, mp0, mp1);
                }
                else if (GetModuloNext(b_indices_array[1], count) == b_indices_array[0])
                {
                    mp0.contactFeature.indexB = b_indices_array[1];
                    mp1.contactFeature.indexB = b_indices_array[1];
                    const auto ln = GetFwdPerpendicular(GetUnitVector(v0 - v1));
                    return Manifold::GetForFaceB(ln, lp, mp0, mp1);
                }
                else
                {
                    //assert(false);
                }
                return Manifold{};
            }
            default:
                break;
        }
    }
    else // uniqA == uniqB
    {
        switch (uniqA)
        {
            case 1:
            {
                return Manifold::GetForCircles(proxyA.GetVertex(a_indices_array[0]), a_indices_array[0],
                                               proxyB.GetVertex(b_indices_array[0]), b_indices_array[0]);
            }
            case 2:
            {
                const auto v0 = proxyA.GetVertex(a_indices_array[0]);
                const auto v1 = proxyA.GetVertex(a_indices_array[1]);
                const auto lp = (v0 + v1) / Real{2};
                const auto count = proxyA.GetVertexCount();
                auto mp0 = Manifold::Point{};
                auto mp1 = Manifold::Point{};
                mp0.contactFeature.typeB = ContactFeature::e_vertex;
                mp0.contactFeature.indexB = b_indices_array[0];
                mp0.localPoint = proxyB.GetVertex(mp0.contactFeature.indexB);
                mp1.contactFeature.typeB = ContactFeature::e_vertex;
                mp1.contactFeature.indexB = b_indices_array[1];
                mp1.localPoint = proxyB.GetVertex(mp1.contactFeature.indexB);
                if ((a_indices_array[1] - a_indices_array[0]) == 1)
                {
                    mp0.contactFeature.typeA = ContactFeature::e_face;
                    mp0.contactFeature.indexA = a_indices_array[0];
                    mp1.contactFeature.typeA = ContactFeature::e_face;
                    mp1.contactFeature.indexA = a_indices_array[0];
                    const auto ln = GetFwdPerpendicular(GetUnitVector(v1 - v0));
                    return Manifold::GetForFaceA(ln, lp, mp0, mp1);
                }
                if (GetModuloNext(a_indices_array[1], count) == a_indices_array[0])
                {
                    mp0.contactFeature.typeA = ContactFeature::e_face;
                    mp0.contactFeature.indexA = a_indices_array[1];
                    mp1.contactFeature.typeA = ContactFeature::e_face;
                    mp1.contactFeature.indexA = a_indices_array[1];
                    const auto ln = GetFwdPerpendicular(GetUnitVector(v0 - v1));
                    return Manifold::GetForFaceA(ln, lp, mp0, mp1);
                }
                assert(false);
                break;
            }
            case 3:
            {
                const auto ln = UnitVec2::GetLeft();
                const auto lp = Length2D{};
                return Manifold::GetForFaceA(ln, lp);
            }
            default:
                break;
        }
    }

    return Manifold{};
}

const char* box2d::GetName(Manifold::Type type) noexcept
{
    switch (type)
    {
        case Manifold::e_unset: return "unset";
        case Manifold::e_circles: return "circles";
        case Manifold::e_faceA: return "face-a";
        case Manifold::e_faceB: return "face-b";
    }
    return "unknown";
}

bool box2d::operator==(const Manifold::Point& lhs, const Manifold::Point& rhs)
{
    if (lhs.localPoint != rhs.localPoint)
    {
        return false;
    }
    if (lhs.contactFeature != rhs.contactFeature)
    {
        return false;
    }
    if (lhs.normalImpulse != rhs.normalImpulse)
    {
        return false;
    }
    if (lhs.tangentImpulse != rhs.tangentImpulse)
    {
        return false;
    }
    return true;
}

bool box2d::operator!=(const Manifold::Point& lhs, const Manifold::Point& rhs)
{
    return !(lhs == rhs);
}

bool box2d::operator==(const Manifold& lhs, const Manifold& rhs)
{
    if (lhs.GetType() != rhs.GetType())
    {
        return false;
    }
    
    if (lhs.GetLocalPoint() != rhs.GetLocalPoint())
    {
        return false;
    }
    
    if (IsValid(lhs.GetLocalNormal()) != IsValid(rhs.GetLocalNormal()))
    {
        return false;
    }

    if (IsValid(lhs.GetLocalNormal()) && (lhs.GetLocalNormal() != rhs.GetLocalNormal()))
    {
        return false;
    }
    
    if (lhs.GetPointCount() != rhs.GetPointCount())
    {
        return false;
    }

    const auto count = lhs.GetPointCount();
    assert(count <= 2);
    switch (count)
    {
        case 0:
            break;
        case 1:
            if (lhs.GetPoint(0) != rhs.GetPoint(0))
            {
                return false;
            }
            break;
        case 2:
            if (lhs.GetPoint(0) != rhs.GetPoint(0))
            {
                if (lhs.GetPoint(0) != rhs.GetPoint(1))
                {
                    return false;
                }
                if (lhs.GetPoint(1) != rhs.GetPoint(0))
                {
                    return false;
                }
            }
            else if (lhs.GetPoint(1) != rhs.GetPoint(1))
            {
                return false;
            }
            break;
    }

    return true;
}

bool box2d::operator!=(const Manifold& lhs, const Manifold& rhs)
{
    return !(lhs == rhs);
}

Length2D box2d::GetLocalPoint(const DistanceProxy& proxy, ContactFeature::Type type, ContactFeature::Index index)
{
    switch (type)
    {
        case ContactFeature::e_vertex:
            return proxy.GetVertex(index);
        case ContactFeature::e_face:
        {
            return proxy.GetVertex(index);
        }
    }
    return GetInvalid<Length2D>();
}
