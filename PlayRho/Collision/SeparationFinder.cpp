/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Collision/SeparationFinder.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>

namespace playrho {

SeparationFinder SeparationFinder::Get(IndexPair3 indices,
                                       const DistanceProxy& proxyA, const Transformation2D& xfA,
                                       const DistanceProxy& proxyB, const Transformation2D& xfB)
{
    assert(!IsEmpty(indices));
    assert(proxyA.GetVertexCount() > 0);
    assert(proxyB.GetVertexCount() > 0);
    
    const auto numIndices = GetNumValidIndices(indices);
    const auto type = (numIndices == 1)? e_points: ((std::get<0>(indices[0]) == std::get<0>(indices[1]))? e_faceB: e_faceA);
    
    switch (type)
    {
        case e_points:
        {
            const auto ip0 = indices[0];
            const auto localPointA = proxyA.GetVertex(std::get<0>(ip0));
            const auto localPointB = proxyB.GetVertex(std::get<1>(ip0));
            const auto pointA = Transform(localPointA, xfA);
            const auto pointB = Transform(localPointB, xfB);
            const auto axis = GetUnitVector(pointB - pointA, UnitVec2::GetZero());
            return SeparationFinder{proxyA, proxyB, axis, GetInvalid<Length2>(), type};
        }
        case e_faceB:
        {
            const auto ip0 = indices[0];
            const auto ip1 = indices[1];
            
            // Two points on B and one on A.
            const auto localPointB1 = proxyB.GetVertex(std::get<1>(ip0));
            const auto localPointB2 = proxyB.GetVertex(std::get<1>(ip1));
            const auto axis = GetUnitVector(GetFwdPerpendicular(localPointB2 - localPointB1),
                                            UnitVec2::GetZero());
            const auto normal = Rotate(axis, xfB.q);
            const auto localPoint = (localPointB1 + localPointB2) / 2;
            const auto pointB = Transform(localPoint, xfB);
            const auto localPointA = proxyA.GetVertex(std::get<0>(ip0));
            const auto pointA = Transform(localPointA, xfA);
            const auto deltaPoint = pointA - pointB;
            return SeparationFinder{
                proxyA, proxyB,
                (Dot(deltaPoint, normal) < 0_m)? -axis: axis,
                localPoint, type
            };
        }
        case e_faceA:
        {
            const auto ip0 = indices[0];
            const auto ip1 = indices[1];
            
            // Two points on A and one or two points on B.
            const auto localPointA1 = proxyA.GetVertex(std::get<0>(ip0));
            const auto localPointA2 = proxyA.GetVertex(std::get<0>(ip1));
            const auto axis = GetUnitVector(GetFwdPerpendicular(localPointA2 - localPointA1),
                                            UnitVec2::GetZero());
            const auto normal = Rotate(axis, xfA.q);
            const auto localPoint = (localPointA1 + localPointA2) / 2;
            const auto pointA = Transform(localPoint, xfA);
            const auto localPointB = proxyB.GetVertex(std::get<1>(ip0));
            const auto pointB = Transform(localPointB, xfB);
            const auto deltaPoint = pointB - pointA;
            return SeparationFinder{
                proxyA, proxyB,
                (Dot(deltaPoint, normal) < 0_m)? -axis: axis,
                localPoint, type
            };
        }
    }

    PLAYRHO_UNREACHABLE;
    return SeparationFinder{proxyA, proxyB, UnitVec2{}, GetInvalid<Length2>(), type};
}

LengthIndexPair SeparationFinder::FindMinSeparation(const Transformation2D& xfA,
                                                    const Transformation2D& xfB) const
{
    switch (m_type)
    {
        case e_points: return FindMinSeparationForPoints(xfA, xfB);
        case e_faceA: return FindMinSeparationForFaceA(xfA, xfB);
        case e_faceB: return FindMinSeparationForFaceB(xfA, xfB);
    }
    PLAYRHO_UNREACHABLE;
    return LengthIndexPair{0, InvalidIndexPair};
}

Length SeparationFinder::Evaluate(const Transformation2D& xfA, const Transformation2D& xfB,
                                  IndexPair indexPair) const
{
    switch (m_type)
    {
        case e_points: return EvaluateForPoints(xfA, xfB, indexPair);
        case e_faceA: return EvaluateForFaceA(xfA, xfB, indexPair);
        case e_faceB: return EvaluateForFaceB(xfA, xfB, indexPair);
    }
    PLAYRHO_UNREACHABLE;
    return 0_m;
}

LengthIndexPair
SeparationFinder::FindMinSeparationForPoints(const Transformation2D& xfA,
                                             const Transformation2D& xfB) const
{
    const auto dirA = InverseRotate(+m_axis, xfA.q);
    const auto dirB = InverseRotate(-m_axis, xfB.q);
    const auto indexA = GetSupportIndex(m_proxyA, dirA);
    const auto indexB = GetSupportIndex(m_proxyB, dirB);
    const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
    const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
    const auto delta = pointB - pointA;
    return LengthIndexPair{Dot(delta, m_axis), IndexPair{indexA, indexB}};
}

LengthIndexPair
SeparationFinder::FindMinSeparationForFaceA(const Transformation2D& xfA,
                                            const Transformation2D& xfB) const
{
    const auto normal = Rotate(m_axis, xfA.q);
    const auto indexA = InvalidVertex;
    const auto pointA = Transform(m_localPoint, xfA);
    const auto dir = InverseRotate(-normal, xfB.q);
    const auto indexB = GetSupportIndex(m_proxyB, dir);
    const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
    const auto delta = pointB - pointA;
    return LengthIndexPair{Dot(delta, normal), IndexPair{indexA, indexB}};
}

LengthIndexPair
SeparationFinder::FindMinSeparationForFaceB(const Transformation2D& xfA,
                                            const Transformation2D& xfB) const
{
    const auto normal = Rotate(m_axis, xfB.q);
    const auto dir = InverseRotate(-normal, xfA.q);
    const auto indexA = GetSupportIndex(m_proxyA, dir);
    const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
    const auto indexB = InvalidVertex;
    const auto pointB = Transform(m_localPoint, xfB);
    const auto delta = pointA - pointB;
    return LengthIndexPair{Dot(delta, normal), IndexPair{indexA, indexB}};
}

Length SeparationFinder::EvaluateForPoints(const Transformation2D& xfA, const Transformation2D& xfB,
                                           IndexPair indexPair) const
{
    const auto pointA = Transform(m_proxyA.GetVertex(std::get<0>(indexPair)), xfA);
    const auto pointB = Transform(m_proxyB.GetVertex(std::get<1>(indexPair)), xfB);
    const auto delta = pointB - pointA;
    return Dot(delta, m_axis);
}

Length SeparationFinder::EvaluateForFaceA(const Transformation2D& xfA, const Transformation2D& xfB,
                                          IndexPair indexPair) const
{
    const auto normal = Rotate(m_axis, xfA.q);
    const auto pointA = Transform(m_localPoint, xfA);
    const auto pointB = Transform(m_proxyB.GetVertex(std::get<1>(indexPair)), xfB);
    const auto delta = pointB - pointA;
    return Dot(delta, normal);
}

Length SeparationFinder::EvaluateForFaceB(const Transformation2D& xfA, const Transformation2D& xfB,
                                          IndexPair indexPair) const
{
    const auto normal = Rotate(m_axis, xfB.q);
    const auto pointB = Transform(m_localPoint, xfB);
    const auto pointA = Transform(m_proxyA.GetVertex(std::get<0>(indexPair)), xfA);
    const auto delta = pointA - pointB;
    return Dot(delta, normal);
}

} // namespace playrho
