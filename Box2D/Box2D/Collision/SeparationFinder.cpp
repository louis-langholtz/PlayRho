/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/SeparationFinder.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>

using namespace box2d;

SeparationFinder SeparationFinder::Get(Span<const IndexPair> indices,
									   const DistanceProxy& proxyA, const Transformation& xfA,
									   const DistanceProxy& proxyB, const Transformation& xfB)
{
	assert(indices.size() > 0);
	assert(indices.size() <= 3); // < 3 or <= 3?
	assert(proxyA.GetVertexCount() > 0);
	assert(proxyB.GetVertexCount() > 0);
	
	const auto type = (indices.size() == 1)? e_points: ((indices[0].a == indices[1].a)? e_faceB: e_faceA);
	
	switch (type)
	{
		case e_points:
		{
			const auto ip0 = indices[0];
			const auto localPointA = proxyA.GetVertex(ip0.a);
			const auto localPointB = proxyB.GetVertex(ip0.b);
			const auto pointA = Transform(localPointA, xfA);
			const auto pointB = Transform(localPointB, xfB);
			const auto axis = GetUnitVector(pointB - pointA, UnitVec2::GetZero());
			return SeparationFinder{proxyA, proxyB, axis, GetInvalid<Vec2>(), type};
		}
		case e_faceB:
		{
			const auto ip0 = indices[0];
			const auto ip1 = indices[1];
			
			// Two points on B and one on A.
			const auto localPointB1 = proxyB.GetVertex(ip0.b);
			const auto localPointB2 = proxyB.GetVertex(ip1.b);
			
			const auto axis = GetUnitVector(GetFwdPerpendicular(localPointB2 - localPointB1),
											UnitVec2::GetZero());
			const auto normal = Rotate(axis, xfB.q);
			
			const auto localPoint = (localPointB1 + localPointB2) / RealNum(2);
			const auto pointB = Transform(localPoint, xfB);
			
			const auto localPointA = proxyA.GetVertex(ip0.a);
			const auto pointA = Transform(localPointA, xfA);
			
			return SeparationFinder{
				proxyA, proxyB,
				(Dot(pointA - pointB, normal) < 0)? -axis: axis,
				localPoint, type
			};
		}
		case e_faceA:
		{
			const auto ip0 = indices[0];
			const auto ip1 = indices[1];
			
			// Two points on A and one or two points on B.
			const auto localPointA1 = proxyA.GetVertex(ip0.a);
			const auto localPointA2 = proxyA.GetVertex(ip1.a);
			
			const auto axis = GetUnitVector(GetFwdPerpendicular(localPointA2 - localPointA1), UnitVec2::GetZero());
			const auto normal = Rotate(axis, xfA.q);
			
			const auto localPoint = (localPointA1 + localPointA2) / RealNum(2);
			const auto pointA = Transform(localPoint, xfA);
			
			const auto localPointB = proxyB.GetVertex(ip0.b);
			const auto pointB = Transform(localPointB, xfB);
			
			return SeparationFinder{
				proxyA, proxyB,
				(Dot(pointB - pointA, normal) < 0)? -axis: axis,
				localPoint, type
			};
		}
	}
}

SeparationFinder::Data SeparationFinder::FindMinSeparationForPoints(const Transformation& xfA, const Transformation& xfB) const
{
	const auto indexA = GetSupportIndex(m_proxyA, Vec2{InverseRotate(m_axis, xfA.q)});
	const auto indexB = GetSupportIndex(m_proxyB, Vec2{InverseRotate(-m_axis, xfB.q)});
	const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
	const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
	return Data{IndexPair{indexA, indexB}, Dot(pointB - pointA, m_axis)};
}

SeparationFinder::Data SeparationFinder::FindMinSeparationForFaceA(const Transformation& xfA, const Transformation& xfB) const
{
	const auto normal = Rotate(m_axis, xfA.q);
	const auto indexA = IndexPair::InvalidIndex;
	const auto pointA = Transform(m_localPoint, xfA);
	const auto indexB = GetSupportIndex(m_proxyB, Vec2{InverseRotate(-normal, xfB.q)});
	const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
	return Data{IndexPair{indexA, indexB}, Dot(pointB - pointA, normal)};
}

SeparationFinder::Data SeparationFinder::FindMinSeparationForFaceB(const Transformation& xfA, const Transformation& xfB) const
{
	const auto normal = Rotate(m_axis, xfB.q);
	const auto indexA = GetSupportIndex(m_proxyA, Vec2{InverseRotate(-normal, xfA.q)});
	const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
	const auto indexB = IndexPair::InvalidIndex;
	const auto pointB = Transform(m_localPoint, xfB);
	return Data{IndexPair{indexA, indexB}, Dot(pointA - pointB, normal)};
}

RealNum SeparationFinder::EvaluateForPoints(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
{
	const auto pointA = Transform(m_proxyA.GetVertex(indexPair.a), xfA);
	const auto pointB = Transform(m_proxyB.GetVertex(indexPair.b), xfB);
	return Dot(pointB - pointA, m_axis);
}

RealNum SeparationFinder::EvaluateForFaceA(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
{
	const auto normal = Rotate(m_axis, xfA.q);
	const auto pointA = Transform(m_localPoint, xfA);
	const auto pointB = Transform(m_proxyB.GetVertex(indexPair.b), xfB);
	return Dot(pointB - pointA, normal);
}

RealNum SeparationFinder::EvaluateForFaceB(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
{
	const auto normal = Rotate(m_axis, xfB.q);
	const auto pointB = Transform(m_localPoint, xfB);
	const auto pointA = Transform(m_proxyA.GetVertex(indexPair.a), xfA);
	return Dot(pointA - pointB, normal);
}
