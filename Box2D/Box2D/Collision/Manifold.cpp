/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <bitset>
#include <algorithm>

using namespace box2d;

Manifold box2d::GetManifold(const DistanceProxy& proxyA, const Transformation& transformA,
							const DistanceProxy& proxyB, const Transformation& transformB)
{
	const auto distanceInfo = Distance(proxyA, transformA, proxyB, transformB);
	const auto totalRadius = proxyA.GetRadius() + proxyB.GetRadius();
	const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);

	const auto distance = Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b));
	if (distance > totalRadius)
	{
		// no collision
		return Manifold{};
	}

	using index_type = IndexPair::size_type;

	index_type a_indices_array[Simplex::MaxEdges];
	index_type b_indices_array[Simplex::MaxEdges];
	auto uniqA = size_t{0};
	auto uniqB = size_t{0};
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

	if (uniqA <= uniqB)
	{
		switch (uniqA)
		{
			case 1:
			{
				const auto lp = proxyA.GetVertex(a_indices_array[0]);
				const auto mp0 = Manifold::Point{
					proxyB.GetVertex(b_indices_array[0]),
					ContactFeature{
						ContactFeature::e_vertex,
						a_indices_array[0],
						(uniqB < 2)? ContactFeature::e_vertex: ContactFeature::e_face,
						b_indices_array[0]
					}
				};
				return Manifold::GetForCircles(lp, mp0);
			}
			case 2:
			{
				auto mp0 = Manifold::Point{};
				auto mp1 = Manifold::Point{};
				mp0.contactFeature.typeA = ContactFeature::e_face;
				mp1.contactFeature.typeA = ContactFeature::e_face;
				const auto v0 = proxyA.GetVertex(a_indices_array[0]);
				const auto v1 = proxyA.GetVertex(a_indices_array[1]);
				const auto lp = (v0 + v1) / 2;
				const auto count = proxyA.GetVertexCount();
				if ((a_indices_array[1] - a_indices_array[0]) == 1)
				{
					mp0.contactFeature.indexA = a_indices_array[0];
					mp1.contactFeature.indexA = a_indices_array[0];
					const auto ln = GetFwdPerpendicular(GetUnitVector(v1 - v0));
					return Manifold::GetForFaceA(ln, lp, mp0, mp1);
				}
				else if ((a_indices_array[1] + 1) % count == a_indices_array[0])
				{
					mp0.contactFeature.indexA = a_indices_array[1];
					mp1.contactFeature.indexA = a_indices_array[1];
					const auto ln = GetFwdPerpendicular(GetUnitVector(v0 - v1));
					return Manifold::GetForFaceA(ln, lp, mp0, mp1);					
				}
				else
				{
					assert(false);
				}
				return Manifold{};
			}
			case 3:
			{
				const auto ln = UnitVec2{};
				const auto lp = Vec2{};
				return Manifold::GetForFaceA(ln, lp);
			}
			default:
				break;
		}
	}
	else // uniqB < uniqA
	{
		switch (uniqB)
		{
			case 1:
			{
				const auto lp = (uniqA < 2)?
					proxyA.GetVertex(a_indices_array[0]):
					(proxyA.GetVertex(a_indices_array[1]) + proxyA.GetVertex(a_indices_array[0])) / 2;
				const auto mp0 = Manifold::Point{
					proxyB.GetVertex(b_indices_array[0]),
					ContactFeature{
						(uniqA < 2)? ContactFeature::e_vertex: ContactFeature::e_face,
						a_indices_array[0],
						ContactFeature::e_vertex,
						b_indices_array[0]
					}
				};
				return Manifold::GetForCircles(lp, mp0);
			}
			case 2:
			{
				auto mp0 = Manifold::Point{};
				auto mp1 = Manifold::Point{};
				mp0.contactFeature.typeB = ContactFeature::e_face;
				mp1.contactFeature.typeB = ContactFeature::e_face;
				const auto v0 = proxyB.GetVertex(b_indices_array[0]);
				const auto v1 = proxyB.GetVertex(b_indices_array[1]);
				const auto lp = (v0 + v1) / 2;
				const auto count = proxyB.GetVertexCount();
				if ((b_indices_array[1] - b_indices_array[0]) == 1)
				{
					mp0.contactFeature.indexB = b_indices_array[0];
					mp1.contactFeature.indexB = b_indices_array[0];
					const auto ln = GetFwdPerpendicular(GetUnitVector(v1 - v0));
					return Manifold::GetForFaceB(ln, lp, mp0, mp1);
				}
				else if ((b_indices_array[1] + 1) % count == b_indices_array[0])
				{
					mp0.contactFeature.indexB = b_indices_array[1];
					mp1.contactFeature.indexB = b_indices_array[1];
					const auto ln = GetFwdPerpendicular(GetUnitVector(v0 - v1));
					return Manifold::GetForFaceB(ln, lp, mp0, mp1);					
				}
				else
				{
					assert(false);
				}
				return Manifold{};
			}
			case 3:
			{
				const auto ln = UnitVec2{};
				const auto lp = Vec2{};
				return Manifold::GetForFaceB(ln, lp);
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
	
	if (lhs.GetLocalNormal() != rhs.GetLocalNormal())
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

Vec2 box2d::GetLocalPoint(const DistanceProxy& proxy, ContactFeature::Type type, ContactFeature::index_t index)
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
	return GetInvalid<Vec2>();
}
