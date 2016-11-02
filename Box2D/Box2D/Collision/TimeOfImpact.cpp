/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Collision/Distance.h>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/SimplexCache.hpp>
#include <Box2D/Collision/TimeOfImpact.h>
#include <Box2D/Common/Timer.h>

#include <stdio.h>

namespace box2d {

struct Separation
{
	Separation() noexcept = default;
	Separation(const Separation& copy) noexcept = default;
	
	constexpr Separation(IndexPair ip, float_t d) noexcept: indexPair{ip}, distance{d} {}

	IndexPair indexPair; ///< Pair of indices of vertices for which distance is being returned for.
	float_t distance; ///< Distance of separation (in meters) between vertices indexed by the index-pair.
};

/// Separation function.
class SeparationFunction
{
public:
	enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	};

	SeparationFunction(const SimplexCache& cache,
		const DistanceProxy& proxyA, const Sweep& sweepA,
		const DistanceProxy& proxyB, const Sweep& sweepB,
		float_t t1):
		m_proxyA{proxyA}, m_proxyB{proxyB}, m_sweepA{sweepA}, m_sweepB{sweepB},
		m_type{(cache.GetNumIndices() == 1)? e_points: ((cache.GetIndexPair(0).a == cache.GetIndexPair(1).a)? e_faceB: e_faceA)}
	{
		assert(cache.GetIndices().size() > 0);
		assert(cache.GetIndices().size() <= 3); // < 3 or <= 3?
		assert(proxyA.GetVertexCount() > 0);
		assert(proxyB.GetVertexCount() > 0);
		
		const auto xfA = GetTransformation(m_sweepA, t1);
		const auto xfB = GetTransformation(m_sweepB, t1);

		switch (m_type)
		{
		case e_points:
		{
			const auto ip0 = cache.GetIndexPair(0);
			const auto localPointA = proxyA.GetVertex(ip0.a);
			const auto localPointB = proxyB.GetVertex(ip0.b);
			const auto pointA = Transform(localPointA, xfA);
			const auto pointB = Transform(localPointB, xfB);
			m_axis = GetUnitVector(pointB - pointA);
			break;
		}
		case e_faceB:
		{
			const auto ip0 = cache.GetIndexPair(0);
			const auto ip1 = cache.GetIndexPair(1);

			// Two points on B and one on A.
			const auto localPointB1 = proxyB.GetVertex(ip0.b);
			const auto localPointB2 = proxyB.GetVertex(ip1.b);

			m_axis = GetUnitVector(GetFwdPerpendicular(localPointB2 - localPointB1));
			const auto normal = Rotate(m_axis, xfB.q);

			m_localPoint = (localPointB1 + localPointB2) / float_t(2);
			const auto pointB = Transform(m_localPoint, xfB);

			const auto localPointA = proxyA.GetVertex(ip0.a);
			const auto pointA = Transform(localPointA, xfA);

			auto s = Dot(pointA - pointB, normal);
			if (s < float_t{0})
			{
				m_axis = -m_axis;
			}
			break;
		}
		case e_faceA:
		{
			const auto ip0 = cache.GetIndexPair(0);
			const auto ip1 = cache.GetIndexPair(1);

			// Two points on A and one or two points on B.
			const auto localPointA1 = proxyA.GetVertex(ip0.a);
			const auto localPointA2 = proxyA.GetVertex(ip1.a);
			
			m_axis = GetUnitVector(GetFwdPerpendicular(localPointA2 - localPointA1));
			const auto normal = Rotate(m_axis, xfA.q);

			m_localPoint = (localPointA1 + localPointA2) / float_t(2);
			const auto pointA = Transform(m_localPoint, xfA);

			const auto localPointB = proxyB.GetVertex(ip0.b);
			const auto pointB = Transform(localPointB, xfB);

			auto s = Dot(pointB - pointA, normal);
			if (s < float_t{0})
			{
				m_axis = -m_axis;
			}
			break;
		}
		}
	}

	/// Finds the minimum separation.
	/// @param t Time factor in [0, 1] for which the calculation should be performed.
	/// @return indexes of proxy A's and proxy B's vertices that have the minimum distance between them and what that distance is.
	Separation FindMinSeparation(float_t t) const
	{
		const auto xfA = GetTransformation(m_sweepA, t);
		const auto xfB = GetTransformation(m_sweepB, t);

		switch (m_type)
		{
			case e_points: return FindMinSeparationForPoints(xfA, xfB);
			case e_faceA: return FindMinSeparationForFaceA(xfA, xfB);
			case e_faceB: return FindMinSeparationForFaceB(xfA, xfB);
		}

		// Should never be reached
		assert(false);
		return Separation{IndexPair{IndexPair::InvalidIndex, IndexPair::InvalidIndex}, 0};
	}
	
	/// Evaluates the separation of the identified proxy vertices at the given time factor.
	/// @param indexPair Indexes of the proxy A and proxy B vertexes.
	/// @param t Time factor in range of [0,1] into the future, where 0 indicates alpha0.
	/// @return Separation distance.
	float_t Evaluate(IndexPair indexPair, float_t t) const
	{
		const auto xfA = GetTransformation(m_sweepA, t);
		const auto xfB = GetTransformation(m_sweepB, t);

		switch (m_type)
		{
			case e_points: return EvaluateForPoints(indexPair, xfA, xfB);
			case e_faceA: return EvaluateForFaceA(indexPair, xfA, xfB);
			case e_faceB: return EvaluateForFaceB(indexPair, xfA, xfB);
			default: break;
		}
		assert(false);
		return float_t{0};
	}

	const DistanceProxy& m_proxyA;
	const DistanceProxy& m_proxyB;
	const Sweep m_sweepA, m_sweepB;
	const Type m_type;
	
private:
	Separation FindMinSeparationForPoints(const Transformation& xfA, const Transformation& xfB) const
	{
		const auto indexA = GetSupportIndex(m_proxyA, InverseRotate(m_axis, xfA.q));
		const auto indexB = GetSupportIndex(m_proxyB, InverseRotate(-m_axis, xfB.q));
		const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
		const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
		return Separation{IndexPair{indexA, indexB}, Dot(pointB - pointA, m_axis)};
	}
	
	Separation FindMinSeparationForFaceA(const Transformation& xfA, const Transformation& xfB) const
	{
		const auto normal = Rotate(m_axis, xfA.q);
		const auto indexA = IndexPair::InvalidIndex;
		const auto pointA = Transform(m_localPoint, xfA);
		const auto indexB = GetSupportIndex(m_proxyB, InverseRotate(-normal, xfB.q));
		const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
		return Separation{IndexPair{indexA, indexB}, Dot(pointB - pointA, normal)};
	}
	
	Separation FindMinSeparationForFaceB(const Transformation& xfA, const Transformation& xfB) const
	{
		const auto normal = Rotate(m_axis, xfB.q);
		const auto indexA = GetSupportIndex(m_proxyA, InverseRotate(-normal, xfA.q));
		const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
		const auto indexB = IndexPair::InvalidIndex;
		const auto pointB = Transform(m_localPoint, xfB);
		return Separation{IndexPair{indexA, indexB}, Dot(pointA - pointB, normal)};
	}
	
	float_t EvaluateForPoints(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
	{
		const auto pointA = Transform(m_proxyA.GetVertex(indexPair.a), xfA);
		const auto pointB = Transform(m_proxyB.GetVertex(indexPair.b), xfB);
		return Dot(pointB - pointA, m_axis);
	}
	
	float_t EvaluateForFaceA(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
	{
		const auto normal = Rotate(m_axis, xfA.q);
		const auto pointA = Transform(m_localPoint, xfA);
		const auto pointB = Transform(m_proxyB.GetVertex(indexPair.b), xfB);
		return Dot(pointB - pointA, normal);
	}
	
	float_t EvaluateForFaceB(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
	{
		const auto normal = Rotate(m_axis, xfB.q);
		const auto pointB = Transform(m_localPoint, xfB);
		const auto pointA = Transform(m_proxyA.GetVertex(indexPair.a), xfA);
		return Dot(pointA - pointB, normal);
	}
	
	Vec2 m_axis; ///< Axis. @detail Normalized vector (a pure directional vector) of the axis of separation.
	Vec2 m_localPoint; // used if type is e_faceA or e_faceB
};

TOIOutput TimeOfImpact(const DistanceProxy& proxyA, const Sweep& sweepA,
					   const DistanceProxy& proxyB, const Sweep& sweepB,
					   const float_t tMax)
{
	// CCD via the local separating axis method. This seeks progression
	// by computing the largest time at which separation is maintained.
	assert(IsValid(sweepA.pos0));
	assert(IsValid(sweepA.pos1));
	assert(IsValid(sweepB.pos0));
	assert(IsValid(sweepB.pos1));
	
	auto stats = TOIOutput::Stats{0, 0, 0, 0, 0};
	auto output = TOIOutput{TOIOutput::e_unknown, tMax, stats};

	const auto totalRadius = proxyA.GetRadius() + proxyB.GetRadius(); // 2 polygons = 2 * PolygonRadius = 4 * LinearSlop
	const auto target = Max(LinearSlop, totalRadius - BOX2D_MAGIC(LinearSlop * float_t{3}));
	const auto tolerance = BOX2D_MAGIC(LinearSlop / 4);
	assert(target >= tolerance);
	const auto maxTarget = target + tolerance;
	const auto minTarget = target - tolerance;
	const auto maxTargetSquared = Square(maxTarget);

	auto t1 = float_t{0}; // Will be set to value of t2

	// Prepare input for distance query.
	SimplexCache cache;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for(;;)
	{
		{
			// Get the distance between shapes. We can also use the results
			// to get a separating axis.
			const auto distanceInfo = Distance(cache,
											   proxyA, GetTransformation(sweepA, t1),
											   proxyB, GetTransformation(sweepB, t1));
			++stats.toi_iters;
			stats.sum_dist_iters += distanceInfo.iterations;
			stats.max_dist_iters = Max(stats.max_dist_iters, distanceInfo.iterations);
			const auto distanceSquared = LengthSquared(distanceInfo.witnessPoints.a - distanceInfo.witnessPoints.b);
			
			// If the shapes aren't separated, give up on continuous collision.
			if (distanceSquared <= float_t{0}) // Failure!
			{
				output = TOIOutput{TOIOutput::e_overlapped, 0, stats};
				break;
			}

			if (distanceSquared < maxTargetSquared) // Victory!
			{
				output = TOIOutput{TOIOutput::e_touching, t1, stats};
				break;
			}
		}

		// Initialize the separating axis.
		SeparationFunction fcn(cache, proxyA, sweepA, proxyB, sweepB, t1);

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		auto done = false;
		auto t2 = tMax; // Will be set to the value of t
		for (auto pushBackIter = decltype(MaxShapeVertices){0}; pushBackIter < MaxShapeVertices; ++pushBackIter)
		{
			// Find the deepest point at t2. Store the witness point indices.
			const auto minSeparation = fcn.FindMinSeparation(t2);

			// Is the final configuration separated?
			if (minSeparation.distance > maxTarget)
			{
				// Victory!
				assert(t2 == tMax);
				// Formerly this used tMax as in...
				// output = TOIOutput{TOIOutput::e_separated, tMax};
				// t2 seems more appropriate however given s2 was derived from it.
				// Meanwhile t2 always seems equal to input.tMax at this point.
				output = TOIOutput{TOIOutput::e_separated, t2, stats};
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (minSeparation.distance > minTarget)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			const auto evaluatedDistance = fcn.Evaluate(minSeparation.indexPair, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			//assert(s1 >= minTarget);
			if (evaluatedDistance < minTarget)
			{
				output = TOIOutput{TOIOutput::e_failed, t1, stats};
				done = true;
				break;
			}

			// Check for touching
			if (evaluatedDistance <= maxTarget)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output = TOIOutput{TOIOutput::e_touching, t1, stats};
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			auto rootIterCount = decltype(MaxTOIRootIterCount){0};
			auto a1 = t1;
			auto a2 = t2;
			auto s1 = evaluatedDistance;
			auto s2 = minSeparation.distance;
			do
			{
				// Uses secant method to improve convergence (see https://en.wikipedia.org/wiki/Secant_method ).
				// Uses bisection method to guarantee progress (see https://en.wikipedia.org/wiki/Bisection_method ).
				const auto t = (rootIterCount & 1)?
					a1 + (target - s1) * (a2 - a1) / (s2 - s1):
					(a1 + a2) / float_t{2};
				++rootIterCount;

				const auto s = fcn.Evaluate(minSeparation.indexPair, t);

				if (Abs(s - target) < tolerance)
				{
					t2 = t; // t2 holds a tentative value for t1
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}				
			}
			while (rootIterCount < MaxTOIRootIterCount);

			stats.sum_root_iters += rootIterCount;
			stats.max_root_iters = Max(stats.max_root_iters, rootIterCount);
		}

		if (done)
			break;

		if (stats.toi_iters == MaxTOIIterations)
		{
			// Root finder got stuck. Semi-victory.
			output = TOIOutput{TOIOutput::e_failed, t1, stats};
			break;
		}
	}

	return output;
}

bool TestOverlap(const Shape& shapeA, child_count_t indexA, const Transformation& xfA,
				 const Shape& shapeB, child_count_t indexB, const Transformation& xfB)
{
	const auto proxyA = GetDistanceProxy(shapeA, indexA);
	const auto proxyB = GetDistanceProxy(shapeB, indexB);
	
	SimplexCache cache;
	const auto distanceInfo = Distance(cache, proxyA, xfA, proxyB, xfB);
	const auto distanceSquared = LengthSquared(distanceInfo.witnessPoints.a - distanceInfo.witnessPoints.b);
	const auto totalRadiusSquared = Square(proxyA.GetRadius() + proxyB.GetRadius());
	const auto separation_amount = distanceSquared - totalRadiusSquared;
	return (separation_amount < 0) || almost_zero(separation_amount);
}

} // namespace box2d
