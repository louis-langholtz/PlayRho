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

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Distance.h>
#include <Box2D/Collision/TimeOfImpact.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Common/Timer.h>

#include <stdio.h>

namespace box2d {

float_t toiTime, toiMaxTime;
uint32 toiCalls, toiIters, toiMaxIters;
uint32 toiRootIters, toiMaxRootIters;

struct Separation
{
	Separation() noexcept = default;
	Separation(const Separation& copy) noexcept = default;
	
	constexpr Separation(IndexPair ip, float_t d) noexcept: indexPair{ip}, distance{d} {}

	IndexPair indexPair;
	float_t distance;
};

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
		m_proxyA(proxyA), m_proxyB(proxyB), m_sweepA(sweepA), m_sweepB(sweepB),
		m_type((cache.GetCount() != 1)? ((cache.GetIndexA(0) == cache.GetIndexA(1))? e_faceB: e_faceA): e_points)
	{
		assert(cache.GetCount() > 0);
		assert(cache.GetCount() <= 3); // < 3 or <= 3?
		assert(proxyA.GetVertexCount() > 0);
		assert(proxyB.GetVertexCount() > 0);
		
		const auto xfA = GetTransform(m_sweepA, t1);
		const auto xfB = GetTransform(m_sweepB, t1);

		switch (m_type)
		{
		case e_points:
		{
			const auto localPointA = proxyA.GetVertex(cache.GetIndexA(0));
			const auto localPointB = proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointA = Mul(xfA, localPointA);
			const auto pointB = Mul(xfB, localPointB);
			m_axis = Normalize(pointB - pointA);
			break;
		}
		case e_faceB:
		{
			// Two points on B and one on A.
			const auto localPointB1 = proxyB.GetVertex(cache.GetIndexB(0));
			const auto localPointB2 = proxyB.GetVertex(cache.GetIndexB(1));

			m_axis = Normalize(GetForwardPerpendicular(localPointB2 - localPointB1));
			const auto normal = Mul(xfB.q, m_axis);

			m_localPoint = (localPointB1 + localPointB2) / float_t(2);
			const auto pointB = Mul(xfB, m_localPoint);

			const auto localPointA = proxyA.GetVertex(cache.GetIndexA(0));
			const auto pointA = Mul(xfA, localPointA);

			auto s = Dot(pointA - pointB, normal);
			if (s < float_t{0})
			{
				m_axis = -m_axis;
			}
			break;
		}
		case e_faceA:
		{
			// Two points on A and one or two points on B.
			const auto localPointA1 = proxyA.GetVertex(cache.GetIndexA(0));
			const auto localPointA2 = proxyA.GetVertex(cache.GetIndexA(1));
			
			m_axis = Normalize(GetForwardPerpendicular(localPointA2 - localPointA1));
			const auto normal = Mul(xfA.q, m_axis);

			m_localPoint = (localPointA1 + localPointA2) / float_t(2);
			const auto pointA = Mul(xfA, m_localPoint);

			const auto localPointB = proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointB = Mul(xfB, localPointB);

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
		const auto xfA = GetTransform(m_sweepA, t);
		const auto xfB = GetTransform(m_sweepB, t);

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
		const auto xfA = GetTransform(m_sweepA, t);
		const auto xfB = GetTransform(m_sweepB, t);

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
	Vec2 m_localPoint; // used if type is e_faceA or e_faceB
	Vec2 m_axis;
	
private:
	Separation FindMinSeparationForPoints(const Transform& xfA, const Transform& xfB) const
	{
		const auto indexA = m_proxyA.GetSupportIndex(MulT(xfA.q,  m_axis));
		const auto pointA = Mul(xfA, m_proxyA.GetVertex(indexA));
		const auto indexB = m_proxyB.GetSupportIndex(MulT(xfB.q, -m_axis));
		const auto pointB = Mul(xfB, m_proxyB.GetVertex(indexB));
		return Separation{IndexPair{indexA, indexB}, Dot(pointB - pointA, m_axis)};
	}
	
	Separation FindMinSeparationForFaceA(const Transform& xfA, const Transform& xfB) const
	{
		const auto normal = Mul(xfA.q, m_axis);
		const auto indexA = static_cast<DistanceProxy::size_type>(-1);
		const auto pointA = Mul(xfA, m_localPoint);
		const auto indexB = m_proxyB.GetSupportIndex(MulT(xfB.q, -normal));
		const auto pointB = Mul(xfB, m_proxyB.GetVertex(indexB));
		return Separation{IndexPair{indexA, indexB}, Dot(pointB - pointA, normal)};
	}
	
	Separation FindMinSeparationForFaceB(const Transform& xfA, const Transform& xfB) const
	{
		const auto normal = Mul(xfB.q, m_axis);
		const auto indexA = m_proxyA.GetSupportIndex(MulT(xfA.q, -normal));
		const auto pointA = Mul(xfA, m_proxyA.GetVertex(indexA));
		const auto indexB = static_cast<DistanceProxy::size_type>(-1);
		const auto pointB = Mul(xfB, m_localPoint);
		return Separation{IndexPair{indexA, indexB}, Dot(pointA - pointB, normal)};
	}
	
	float_t EvaluateForPoints(IndexPair indexPair, const Transform& xfA, const Transform& xfB) const
	{
		const auto localPointA = m_proxyA.GetVertex(indexPair.a);
		const auto localPointB = m_proxyB.GetVertex(indexPair.b);
		const auto pointA = Mul(xfA, localPointA);
		const auto pointB = Mul(xfB, localPointB);
		return Dot(pointB - pointA, m_axis);
	}
	
	float_t EvaluateForFaceA(IndexPair indexPair, const Transform& xfA, const Transform& xfB) const
	{
		const auto normal = Mul(xfA.q, m_axis);
		const auto pointA = Mul(xfA, m_localPoint);
		const auto localPointB = m_proxyB.GetVertex(indexPair.b);
		const auto pointB = Mul(xfB, localPointB);
		return Dot(pointB - pointA, normal);
	}
	
	float_t EvaluateForFaceB(IndexPair indexPair, const Transform& xfA, const Transform& xfB) const
	{
		const auto normal = Mul(xfB.q, m_axis);
		const auto pointB = Mul(xfB, m_localPoint);
		const auto localPointA = m_proxyA.GetVertex(indexPair.a);
		const auto pointA = Mul(xfA, localPointA);
		return Dot(pointA - pointB, normal);
	}
};

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
TOIOutput TimeOfImpact(DistanceProxy proxyA, Sweep sweepA, DistanceProxy proxyB, Sweep sweepB, float_t tMax)
{
	++toiCalls;

	auto output = TOIOutput{TOIOutput::e_unknown, tMax};

	// Large rotations can make the root finder fail, so we normalize the  sweep angles.
	sweepA = GetAnglesNormalized(sweepA);
	sweepB = GetAnglesNormalized(sweepB);

	const auto totalRadius = proxyA.GetRadius() + proxyB.GetRadius();
	const auto target = Max(LinearSlop, totalRadius - (float_t{3} * LinearSlop));
	constexpr auto tolerance = LinearSlop / float_t{4};
	assert(target >= tolerance);
	const auto maxTarget = target + tolerance;
	const auto minTarget = target - tolerance;

	auto t1 = float_t{0};
	auto iter = decltype(MaxTOIIterations){0};

	// Prepare input for distance query.
	SimplexCache cache;
	DistanceInput distanceInput;
	distanceInput.proxyA = proxyA;
	distanceInput.proxyB = proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for(;;)
	{
		distanceInput.transformA = GetTransform(sweepA, t1);
		distanceInput.transformB = GetTransform(sweepB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		const auto distanceOutput = Distance(cache, distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= float_t{0}) // Failure!
		{
			output = TOIOutput{TOIOutput::e_overlapped, 0};
			break;
		}

		if (distanceOutput.distance < maxTarget) // Victory!
		{
			output = TOIOutput{TOIOutput::e_touching, t1};
			break;
		}

		// Initialize the separating axis.
		SeparationFunction fcn(cache, proxyA, sweepA, proxyB, sweepB, t1);
#if 0
		// Dump the curve seen by the root finder
		{
			const int32 N = 100;
			float_t dx = float_t{1} / N;
			float_t xs[N+1];
			float_t fs[N+1];

			float_t x = float_t{0};

			for (auto i = decltype(N){0}; i <= N; ++i)
			{
				const auto xfA = GetTransform(sweepA, x);
				const auto xfB = GetTransform(sweepB, x);
				float_t f = fcn.Evaluate(xfA, xfB) - target;

				printf("%g %g\n", x, f);

				xs[i] = x;
				fs[i] = f;

				x += dx;
			}
		}
#endif

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		auto done = false;
		auto t2 = tMax;
		for (auto pushBackIter = decltype(MaxPolygonVertices){0}; pushBackIter < MaxPolygonVertices; ++pushBackIter)
		{
			// Find the deepest point at t2. Store the witness point indices.
			const auto minSeparation = fcn.FindMinSeparation(t2);
			auto s2 = minSeparation.distance;

			// Is the final configuration separated?
			if (s2 > maxTarget)
			{
				// Victory!
				assert(t2 == tMax);
				// Formerly this used input.tMax as in...
				// output = TOIOutput{TOIOutput::e_separated, input.tMax};
				// t2 seems more appropriate however given s2 was derived from it.
				// Meanwhile t2 always seems equal to input.tMax at this point.
				output = TOIOutput{TOIOutput::e_separated, t2};
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > minTarget)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			auto s1 = fcn.Evaluate(minSeparation.indexPair, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < minTarget)
			{
				output = TOIOutput{TOIOutput::e_failed, t1};
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= maxTarget)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output = TOIOutput{TOIOutput::e_touching, t1};
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			auto rootIterCount = decltype(MaxTOIRootIterCount){0};
			auto a1 = t1;
			auto a2 = t2;
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

			toiRootIters += rootIterCount;
			toiMaxRootIters = Max(toiMaxRootIters, rootIterCount);
		}

		++iter;
		++toiIters;

		if (done)
			break;

		if (iter == MaxTOIIterations)
		{
			// Root finder got stuck. Semi-victory.
			output = TOIOutput{TOIOutput::e_failed, t1};
			break;
		}
	}

	toiMaxIters = Max(toiMaxIters, iter);
	
	return output;
}
	
} // namespace box2d
