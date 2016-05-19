/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2Distance.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Common/b2Timer.h>

#include <stdio.h>

namespace box2d {

float_t toiTime, toiMaxTime;
uint32 toiCalls, toiIters, toiMaxIters;
uint32 toiRootIters, toiMaxRootIters;

class b2SeparationFunction
{
public:
	enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	};

	b2SeparationFunction(const b2SimplexCache& cache,
		const b2DistanceProxy& proxyA, const b2Sweep& sweepA,
		const b2DistanceProxy& proxyB, const b2Sweep& sweepB,
		float_t t1):
		m_proxyA(proxyA), m_proxyB(proxyB), m_sweepA(sweepA), m_sweepB(sweepB),
		m_type((cache.GetCount() != 1)? ((cache.GetIndexA(0) == cache.GetIndexA(1))? e_faceB: e_faceA): e_points)
	{
		assert(cache.GetCount() > 0);
		assert(cache.GetCount() <= 3); // < 3 or <= 3?

		const auto xfA = b2GetTransform(m_sweepA, t1);
		const auto xfB = b2GetTransform(m_sweepB, t1);

		switch (m_type)
		{
		case e_points:
		{
			const auto localPointA = m_proxyA.GetVertex(cache.GetIndexA(0));
			const auto localPointB = m_proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointA = b2Mul(xfA, localPointA);
			const auto pointB = b2Mul(xfB, localPointB);
			m_axis = b2Normalize(pointB - pointA);
			break;
		}
		case e_faceB:
		{
			// Two points on B and one on A.
			const auto localPointB1 = proxyB.GetVertex(cache.GetIndexB(0));
			const auto localPointB2 = proxyB.GetVertex(cache.GetIndexB(1));

			m_axis = b2Normalize(b2Cross(localPointB2 - localPointB1, float_t(1)));
			const auto normal = b2Mul(xfB.q, m_axis);

			m_localPoint = (localPointB1 + localPointB2) / float_t(2);
			const auto pointB = b2Mul(xfB, m_localPoint);

			const auto localPointA = proxyA.GetVertex(cache.GetIndexA(0));
			const auto pointA = b2Mul(xfA, localPointA);

			auto s = b2Dot(pointA - pointB, normal);
			if (s < float_t{0})
			{
				m_axis = -m_axis;
			}
			break;
		}
		case e_faceA:
		{
			// Two points on A and one or two points on B.
			const auto localPointA1 = m_proxyA.GetVertex(cache.GetIndexA(0));
			const auto localPointA2 = m_proxyA.GetVertex(cache.GetIndexA(1));
			
			m_axis = b2Normalize(b2Cross(localPointA2 - localPointA1, float_t(1)));
			const auto normal = b2Mul(xfA.q, m_axis);

			m_localPoint = (localPointA1 + localPointA2) / float_t(2);
			const auto pointA = b2Mul(xfA, m_localPoint);

			const auto localPointB = m_proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointB = b2Mul(xfB, localPointB);

			auto s = b2Dot(pointB - pointA, normal);
			if (s < float_t{0})
			{
				m_axis = -m_axis;
			}
			break;
		}
		}
	}

	/// Finds the minimum separation.
	/// @param indexA Returns the index of proxy A's vertex for the returned separation.
	/// @param indexB Returns the index of proxy B's vertex for the returned separation.
	/// @param t Time factor in [0, 1] for which the calculation should be performed.
	/// @return minimum distance between the two identified vertces or zero.
	float_t FindMinSeparation(b2DistanceProxy::size_type* indexA,
							  b2DistanceProxy::size_type* indexB,
							  float_t t) const
	{
		const auto xfA = b2GetTransform(m_sweepA, t);
		const auto xfB = b2GetTransform(m_sweepB, t);

		switch (m_type)
		{
		case e_points:
			{
				const auto axisA = b2MulT(xfA.q,  m_axis);
				const auto axisB = b2MulT(xfB.q, -m_axis);

				*indexA = m_proxyA.GetSupport(axisA);
				*indexB = m_proxyB.GetSupport(axisB);

				const auto localPointA = m_proxyA.GetVertex(*indexA);
				const auto localPointB = m_proxyB.GetVertex(*indexB);
				
				const auto pointA = b2Mul(xfA, localPointA);
				const auto pointB = b2Mul(xfB, localPointB);

				return b2Dot(pointB - pointA, m_axis);
			}

		case e_faceA:
			{
				const auto normal = b2Mul(xfA.q, m_axis);
				const auto pointA = b2Mul(xfA, m_localPoint);

				const auto axisB = b2MulT(xfB.q, -normal);
				
				*indexA = static_cast<b2DistanceProxy::size_type>(-1);
				*indexB = m_proxyB.GetSupport(axisB);

				const auto localPointB = m_proxyB.GetVertex(*indexB);
				const auto pointB = b2Mul(xfB, localPointB);

				return b2Dot(pointB - pointA, normal);
			}

		case e_faceB:
			{
				const auto normal = b2Mul(xfB.q, m_axis);
				const auto pointB = b2Mul(xfB, m_localPoint);

				const auto axisA = b2MulT(xfA.q, -normal);

				*indexB = static_cast<b2DistanceProxy::size_type>(-1);
				*indexA = m_proxyA.GetSupport(axisA);

				const auto localPointA = m_proxyA.GetVertex(*indexA);
				const auto pointA = b2Mul(xfA, localPointA);

				return b2Dot(pointA - pointB, normal);
			}

		default:
			assert(false);
			*indexA = static_cast<b2DistanceProxy::size_type>(-1);
			*indexB = static_cast<b2DistanceProxy::size_type>(-1);
			return float_t{0};
		}
	}
	
	/// Evaluates the separation of the identified proxy vertices at the given time factor.
	/// @param indexA Index of the proxy A vertex.
	/// @param indexB Index of the proxy B vertex.
	/// @param t Time factor in range of [0,1] into the future, where 0 indicates alpha0.
	/// @return Separation distance.
	float_t Evaluate(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB, float_t t) const
	{
		const auto xfA = b2GetTransform(m_sweepA, t);
		const auto xfB = b2GetTransform(m_sweepB, t);

		switch (m_type)
		{
			case e_points: return EvaluatePoints(indexA, indexB, xfA, xfB);
			case e_faceA: return EvaluateFaceA(indexA, indexB, xfA, xfB);
			case e_faceB: return EvaluateFaceB(indexA, indexB, xfA, xfB);
			default: break;
		}
		assert(false);
		return float_t{0};
	}

	const b2DistanceProxy& m_proxyA;
	const b2DistanceProxy& m_proxyB;
	const b2Sweep m_sweepA, m_sweepB;
	const Type m_type;
	Vec2 m_localPoint; // used if type is e_faceA or e_faceB
	Vec2 m_axis;
	
private:
	float_t EvaluatePoints(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB,
						   const b2Transform& xfA, const b2Transform& xfB) const
	{
		const auto localPointA = m_proxyA.GetVertex(indexA);
		const auto localPointB = m_proxyB.GetVertex(indexB);
		const auto pointA = b2Mul(xfA, localPointA);
		const auto pointB = b2Mul(xfB, localPointB);
		return b2Dot(pointB - pointA, m_axis);
	}
	
	float_t EvaluateFaceA(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB,
						  const b2Transform& xfA, const b2Transform& xfB) const
	{
		const auto normal = b2Mul(xfA.q, m_axis);
		const auto pointA = b2Mul(xfA, m_localPoint);
		const auto localPointB = m_proxyB.GetVertex(indexB);
		const auto pointB = b2Mul(xfB, localPointB);
		return b2Dot(pointB - pointA, normal);
	}
	
	float_t EvaluateFaceB(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB,
						  const b2Transform& xfA, const b2Transform& xfB) const
	{
		const auto normal = b2Mul(xfB.q, m_axis);
		const auto pointB = b2Mul(xfB, m_localPoint);
		
		const auto localPointA = m_proxyA.GetVertex(indexA);
		const auto pointA = b2Mul(xfA, localPointA);
		
		return b2Dot(pointA - pointB, normal);
	}
};

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
b2TOIOutput b2TimeOfImpact(const b2TOIInput& input)
{
	++toiCalls;

	auto output = b2TOIOutput{b2TOIOutput::e_unknown, input.tMax};

	const auto& proxyA = input.proxyA;
	const auto& proxyB = input.proxyB;

	auto sweepA = input.sweepA;
	auto sweepB = input.sweepB;

	// Large rotations can make the root finder fail, so we normalize the  sweep angles.
	sweepA.Normalize();
	sweepB.Normalize();

	const auto totalRadius = proxyA.GetRadius() + proxyB.GetRadius();
	const auto target = b2Max(LinearSlop, totalRadius - (float_t{3} * LinearSlop));
	constexpr auto tolerance = LinearSlop / float_t(4);
	assert(target >= tolerance);

	auto t1 = float_t{0};
	auto iter = decltype(MaxTOIIterations){0};

	// Prepare input for distance query.
	b2SimplexCache cache;
	b2DistanceInput distanceInput;
	distanceInput.proxyA = proxyA;
	distanceInput.proxyB = proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for(;;)
	{
		distanceInput.transformA = b2GetTransform(sweepA, t1);
		distanceInput.transformB = b2GetTransform(sweepB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		const auto distanceOutput = b2Distance(cache, distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= float_t{0})
		{
			// Failure!
			output = b2TOIOutput{b2TOIOutput::e_overlapped, float_t{0}};
			break;
		}

		if (distanceOutput.distance < (target + tolerance))
		{
			// Victory!
			output = b2TOIOutput{b2TOIOutput::e_touching, t1};
			break;
		}

		// Initialize the separating axis.
		b2SeparationFunction fcn(cache, proxyA, sweepA, proxyB, sweepB, t1);
#if 0
		// Dump the curve seen by the root finder
		{
			const int32 N = 100;
			float_t dx = float_t(1) / N;
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
		auto t2 = input.tMax;
		auto pushBackIter = decltype(MaxPolygonVertices){0};
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			b2DistanceProxy::size_type indexA, indexB;
			auto s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);

			// Is the final configuration separated?
			if (s2 > (target + tolerance))
			{
				// Victory!
				assert(t2 == input.tMax);
				// Formerly this used input.tMax as in...
				// output = b2TOIOutput{b2TOIOutput::e_separated, input.tMax};
				// t2 seems more appropriate however given s2 was derived from it.
				// Meanwhile t2 always seems equal to input.tMax at this point.
				output = b2TOIOutput{b2TOIOutput::e_separated, t2};
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > (target - tolerance))
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			auto s1 = fcn.Evaluate(indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < (target - tolerance))
			{
				output = b2TOIOutput{b2TOIOutput::e_failed, t1};
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= (target + tolerance))
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output = b2TOIOutput{b2TOIOutput::e_touching, t1};
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			auto rootIterCount = decltype(MaxTOIRootIterCount){0};
			auto a1 = t1;
			auto a2 = t2;
			do
			{
				// Use a mix of the secant rule and bisection.
				const auto t = (rootIterCount & 1)?
					// Secant rule to improve convergence.
					a1 + (target - s1) * (a2 - a1) / (s2 - s1):
					// Bisection to guarantee progress.
					(a1 + a2) / float_t(2);

				++rootIterCount;
				++toiRootIters;

				const auto s = fcn.Evaluate(indexA, indexB, t);

				if (b2Abs(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
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

			toiMaxRootIters = b2Max(toiMaxRootIters, rootIterCount);

			++pushBackIter;

			if (pushBackIter == MaxPolygonVertices)
				break;
		}

		++iter;
		++toiIters;

		if (done)
			break;

		if (iter == MaxTOIIterations)
		{
			// Root finder got stuck. Semi-victory.
			output = b2TOIOutput{b2TOIOutput::e_failed, t1};
			break;
		}
	}

	toiMaxIters = b2Max(toiMaxIters, iter);
	
	return output;
}
	
} // namespace box2d
