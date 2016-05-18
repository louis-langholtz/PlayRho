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

b2Float b2_toiTime, b2_toiMaxTime;
uint32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
uint32 b2_toiRootIters, b2_toiMaxRootIters;

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
		b2Float t1):
		m_proxyA(proxyA), m_proxyB(proxyB), m_sweepA(sweepA), m_sweepB(sweepB),
		m_type((cache.GetCount() != 1)? ((cache.GetIndexA(0) == cache.GetIndexA(1))? e_faceB: e_faceA): e_points)
	{
		b2Assert(cache.GetCount() > 0);
		b2Assert(cache.GetCount() <= 3); // < 3 or <= 3?

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

			m_axis = b2Normalize(b2Cross(localPointB2 - localPointB1, b2Float(1)));
			const auto normal = b2Mul(xfB.q, m_axis);

			m_localPoint = (localPointB1 + localPointB2) / b2Float(2);
			const auto pointB = b2Mul(xfB, m_localPoint);

			const auto localPointA = proxyA.GetVertex(cache.GetIndexA(0));
			const auto pointA = b2Mul(xfA, localPointA);

			auto s = b2Dot(pointA - pointB, normal);
			if (s < b2Float{0})
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
			
			m_axis = b2Normalize(b2Cross(localPointA2 - localPointA1, b2Float(1)));
			const auto normal = b2Mul(xfA.q, m_axis);

			m_localPoint = (localPointA1 + localPointA2) / b2Float(2);
			const auto pointA = b2Mul(xfA, m_localPoint);

			const auto localPointB = m_proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointB = b2Mul(xfB, localPointB);

			auto s = b2Dot(pointB - pointA, normal);
			if (s < b2Float{0})
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
	b2Float FindMinSeparation(b2DistanceProxy::size_type* indexA,
							  b2DistanceProxy::size_type* indexB,
							  b2Float t) const
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
			b2Assert(false);
			*indexA = static_cast<b2DistanceProxy::size_type>(-1);
			*indexB = static_cast<b2DistanceProxy::size_type>(-1);
			return b2Float{0};
		}
	}
	
	/// Evaluates the separation of the identified proxy vertices at the given time factor.
	/// @param indexA Index of the proxy A vertex.
	/// @param indexB Index of the proxy B vertex.
	/// @param t Time factor in range of [0,1] into the future, where 0 indicates alpha0.
	/// @return Separation distance.
	b2Float Evaluate(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB, b2Float t) const
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
		b2Assert(false);
		return b2Float{0};
	}

	const b2DistanceProxy& m_proxyA;
	const b2DistanceProxy& m_proxyB;
	const b2Sweep m_sweepA, m_sweepB;
	const Type m_type;
	b2Vec2 m_localPoint; // used if type is e_faceA or e_faceB
	b2Vec2 m_axis;
	
private:
	b2Float EvaluatePoints(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB,
						   const b2Transform& xfA, const b2Transform& xfB) const
	{
		const auto localPointA = m_proxyA.GetVertex(indexA);
		const auto localPointB = m_proxyB.GetVertex(indexB);
		const auto pointA = b2Mul(xfA, localPointA);
		const auto pointB = b2Mul(xfB, localPointB);
		return b2Dot(pointB - pointA, m_axis);
	}
	
	b2Float EvaluateFaceA(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB,
						  const b2Transform& xfA, const b2Transform& xfB) const
	{
		const auto normal = b2Mul(xfA.q, m_axis);
		const auto pointA = b2Mul(xfA, m_localPoint);
		const auto localPointB = m_proxyB.GetVertex(indexB);
		const auto pointB = b2Mul(xfB, localPointB);
		return b2Dot(pointB - pointA, normal);
	}
	
	b2Float EvaluateFaceB(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB,
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
b2TOIOutput box2d::b2TimeOfImpact(const b2TOIInput& input)
{
	++b2_toiCalls;

	auto output = b2TOIOutput{b2TOIOutput::e_unknown, input.tMax};

	const auto& proxyA = input.proxyA;
	const auto& proxyB = input.proxyB;

	auto sweepA = input.sweepA;
	auto sweepB = input.sweepB;

	// Large rotations can make the root finder fail, so we normalize the  sweep angles.
	sweepA.Normalize();
	sweepB.Normalize();

	const auto totalRadius = proxyA.GetRadius() + proxyB.GetRadius();
	const auto target = b2Max(b2_linearSlop, totalRadius - (b2Float{3} * b2_linearSlop));
	constexpr auto tolerance = b2_linearSlop / b2Float(4);
	b2Assert(target >= tolerance);

	auto t1 = b2Float{0};
	auto iter = decltype(b2_maxTOIIterations){0};

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
		if (distanceOutput.distance <= b2Float{0})
		{
			// Failure!
			output = b2TOIOutput{b2TOIOutput::e_overlapped, b2Float{0}};
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
			b2Float dx = b2Float(1) / N;
			b2Float xs[N+1];
			b2Float fs[N+1];

			b2Float x = b2Float{0};

			for (auto i = decltype(N){0}; i <= N; ++i)
			{
				const auto xfA = GetTransform(sweepA, x);
				const auto xfB = GetTransform(sweepB, x);
				b2Float f = fcn.Evaluate(xfA, xfB) - target;

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
		auto pushBackIter = decltype(b2_maxPolygonVertices){0};
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			b2DistanceProxy::size_type indexA, indexB;
			auto s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);

			// Is the final configuration separated?
			if (s2 > (target + tolerance))
			{
				// Victory!
				b2Assert(t2 == input.tMax);
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
			auto rootIterCount = decltype(b2_maxTOIRootIterCount){0};
			auto a1 = t1;
			auto a2 = t2;
			do
			{
				// Use a mix of the secant rule and bisection.
				const auto t = (rootIterCount & 1)?
					// Secant rule to improve convergence.
					a1 + (target - s1) * (a2 - a1) / (s2 - s1):
					// Bisection to guarantee progress.
					(a1 + a2) / b2Float(2);

				++rootIterCount;
				++b2_toiRootIters;

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
			while (rootIterCount < b2_maxTOIRootIterCount);

			b2_toiMaxRootIters = b2Max(b2_toiMaxRootIters, rootIterCount);

			++pushBackIter;

			if (pushBackIter == b2_maxPolygonVertices)
				break;
		}

		++iter;
		++b2_toiIters;

		if (done)
			break;

		if (iter == b2_maxTOIIterations)
		{
			// Root finder got stuck. Semi-victory.
			output = b2TOIOutput{b2TOIOutput::e_failed, t1};
			break;
		}
	}

	b2_toiMaxIters = b2Max(b2_toiMaxIters, iter);
	
	return output;
}
	
} // namespace box2d
