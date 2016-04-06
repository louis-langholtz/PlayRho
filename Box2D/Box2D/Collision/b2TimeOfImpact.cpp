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

float32 b2_toiTime, b2_toiMaxTime;
int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
int32 b2_toiRootIters, b2_toiMaxRootIters;

//
struct b2SeparationFunction
{
	enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	};

	b2SeparationFunction(const b2SimplexCache& cache,
		const b2DistanceProxy& proxyA, const b2Sweep& sweepA,
		const b2DistanceProxy& proxyB, const b2Sweep& sweepB,
		float32 t1):
		m_proxyA(proxyA), m_proxyB(proxyB), m_sweepA(sweepA), m_sweepB(sweepB),
		m_type((cache.GetCount() != 1)? ((cache.GetIndexA(0) == cache.GetIndexA(1))? e_faceB: e_faceA): e_points)
	{
		b2Assert(cache.GetCount() > 0);
		b2Assert(cache.GetCount() <= 3); // < 3 or <= 3?

		const auto xfA = m_sweepA.GetTransform(t1);
		const auto xfB = m_sweepB.GetTransform(t1);

		switch (m_type)
		{
		case e_points:
		{
			const auto localPointA = m_proxyA.GetVertex(cache.GetIndexA(0));
			const auto localPointB = m_proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointA = b2Mul(xfA, localPointA);
			const auto pointB = b2Mul(xfB, localPointB);
			m_axis = pointB - pointA;
			m_axis.Normalize();
			break;
		}
		case e_faceB:
		{
			// Two points on B and one on A.
			const auto localPointB1 = proxyB.GetVertex(cache.GetIndexB(0));
			const auto localPointB2 = proxyB.GetVertex(cache.GetIndexB(1));

			m_axis = b2Normalize(b2Cross(localPointB2 - localPointB1, 1.0f));
			const auto normal = b2Mul(xfB.q, m_axis);

			m_localPoint = 0.5f * (localPointB1 + localPointB2);
			const auto pointB = b2Mul(xfB, m_localPoint);

			const auto localPointA = proxyA.GetVertex(cache.GetIndexA(0));
			const auto pointA = b2Mul(xfA, localPointA);

			auto s = b2Dot(pointA - pointB, normal);
			if (s < 0.0f)
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
			
			m_axis = b2Normalize(b2Cross(localPointA2 - localPointA1, 1.0f));
			const auto normal = b2Mul(xfA.q, m_axis);

			m_localPoint = 0.5f * (localPointA1 + localPointA2);
			const auto pointA = b2Mul(xfA, m_localPoint);

			const auto localPointB = m_proxyB.GetVertex(cache.GetIndexB(0));
			const auto pointB = b2Mul(xfB, localPointB);

			auto s = b2Dot(pointB - pointA, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
			}
			break;
		}
		}
	}

	//
	float32 FindMinSeparation(b2DistanceProxy::size_type* indexA,
							  b2DistanceProxy::size_type* indexB,
							  float32 t) const
	{
		const auto xfA = m_sweepA.GetTransform(t);
		const auto xfB = m_sweepB.GetTransform(t);

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
			return 0.0f;
		}
	}

	//
	float32 Evaluate(b2DistanceProxy::size_type indexA, b2DistanceProxy::size_type indexB, float32 t) const
	{
		const auto xfA = m_sweepA.GetTransform(t);
		const auto xfB = m_sweepB.GetTransform(t);

		switch (m_type)
		{
		case e_points:
			{
				const auto localPointA = m_proxyA.GetVertex(indexA);
				const auto localPointB = m_proxyB.GetVertex(indexB);

				const auto pointA = b2Mul(xfA, localPointA);
				const auto pointB = b2Mul(xfB, localPointB);
				return b2Dot(pointB - pointA, m_axis);
			}

		case e_faceA:
			{
				const auto normal = b2Mul(xfA.q, m_axis);
				const auto pointA = b2Mul(xfA, m_localPoint);

				const auto localPointB = m_proxyB.GetVertex(indexB);
				const auto pointB = b2Mul(xfB, localPointB);

				return b2Dot(pointB - pointA, normal);
			}

		case e_faceB:
			{
				const auto normal = b2Mul(xfB.q, m_axis);
				const auto pointB = b2Mul(xfB, m_localPoint);

				const auto localPointA = m_proxyA.GetVertex(indexA);
				const auto pointA = b2Mul(xfA, localPointA);

				return b2Dot(pointA - pointB, normal);
			}

		default:
			b2Assert(false);
			return 0.0f;
		}
	}

	const b2DistanceProxy& m_proxyA;
	const b2DistanceProxy& m_proxyB;
	const b2Sweep m_sweepA, m_sweepB;
	const Type m_type;
	b2Vec2 m_localPoint; // used if type is e_faceA or e_faceB
	b2Vec2 m_axis;
};

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
void b2TimeOfImpact(b2TOIOutput& output, const b2TOIInput& input)
{
	b2Timer timer;

	++b2_toiCalls;

	output.state = b2TOIOutput::e_unknown;
	output.t = input.tMax;

	const auto& proxyA = input.proxyA;
	const auto& proxyB = input.proxyB;

	auto sweepA = input.sweepA;
	auto sweepB = input.sweepB;

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweepA.Normalize();
	sweepB.Normalize();

	const auto tMax = input.tMax;

	const auto totalRadius = proxyA.GetRadius() + proxyB.GetRadius();
	const auto target = b2Max(b2_linearSlop, totalRadius - (3.0f * b2_linearSlop));
	constexpr auto tolerance = 0.25f * b2_linearSlop;
	b2Assert(target >= tolerance);

	auto t1 = 0.0f;
	auto iter = decltype(b2_maxTOIIterations){0};

	// Prepare input for distance query.
	b2SimplexCache cache;
	b2DistanceInput distanceInput;
	distanceInput.proxyA = input.proxyA;
	distanceInput.proxyB = input.proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for(;;)
	{
		distanceInput.transformA = sweepA.GetTransform(t1);
		distanceInput.transformB = sweepB.GetTransform(t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		b2DistanceOutput distanceOutput;
		b2Distance(&distanceOutput, &cache, distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0.0f)
		{
			// Failure!
			output.state = b2TOIOutput::e_overlapped;
			output.t = 0.0f;
			break;
		}

		if (distanceOutput.distance < (target + tolerance))
		{
			// Victory!
			output.state = b2TOIOutput::e_touching;
			output.t = t1;
			break;
		}

		// Initialize the separating axis.
		b2SeparationFunction fcn(cache, proxyA, sweepA, proxyB, sweepB, t1);
#if 0
		// Dump the curve seen by the root finder
		{
			const int32 N = 100;
			float32 dx = 1.0f / N;
			float32 xs[N+1];
			float32 fs[N+1];

			float32 x = 0.0f;

			for (auto i = decltype(N){0}; i <= N; ++i)
			{
				const auto xfA = sweepA.GetTransform(x);
				const auto xfB = sweepB.GetTransform(x);
				float32 f = fcn.Evaluate(xfA, xfB) - target;

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
				output.state = b2TOIOutput::e_separated;
				output.t = tMax;
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
				output.state = b2TOIOutput::e_failed;
				output.t = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= (target + tolerance))
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output.state = b2TOIOutput::e_touching;
				output.t = t1;
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
					0.5f * (a1 + a2);

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
			output.state = b2TOIOutput::e_failed;
			output.t = t1;
			break;
		}
	}

	b2_toiMaxIters = b2Max(b2_toiMaxIters, iter);

	const auto time = timer.GetMilliseconds();
	b2_toiMaxTime = b2Max(b2_toiMaxTime, time);
	b2_toiTime += time;
}
