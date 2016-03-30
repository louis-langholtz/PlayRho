/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>

#define B2_DEBUG_SOLVER 0

bool g_blockSolve = true;

struct b2ContactPositionConstraintBodyData
{
	int32 index;
	float32 invMass;
	b2Vec2 localCenter;
	float32 invI;
};

class b2ContactPositionConstraint
{
public:
	b2Vec2 localNormal;
	b2Vec2 localPoint;

	b2ContactPositionConstraintBodyData bodyA;
	b2ContactPositionConstraintBodyData bodyB;

	b2Manifold::Type type;
	float32 radiusA, radiusB;

	int32 GetPointCount() const noexcept { return pointCount; }

	b2Vec2 GetPoint(int32 index) const
	{
		b2Assert(index < pointCount);
		return localPoints[index];
	}

	void ClearPoints()
	{
		pointCount = 0;
	}

	void AddPoint(const b2Vec2& val)
	{
		b2Assert(pointCount < b2_maxManifoldPoints);
		localPoints[pointCount] = val;
		++pointCount;
	}

private:
	int32 pointCount;
	b2Vec2 localPoints[b2_maxManifoldPoints];
};

void b2ContactSolver::Assign(b2ContactVelocityConstraint& var, const b2Contact& val)
{
	var.friction = val.m_friction;
	var.restitution = val.m_restitution;
	var.tangentSpeed = val.m_tangentSpeed;
}

void b2ContactSolver::Assign(b2ContactPositionConstraintBodyData& var, const b2Body& val)
{
	var.index = val.m_islandIndex;
	var.invMass = val.m_invMass;
	var.invI = val.m_invI;
	var.localCenter = val.m_sweep.localCenter;
}

void b2ContactSolver::Assign(b2ContactVelocityConstraintBodyData& var, const b2Body& val)
{
	var.index = val.m_islandIndex;
	var.invMass = val.m_invMass;
	var.invI = val.m_invI;
}

b2ContactSolver::b2ContactSolver(b2ContactSolverDef* def) :
	m_step(def->step),
	m_positions(def->positions),
	m_velocities(def->velocities),
	m_allocator(def->allocator),
	m_contacts(def->contacts),
	m_count(def->count),
	m_positionConstraints(static_cast<b2ContactPositionConstraint*>(m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint)))),
	m_velocityConstraints(static_cast<b2ContactVelocityConstraint*>(m_allocator->Allocate(m_count * sizeof(b2ContactVelocityConstraint))))
{
	// Initialize position independent portions of the constraints.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto contact = m_contacts[i];

		const auto fixtureA = contact->m_fixtureA;
		const auto fixtureB = contact->m_fixtureB;
		const auto shapeA = fixtureA->GetShape();
		const auto shapeB = fixtureB->GetShape();
		const auto radiusA = shapeA->GetRadius();
		const auto radiusB = shapeB->GetRadius();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		const auto manifold = contact->GetManifold();

		const auto pointCount = manifold->GetPointCount();
		b2Assert(pointCount > 0);

		auto& vc = m_velocityConstraints[i];
		Assign(vc, *contact);
		Assign(vc.bodyA, *bodyA);
		Assign(vc.bodyB, *bodyB);

		vc.contactIndex = i;
		vc.K = b2Mat22_zero;
		vc.normalMass = b2Mat22_zero;

		auto& pc = m_positionConstraints[i];
		
		Assign(pc.bodyA, *bodyA);
		Assign(pc.bodyB, *bodyB);

		pc.localNormal = manifold->GetLocalNormal();
		pc.localPoint = manifold->GetLocalPoint();
		pc.radiusA = radiusA;
		pc.radiusB = radiusB;
		pc.type = manifold->GetType();

		pc.ClearPoints();
		vc.ClearPoints();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto& cp = manifold->GetPoint(j);
			b2VelocityConstraintPoint vcp;
	
			if (m_step.warmStarting)
			{
				vcp.normalImpulse = m_step.dtRatio * cp.normalImpulse;
				vcp.tangentImpulse = m_step.dtRatio * cp.tangentImpulse;
			}
			else
			{
				vcp.normalImpulse = 0.0f;
				vcp.tangentImpulse = 0.0f;
			}

			vcp.rA = b2Vec2_zero;
			vcp.rB = b2Vec2_zero;
			vcp.normalMass = 0.0f;
			vcp.tangentMass = 0.0f;
			vcp.velocityBias = 0.0f;
			vc.AddPoint(vcp);

			pc.AddPoint(cp.localPoint);
		}
	}
}

b2ContactSolver::~b2ContactSolver()
{
	m_allocator->Free(m_velocityConstraints);
	m_allocator->Free(m_positionConstraints);
}

// Initialize position dependent portions of the velocity constraints.
void b2ContactSolver::InitializeVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		auto& vc = m_velocityConstraints[i];
		const auto& pc = m_positionConstraints[i];

		const auto radiusA = pc.radiusA;
		const auto radiusB = pc.radiusB;
		const auto manifold = m_contacts[vc.contactIndex]->GetManifold();

		const auto indexA = vc.bodyA.index;
		const auto mA = vc.bodyA.invMass;
		const auto iA = vc.bodyA.invI;

		const auto indexB = vc.bodyB.index;
		const auto mB = vc.bodyB.invMass;
		const auto iB = vc.bodyB.invI;

		const auto localCenterA = pc.bodyA.localCenter;
		const auto localCenterB = pc.bodyB.localCenter;

		const auto cA = m_positions[indexA].c;
		const auto aA = m_positions[indexA].a;
		const auto vA = m_velocities[indexA].v;
		const auto wA = m_velocities[indexA].w;

		const auto cB = m_positions[indexB].c;
		const auto aB = m_positions[indexB].a;
		const auto vB = m_velocities[indexB].v;
		const auto wB = m_velocities[indexB].w;

		b2Assert(manifold->GetPointCount() > 0);

		b2Transform xfA;
		xfA.q = b2Rot(aA);
		xfA.p = cA - b2Mul(xfA.q, localCenterA);

		b2Transform xfB;
		xfB.q = b2Rot(aB);
		xfB.p = cB - b2Mul(xfB.q, localCenterB);

		b2WorldManifold worldManifold;
		worldManifold.Assign(*manifold, xfA, radiusA, xfB, radiusB);

		vc.normal = worldManifold.GetNormal();

		const auto pointCount = vc.GetPointCount();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			auto& vcp = vc.GetPoint(j);

			const auto worldPoint = worldManifold.GetPoint(j);
			vcp.rA = worldPoint - cA;
			vcp.rB = worldPoint - cB;

			const auto rnA = b2Cross(vcp.rA, vc.normal);
			const auto rnB = b2Cross(vcp.rB, vc.normal);

			const auto kNormal = mA + mB + (iA * rnA * rnA) + (iB * rnB * rnB);

			vcp.normalMass = (kNormal > 0.0f)? 1.0f / kNormal : 0.0f;

			const auto tangent = b2Cross(vc.normal, 1.0f);

			const auto rtA = b2Cross(vcp.rA, tangent);
			const auto rtB = b2Cross(vcp.rB, tangent);

			const auto kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			vcp.tangentMass = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

			// Setup a velocity bias for restitution.
			vcp.velocityBias = 0.0f;
			const auto vRel = b2Dot(vc.normal, vB + b2Cross(wB, vcp.rB) - vA - b2Cross(wA, vcp.rA));
			if (vRel < -b2_velocityThreshold)
			{
				vcp.velocityBias = -vc.restitution * vRel;
			}
		}

		// If we have two points, then prepare the block solver.
		if ((vc.GetPointCount() == 2) && g_blockSolve)
		{
			const auto vcp1 = vc.GetPoint(0);
			const auto vcp2 = vc.GetPoint(1);

			const auto rn1A = b2Cross(vcp1.rA, vc.normal);
			const auto rn1B = b2Cross(vcp1.rB, vc.normal);
			const auto rn2A = b2Cross(vcp2.rA, vc.normal);
			const auto rn2B = b2Cross(vcp2.rB, vc.normal);

			const auto k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			const auto k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
			const auto k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

			// Ensure a reasonable condition number.
			const auto k_maxConditionNumber = 1000.0f;
			if ((k11 * k11) < (k_maxConditionNumber * (k11 * k22 - k12 * k12)))
			{
				// K is safe to invert.
				vc.K = b2Mat22(b2Vec2(k11, k12), b2Vec2(k12, k22));
				vc.normalMass = vc.K.GetInverse();
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc.RemovePoint();
			}
		}
	}
}

void b2ContactSolver::WarmStart()
{
	// Warm start.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& vc = m_velocityConstraints[i];

		const auto indexA = vc.bodyA.index;
		const auto mA = vc.bodyA.invMass;
		const auto iA = vc.bodyA.invI;

		const auto indexB = vc.bodyB.index;
		const auto mB = vc.bodyB.invMass;
		const auto iB = vc.bodyB.invI;
		
		const auto pointCount = vc.GetPointCount();

		auto vA = m_velocities[indexA].v;
		auto wA = m_velocities[indexA].w;
		auto vB = m_velocities[indexB].v;
		auto wB = m_velocities[indexB].w;

		const auto normal = vc.normal;
		const auto tangent = b2Cross(normal, 1.0f);

		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto vcp = vc.GetPoint(j);
			const auto P = vcp.normalImpulse * normal + vcp.tangentImpulse * tangent;
			wA -= iA * b2Cross(vcp.rA, P);
			vA -= mA * P;
			wB += iB * b2Cross(vcp.rB, P);
			vB += mB * P;
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		auto& vc = m_velocityConstraints[i];

		const auto indexA = vc.bodyA.index;
		const auto mA = vc.bodyA.invMass;
		const auto iA = vc.bodyA.invI;

		const auto indexB = vc.bodyB.index;
		const auto mB = vc.bodyB.invMass;
		const auto iB = vc.bodyB.invI;
		
		const auto pointCount = vc.GetPointCount();

		auto vA = m_velocities[indexA].v;
		auto wA = m_velocities[indexA].w;
		auto vB = m_velocities[indexB].v;
		auto wB = m_velocities[indexB].w;

		const auto normal = vc.normal;
		const auto tangent = b2Cross(normal, 1.0f);
		const auto friction = vc.friction;

		b2Assert((pointCount == 1) || (pointCount == 2));

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			auto& vcp = vc.GetPoint(j);

			// Relative velocity at contact
			const auto dv = vB + b2Cross(wB, vcp.rB) - vA - b2Cross(wA, vcp.rA);

			// Compute tangent force
			const auto vt = b2Dot(dv, tangent) - vc.tangentSpeed;
			auto lambda = vcp.tangentMass * (-vt);

			// b2Clamp the accumulated force
			const auto maxFriction = friction * vcp.normalImpulse;
			const auto newImpulse = b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - vcp.tangentImpulse;
			vcp.tangentImpulse = newImpulse;

			// Apply contact impulse
			const auto P = lambda * tangent;

			vA -= mA * P;
			wA -= iA * b2Cross(vcp.rA, P);

			vB += mB * P;
			wB += iB * b2Cross(vcp.rB, P);
		}

		// Solve normal constraints
		if ((pointCount == 1) || (!g_blockSolve))
		{
			for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
			{
				auto& vcp = vc.GetPoint(j);

				// Relative velocity at contact
				const auto dv = vB + b2Cross(wB, vcp.rB) - vA - b2Cross(wA, vcp.rA);

				// Compute normal impulse
				const auto vn = b2Dot(dv, normal);
				auto lambda = -vcp.normalMass * (vn - vcp.velocityBias);

				// b2Clamp the accumulated impulse
				const auto newImpulse = b2Max(vcp.normalImpulse + lambda, 0.0f);
				lambda = newImpulse - vcp.normalImpulse;
				vcp.normalImpulse = newImpulse;

				// Apply contact impulse
				const auto P = lambda * normal;
				vA -= mA * P;
				wA -= iA * b2Cross(vcp.rA, P);

				vB += mB * P;
				wB += iB * b2Cross(vcp.rB, P);
			}
		}
		else
		{
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			// 
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			// 
			// x = a + d
			// 
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse 
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			auto& cp1 = vc.GetPoint(0);
			auto& cp2 = vc.GetPoint(1);

			const auto a = b2Vec2(cp1.normalImpulse, cp2.normalImpulse);
			b2Assert(a.x >= 0.0f && a.y >= 0.0f);

			// Relative velocity at contact
			auto dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
			auto dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);

			// Compute normal velocity
			auto vn1 = b2Dot(dv1, normal);
			auto vn2 = b2Dot(dv2, normal);

			// Compute b'
			const auto b = b2Vec2{vn1 - cp1.velocityBias, vn2 - cp2.velocityBias} - b2Mul(vc.K, a);

			const auto k_errorTol = 1e-3f;
			B2_NOT_USED(k_errorTol);

			for (;;)
			{
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				auto x = - b2Mul(vc.normalMass, b);

				if ((x.x >= 0.0f) && (x.y >= 0.0f))
				{
					// Get the incremental impulse
					const auto d = x - a;

					// Apply incremental impulse
					const auto P1 = d.x * normal;
					const auto P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));

					// Accumulate
					cp1.normalImpulse = x.x;
					cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
					dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
					b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1' 
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.x = - cp1.normalMass * b.x;
				x.y = 0.0f;
				vn2 = vc.K.ex.y * x.x + b.y;

				if ((x.x >= 0.0f) && (vn2 >= 0.0f))
				{
					// Get the incremental impulse
					const auto d = x - a;

					// Apply incremental impulse
					const auto P1 = d.x * normal;
					const auto P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));

					// Accumulate
					cp1.normalImpulse = x.x;
					cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
#endif
					break;
				}


				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1' 
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.x = 0.0f;
				x.y = - cp2.normalMass * b.y;
				vn1 = vc.K.ey.x * x.y + b.x;

				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					const auto d = x - a;

					// Apply incremental impulse
					const auto P1 = d.x * normal;
					const auto P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));

					// Accumulate
					cp1.normalImpulse = x.x;
					cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				// 
				// vn1 = b1
				// vn2 = b2;
				x.x = 0.0f;
				x.y = 0.0f;
				vn1 = b.x;
				vn2 = b.y;

				if (vn1 >= 0.0f && vn2 >= 0.0f )
				{
					// Resubstitute for the incremental impulse
					const auto d = x - a;

					// Apply incremental impulse
					const auto P1 = d.x * normal;
					const auto P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));

					// Accumulate
					cp1.normalImpulse = x.x;
					cp2.normalImpulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b2ContactSolver::StoreImpulses()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& vc = m_velocityConstraints[i];
		auto& manifold = *(m_contacts[vc.contactIndex]->GetManifold());

		const auto point_count = vc.GetPointCount();
		for (auto j = decltype(point_count){0}; j < point_count; ++j)
		{
			auto& manifold_point = manifold.GetPoint(j);
			const auto& vc_point = vc.GetPoint(j);
			manifold_point.normalImpulse = vc_point.normalImpulse;
			manifold_point.tangentImpulse = vc_point.tangentImpulse;
		}
	}
}

class b2PositionSolverManifold
{
public:
	b2PositionSolverManifold() = delete;

	b2PositionSolverManifold(const b2ContactPositionConstraint& pc,
							 const b2Transform& xfA, const b2Transform& xfB, int32 index)
	{
		b2Assert(pc.GetPointCount() > 0);

		switch (pc.type)
		{
		case b2Manifold::e_circles:
			{
				const auto pointA = b2Mul(xfA, pc.localPoint);
				const auto pointB = b2Mul(xfB, pc.GetPoint(0));
				normal = b2Normalize(pointB - pointA);
				point = 0.5f * (pointA + pointB);
				separation = b2Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
			}
			break;

		case b2Manifold::e_faceA:
			{
				normal = b2Mul(xfA.q, pc.localNormal);
				const auto planePoint = b2Mul(xfA, pc.localPoint);

				const auto clipPoint = b2Mul(xfB, pc.GetPoint(index));
				separation = b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
				point = clipPoint;
			}
			break;

		case b2Manifold::e_faceB:
			{
				normal = b2Mul(xfB.q, pc.localNormal);
				const auto planePoint = b2Mul(xfB, pc.localPoint);

				const auto clipPoint = b2Mul(xfA, pc.GetPoint(index));
				separation = b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
				point = clipPoint;

				// Ensure normal points from A to B
				normal = -normal;
			}
			break;
		}
	}

	b2Vec2 GetNormal() const noexcept { return normal; }
	b2Vec2 GetPoint() const noexcept { return point; }
	float32 GetSeparation() const noexcept { return separation; }

private:
	b2Vec2 normal;
	b2Vec2 point;
	float32 separation;
};

// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints()
{
	auto minSeparation = 0.0f;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];

		const auto indexA = pc.bodyA.index;
		const auto localCenterA = pc.bodyA.localCenter;
		const auto mA = pc.bodyA.invMass;
		const auto iA = pc.bodyA.invI;

		const auto indexB = pc.bodyB.index;
		const auto localCenterB = pc.bodyB.localCenter;
		const auto mB = pc.bodyB.invMass;
		const auto iB = pc.bodyB.invI;

		const auto pointCount = pc.GetPointCount();

		auto cA = m_positions[indexA].c;
		auto aA = m_positions[indexA].a;

		auto cB = m_positions[indexB].c;
		auto aB = m_positions[indexB].a;

		// Solve normal constraints
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q = b2Rot(aA);
			xfB.q = b2Rot(aB);
			xfA.p = cA - b2Mul(xfA.q, localCenterA);
			xfB.p = cB - b2Mul(xfB.q, localCenterB);

			const auto psm = b2PositionSolverManifold(pc, xfA, xfB, j);
			const auto normal = psm.GetNormal();
			const auto point = psm.GetPoint();
			const auto separation = psm.GetSeparation();

			const auto rA = point - cA;
			const auto rB = point - cB;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			const auto C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			const auto rnA = b2Cross(rA, normal);
			const auto rnB = b2Cross(rB, normal);
			const auto K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			const auto impulse = (K > 0.0f)? - C / K : 0.0f;

			const auto P = impulse * normal;

			cA -= mA * P;
			aA -= iA * b2Cross(rA, P);

			cB += mB * P;
			aB += iB * b2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * b2_linearSlop;
}

// Sequential position solver for position constraints.
bool b2ContactSolver::SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB)
{
	auto minSeparation = 0.0f;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];

		const auto indexA = pc.bodyA.index;
		const auto localCenterA = pc.bodyA.localCenter;

		const auto indexB = pc.bodyB.index;
		const auto localCenterB = pc.bodyB.localCenter;
		
		auto mA = 0.0f;
		auto iA = 0.0f;
		if ((indexA == toiIndexA) || (indexA == toiIndexB))
		{
			mA = pc.bodyA.invMass;
			iA = pc.bodyA.invI;
		}

		auto mB = 0.0f;
		auto iB = 0.f;
		if ((indexB == toiIndexA) || (indexB == toiIndexB))
		{
			mB = pc.bodyB.invMass;
			iB = pc.bodyB.invI;
		}

		auto cA = m_positions[indexA].c;
		auto aA = m_positions[indexA].a;

		auto cB = m_positions[indexB].c;
		auto aB = m_positions[indexB].a;

		// Solve normal constraints
		const auto pointCount = pc.GetPointCount();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q = b2Rot(aA);
			xfB.q = b2Rot(aB);
			xfA.p = cA - b2Mul(xfA.q, localCenterA);
			xfB.p = cB - b2Mul(xfB.q, localCenterB);

			const auto psm = b2PositionSolverManifold(pc, xfA, xfB, j);
			const auto normal = psm.GetNormal();
			const auto point = psm.GetPoint();
			const auto separation = psm.GetSeparation();

			const auto rA = point - cA;
			const auto rB = point - cB;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			const auto C = b2Clamp(b2_toiBaugarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			const auto rnA = b2Cross(rA, normal);
			const auto rnB = b2Cross(rB, normal);
			const auto K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			const auto impulse = (K > 0.0f) ? - C / K : 0.0f;

			const auto P = impulse * normal;

			cA -= mA * P;
			aA -= iA * b2Cross(rA, P);

			cB += mB * P;
			aB += iB * b2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5f * b2_linearSlop;
}
