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

#if !defined(NDEBUG)
// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
// #define B2_DEBUG_SOLVER 1
#endif

#if defined(B2_DEBUG_SOLVER)
static constexpr auto k_errorTol = b2Float(2e-3); ///< error tolerance
static constexpr auto k_majorErrorTol = b2Float(1e-2); ///< error tolerance
#endif

bool g_blockSolve = true;

struct b2ContactPositionConstraintBodyData
{
	using index_t = b2_size_t;

	index_t index; ///< Index within island of the associated body.
	b2Float invMass; ///< Inverse mass of associated body.
	b2Vec2 localCenter;
	b2Float invI; ///< Inverse rotational inertia about the center of mass of the associated body.
};

class b2ContactPositionConstraint
{
public:
	using size_type = std::remove_cv<decltype(b2_maxManifoldPoints)>::type;

	b2Vec2 localNormal;
	b2Vec2 localPoint;

	b2ContactPositionConstraintBodyData bodyA;
	b2ContactPositionConstraintBodyData bodyB;

	b2Manifold::Type type = b2Manifold::e_unset;
	b2Float radiusA, radiusB;

	size_type GetPointCount() const noexcept { return pointCount; }

	b2Vec2 GetPoint(size_type index) const
	{
		b2Assert((0 <= index) && (index < pointCount));
		return localPoints[index];
	}

	void ClearPoints() noexcept
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
	size_type pointCount = 0;
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
	b2Assert(val.IsValidIslandIndex());
	var.index = val.m_islandIndex;
	var.invMass = val.m_invMass;
	var.invI = val.m_invI;
	var.localCenter = val.m_sweep.localCenter;
}

void b2ContactSolver::Assign(b2ContactVelocityConstraintBodyData& var, const b2Body& val)
{
	b2Assert(val.IsValidIslandIndex());
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
		const auto& contact = *m_contacts[i];

		const auto fixtureA = contact.m_fixtureA;
		const auto fixtureB = contact.m_fixtureB;
		const auto shapeA = fixtureA->GetShape();
		const auto shapeB = fixtureB->GetShape();
		const auto radiusA = shapeA->GetRadius();
		const auto radiusB = shapeB->GetRadius();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		const auto manifold = contact.GetManifold();

		const auto pointCount = manifold->GetPointCount();
		b2Assert(pointCount > 0);

		auto& vc = m_velocityConstraints[i];
		Assign(vc, contact);
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
			const auto& mp = manifold->GetPoint(j); ///< Manifold point.
			b2VelocityConstraintPoint vcp;
	
			if (m_step.warmStarting)
			{
				vcp.normalImpulse = m_step.dtRatio * mp.normalImpulse;
				vcp.tangentImpulse = m_step.dtRatio * mp.tangentImpulse;
			}
			else
			{
				vcp.normalImpulse = b2Float{0};
				vcp.tangentImpulse = b2Float{0};
			}

			vcp.rA = b2Vec2_zero;
			vcp.rB = b2Vec2_zero;
			vcp.normalMass = b2Float{0};
			vcp.tangentMass = b2Float{0};
			vcp.velocityBias = b2Float{0};
			vc.AddPoint(vcp);

			pc.AddPoint(mp.localPoint);
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
		const auto invMassA = vc.bodyA.invMass;
		const auto invInertiaA = vc.bodyA.invI;

		const auto indexB = vc.bodyB.index;
		const auto invMassB = vc.bodyB.invMass;
		const auto invInertiaB = vc.bodyB.invI;

		const auto localCenterA = pc.bodyA.localCenter;
		const auto localCenterB = pc.bodyB.localCenter;

		b2Assert(indexA >= 0);
		const auto cA = m_positions[indexA].c;
		const auto aA = m_positions[indexA].a;
		const auto vA = m_velocities[indexA].v;
		const auto wA = m_velocities[indexA].w;

		b2Assert(indexB >= 0);
		const auto cB = m_positions[indexB].c;
		const auto aB = m_positions[indexB].a;
		const auto vB = m_velocities[indexB].v;
		const auto wB = m_velocities[indexB].w;

		b2Assert(manifold->GetPointCount() > 0);

		const auto xfA = b2Displace(cA, localCenterA, b2Rot(aA));
		const auto xfB = b2Displace(cB, localCenterB, b2Rot(aB));
		const b2WorldManifold worldManifold(*manifold, xfA, radiusA, xfB, radiusB);

		vc.normal = worldManifold.GetNormal();

		const auto pointCount = vc.GetPointCount();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto worldPoint = worldManifold.GetPoint(j);
			const auto vcp_rA = worldPoint - cA;
			const auto vcp_rB = worldPoint - cB;
			
			const auto rnA = b2Cross(vcp_rA, vc.normal);
			const auto rnB = b2Cross(vcp_rB, vc.normal);

			const auto kNormal = invMassA + invMassB + (invInertiaA * b2Square(rnA)) + (invInertiaB * b2Square(rnB));

			const auto tangent = b2Cross(vc.normal, b2Float(1));

			const auto rtA = b2Cross(vcp_rA, tangent);
			const auto rtB = b2Cross(vcp_rB, tangent);

			const auto kTangent = invMassA + invMassB + (invInertiaA * b2Square(rtA)) + (invInertiaB * b2Square(rtB));

			// Relative velocity at contact
			const auto dv = (vB + b2Cross(wB, vcp_rB)) - (vA + b2Cross(wA, vcp_rA));
			const auto vRel = b2Dot(dv, vc.normal);

			auto& vcp = vc.GetPoint(j); ///< Velocity constraint point.
			vcp.rA = vcp_rA;
			vcp.rB = vcp_rB;
			vcp.normalMass = (kNormal > b2Float{0})? b2Float(1) / kNormal : b2Float{0};
			vcp.tangentMass = (kTangent > b2Float{0}) ? b2Float(1) /  kTangent : b2Float{0};
			vcp.velocityBias = (vRel < -b2_velocityThreshold)? -vc.restitution * vRel: b2Float{0};
		}

		// If we have two points, then prepare the block solver.
		if ((pointCount == 2) && g_blockSolve)
		{
			const auto vcp1 = vc.GetPoint(0);
			const auto vcp2 = vc.GetPoint(1);

			const auto rn1A = b2Cross(vcp1.rA, vc.normal);
			const auto rn1B = b2Cross(vcp1.rB, vc.normal);
			const auto rn2A = b2Cross(vcp2.rA, vc.normal);
			const auto rn2B = b2Cross(vcp2.rB, vc.normal);

			const auto k11 = invMassA + invMassB + (invInertiaA * b2Square(rn1A)) + (invInertiaB * b2Square(rn1B));
			const auto k22 = invMassA + invMassB + (invInertiaA * b2Square(rn2A)) + (invInertiaB * b2Square(rn2B));
			const auto k12 = invMassA + invMassB + (invInertiaA * rn1A * rn2A) + (invInertiaB * rn1B * rn2B);

			// Ensure a reasonable condition number.
			constexpr auto k_maxConditionNumber = b2Float(1000);
			if (b2Square(k11) < (k_maxConditionNumber * (k11 * k22 - b2Square(k12))))
			{
				// K is safe to invert.
				vc.K = b2Mat22{b2Vec2{k11, k12}, b2Vec2{k12, k22}};
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

static inline void WarmStart(const b2ContactVelocityConstraint& vc, b2Velocity& bodyA, b2Velocity& bodyB)
{
	const auto tangent = b2Cross(vc.normal, b2Float(1));
	const auto pointCount = vc.GetPointCount();	
	for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
	{
		const auto vcp = vc.GetPoint(j); ///< Velocity constraint point.
		const auto P = vcp.normalImpulse * vc.normal + vcp.tangentImpulse * tangent;
		bodyA.v -= vc.bodyA.invMass * P;
		bodyA.w -= vc.bodyA.invI * b2Cross(vcp.rA, P);
		bodyB.v += vc.bodyB.invMass * P;
		bodyB.w += vc.bodyB.invI * b2Cross(vcp.rB, P);
	}
}

void b2ContactSolver::WarmStart()
{
	// Warm start.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& vc = m_velocityConstraints[i];
		::WarmStart(vc, m_velocities[vc.bodyA.index], m_velocities[vc.bodyB.index]);
	}
}

static inline void SolveTangentConstraint(const b2ContactVelocityConstraint& vc, b2Vec2 tangent,
										 b2Velocity& bodyA, b2Velocity& bodyB,
										 b2VelocityConstraintPoint& vcp)
{
	// Relative velocity at contact
	const auto dv = (bodyB.v + b2Cross(bodyB.w, vcp.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp.rA));
	
	// Compute tangent force
	const auto vt = b2Dot(dv, tangent) - vc.tangentSpeed;
	const auto lambda = vcp.tangentMass * (-vt);
	
	// Clamp the accumulated force
	const auto maxFriction = vc.friction * vcp.normalImpulse;
	const auto oldImpulse = vcp.tangentImpulse;
	const auto newImpulse = b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Save new impulse
	vcp.tangentImpulse = newImpulse;
	
	// Apply contact impulse
	const auto P = incImpulse * tangent;
	bodyA.v -= vc.bodyA.invMass * P;
	bodyA.w -= vc.bodyA.invI * b2Cross(vcp.rA, P);
	bodyB.v += vc.bodyB.invMass * P;
	bodyB.w += vc.bodyB.invI * b2Cross(vcp.rB, P);
}

static inline void SolveNormalConstraint(const b2ContactVelocityConstraint& vc,
										 b2Velocity& bodyA, b2Velocity& bodyB,
										 b2VelocityConstraintPoint& vcp)
{
	// Relative velocity at contact
	const auto dv = (bodyB.v + b2Cross(bodyB.w, vcp.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp.rA));
	
	// Compute normal impulse
	const auto vn = b2Dot(dv, vc.normal);
	const auto lambda = -vcp.normalMass * (vn - vcp.velocityBias);
	
	// b2Clamp the accumulated impulse
	const auto oldImpulse = vcp.normalImpulse;
	const auto newImpulse = b2Max(vcp.normalImpulse + lambda, b2Float{0});
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Save new impulse
	vcp.normalImpulse = newImpulse;
	
	// Apply contact impulse
	const auto P = incImpulse * vc.normal;
	bodyA.v -= vc.bodyA.invMass * P;
	bodyA.w -= vc.bodyA.invI * b2Cross(vcp.rA, P);
	bodyB.v += vc.bodyB.invMass * P;
	bodyB.w += vc.bodyB.invI * b2Cross(vcp.rB, P);
}

static inline void BlockSolveUpdate(const b2ContactVelocityConstraint& vc,
									b2Vec2 oldImpulse, b2Vec2 newImpulse,
									b2Velocity& bodyA, b2Velocity& bodyB,
									b2VelocityConstraintPoint& vcp1, b2VelocityConstraintPoint& vcp2)
{
	// Get the incremental impulse
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Apply incremental impulse
	const auto P1 = incImpulse.x * vc.normal;
	const auto P2 = incImpulse.y * vc.normal;
	const auto P = P1 + P2;
	bodyA.v -= vc.bodyA.invMass * P;
	bodyA.w -= vc.bodyA.invI * (b2Cross(vcp1.rA, P1) + b2Cross(vcp2.rA, P2));
	bodyB.v += vc.bodyB.invMass * P;
	bodyB.w += vc.bodyB.invI * (b2Cross(vcp1.rB, P1) + b2Cross(vcp2.rB, P2));
	
	// Save new impulse
	vcp1.normalImpulse = newImpulse.x;
	vcp2.normalImpulse = newImpulse.y;
}

static inline bool BlockSolveNormalCase1(const b2ContactVelocityConstraint& vc,
										 b2Vec2 oldImpulse, b2Vec2 b_prime,
										 b2Velocity& bodyA, b2Velocity& bodyB,
										 b2VelocityConstraintPoint& vcp1, b2VelocityConstraintPoint& vcp2)
{
	//
	// Case 1: vn = 0
	//
	// 0 = A * x + b'
	//
	// Solve for x:
	//
	// x = -inv(A) * b'
	//
	const auto newImpulse = -b2Mul(vc.normalMass, b_prime);
	if ((newImpulse.x >= b2Float{0}) && (newImpulse.y >= b2Float{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv1 = (bodyB.v + b2Cross(bodyB.w, vcp1.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp1.rA));
		const auto post_dv2 = (bodyB.v + b2Cross(bodyB.w, vcp2.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto post_vn1 = b2Dot(post_dv1, vc.normal);
		const auto post_vn2 = b2Dot(post_dv2, vc.normal);
		
		
		b2Assert(b2Abs(post_vn1 - vcp1.velocityBias) < k_majorErrorTol);
		b2Assert(b2Abs(post_vn2 - vcp2.velocityBias) < k_majorErrorTol);

		b2Assert(b2Abs(post_vn1 - vcp1.velocityBias) < k_errorTol);
		b2Assert(b2Abs(post_vn2 - vcp2.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase2(const b2ContactVelocityConstraint& vc,
										 b2Vec2 oldImpulse, b2Vec2 b_prime,
										 b2Velocity& bodyA, b2Velocity& bodyB,
										 b2VelocityConstraintPoint& vcp1, b2VelocityConstraintPoint& vcp2)
{
	//
	// Case 2: vn1 = 0 and x2 = 0
	//
	//   0 = a11 * x1 + a12 * 0 + b1' 
	// vn2 = a21 * x1 + a22 * 0 + b2'
	//
	const auto newImpulse = b2Vec2{-vcp1.normalMass * b_prime.x, b2Float{0}};
	const auto vn2 = vc.K.ex.y * newImpulse.x + b_prime.y;
	if ((newImpulse.x >= b2Float{0}) && (vn2 >= b2Float{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv1 = (bodyB.v + b2Cross(bodyB.w, vcp1.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp1.rA));
		
		// Compute normal velocity
		const auto post_vn1 = b2Dot(post_dv1, vc.normal);
		
		b2Assert(b2Abs(post_vn1 - vcp1.velocityBias) < k_majorErrorTol);
		b2Assert(b2Abs(post_vn1 - vcp1.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase3(const b2ContactVelocityConstraint& vc,
										 b2Vec2 oldImpulse, b2Vec2 b_prime,
										 b2Velocity& bodyA, b2Velocity& bodyB,
										 b2VelocityConstraintPoint& vcp1, b2VelocityConstraintPoint& vcp2)
{
	//
	// Case 3: vn2 = 0 and x1 = 0
	//
	// vn1 = a11 * 0 + a12 * x2 + b1' 
	//   0 = a21 * 0 + a22 * x2 + b2'
	//
	const auto newImpulse = b2Vec2{b2Float{0}, -vcp2.normalMass * b_prime.y};
	const auto vn1 = vc.K.ey.x * newImpulse.y + b_prime.x;
	if ((newImpulse.y >= b2Float{0}) && (vn1 >= b2Float{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv2 = (bodyB.v + b2Cross(bodyB.w, vcp2.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto post_vn2 = b2Dot(post_dv2, vc.normal);

		b2Assert(b2Abs(post_vn2 - vcp2.velocityBias) < k_majorErrorTol);
		b2Assert(b2Abs(post_vn2 - vcp2.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase4(const b2ContactVelocityConstraint& vc,
										 b2Vec2 oldImpulse, b2Vec2 b_prime,
										 b2Velocity& bodyA, b2Velocity& bodyB,
										 b2VelocityConstraintPoint& vcp1, b2VelocityConstraintPoint& vcp2)
{
	//
	// Case 4: x1 = 0 and x2 = 0
	// 
	// vn1 = b1
	// vn2 = b2;
	const auto newImpulse = b2Vec2{b2Float{0}, b2Float{0}};
	const auto vn1 = b_prime.x;
	const auto vn2 = b_prime.y;
	if ((vn1 >= b2Float{0}) && (vn2 >= b2Float{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		return true;
	}
	return false;
}

static inline void BlockSolveNormalConstraint(const b2ContactVelocityConstraint& vc,
											  b2Velocity& bodyA, b2Velocity& bodyB,
											  b2VelocityConstraintPoint& vcp1, b2VelocityConstraintPoint& vcp2)
{
	// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
	// Build the mini LCP for this contact patch
	//
	// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
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

	const auto oldImpulse = b2Vec2{vcp1.normalImpulse, vcp2.normalImpulse};
	b2Assert((oldImpulse.x >= b2Float{0}) && (oldImpulse.y >= b2Float{0}));
	
	const auto b_prime = [=]{
		// Relative velocity at contact
		const auto dv1 = (bodyB.v + b2Cross(bodyB.w, vcp1.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp1.rA));
		const auto dv2 = (bodyB.v + b2Cross(bodyB.w, vcp2.rB)) - (bodyA.v + b2Cross(bodyA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto normal_vn1 = b2Dot(dv1, vc.normal);
		const auto normal_vn2 = b2Dot(dv2, vc.normal);
		
		// Compute b
		const auto b = b2Vec2{normal_vn1 - vcp1.velocityBias, normal_vn2 - vcp2.velocityBias};
		
		// Return b'
		return b - b2Mul(vc.K, oldImpulse);
	}();
	
	
	if (BlockSolveNormalCase1(vc, oldImpulse, b_prime, bodyA, bodyB, vcp1, vcp2))
		return;
	if (BlockSolveNormalCase2(vc, oldImpulse, b_prime, bodyA, bodyB, vcp1, vcp2))
		return;
	if (BlockSolveNormalCase3(vc, oldImpulse, b_prime, bodyA, bodyB, vcp1, vcp2))
		return;
	if (BlockSolveNormalCase4(vc, oldImpulse, b_prime, bodyA, bodyB, vcp1, vcp2))
		return;
	
	// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
}

static inline void SolveVelocityConstraint(b2ContactVelocityConstraint& vc, b2Velocity& bodyA, b2Velocity& bodyB)
{
	const auto pointCount = vc.GetPointCount();
	b2Assert((pointCount == 1) || (pointCount == 2));

	// Solve tangent constraints first, because non-penetration is more important than friction.
	// Solve normal constraints second.

	if (pointCount == 1)
	{
		auto& vcp = vc.GetPoint(0);

		{
			const auto tangent = b2Cross(vc.normal, b2Float(1));
			SolveTangentConstraint(vc, tangent, bodyA, bodyB, vcp);
		}

		SolveNormalConstraint(vc, bodyA, bodyB, vcp);
	}
	else // pointCount == 2
	{
		auto& vcp1 = vc.GetPoint(0); ///< Velocity constraint point.
		auto& vcp2 = vc.GetPoint(1); ///< Velocity constraint point.

		{
			const auto tangent = b2Cross(vc.normal, b2Float(1));
			SolveTangentConstraint(vc, tangent, bodyA, bodyB, vcp1);
			SolveTangentConstraint(vc, tangent, bodyA, bodyB, vcp2);
		}

		if (!g_blockSolve)
		{
			SolveNormalConstraint(vc, bodyA, bodyB, vcp1);
			SolveNormalConstraint(vc, bodyA, bodyB, vcp2);
		}
		else
		{
			BlockSolveNormalConstraint(vc, bodyA, bodyB, vcp1, vcp2);
		}
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		auto& vc = m_velocityConstraints[i];
		SolveVelocityConstraint(vc, m_velocities[vc.bodyA.index], m_velocities[vc.bodyB.index]);
	}
}

static void b2AssignImpulses(b2ManifoldPoint& var, const b2VelocityConstraintPoint& val)
{
	var.normalImpulse = val.normalImpulse;
	var.tangentImpulse = val.tangentImpulse;
}

void b2ContactSolver::StoreImpulses()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& vc = m_velocityConstraints[i];
		auto& manifold = *(m_contacts[vc.contactIndex]->GetManifold());

		const auto point_count = vc.GetPointCount();
		for (auto j = decltype(point_count){0}; j < point_count; ++j)
			b2AssignImpulses(manifold.GetPoint(j), vc.GetPoint(j));
	}
}

class b2PositionSolverManifold
{
public:
	using index_t = std::remove_cv<decltype(b2_maxManifoldPoints)>::type;

	b2PositionSolverManifold() = delete;

	b2PositionSolverManifold(const b2ContactPositionConstraint& pc,
							 const b2Transform& xfA, const b2Transform& xfB, index_t index)
	{
		b2Assert(pc.GetPointCount() > 0);
		b2Assert(pc.type != b2Manifold::e_unset);

		switch (pc.type)
		{
		case b2Manifold::e_unset:
			break;

		case b2Manifold::e_circles:
			{
				b2Assert(index == 0);
				const auto pointA = b2Mul(xfA, pc.localPoint);
				const auto pointB = b2Mul(xfB, pc.GetPoint(0));
				normal = b2Normalize(pointB - pointA);
				point = (pointA + pointB) / b2Float(2);
				separation = b2Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
			}
			break;

		case b2Manifold::e_faceA:
			{
				const auto planePoint = b2Mul(xfA, pc.localPoint);
				const auto clipPoint = b2Mul(xfB, pc.GetPoint(index));
				normal = b2Mul(xfA.q, pc.localNormal);
				separation = b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
				point = clipPoint;
			}
			break;

		case b2Manifold::e_faceB:
			{
				const auto planePoint = b2Mul(xfB, pc.localPoint);
				const auto clipPoint = b2Mul(xfA, pc.GetPoint(index));
				normal = b2Mul(xfB.q, pc.localNormal);
				separation = b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
				point = clipPoint;
				normal = -normal; // Ensure normal points from A to B
			}
			break;
		}
	}

	b2Vec2 GetNormal() const noexcept { return normal; }
	b2Vec2 GetPoint() const noexcept { return point; }
	b2Float GetSeparation() const noexcept { return separation; }

private:
	b2Vec2 normal;
	b2Vec2 point;
	b2Float separation;
};

// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints()
{
	auto minSeparation = b2_maxFloat;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];

		const auto indexA = pc.bodyA.index;
		const auto localCenterA = pc.bodyA.localCenter;
		const auto invMassA = pc.bodyA.invMass;
		const auto invInertiaA = pc.bodyA.invI;

		const auto indexB = pc.bodyB.index;
		const auto localCenterB = pc.bodyB.localCenter;
		const auto invMassB = pc.bodyB.invMass;
		const auto invInertiaB = pc.bodyB.invI;

		auto cA = m_positions[indexA].c;
		auto aA = m_positions[indexA].a;

		auto cB = m_positions[indexB].c;
		auto aB = m_positions[indexB].a;

		// Solve normal constraints
		const auto pointCount = pc.GetPointCount();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto xfA = b2Displace(cA, localCenterA, b2Rot(aA));
			const auto xfB = b2Displace(cB, localCenterB, b2Rot(aB));
			const auto psm = b2PositionSolverManifold(pc, xfA, xfB, j);
			const auto normal = psm.GetNormal();
			const auto point = psm.GetPoint();
			const auto separation = psm.GetSeparation();

			const auto rA = point - cA;
			const auto rB = point - cB;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			const auto C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, b2Float{0});

			// Compute the effective mass.
			const auto rnA = b2Cross(rA, normal);
			const auto rnB = b2Cross(rB, normal);
			const auto K = invMassA + invMassB + (invInertiaA * b2Square(rnA)) + (invInertiaB * b2Square(rnB));

			// Compute normal impulse
			const auto impulse = (K > b2Float{0})? - C / K : b2Float{0};

			const auto P = impulse * normal;

			cA -= invMassA * P;
			aA -= invInertiaA * b2Cross(rA, P);

			cB += invMassB * P;
			aB += invInertiaB * b2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= (-b2_linearSlop * b2Float(3));
}

// Sequential position solver for position constraints.
bool b2ContactSolver::SolveTOIPositionConstraints(size_type toiIndexA, size_type toiIndexB)
{
	auto minSeparation = b2_maxFloat;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];

		const auto indexA = pc.bodyA.index;
		const auto localCenterA = pc.bodyA.localCenter;

		const auto indexB = pc.bodyB.index;
		const auto localCenterB = pc.bodyB.localCenter;
		
		auto invMassA = b2Float{0};
		auto invInertiaA = b2Float{0};
		if ((indexA == toiIndexA) || (indexA == toiIndexB))
		{
			invMassA = pc.bodyA.invMass;
			invInertiaA = pc.bodyA.invI;
		}

		auto invMassB = b2Float{0};
		auto invInertiaB = b2Float{0};
		if ((indexB == toiIndexA) || (indexB == toiIndexB))
		{
			invMassB = pc.bodyB.invMass;
			invInertiaB = pc.bodyB.invI;
		}

		auto cA = m_positions[indexA].c;
		auto aA = m_positions[indexA].a;
		auto cB = m_positions[indexB].c;
		auto aB = m_positions[indexB].a;

		// Solve normal constraints
		const auto pointCount = pc.GetPointCount();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto xfA = b2Displace(cA, localCenterA, b2Rot(aA));
			const auto xfB = b2Displace(cB, localCenterB, b2Rot(aB));
			const auto psm = b2PositionSolverManifold(pc, xfA, xfB, j);
			const auto normal = psm.GetNormal();
			const auto point = psm.GetPoint();
			const auto separation = psm.GetSeparation();

			const auto rA = point - cA;
			const auto rB = point - cB;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			const auto C = b2Clamp(b2_toiBaugarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, b2Float{0});

			// Compute the effective mass.
			const auto rnA = b2Cross(rA, normal);
			const auto rnB = b2Cross(rB, normal);
			const auto K = invMassA + invMassB + (invInertiaA * b2Square(rnA)) + (invInertiaB * b2Square(rnB));

			// Compute normal impulse
			const auto impulse = (K > b2Float{0}) ? - C / K : b2Float{0};

			const auto P = impulse * normal;

			cA -= invMassA * P;
			aA -= invInertiaA * b2Cross(rA, P);

			cB += invMassB * P;
			aB += invInertiaB * b2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;
		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= (-b2_linearSlop * b2Float(1.5));
}
