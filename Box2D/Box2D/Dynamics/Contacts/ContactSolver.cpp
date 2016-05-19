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

#include <Box2D/Dynamics/Contacts/ContactSolver.h>

#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/World.h>

namespace box2d {

#if !defined(NDEBUG)
// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
// #define B2_DEBUG_SOLVER 1
#endif

#if defined(B2_DEBUG_SOLVER)
static constexpr auto k_errorTol = float_t(2e-3); ///< error tolerance
static constexpr auto k_majorErrorTol = float_t(1e-2); ///< error tolerance
#endif

bool g_blockSolve = true;

struct ContactPositionConstraintBodyData
{
	using index_t = size_t;

	index_t index; ///< Index within island of the associated body.
	float_t invMass; ///< Inverse mass of associated body.
	Vec2 localCenter; ///< Local center of the associated body's sweep.
	float_t invI; ///< Inverse rotational inertia about the center of mass of the associated body.
};

class ContactPositionConstraint
{
public:
	using size_type = std::remove_cv<decltype(MaxManifoldPoints)>::type;

	Vec2 localNormal;
	Vec2 localPoint;

	ContactPositionConstraintBodyData bodyA;
	ContactPositionConstraintBodyData bodyB;

	Manifold::Type type = Manifold::e_unset;

	float_t radiusA; ///< "Radius" distance from the associated shape of fixture A.
	float_t radiusB; ///< "Radius" distance from the associated shape of fixture B.

	size_type GetPointCount() const noexcept { return pointCount; }

	Vec2 GetPoint(size_type index) const
	{
		assert((0 <= index) && (index < pointCount));
		return localPoints[index];
	}

	void ClearPoints() noexcept
	{
		pointCount = 0;
	}

	void AddPoint(const Vec2& val)
	{
		assert(pointCount < MaxManifoldPoints);
		localPoints[pointCount] = val;
		++pointCount;
	}

private:
	size_type pointCount = 0;
	Vec2 localPoints[MaxManifoldPoints];
};

void ContactSolver::Assign(ContactVelocityConstraint& var, const Contact& val)
{
	var.friction = val.m_friction;
	var.restitution = val.m_restitution;
	var.tangentSpeed = val.m_tangentSpeed;
}

void ContactSolver::Assign(ContactPositionConstraintBodyData& var, const Body& val)
{
	assert(val.IsValidIslandIndex());
	var.index = val.m_islandIndex;
	var.invMass = val.m_invMass;
	var.invI = val.m_invI;
	var.localCenter = val.m_sweep.localCenter;
}

void ContactSolver::Assign(ContactVelocityConstraintBodyData& var, const Body& val)
{
	assert(val.IsValidIslandIndex());
	var.index = val.m_islandIndex;
	var.invMass = val.m_invMass;
	var.invI = val.m_invI;
}

ContactSolver::ContactSolver(ContactSolverDef* def) :
	m_step(def->step),
	m_positions(def->positions),
	m_velocities(def->velocities),
	m_allocator(def->allocator),
	m_contacts(def->contacts),
	m_count(def->count),
	m_positionConstraints(static_cast<ContactPositionConstraint*>(m_allocator->Allocate(m_count * sizeof(ContactPositionConstraint)))),
	m_velocityConstraints(static_cast<ContactVelocityConstraint*>(m_allocator->Allocate(m_count * sizeof(ContactVelocityConstraint))))
{
	// Initialize position independent portions of the constraints.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& contact = *m_contacts[i];

		const auto fixtureA = contact.m_fixtureA;
		const auto fixtureB = contact.m_fixtureB;
		const auto shapeA = fixtureA->GetShape();
		const auto shapeB = fixtureB->GetShape();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		const auto manifold = contact.GetManifold();

		auto& vc = m_velocityConstraints[i];
		Assign(vc, contact);
		Assign(vc.bodyA, *bodyA);
		Assign(vc.bodyB, *bodyB);
		vc.contactIndex = i;
		vc.K = Mat22_zero;
		vc.normalMass = Mat22_zero;
		vc.ClearPoints();
		
		auto& pc = m_positionConstraints[i];
		Assign(pc.bodyA, *bodyA);
		Assign(pc.bodyB, *bodyB);
		pc.localNormal = manifold->GetLocalNormal();
		pc.localPoint = manifold->GetLocalPoint();
		pc.radiusA = shapeA->GetRadius();
		pc.radiusB = shapeB->GetRadius();
		pc.type = manifold->GetType();
		pc.ClearPoints();

		const auto pointCount = manifold->GetPointCount();
		assert(pointCount > 0);
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto& mp = manifold->GetPoint(j); ///< Manifold point.
			pc.AddPoint(mp.localPoint);
	
			VelocityConstraintPoint vcp;
			vcp.normalImpulse = m_step.warmStarting? m_step.dtRatio * mp.normalImpulse: float_t{0};
			vcp.tangentImpulse = m_step.warmStarting? m_step.dtRatio * mp.tangentImpulse: float_t{0};
			vcp.rA = Vec2_zero;
			vcp.rB = Vec2_zero;
			vcp.normalMass = float_t{0};
			vcp.tangentMass = float_t{0};
			vcp.velocityBias = float_t{0};
			vc.AddPoint(vcp);
		}
	}
}

ContactSolver::~ContactSolver()
{
	m_allocator->Free(m_velocityConstraints);
	m_allocator->Free(m_positionConstraints);
}

static inline void Initialize(VelocityConstraintPoint& vcp,
							  const ContactVelocityConstraint& vc, Vec2 worldPoint,
							  Position posA, Velocity velA, Position posB, Velocity velB)
{
	const auto vcp_rA = worldPoint - posA.c;
	const auto vcp_rB = worldPoint - posB.c;
	
	const auto rnA = Cross(vcp_rA, vc.normal);
	const auto rnB = Cross(vcp_rB, vc.normal);
	
	const auto kNormal = vc.bodyA.invMass + vc.bodyB.invMass + (vc.bodyA.invI * Square(rnA)) + (vc.bodyB.invI * Square(rnB));
	
	const auto tangent = Cross(vc.normal, float_t(1));
	
	const auto rtA = Cross(vcp_rA, tangent);
	const auto rtB = Cross(vcp_rB, tangent);
	
	const auto kTangent = vc.bodyA.invMass + vc.bodyB.invMass + (vc.bodyA.invI * Square(rtA)) + (vc.bodyB.invI * Square(rtB));
	
	// Relative velocity at contact
	const auto dv = (velB.v + Cross(velB.w, vcp_rB)) - (velA.v + Cross(velA.w, vcp_rA));
	const auto vRel = Dot(dv, vc.normal);
	
	vcp.rA = vcp_rA;
	vcp.rB = vcp_rB;
	vcp.normalMass = (kNormal > float_t{0})? float_t(1) / kNormal : float_t{0};
	vcp.tangentMass = (kTangent > float_t{0}) ? float_t(1) /  kTangent : float_t{0};
	vcp.velocityBias = (vRel < -VelocityThreshold)? -vc.restitution * vRel: float_t{0};
	
	// The following fields are assumed to be set already (by ContactSolver constructor).
	// vcp.normalImpulse
	// vcp.tangentImpulse
}

void ContactSolver::InitializeVelocityConstraint(ContactVelocityConstraint& vc, const ContactPositionConstraint& pc)
{
	const auto manifold = m_contacts[vc.contactIndex]->GetManifold();
	
	assert(vc.bodyA.index >= 0);
	const auto posA = m_positions[vc.bodyA.index];
	const auto velA = m_velocities[vc.bodyA.index];
	
	assert(vc.bodyB.index >= 0);
	const auto posB = m_positions[vc.bodyB.index];
	const auto velB = m_velocities[vc.bodyB.index];
	
	assert(manifold->GetPointCount() > 0);
	
	const auto xfA = GetTransform(posA, pc.bodyA.localCenter);
	const auto xfB = GetTransform(posB, pc.bodyB.localCenter);
	const auto worldManifold = WorldManifold{*manifold, xfA, pc.radiusA, xfB, pc.radiusB};
	
	vc.normal = worldManifold.GetNormal();
	
	const auto pointCount = vc.GetPointCount();
	for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
	{
		const auto worldPoint = worldManifold.GetPoint(j);
		auto& vcp = vc.GetPoint(j); ///< Velocity constraint point.
		Initialize(vcp, vc, worldPoint, posA, velA, posB, velB);	
	}
	
	// If we have two points, then prepare the block solver.
	if ((pointCount == 2) && g_blockSolve)
	{
		const auto vcp1 = vc.GetPoint(0);
		const auto vcp2 = vc.GetPoint(1);
		
		const auto rn1A = Cross(vcp1.rA, vc.normal);
		const auto rn1B = Cross(vcp1.rB, vc.normal);
		const auto rn2A = Cross(vcp2.rA, vc.normal);
		const auto rn2B = Cross(vcp2.rB, vc.normal);
		
		const auto k11 = vc.bodyA.invMass + vc.bodyB.invMass + (vc.bodyA.invI * Square(rn1A)) + (vc.bodyB.invI * Square(rn1B));
		const auto k22 = vc.bodyA.invMass + vc.bodyB.invMass + (vc.bodyA.invI * Square(rn2A)) + (vc.bodyB.invI * Square(rn2B));
		const auto k12 = vc.bodyA.invMass + vc.bodyB.invMass + (vc.bodyA.invI * rn1A * rn2A) + (vc.bodyB.invI * rn1B * rn2B);
		
		// Ensure a reasonable condition number.
		constexpr auto k_maxConditionNumber = float_t(1000);
		if (Square(k11) < (k_maxConditionNumber * (k11 * k22 - Square(k12))))
		{
			// K is safe to invert.
			vc.K = Mat22{Vec2{k11, k12}, Vec2{k12, k22}};
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

// Initialize position dependent portions of the velocity constraints.
void ContactSolver::InitializeVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		InitializeVelocityConstraint(m_velocityConstraints[i], m_positionConstraints[i]);
	}
}

static inline void WarmStart(const ContactVelocityConstraint& vc, Velocity& bodyA, Velocity& bodyB)
{
	const auto tangent = Cross(vc.normal, float_t(1));
	const auto pointCount = vc.GetPointCount();	
	for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
	{
		const auto vcp = vc.GetPoint(j); ///< Velocity constraint point.
		const auto P = vcp.normalImpulse * vc.normal + vcp.tangentImpulse * tangent;
		bodyA.v -= vc.bodyA.invMass * P;
		bodyA.w -= vc.bodyA.invI * Cross(vcp.rA, P);
		bodyB.v += vc.bodyB.invMass * P;
		bodyB.w += vc.bodyB.invI * Cross(vcp.rB, P);
	}
}

void ContactSolver::WarmStart()
{
	// Warm start.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& vc = m_velocityConstraints[i];
		::box2d::WarmStart(vc, m_velocities[vc.bodyA.index], m_velocities[vc.bodyB.index]);
	}
}

static inline void SolveTangentConstraint(const ContactVelocityConstraint& vc, Vec2 tangent,
										 Velocity& bodyA, Velocity& bodyB,
										 VelocityConstraintPoint& vcp)
{
	// Relative velocity at contact
	const auto dv = (bodyB.v + Cross(bodyB.w, vcp.rB)) - (bodyA.v + Cross(bodyA.w, vcp.rA));
	
	// Compute tangent force
	const auto vt = Dot(dv, tangent) - vc.tangentSpeed;
	const auto lambda = vcp.tangentMass * (-vt);
	
	// Clamp the accumulated force
	const auto maxFriction = vc.friction * vcp.normalImpulse;
	const auto oldImpulse = vcp.tangentImpulse;
	const auto newImpulse = Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Save new impulse
	vcp.tangentImpulse = newImpulse;
	
	// Apply contact impulse
	const auto P = incImpulse * tangent;
	bodyA.v -= vc.bodyA.invMass * P;
	bodyA.w -= vc.bodyA.invI * Cross(vcp.rA, P);
	bodyB.v += vc.bodyB.invMass * P;
	bodyB.w += vc.bodyB.invI * Cross(vcp.rB, P);
}

static inline void SolveNormalConstraint(const ContactVelocityConstraint& vc,
										 Velocity& bodyA, Velocity& bodyB,
										 VelocityConstraintPoint& vcp)
{
	// Relative velocity at contact
	const auto dv = (bodyB.v + Cross(bodyB.w, vcp.rB)) - (bodyA.v + Cross(bodyA.w, vcp.rA));
	
	// Compute normal impulse
	const auto vn = Dot(dv, vc.normal);
	const auto lambda = -vcp.normalMass * (vn - vcp.velocityBias);
	
	// Clamp the accumulated impulse
	const auto oldImpulse = vcp.normalImpulse;
	const auto newImpulse = Max(vcp.normalImpulse + lambda, float_t{0});
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Save new impulse
	vcp.normalImpulse = newImpulse;
	
	// Apply contact impulse
	const auto P = incImpulse * vc.normal;
	bodyA.v -= vc.bodyA.invMass * P;
	bodyA.w -= vc.bodyA.invI * Cross(vcp.rA, P);
	bodyB.v += vc.bodyB.invMass * P;
	bodyB.w += vc.bodyB.invI * Cross(vcp.rB, P);
}

static inline void BlockSolveUpdate(const ContactVelocityConstraint& vc,
									Vec2 oldImpulse, Vec2 newImpulse,
									Velocity& bodyA, Velocity& bodyB,
									VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	// Get the incremental impulse
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Apply incremental impulse
	const auto P1 = incImpulse.x * vc.normal;
	const auto P2 = incImpulse.y * vc.normal;
	const auto P = P1 + P2;
	bodyA.v -= vc.bodyA.invMass * P;
	bodyA.w -= vc.bodyA.invI * (Cross(vcp1.rA, P1) + Cross(vcp2.rA, P2));
	bodyB.v += vc.bodyB.invMass * P;
	bodyB.w += vc.bodyB.invI * (Cross(vcp1.rB, P1) + Cross(vcp2.rB, P2));
	
	// Save new impulse
	vcp1.normalImpulse = newImpulse.x;
	vcp2.normalImpulse = newImpulse.y;
}

static inline bool BlockSolveNormalCase1(const ContactVelocityConstraint& vc,
										 Vec2 oldImpulse, Vec2 b_prime,
										 Velocity& bodyA, Velocity& bodyB,
										 VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
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
	const auto newImpulse = -Mul(vc.normalMass, b_prime);
	if ((newImpulse.x >= float_t{0}) && (newImpulse.y >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv1 = (bodyB.v + Cross(bodyB.w, vcp1.rB)) - (bodyA.v + Cross(bodyA.w, vcp1.rA));
		const auto post_dv2 = (bodyB.v + Cross(bodyB.w, vcp2.rB)) - (bodyA.v + Cross(bodyA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto post_vn1 = Dot(post_dv1, vc.normal);
		const auto post_vn2 = Dot(post_dv2, vc.normal);
		
		
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn2 - vcp2.velocityBias) < k_majorErrorTol);

		assert(Abs(post_vn1 - vcp1.velocityBias) < k_errorTol);
		assert(Abs(post_vn2 - vcp2.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase2(const ContactVelocityConstraint& vc,
										 Vec2 oldImpulse, Vec2 b_prime,
										 Velocity& bodyA, Velocity& bodyB,
										 VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	//
	// Case 2: vn1 = 0 and x2 = 0
	//
	//   0 = a11 * x1 + a12 * 0 + b1' 
	// vn2 = a21 * x1 + a22 * 0 + b2'
	//
	const auto newImpulse = Vec2{-vcp1.normalMass * b_prime.x, float_t{0}};
	const auto vn2 = vc.K.ex.y * newImpulse.x + b_prime.y;
	if ((newImpulse.x >= float_t{0}) && (vn2 >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv1 = (bodyB.v + Cross(bodyB.w, vcp1.rB)) - (bodyA.v + Cross(bodyA.w, vcp1.rA));
		
		// Compute normal velocity
		const auto post_vn1 = Dot(post_dv1, vc.normal);
		
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase3(const ContactVelocityConstraint& vc,
										 Vec2 oldImpulse, Vec2 b_prime,
										 Velocity& bodyA, Velocity& bodyB,
										 VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	//
	// Case 3: vn2 = 0 and x1 = 0
	//
	// vn1 = a11 * 0 + a12 * x2 + b1' 
	//   0 = a21 * 0 + a22 * x2 + b2'
	//
	const auto newImpulse = Vec2{float_t{0}, -vcp2.normalMass * b_prime.y};
	const auto vn1 = vc.K.ey.x * newImpulse.y + b_prime.x;
	if ((newImpulse.y >= float_t{0}) && (vn1 >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv2 = (bodyB.v + Cross(bodyB.w, vcp2.rB)) - (bodyA.v + Cross(bodyA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto post_vn2 = Dot(post_dv2, vc.normal);

		assert(Abs(post_vn2 - vcp2.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn2 - vcp2.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase4(const ContactVelocityConstraint& vc,
										 Vec2 oldImpulse, Vec2 b_prime,
										 Velocity& bodyA, Velocity& bodyB,
										 VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	//
	// Case 4: x1 = 0 and x2 = 0
	// 
	// vn1 = b1
	// vn2 = b2;
	const auto newImpulse = Vec2{float_t{0}, float_t{0}};
	const auto vn1 = b_prime.x;
	const auto vn2 = b_prime.y;
	if ((vn1 >= float_t{0}) && (vn2 >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, bodyA, bodyB, vcp1, vcp2);
		return true;
	}
	return false;
}

static inline void BlockSolveNormalConstraint(const ContactVelocityConstraint& vc,
											  Velocity& bodyA, Velocity& bodyB,
											  VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
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

	const auto oldImpulse = Vec2{vcp1.normalImpulse, vcp2.normalImpulse};
	assert((oldImpulse.x >= float_t{0}) && (oldImpulse.y >= float_t{0}));
	
	const auto b_prime = [=]{
		// Relative velocity at contact
		const auto dv1 = (bodyB.v + Cross(bodyB.w, vcp1.rB)) - (bodyA.v + Cross(bodyA.w, vcp1.rA));
		const auto dv2 = (bodyB.v + Cross(bodyB.w, vcp2.rB)) - (bodyA.v + Cross(bodyA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto normal_vn1 = Dot(dv1, vc.normal);
		const auto normal_vn2 = Dot(dv2, vc.normal);
		
		// Compute b
		const auto b = Vec2{normal_vn1 - vcp1.velocityBias, normal_vn2 - vcp2.velocityBias};
		
		// Return b'
		return b - Mul(vc.K, oldImpulse);
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

static inline void SolveVelocityConstraint(ContactVelocityConstraint& vc, Velocity& bodyA, Velocity& bodyB)
{
	const auto pointCount = vc.GetPointCount();
	assert((pointCount == 1) || (pointCount == 2));

	// Solve tangent constraints first, because non-penetration is more important than friction.
	// Solve normal constraints second.

	if (pointCount == 1)
	{
		auto& vcp = vc.GetPoint(0);

		{
			const auto tangent = Cross(vc.normal, float_t(1));
			SolveTangentConstraint(vc, tangent, bodyA, bodyB, vcp);
		}

		SolveNormalConstraint(vc, bodyA, bodyB, vcp);
	}
	else // pointCount == 2
	{
		auto& vcp1 = vc.GetPoint(0); ///< Velocity constraint point.
		auto& vcp2 = vc.GetPoint(1); ///< Velocity constraint point.

		{
			const auto tangent = Cross(vc.normal, float_t(1));
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

void ContactSolver::SolveVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		auto& vc = m_velocityConstraints[i];
		SolveVelocityConstraint(vc, m_velocities[vc.bodyA.index], m_velocities[vc.bodyB.index]);
	}
}

static void AssignImpulses(ManifoldPoint& var, const VelocityConstraintPoint& val)
{
	var.normalImpulse = val.normalImpulse;
	var.tangentImpulse = val.tangentImpulse;
}

void ContactSolver::StoreImpulses()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& vc = m_velocityConstraints[i];
		auto& manifold = *(m_contacts[vc.contactIndex]->GetManifold());

		const auto point_count = vc.GetPointCount();
		for (auto j = decltype(point_count){0}; j < point_count; ++j)
			AssignImpulses(manifold.GetPoint(j), vc.GetPoint(j));
	}
}

class PositionSolverManifold
{
public:
	using index_t = std::remove_cv<decltype(MaxManifoldPoints)>::type;

	PositionSolverManifold() = delete;

	PositionSolverManifold(const ContactPositionConstraint& pc,
							 const Transform& xfA, const Transform& xfB, index_t index)
	{
		assert(pc.GetPointCount() > 0);
		assert(pc.type != Manifold::e_unset);

		// Note for valid manifold types:
		//   Sum the radius values and subtract this sum to reduce FP losses in cases where the radius
		//   values would otherwise be insignificant compared to the values being subtracted from.
		switch (pc.type)
		{
		case Manifold::e_unset:
			break;

		case Manifold::e_circles:
			{
				assert(index == 0);
				const auto pointA = Mul(xfA, pc.localPoint);
				const auto pointB = Mul(xfB, pc.GetPoint(0));
				const auto delta = pointB - pointA;
				normal = Normalize(delta);
				point = (pointA + pointB) / float_t(2);
				const auto totalRadius = pc.radiusA + pc.radiusB;
				separation = Dot(delta, normal) - totalRadius;
			}
			break;

		case Manifold::e_faceA:
			{
				const auto planePoint = Mul(xfA, pc.localPoint);
				const auto clipPoint = Mul(xfB, pc.GetPoint(index));
				normal = Mul(xfA.q, pc.localNormal);
				const auto totalRadius = pc.radiusA + pc.radiusB;
				separation = Dot(clipPoint - planePoint, normal) - totalRadius;
				point = clipPoint;
			}
			break;

		case Manifold::e_faceB:
			{
				const auto planePoint = Mul(xfB, pc.localPoint);
				const auto clipPoint = Mul(xfA, pc.GetPoint(index));
				normal = Mul(xfB.q, pc.localNormal);
				const auto totalRadius = pc.radiusA + pc.radiusB;
				separation = Dot(clipPoint - planePoint, normal) - totalRadius;
				point = clipPoint;
				normal = -normal; // Ensure normal points from A to B
			}
			break;
		}
	}

	Vec2 GetNormal() const noexcept { return normal; }
	Vec2 GetPoint() const noexcept { return point; }

	/// Gets the "separation" between the two relavent points of the contact position constraint.
	/// @return separation value.
	float_t GetSeparation() const noexcept { return separation; }

private:
	Vec2 normal;
	Vec2 point;
	float_t separation;
};

static inline float_t SolvePositionConstraint(const ContactPositionConstraint& pc,
											  Position& posA, Position& posB)
{
	auto minSeparation = MaxFloat;

	// Solve normal constraints
	const auto pointCount = pc.GetPointCount();
	for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
	{
		const auto xfA = GetTransform(posA, pc.bodyA.localCenter);
		const auto xfB = GetTransform(posB, pc.bodyB.localCenter);
		const auto psm = PositionSolverManifold(pc, xfA, xfB, j);
		const auto normal = psm.GetNormal();
		const auto point = psm.GetPoint();
		const auto separation = psm.GetSeparation();
		
		const auto rA = point - posA.c;
		const auto rB = point - posB.c;
		
		// Track max constraint error.
		minSeparation = Min(minSeparation, separation);
		
		// Prevent large corrections and allow slop.
		const auto C = Clamp(Baumgarte * (separation + LinearSlop), -MaxLinearCorrection, float_t{0});
		
		// Compute the effective mass.
		const auto rnA = Cross(rA, normal);
		const auto rnB = Cross(rB, normal);
		const auto K = pc.bodyA.invMass + pc.bodyB.invMass + (pc.bodyA.invI * Square(rnA)) + (pc.bodyB.invI * Square(rnB));
		
		// Compute normal impulse
		const auto impulse = (K > float_t{0})? - C / K : float_t{0};
		
		const auto P = impulse * normal;
		
		posA.c -= pc.bodyA.invMass * P;
		posA.a -= pc.bodyA.invI * Cross(rA, P);
		posB.c += pc.bodyB.invMass * P;
		posB.a += pc.bodyB.invI * Cross(rB, P);
	}
	
	return minSeparation;
}

// Sequential solver.
bool ContactSolver::SolvePositionConstraints()
{
	auto minSeparation = MaxFloat;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];
		minSeparation = Min(minSeparation,
							  SolvePositionConstraint(pc, m_positions[pc.bodyA.index], m_positions[pc.bodyB.index]));
	}

	// Can't expect minSpeparation >= -LinearSlop because we don't push the separation above -LinearSlop.
	return minSeparation >= MinSeparationThreshold;
}

static inline float_t SolveTOIPositionConstraint(const ContactPositionConstraint& pc,
												 ContactSolver::size_type indexA, ContactSolver::size_type indexB,
												 Position& posA, Position& posB)
{
	auto minSeparation = MaxFloat;
	
	const auto isA = (indexA == pc.bodyA.index) || (indexB == pc.bodyA.index);
	const auto invMassA = isA? pc.bodyA.invMass: float_t{0};
	const auto invInertiaA = isA? pc.bodyA.invI: float_t{0};
	
	const auto isB = (indexA == pc.bodyB.index) || (indexB == pc.bodyB.index);
	const auto invMassB = isB? pc.bodyB.invMass: float_t{0};
	const auto invInertiaB = isB? pc.bodyB.invI: float_t{0};

	// Solve normal constraints
	const auto pointCount = pc.GetPointCount();
	for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
	{
		const auto xfA = GetTransform(posA, pc.bodyA.localCenter);
		const auto xfB = GetTransform(posB, pc.bodyB.localCenter);
		const auto psm = PositionSolverManifold(pc, xfA, xfB, j);
		const auto normal = psm.GetNormal();
		const auto point = psm.GetPoint();
		const auto separation = psm.GetSeparation();
		
		const auto rA = point - posA.c;
		const auto rB = point - posB.c;
		
		// Track max constraint error.
		minSeparation = Min(minSeparation, separation);
		
		// Prevent large corrections and allow slop.
		const auto C = Clamp(ToiBaugarte * (separation + LinearSlop), -MaxLinearCorrection, float_t{0});
		
		// Compute the effective mass.
		const auto rnA = Cross(rA, normal);
		const auto rnB = Cross(rB, normal);
		const auto K = invMassA + invMassB + (invInertiaA * Square(rnA)) + (invInertiaB * Square(rnB));
		
		// Compute normal impulse
		const auto impulse = (K > float_t{0}) ? - C / K : float_t{0};
		
		const auto P = impulse * normal;

		posA.c -= invMassA * P;
		posA.a -= invInertiaA * Cross(rA, P);
		posB.c += invMassB * P;
		posB.a += invInertiaB * Cross(rB, P);
	}
	
	return minSeparation;
}

bool ContactSolver::SolveTOIPositionConstraints(size_type indexA, size_type indexB)
{
	auto minSeparation = MaxFloat;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];
		const auto s = SolveTOIPositionConstraint(pc, indexA, indexB, m_positions[pc.bodyA.index], m_positions[pc.bodyB.index]);
		minSeparation = Min(minSeparation, s);
	}

	// Can't expect minSpeparation >= -LinearSlop because we don't push the separation above -LinearSlop.
	return minSeparation >= MinToiSeparation;
}
	
} // namespace box2d
