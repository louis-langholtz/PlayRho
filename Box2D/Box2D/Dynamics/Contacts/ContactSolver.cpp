/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Contacts/ContactSolver.h>
#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Dynamics/Contacts/PositionSolverManifold.hpp>
#include <Box2D/Dynamics/Contacts/ContactVelocityConstraint.hpp>
#include <Box2D/Dynamics/Contacts/ContactPositionConstraint.hpp>

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

/// Updates the velocity constraint data with the given data.
/// @detail Specifically this:
///   1. Sets the normal to the calculated world manifold normal.
///   2. Sets the velocity constraint point information (short of the impulse data).
///   3. Sets the K value (for the 2-point block solver).
///   4. Checks for redundant velocity constraint point and removes it if found.
/// @param vc Velocity constraint.
/// @param pc Position constraint.
static inline void UpdateVelocityConstraint(ContactVelocityConstraint& vc,
											const ContactPositionConstraint& pc,
											const Position* positions, const Velocity* velocities)
{
	assert(vc.bodyA.GetIndex() >= 0);
	const auto posA = positions[vc.bodyA.GetIndex()];
	const auto velA = velocities[vc.bodyA.GetIndex()];
	
	assert(vc.bodyB.GetIndex() >= 0);
	const auto posB = positions[vc.bodyB.GetIndex()];
	const auto velB = velocities[vc.bodyB.GetIndex()];
	
	const auto totalInvMass = vc.bodyA.GetInvMass() + vc.bodyB.GetInvMass();
	
	const auto worldManifold = [&]() {
		const auto xfA = GetTransformation(posA, pc.bodyA.localCenter);
		const auto xfB = GetTransformation(posB, pc.bodyB.localCenter);
		return GetWorldManifold(pc.manifold, xfA, pc.radiusA, xfB, pc.radiusB);
	}();
	
	vc.normal = worldManifold.GetNormal();
	
	const auto pointCount = vc.GetPointCount();
	for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
	{
		const auto worldPoint = worldManifold.GetPoint(j);
		const auto vcp_rA = worldPoint - posA.c;
		const auto vcp_rB = worldPoint - posB.c;
		auto& vcp = vc.Point(j);
		vcp.rA = vcp_rA;
		vcp.rB = vcp_rB;
		vcp.normalMass = [&]() {
			const auto rnA = Cross(vcp_rA, vc.normal);
			const auto rnB = Cross(vcp_rB, vc.normal);
			const auto kNormal = totalInvMass + (vc.bodyA.GetInvRotI() * Square(rnA)) + (vc.bodyB.GetInvRotI() * Square(rnB));
			return (kNormal > float_t{0})? float_t{1} / kNormal : float_t{0};
		}();
		vcp.tangentMass = [&]() {
			const auto tangent = GetFwdPerpendicular(vc.normal);
			const auto rtA = Cross(vcp_rA, tangent);
			const auto rtB = Cross(vcp_rB, tangent);
			const auto kTangent = totalInvMass + (vc.bodyA.GetInvRotI() * Square(rtA)) + (vc.bodyB.GetInvRotI() * Square(rtB));
			return (kTangent > float_t{0}) ? float_t{1} /  kTangent : float_t{0};
		}();
		vcp.velocityBias = [&]() {
			const auto vn = Dot(GetContactRelVelocity(velA, vcp_rA, velB, vcp_rB), vc.normal);
			return (vn < -VelocityThreshold)? -vc.GetRestitution() * vn: float_t{0};
		}();
	}
	
	// If we have two points, then prepare the block solver.
	if ((pointCount == 2) && g_blockSolve)
	{
		const auto vcp1 = vc.Point(0);
		const auto rn1A = Cross(vcp1.rA, vc.normal);
		const auto rn1B = Cross(vcp1.rB, vc.normal);

		const auto vcp2 = vc.Point(1);
		const auto rn2A = Cross(vcp2.rA, vc.normal);
		const auto rn2B = Cross(vcp2.rB, vc.normal);
		
		const auto k11 = totalInvMass + (vc.bodyA.GetInvRotI() * Square(rn1A)) + (vc.bodyB.GetInvRotI() * Square(rn1B));
		const auto k22 = totalInvMass + (vc.bodyA.GetInvRotI() * Square(rn2A)) + (vc.bodyB.GetInvRotI() * Square(rn2B));
		const auto k12 = totalInvMass + (vc.bodyA.GetInvRotI() * rn1A * rn2A)  + (vc.bodyB.GetInvRotI() * rn1B * rn2B);
		
		// Ensure a reasonable condition number.
		constexpr auto k_maxConditionNumber = BOX2D_MAGIC(float_t(1000));
		if (Square(k11) < (k_maxConditionNumber * (k11 * k22 - Square(k12))))
		{
			// K is safe to invert.
			vc.SetK(Mat22{Vec2{k11, k12}, Vec2{k12, k22}});
		}
		else
		{
			// The constraints are redundant, just use one.
			// TODO_ERIN use deepest?
			vc.RemovePoint();
		}
	}
}
	
/// Solves the velocities and constraint point for the tangent constraint.
/// @detail This updates the tangent impulse on the given velocity constraint point and
///   updates the two given velocity structures.
/// @param vc Contact velocity constraint.
/// @param velA Velocity structure for body A. This is an input and output parameter modified to meet the constraint.
/// @param velB Velocity structure for body B. This is an input and output parameter modified to meet the constraint.
/// @param vcp Velocity constraint point. This is an input and output parameter whose tangent impulse is modified.
static void SolveTangentConstraint(const ContactVelocityConstraint& vc,
								   Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp)
{
	assert(IsValid(velA));
	assert(IsValid(velB));

	const auto tangent = GetFwdPerpendicular(vc.normal);
	
	// Compute tangent force
	const auto vt = Dot(GetContactRelVelocity(velA, vcp.rA, velB, vcp.rB), tangent) - vc.GetTangentSpeed();
	const auto lambda = vcp.tangentMass * (-vt);
	
	// Clamp the accumulated force
	const auto maxImpulse = vc.GetFriction() * vcp.normalImpulse;
	const auto oldImpulse = vcp.tangentImpulse;
	const auto newImpulse = Clamp(vcp.tangentImpulse + lambda, -maxImpulse, maxImpulse);
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Save new impulse
	vcp.tangentImpulse = newImpulse;
	
	// Apply contact impulse
	const auto P = incImpulse * tangent;
	velA.v -= vc.bodyA.GetInvMass() * P;
	velA.w -= vc.bodyA.GetInvRotI() * Cross(vcp.rA, P);
	velB.v += vc.bodyB.GetInvMass() * P;
	velB.w += vc.bodyB.GetInvRotI() * Cross(vcp.rB, P);
}

/// Solves the velocities and constraint point for the normal constraint.
static void SolveNormalConstraint(const ContactVelocityConstraint& vc,
								  Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp)
{
	assert(IsValid(velA));
	assert(IsValid(velB));
	
	// Compute normal impulse
	const auto vn = Dot(GetContactRelVelocity(velA, vcp.rA, velB, vcp.rB), vc.normal);
	const auto lambda = -vcp.normalMass * (vn - vcp.velocityBias);
	
	// Clamp the accumulated impulse
	const auto oldImpulse = vcp.normalImpulse;
	const auto newImpulse = Max(oldImpulse + lambda, float_t{0});
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Save new impulse
	vcp.normalImpulse = newImpulse;
	
	// Apply contact impulse
	const auto P = incImpulse * vc.normal;
	velA.v -= vc.bodyA.GetInvMass() * P;
	velA.w -= vc.bodyA.GetInvRotI() * Cross(vcp.rA, P);
	velB.v += vc.bodyB.GetInvMass() * P;
	velB.w += vc.bodyB.GetInvRotI() * Cross(vcp.rB, P);
}

static inline void BlockSolveUpdate(const ContactVelocityConstraint& vc, const Vec2 oldImpulse, const Vec2 newImpulse,
									Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	assert(IsValid(oldImpulse));
	assert(IsValid(newImpulse));
	assert(IsValid(velA));
	assert(IsValid(velB));
	
	// Get the incremental impulse
	const auto incImpulse = newImpulse - oldImpulse;
	
	// Apply incremental impulse
	const auto P1 = incImpulse.x * vc.normal;
	const auto P2 = incImpulse.y * vc.normal;
	const auto P = P1 + P2;
	velA.v -= vc.bodyA.GetInvMass() * P;
	velA.w -= vc.bodyA.GetInvRotI() * (Cross(vcp1.rA, P1) + Cross(vcp2.rA, P2));
	velB.v += vc.bodyB.GetInvMass() * P;
	velB.w += vc.bodyB.GetInvRotI() * (Cross(vcp1.rB, P1) + Cross(vcp2.rB, P2));
	
	// Save new impulse
	vcp1.normalImpulse = newImpulse.x;
	vcp2.normalImpulse = newImpulse.y;
}

static inline bool BlockSolveNormalCase1(const ContactVelocityConstraint& vc, const Vec2 oldImpulse, const Vec2 b_prime,
										 Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
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
	const auto newImpulse = -Transform(b_prime, vc.GetNormalMass());
	if ((newImpulse.x >= float_t{0}) && (newImpulse.y >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, velA, velB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv1 = (velB.v + Cross(velB.w, vcp1.rB)) - (velA.v + Cross(velA.w, vcp1.rA));
		const auto post_dv2 = (velB.v + Cross(velB.w, vcp2.rB)) - (velA.v + Cross(velA.w, vcp2.rA));
		
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

static inline bool BlockSolveNormalCase2(const ContactVelocityConstraint& vc, const Vec2 oldImpulse, const Vec2 b_prime,
										 Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	//
	// Case 2: vn1 = 0 and x2 = 0
	//
	//   0 = a11 * x1 + a12 * 0 + b1' 
	// vn2 = a21 * x1 + a22 * 0 + b2'
	//
	const auto newImpulse = Vec2{-vcp1.normalMass * b_prime.x, float_t{0}};
	const auto vn2 = vc.GetK().ex.y * newImpulse.x + b_prime.y;
	if ((newImpulse.x >= float_t{0}) && (vn2 >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, velA, velB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv1 = (velB.v + Cross(velB.w, vcp1.rB)) - (velA.v + Cross(velA.w, vcp1.rA));
		
		// Compute normal velocity
		const auto post_vn1 = Dot(post_dv1, vc.normal);
		
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase3(const ContactVelocityConstraint& vc, const Vec2 oldImpulse, const Vec2 b_prime,
										 Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	//
	// Case 3: vn2 = 0 and x1 = 0
	//
	// vn1 = a11 * 0 + a12 * x2 + b1' 
	//   0 = a21 * 0 + a22 * x2 + b2'
	//
	const auto newImpulse = Vec2{float_t{0}, -vcp2.normalMass * b_prime.y};
	const auto vn1 = vc.GetK().ey.x * newImpulse.y + b_prime.x;
	if ((newImpulse.y >= float_t{0}) && (vn1 >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, velA, velB, vcp1, vcp2);
		
#if defined(B2_DEBUG_SOLVER)
		// Postconditions
		const auto post_dv2 = (velB.v + Cross(velB.w, vcp2.rB)) - (velA.v + Cross(velA.w, vcp2.rA));
		
		// Compute normal velocity
		const auto post_vn2 = Dot(post_dv2, vc.normal);

		assert(Abs(post_vn2 - vcp2.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn2 - vcp2.velocityBias) < k_errorTol);
#endif
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase4(const ContactVelocityConstraint& vc, const Vec2 oldImpulse, const Vec2 b_prime,
										 Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
{
	//
	// Case 4: x1 = 0 and x2 = 0
	// 
	// vn1 = b1
	// vn2 = b2;
	const auto newImpulse = Vec2_zero;
	const auto vn1 = b_prime.x;
	const auto vn2 = b_prime.y;
	if ((vn1 >= float_t{0}) && (vn2 >= float_t{0}))
	{
		BlockSolveUpdate(vc, oldImpulse, newImpulse, velA, velB, vcp1, vcp2);
		return true;
	}
	return false;
}

static inline void BlockSolveNormalConstraint(const ContactVelocityConstraint& vc,
											  Velocity& velA, Velocity& velB, VelocityConstraintPoint& vcp1, VelocityConstraintPoint& vcp2)
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

	assert(IsValid(velA));
	assert(IsValid(velB));
	
	const auto oldImpulse = Vec2{vcp1.normalImpulse, vcp2.normalImpulse};
	assert((oldImpulse.x >= float_t{0}) && (oldImpulse.y >= float_t{0}));
	
	const auto b_prime = [=]{
		// Compute normal velocity
		const auto vn1 = Dot(GetContactRelVelocity(velA, vcp1.rA, velB, vcp1.rB), vc.normal);
		const auto vn2 = Dot(GetContactRelVelocity(velA, vcp2.rA, velB, vcp2.rB), vc.normal);
		
		// Compute b
		const auto b = Vec2{vn1 - vcp1.velocityBias, vn2 - vcp2.velocityBias};
		
		// Return b'
		return b - Transform(oldImpulse, vc.GetK());
	}();
	
	
	if (BlockSolveNormalCase1(vc, oldImpulse, b_prime, velA, velB, vcp1, vcp2))
		return;
	if (BlockSolveNormalCase2(vc, oldImpulse, b_prime, velA, velB, vcp1, vcp2))
		return;
	if (BlockSolveNormalCase3(vc, oldImpulse, b_prime, velA, velB, vcp1, vcp2))
		return;
	if (BlockSolveNormalCase4(vc, oldImpulse, b_prime, velA, velB, vcp1, vcp2))
		return;
	
	// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
}

/// Solves the velocity constraint.
/// @detail This updates the tangent and normal impulses of the velocity constraint points of the given velocity
///   constraint and updates the given velocities.
static inline void SolveVelocityConstraint(ContactVelocityConstraint& vc, Velocity& velA, Velocity& velB)
{
	const auto pointCount = vc.GetPointCount();
	assert((pointCount == 1) || (pointCount == 2));

	assert(IsValid(velA));
	assert(IsValid(velB));

	// Solve tangent constraints first, because non-penetration is more important than friction.
	// Solve normal constraints second.

	if (pointCount == 1)
	{
		auto& vcp = vc.Point(0);

		SolveTangentConstraint(vc, velA, velB, vcp);
		SolveNormalConstraint(vc, velA, velB, vcp);
	}
	else // pointCount == 2
	{
		auto& vcp1 = vc.Point(0); ///< Velocity constraint point.
		auto& vcp2 = vc.Point(1); ///< Velocity constraint point.

		SolveTangentConstraint(vc, velA, velB, vcp1);
		SolveTangentConstraint(vc, velA, velB, vcp2);

		if (!g_blockSolve)
		{
			SolveNormalConstraint(vc, velA, velB, vcp1);
			SolveNormalConstraint(vc, velA, velB, vcp2);
		}
		else
		{
			BlockSolveNormalConstraint(vc, velA, velB, vcp1, vcp2);
		}
	}
}

/// Solves position constraint.
/// @detail
/// This updates the two given positions for every point in the contact position constraint
/// and returns the minimum separation value from the position solver manifold for each point.
static float_t Solve(const ContactPositionConstraint& pc,
					 Position& positionA, Position& positionB, float_t baumgarte)
{
	// see http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
	auto minSeparation = MaxFloat;
	
	auto posA = positionA;
	auto posB = positionB;
	
	{
		const auto invMassA = pc.bodyA.invMass;
		const auto invInertiaA = pc.bodyA.invI;
		const auto localCenterA = pc.bodyA.localCenter;
		
		const auto invMassB = pc.bodyB.invMass;
		const auto invInertiaB = pc.bodyB.invI;
		const auto localCenterB = pc.bodyB.localCenter;
		
		//baumgarte += (invMassA == 0 || invMassB == 0) * (float_t(1) - baumgarte);
		// Compute inverse mass total.
		// This must be > 0 unless doing TOI solving and neither bodies were the bodies specified.
		const auto invMassTotal = invMassA + invMassB;
		assert(invMassTotal >= 0);
		
		const auto totalRadius = pc.radiusA + pc.radiusB;
		
		// Solve normal constraints
		const auto pointCount = pc.manifold.GetPointCount();
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			const auto psm = [&]() {
				const auto xfA = GetTransformation(posA, localCenterA);
				const auto xfB = GetTransformation(posB, localCenterB);
				return GetPSM(pc.manifold, totalRadius, xfA, xfB, j);
			}();
			
			// Track max constraint error.
			minSeparation = Min(minSeparation, psm.separation);
			
			if (invMassTotal > float_t{0})
			{
				const auto rA = psm.point - posA.c;
				const auto rB = psm.point - posB.c;
				
				// Compute the effective mass.
				const auto K = [&]() {
					const auto rnA = Cross(rA, psm.normal);
					const auto rnB = Cross(rB, psm.normal);
					return invMassTotal + (invInertiaA * Square(rnA)) + (invInertiaB * Square(rnB));
				}();
				
				// Prevent large corrections and don't push the separation above -LinearSlop.
				//const auto C = Clamp(baumgarte * (psm.separation + LinearSlop * (invMassA != 0 && invMassB != 0)),
				const auto C = Clamp(baumgarte * (psm.separation + LinearSlop),
									 BOX2D_MAGIC(-MaxLinearCorrection), float_t{0});
				//BOX2D_MAGIC(-MaxLinearCorrection), float_t{0});
				
				// Compute normal impulse
				const auto P = psm.normal * -C / K;
				
				posA.c -= invMassA * P;
				posA.a -= invInertiaA * Cross(rA, P);
				posB.c += invMassB * P;
				posB.a += invInertiaB * Cross(rB, P);
			}
		}
	}
	
	positionA = posA;
	positionB = posB;
	
	return minSeparation;
}

void ContactSolver::UpdateVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		UpdateVelocityConstraint(m_velocityConstraints[i], m_positionConstraints[i], m_positions, m_velocities);
	}
}

void ContactSolver::SolveVelocityConstraints()
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		auto& vc = m_velocityConstraints[i];
		SolveVelocityConstraint(vc, m_velocities[vc.bodyA.GetIndex()], m_velocities[vc.bodyB.GetIndex()]);
	}
}

// Sequential solver.
bool ContactSolver::SolvePositionConstraints()
{
	auto minSeparation = MaxFloat;
	
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto& pc = m_positionConstraints[i];
		assert(pc.bodyA.index != pc.bodyB.index);
		const auto separation = Solve(pc, m_positions[pc.bodyA.index], m_positions[pc.bodyB.index], Baumgarte);
		minSeparation = Min(minSeparation, separation);
	}
	
	// Can't expect minSpeparation >= -LinearSlop because we don't push the separation above -LinearSlop.
	//return minSeparation >= MinSeparationThreshold;
	return minSeparation >= -LinearSlop * 2;
}
	
bool ContactSolver::SolveTOIPositionConstraints(island_count_t indexA, island_count_t indexB)
{
	auto minSeparation = MaxFloat;

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		auto pc = m_positionConstraints[i];

		assert(pc.bodyA.index != pc.bodyB.index); // Confirm ContactManager::Add() did its job.
		
		// Modify local copy of the position constraint to only let position
		// of bodies identified by either given indexes be changed.

		if ((pc.bodyA.index != indexA) && (pc.bodyA.index != indexB))
		{
			pc.bodyA.invMass = float_t{0};
			pc.bodyA.invI = float_t{0};
		}
		if ((pc.bodyB.index != indexA) && (pc.bodyB.index != indexB))
		{
			pc.bodyB.invMass = float_t{0};
			pc.bodyB.invI = float_t{0};
		}

		const auto separation = Solve(pc, m_positions[pc.bodyA.index], m_positions[pc.bodyB.index], ToiBaumgarte);
		minSeparation = Min(minSeparation, separation);
	}

	// Can't expect minSpeparation >= -LinearSlop because we don't push the separation above -LinearSlop.
	//return minSeparation >= MinToiSeparation;
	return minSeparation >= -LinearSlop * float_t(3) / float_t(2); // 1.5;
}
	
} // namespace box2d
