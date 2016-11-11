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
#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>
#include <Box2D/Dynamics/Contacts/PositionConstraint.hpp>

using namespace box2d;

#if !defined(NDEBUG)
// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
//#define B2_DEBUG_SOLVER 1
#endif

#if defined(B2_DEBUG_SOLVER)
static constexpr auto k_errorTol = float_t(2e-3); ///< error tolerance
static constexpr auto k_majorErrorTol = float_t(1e-2); ///< error tolerance
#endif

struct VelocityPair
{
	Velocity vel_a;
	Velocity vel_b;
};

/// Solves the tangential portion of the velocity constraint.
/// @detail This updates the tangent impulses on the velocity constraint points and
///   updates the two given velocity structures.
/// @param vc Contact velocity constraint.
/// @param velA Velocity structure for body A. This is an input and output parameter modified to meet the constraint.
/// @param velB Velocity structure for body B. This is an input and output parameter modified to meet the constraint.
static inline void SolveTangentConstraint(VelocityConstraint& vc, Velocity& velA, Velocity& velB)
{
	assert(IsValid(velA));
	assert(IsValid(velB));

	const auto tangent = GetTangent(vc);
	if (IsValid(tangent))
	{
		const auto count = vc.GetPointCount();
		assert((count == 1) || (count == 2));
		
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto rA = GetPointRelPosA(vc, i);
			const auto rB = GetPointRelPosB(vc, i);

			// Compute tangent force
			const auto lambda = GetTangentMassAtPoint(vc, i) * (vc.GetTangentSpeed() - Dot(GetContactRelVelocity(velA, rA, velB, rB), tangent));
			
			// Clamp the accumulated force
			const auto maxImpulse = vc.GetFriction() * GetNormalImpulseAtPoint(vc, i);
			const auto oldImpulse = GetTangentImpulseAtPoint(vc, i);
			const auto newImpulse = Clamp(GetTangentImpulseAtPoint(vc, i) + lambda, -maxImpulse, maxImpulse);
			const auto incImpulse = newImpulse - oldImpulse;
			
			// Save new impulse
			SetTangentImpulseAtPoint(vc, i, newImpulse);
			
			// Apply contact impulse
			const auto P = incImpulse * tangent;
			velA -= Velocity{vc.bodyA.GetInvMass() * P, 1_rad * vc.bodyA.GetInvRotI() * Cross(rA, P)};
			velB += Velocity{vc.bodyB.GetInvMass() * P, 1_rad * vc.bodyB.GetInvRotI() * Cross(rB, P)};
		}
	}
}

static inline void SeqSolveNormalConstraint(VelocityConstraint& vc, Velocity& velA, Velocity& velB)
{
	assert(IsValid(velA));
	assert(IsValid(velB));
	
	const auto normal = GetNormal(vc);
	if (IsValid(normal))
	{
		const auto count = vc.GetPointCount();	
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto rA = GetPointRelPosA(vc, i);
			const auto rB = GetPointRelPosB(vc, i);
			
			// Compute normal impulse
			const auto lambda = [&](){
				const auto dv = GetContactRelVelocity(velA, rA, velB, rB);
				const auto vn = Dot(dv, normal);
				return GetNormalMassAtPoint(vc, i) * (vn - GetVelocityBiasAtPoint(vc, i));
			}();
			
			// Clamp the accumulated impulse
			const auto oldImpulse = GetNormalImpulseAtPoint(vc, i);
			const auto newImpulse = Max(oldImpulse - lambda, float_t{0});
			const auto incImpulse = newImpulse - oldImpulse;
			
			// Save new impulse
			SetNormalImpulseAtPoint(vc, i, newImpulse);
			
			// Apply contact impulse
			const auto P = incImpulse * normal;
			velA -= Velocity{vc.bodyA.GetInvMass() * P, 1_rad * vc.bodyA.GetInvRotI() * Cross(rA, P)};
			velB += Velocity{vc.bodyB.GetInvMass() * P, 1_rad * vc.bodyB.GetInvRotI() * Cross(rB, P)};
		}
	}
}

static inline VelocityPair ApplyImpulses(const VelocityConstraint& vc, const Vec2 impulses)
{
	assert(IsValid(impulses));

	// Apply incremental impulse
	const auto normal = GetNormal(vc);
	const auto P0 = impulses[0] * normal;
	const auto P1 = impulses[1] * normal;
	const auto P = P0 + P1;
	return VelocityPair{
		-Velocity{
			vc.bodyA.GetInvMass() * P,
			1_rad * vc.bodyA.GetInvRotI() * (Cross(GetPointRelPosA(vc, 0), P0) + Cross(GetPointRelPosA(vc, 1), P1))
		},
		+Velocity{
			vc.bodyB.GetInvMass() * P,
			1_rad * vc.bodyB.GetInvRotI() * (Cross(GetPointRelPosB(vc, 0), P0) + Cross(GetPointRelPosB(vc, 1), P1))
		}
	};
}

static inline void BlockSolveUpdate(VelocityConstraint& vc, Velocity& velA, Velocity& velB,
									const Vec2 newImpulses)
{
	assert(IsValid(velA));
	assert(IsValid(velB));
	
	const auto delta_v = ApplyImpulses(vc, newImpulses - GetNormalImpulses(vc));
	velA += delta_v.vel_a;
	velB += delta_v.vel_b;
	SetNormalImpulses(vc, newImpulses);
}

static inline bool BlockSolveNormalCase1(VelocityConstraint& vc, Velocity& velA, Velocity& velB,
										 const Vec2 b_prime)
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
	const auto normalMass = vc.GetNormalMass();
	assert(IsValid(normalMass));

	const auto newImpulses = -Transform(b_prime, normalMass);
	if ((newImpulses[0] >= float_t{0}) && (newImpulses[1] >= float_t{0}))
	{
		BlockSolveUpdate(vc, velA, velB, newImpulses);
		
#if defined(B2_DEBUG_SOLVER)
		auto& vcp1 = vc.PointAt(0);
		auto& vcp2 = vc.PointAt(1);

		// Postconditions
		const auto post_dv1 = (velB.v + (velB.w * GetRevPerpendicular(vcp1.rB))) - (velA.v + (velA.w * GetRevPerpendicular(vcp1.rA)));
		const auto post_dv2 = (velB.v + (velB.w * GetRevPerpendicular(vcp2.rB))) - (velA.v + (velA.w * GetRevPerpendicular(vcp2.rA)));
		
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

static inline bool BlockSolveNormalCase2(VelocityConstraint& vc, Velocity& velA, Velocity& velB,
										 const Vec2 b_prime)
{
	//
	// Case 2: vn1 = 0 and x2 = 0
	//
	//   0 = a11 * x1 + a12 * 0 + b1' 
	// vn2 = a21 * x1 + a22 * 0 + b2'
	//
	const auto newImpulse = Vec2{-GetNormalMassAtPoint(vc, 0) * b_prime.x, float_t{0}};
	const auto K = vc.GetK();
	assert(IsValid(K));
	const auto vn2 = K.ex.y * newImpulse.x + b_prime.y;
	if ((newImpulse.x >= float_t{0}) && (vn2 >= float_t{0}))
	{
		BlockSolveUpdate(vc, velA, velB, newImpulse);
		
#if defined(B2_DEBUG_SOLVER)
		auto& vcp1 = vc.PointAt(0);
	
		// Postconditions
		const auto post_dv1 = (velB.v + (velB.w * GetRevPerpendicular(vcp1.rB))) - (velA.v + (velA.w * GetRevPerpendicular(vcp1.rA)));
		
		// Compute normal velocity
		const auto post_vn1 = Dot(post_dv1, vc.normal);
		
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn1 - vcp1.velocityBias) < k_errorTol);
#endif
		
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase3(VelocityConstraint& vc, Velocity& velA, Velocity& velB,
										 const Vec2 b_prime)
{
	//
	// Case 3: vn2 = 0 and x1 = 0
	//
	// vn1 = a11 * 0 + a12 * x2 + b1' 
	//   0 = a21 * 0 + a22 * x2 + b2'
	//
	const auto newImpulse = Vec2{float_t{0}, -GetNormalMassAtPoint(vc, 1) * b_prime.y};
	const auto K = vc.GetK();
	assert(IsValid(K));
	const auto vn1 = K.ey.x * newImpulse.y + b_prime.x;
	if ((newImpulse.y >= float_t{0}) && (vn1 >= float_t{0}))
	{
		BlockSolveUpdate(vc, velA, velB, newImpulse);
		
#if defined(B2_DEBUG_SOLVER)
		auto& vcp2 = vc.PointAt(1);

		// Postconditions
		const auto post_dv2 = (velB.v + (velB.w * GetRevPerpendicular(vcp2.rB))) - (velA.v + (velA.w * GetRevPerpendicular(vcp2.rA)));
		
		// Compute normal velocity
		const auto post_vn2 = Dot(post_dv2, vc.normal);

		assert(Abs(post_vn2 - vcp2.velocityBias) < k_majorErrorTol);
		assert(Abs(post_vn2 - vcp2.velocityBias) < k_errorTol);
#endif
		
		return true;
	}
	return false;
}

static inline bool BlockSolveNormalCase4(VelocityConstraint& vc, Velocity& velA, Velocity& velB,
										 const Vec2 b_prime)
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
		BlockSolveUpdate(vc, velA, velB, newImpulse);
		return true;
	}
	return false;
}

static inline void BlockSolveNormalConstraint(VelocityConstraint& vc, Velocity& velA, Velocity& velB)
{
	const auto K = vc.GetK();
	assert(IsValid(K));
	
	const auto normal = GetNormal(vc);
	if (!IsValid(normal))
	{
		return;
	}
	
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

	const auto b_prime = [=]{
		// Compute normal velocity
		const auto vn1 = Dot(GetContactRelVelocity(velA, GetPointRelPosA(vc, 0), velB, GetPointRelPosB(vc, 0)), normal);
		const auto vn2 = Dot(GetContactRelVelocity(velA, GetPointRelPosA(vc, 1), velB, GetPointRelPosB(vc, 1)), normal);
		
		// Compute b
		const auto b = Vec2{vn1 - GetVelocityBiasAtPoint(vc, 0), vn2 - GetVelocityBiasAtPoint(vc, 1)};
		
		// Return b'
		return b - Transform(GetNormalImpulses(vc), K);
	}();
	
	
	if (BlockSolveNormalCase1(vc, velA, velB, b_prime))
		return;
	if (BlockSolveNormalCase2(vc, velA, velB, b_prime))
		return;
	if (BlockSolveNormalCase3(vc, velA, velB, b_prime))
		return;
	if (BlockSolveNormalCase4(vc, velA, velB, b_prime))
		return;
	
	// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
}

/// Solves the normal portion of the velocity constraint.	
static inline void SolveNormalConstraint(VelocityConstraint& vc, Velocity& velA, Velocity& velB)
{
	const auto count = vc.GetPointCount();
	assert((count == 1) || (count == 2));
	
	if ((count == 1) || (!IsValid(vc.GetK())))
	{
		SeqSolveNormalConstraint(vc, velA, velB);
	}
	else
	{
		BlockSolveNormalConstraint(vc, velA, velB);
	}
}
	
void box2d::SolveVelocityConstraint(VelocityConstraint& vc, Velocity& velA, Velocity& velB)
{
	// Solve tangent constraints first (before normal constraints) because non-penetration is more important than friction.
	SolveTangentConstraint(vc, velA, velB);
	SolveNormalConstraint(vc, velA, velB);
}

PositionSolution box2d::SolvePositionConstraint(const PositionConstraint& pc,
												Position posA, Position posB,
												float_t resolution_rate,
												float_t max_separation,
												float_t max_correction)
{
	assert(IsValid(posA));
	assert(IsValid(posB));
	assert(IsValid(resolution_rate));
	assert(IsValid(max_separation));
	assert(IsValid(max_correction));
	
	const auto invMassA = pc.bodyA.invMass;
	const auto invInertiaA = pc.bodyA.invI;
	const auto localCenterA = pc.bodyA.localCenter;
	
	const auto invMassB = pc.bodyB.invMass;
	const auto invInertiaB = pc.bodyB.invI;
	const auto localCenterB = pc.bodyB.localCenter;
	
	// Compute inverse mass total.
	// This must be > 0 unless doing TOI solving and neither bodies were the bodies specified.
	const auto invMassTotal = invMassA + invMassB;
	assert(invMassTotal >= 0);
	
	const auto totalRadius = pc.radiusA + pc.radiusB;
	
	const auto idx_fn = [=](Manifold::size_type index, Position pos_a, Position pos_b) {
		const auto psm = GetPSM(pc.manifold, index, pos_a, localCenterA, pos_b, localCenterB);
		
		const auto separation = psm.m_separation - totalRadius;
		// Positive separation means shapes not overlapping and not touching.
		// Zero separation means shapes are touching.
		// Negative separation means shapes are overlapping.
		
		const auto rA = psm.m_point - pos_a.c;
		const auto rB = psm.m_point - pos_b.c;
		
		// Compute the effective mass.
		const auto K = [&]() {
			const auto rnA = Cross(rA, psm.m_normal);
			const auto rnB = Cross(rB, psm.m_normal);
			return invMassTotal + (invInertiaA * Square(rnA)) + (invInertiaB * Square(rnB));
		}();
		
		// Prevent large corrections & don't push separation above max_separation (-LinearSlop).
		const auto C = Clamp(resolution_rate * (separation - max_separation),
							 -max_correction, float_t{0});
		
		// Compute normal impulse
		const auto P = psm.m_normal * -C / K;
		
		return PositionSolution{
			-Position{invMassA * P, 1_rad * invInertiaA * Cross(rA, P)},
			+Position{invMassB * P, 1_rad * invInertiaB * Cross(rB, P)},
			separation
		};
	};

	// Solve normal constraints
	const auto pointCount = pc.manifold.GetPointCount();
	if (pointCount == 1)
	{
		return PositionSolution{posA, posB, 0} + idx_fn(0, posA, posB);
	}
	if (pointCount == 2)
	{
		// solve most penatrating point first or solve simultaneously if about the same penetration
		const auto psm0 = GetPSM(pc.manifold, 0, posA, localCenterA, posB, localCenterB);
		const auto psm1 = GetPSM(pc.manifold, 1, posA, localCenterA, posB, localCenterB);
		if (almost_equal(psm0.m_separation, psm1.m_separation))
		{
			const auto s0 = idx_fn(0, posA, posB);
			const auto s1 = idx_fn(1, posA, posB);
			return PositionSolution{posA + s0.pos_a + s1.pos_a, posB + s0.pos_b + s1.pos_b, s0.min_separation};
		}
		if (psm0.m_separation < psm1.m_separation)
		{
			const auto s0 = idx_fn(0, posA, posB);
			posA += s0.pos_a;
			posB += s0.pos_b;
			const auto s1 = idx_fn(1, posA, posB);
			posA += s1.pos_a;
			posB += s1.pos_b;
			return PositionSolution{posA, posB, s0.min_separation};
		}
		// psm1.separation < psm0.separation
		{
			const auto s1 = idx_fn(1, posA, posB);
			posA += s1.pos_a;
			posB += s1.pos_b;
			const auto s0 = idx_fn(0, posA, posB);
			posA += s0.pos_a;
			posB += s0.pos_b;
			return PositionSolution{posA, posB, s1.min_separation};
		}
	}
	return PositionSolution{posA, posB, MaxFloat};
}

bool box2d::SolvePositionConstraints(Span<const PositionConstraint> positionConstraints,
									 Span<Position> positions)
{
	auto minSeparation = MaxFloat;
	
	for (auto&& pc: positionConstraints)
	{
		assert(pc.bodyA.index != pc.bodyB.index);
		const auto solution = SolvePositionConstraint(pc, positions[pc.bodyA.index], positions[pc.bodyB.index],
													  Baumgarte, -LinearSlop, MaxLinearCorrection);
		positions[pc.bodyA.index] = solution.pos_a;
		positions[pc.bodyB.index] = solution.pos_b;
		minSeparation = Min(minSeparation, solution.min_separation);
	}
	
	// Can't expect minSpeparation >= -LinearSlop because we don't push the separation above -LinearSlop.
	//return minSeparation >= MinSeparationThreshold;
	return minSeparation >= -LinearSlop * 3;
}

bool box2d::SolveTOIPositionConstraints(Span<const PositionConstraint> positionConstraints,
										Span<Position> positions,
										island_count_t indexA, island_count_t indexB)
{
	auto minSeparation = MaxFloat;
	
	for (auto&& positionConstraint: positionConstraints)
	{
		auto pc = positionConstraint;
		
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
		
		const auto solution = SolvePositionConstraint(pc, positions[pc.bodyA.index], positions[pc.bodyB.index],
													  ToiBaumgarte, -LinearSlop, MaxLinearCorrection);
		positions[pc.bodyA.index] = solution.pos_a;
		positions[pc.bodyB.index] = solution.pos_b;
		minSeparation = Min(minSeparation, solution.min_separation);
	}
	
	// Can't expect minSpeparation >= -LinearSlop because we don't push the separation above -LinearSlop.
	//return minSeparation >= MinToiSeparation;
	return minSeparation >= -LinearSlop * float_t(1.5);
}

