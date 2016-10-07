/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_CONTACT_SOLVER_H
#define B2_CONTACT_SOLVER_H

#include <Box2D/Common/Math.h>

namespace box2d {

	class VelocityConstraint;
	class PositionConstraint;

	/// Contact Solver.
	/// @note This data structure is at least 36-bytes large.
	class ContactSolver
	{
	public:
		/// Minimum separation for position constraints.
		static constexpr auto MinSeparationThreshold = BOX2D_MAGIC(-LinearSlop * 3);
		
		/// Minimum time of impact separation for TOI position constraints.
		static constexpr auto MinToiSeparation = BOX2D_MAGIC(-LinearSlop * float_t{3} / float_t{2}); // aka -LinearSlop * 1.5
		
		/// Initializing constructor.
		/// @param positions Array of positions, one for every body referenced by a contact.
		/// @param velocities Array of velocities, for every body referenced by a contact.
		/// @param count Count of contacts.
		/// @param positionConstraints Array of position-constraints (1 per contact).
		///   Must be non-null if count is greater than zero.
		/// @param velocityConstraints Array of velocity-constraints (1 per contact).
		///   Must be non-null if count is greater than zero.
		/// @note Behavior is undefined if count is greater than zero and any of the arrays are
		///   <code>nullptr</code>.
		ContactSolver(Position* positions, Velocity* velocities,
					  contact_count_t count,
					  PositionConstraint* positionConstraints,
					  VelocityConstraint* velocityConstraints) noexcept :
			m_positions{positions},
			m_velocities{velocities},
			m_count{count},
			m_positionConstraints{positionConstraints},
			m_velocityConstraints{velocityConstraints}
		{
			assert((count == 0) || (positions && velocities && positionConstraints && velocityConstraints));
		}
		
		~ContactSolver() = default;
		
		ContactSolver() = delete;
		ContactSolver(const ContactSolver& copy) = delete;
		
		/// Updates velocity constraints.
		/// @detail
		/// Updates the position dependent portions of the velocity constraints with the
		/// information from the current position constraints.
		/// @note This MUST be called prior to calling <code>SolveVelocityConstraints</code>.
		/// @post Velocity constraints will have their "normal" field set to the world manifold normal for them.
		/// @post Velocity constraints will have their constraint points updated.
		/// @sa SolveVelocityConstraints.
		void UpdateVelocityConstraints();
				
		/// "Solves" the velocity constraints.
		/// @detail Updates the velocities and velocity constraint points' normal and tangent impulses.
		/// @pre <code>UpdateVelocityConstraints</code> has been called on this object.
		void SolveVelocityConstraints();
		
		/// Solves position constraints.
		/// @detail This updates positions (and nothing else).
		/// @return true if the minimum separation is above the minimum separation threshold, false otherwise.
		/// @sa MinSeparationThreshold.
		bool SolvePositionConstraints();
		
		/// Solves TOI position constraints.
		/// @detail Sequential position solver for TOI-based position constraints.
		///   This only updates positions for the bodies identified by the given indexes (and nothing else).
		/// @param indexA Index within the island of body A.
		/// @param indexB Index within the island of body B.
		/// @return true if the minimum separation is above the minimum TOI separation value, false otherwise.
		bool SolveTOIPositionConstraints(island_count_t indexA, island_count_t indexB);
		
	private:
		
		Position* const m_positions; ///< Array of positions (8-bytes).
		Velocity* const m_velocities; ///< Array of velocities (8-bytes).
		
		const contact_count_t m_count; ///< Count of elements (contacts) in the contact position-constraint and velocity-constraint arrays (4-bytes).
		PositionConstraint* const m_positionConstraints; ///< Array of position-constraints (1 per contact, 8-bytes).
		VelocityConstraint* const m_velocityConstraints; ///< Array of velocity-constraints (1 per contact, 8-bytes).
	};
	
	struct PositionSolution
	{
		Position pos_a;
		Position pos_b;
		float_t min_separation;
	};

	/// Solves the given position constraint.
	/// @detail
	/// This updates the two given positions for every point in the contact position constraint
	/// and returns the minimum separation value from the position solver manifold for each point.
	/// @param resolution_rate Resolution rate. Value greater than zero and less than or equal to one.
	///   Defines the percentage of the overlap that should get resolved in a single call to this
	///   function. Recommended values are: <code>Baumgarte</code> or <code>ToiBaumgarte</code>.
	/// @param max_separation Maximum separation to create. Recommended value: <code>-LinearSlop</code>.
	/// @param max_correction Maximum correction. Maximum amount of overlap to resolve in a single
	///   call to this function. Recommended value: <code>MaxLinearCorrection</code>.
	/// @sa http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
	/// @return Minimum separation distance of the position constraint's manifold points
	///   (prior to "solving").
	PositionSolution Solve(const PositionConstraint& pc, Position positionA, Position positionB,
						   float_t resolution_rate, float_t max_separation, float_t max_correction);

} // namespace box2d

#endif

