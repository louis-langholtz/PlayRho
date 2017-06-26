/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Span.hpp>

namespace box2d {

    class VelocityConstraint;
    class PositionConstraint;
    class BodyConstraint;
    
    struct PositionSolution
    {
        Position pos_a;
        Position pos_b;
        Length min_separation;
    };

    inline PositionSolution operator+ (PositionSolution lhs, PositionSolution rhs)
    {
        return PositionSolution{
            lhs.pos_a + rhs.pos_a,
            lhs.pos_b + rhs.pos_b,
            lhs.min_separation + rhs.min_separation
        };
    }

    inline PositionSolution operator- (PositionSolution lhs, PositionSolution rhs)
    {
        return PositionSolution{
            lhs.pos_a - rhs.pos_a,
            lhs.pos_b - rhs.pos_b,
            lhs.min_separation - rhs.min_separation
        };
    }

    /// Constraint solver configuration data.
    /// @details
    /// Defines how a constraint solver should resolve a given constraint.
    /// @sa SolvePositionConstraint.
    struct ConstraintSolverConf
    {
        ConstraintSolverConf& UseResolutionRate(RealNum value) noexcept;

        ConstraintSolverConf& UseLinearSlop(Length value) noexcept;
        
        ConstraintSolverConf& UseAngularSlop(Angle value) noexcept;

        ConstraintSolverConf& UseMaxLinearCorrection(Length value) noexcept;

        ConstraintSolverConf& UseMaxAngularCorrection(Angle value) noexcept;

        /// Resolution rate.
        /// @details
        /// Defines the percentage of the overlap that should get resolved in a single solver call.
        /// Value greater than zero and less than or equal to one.
        /// Ideally this would be 1 so that overlap is removed in one time step.
        /// However using values close to 1 often leads to overshoot.
        /// @note Recommended values are: <code>0.2</code> for solving regular constraints
        ///   or <code>0.75</code> for solving TOI constraints.
        RealNum resolutionRate = RealNum(0.2);

        /// Linear slop.
        /// @note The negative of this amount is the maximum amount of separation to create.
        /// @note Recommended value: <code>DefaultLinearSlop</code>.
        Length linearSlop = DefaultLinearSlop;

        /// Angular slop.
        /// @note Recommended value: <code>DefaultAngularSlop</code>.
        Angle angularSlop = DefaultAngularSlop;

        /// Maximum linear correction.
        /// @details
        /// Maximum amount of overlap to resolve in a single solver call. Helps prevent overshoot.
        /// @note Recommended value: <code>linearSlop * 40</code>.
        Length maxLinearCorrection = DefaultLinearSlop * RealNum{20};
        
        /// Maximum angular correction.
        /// @details Maximum angular position correction used when solving constraints.
        /// Helps to prevent overshoot.
        /// @note Recommended value: <code>angularSlop * 4</code>.
        Angle maxAngularCorrection = DefaultAngularSlop * RealNum{4};
    };

    inline ConstraintSolverConf& ConstraintSolverConf::UseResolutionRate(RealNum value) noexcept
    {
        resolutionRate = value;
        return *this;
    }

    inline ConstraintSolverConf& ConstraintSolverConf::UseLinearSlop(Length value) noexcept
    {
        linearSlop = value;
        return *this;
    }

    inline ConstraintSolverConf& ConstraintSolverConf::UseAngularSlop(Angle value) noexcept
    {
        angularSlop = value;
        return *this;
    }

    inline ConstraintSolverConf& ConstraintSolverConf::UseMaxLinearCorrection(Length value) noexcept
    {
        maxLinearCorrection = value;
        return *this;
    }

    inline ConstraintSolverConf& ConstraintSolverConf::UseMaxAngularCorrection(Angle value) noexcept
    {
        maxAngularCorrection = value;
        return *this;
    }

    inline ConstraintSolverConf GetDefaultPositionSolverConf()
    {
        return ConstraintSolverConf{}.UseResolutionRate(RealNum(0.2));
    }
    
    inline ConstraintSolverConf GetDefaultToiPositionSolverConf()
    {
        // For solving TOI events, use a faster/higher resolution rate than normally used.
        return ConstraintSolverConf{}.UseResolutionRate(RealNum(0.75));
    }
    
    namespace GaussSeidel {

        Momentum BlockSolveNormalConstraint(VelocityConstraint& vc);

        Momentum SeqSolveNormalConstraint(VelocityConstraint& vc);

        /// Solves the tangential portion of the velocity constraint.
        /// @details
        /// This imposes friction on the velocity.
        /// Specifically, this updates the tangent impulses on the velocity constraint points and
        ///   updates the two given velocity structures.
        /// @warning Behavior is undefined unless the velocity constraint point count is 1 or 2.
        /// @param vc Velocity constraint.
        Momentum SolveTangentConstraint(VelocityConstraint& vc);

        /// Solves the normal portion of the velocity constraint.
        /// @details
        /// This prevents penetration and applies the contact restitution to the velocity.
        Momentum SolveNormalConstraint(VelocityConstraint& vc);

        /// Solves the velocity constraint.
        ///
        /// @details This updates the tangent and normal impulses of the velocity constraint points of
        ///   the given velocity constraint and updates the given velocities.
        ///
        /// @warning Behavior is undefined unless the velocity constraint point count is 1 or 2.
        /// @note Linear velocity is only changed if the inverse mass of either body is non-zero.
        /// @note Angular velocity is only changed if the inverse rotational inertia of either body is non-zero.
        ///
        /// @pre The velocity constraint must have a valid normal, a valid tangent,
        ///   valid point relative positions, and valid velocity biases.
        ///
        Momentum SolveVelocityConstraint(VelocityConstraint& vc);
        
        /// Solves the given position constraint.
        /// @details
        /// This pushes apart the two given positions for every point in the contact position constraint
        /// and returns the minimum separation value from the position solver manifold for each point.
        /// @sa http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
        /// @return Minimum separation distance of the position constraint's manifold points
        ///   (prior to "solving").
        PositionSolution SolvePositionConstraint(const PositionConstraint& pc,
                                                 const bool moveA, const bool moveB,
                                                 ConstraintSolverConf conf);
        
    } // namespace GaussSidel
    
} // namespace box2d

#endif

