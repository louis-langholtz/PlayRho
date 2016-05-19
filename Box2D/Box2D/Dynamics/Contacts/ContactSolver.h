/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
#include <Box2D/Collision/Collision.h>
#include <Box2D/Dynamics/TimeStep.h>

namespace box2d {

class Contact;
class Body;
class StackAllocator;
struct ContactPositionConstraint;
struct ContactPositionConstraintBodyData;

/// Velocity constraint point.
struct VelocityConstraintPoint
{
	Vec2 rA; ///< Position of body A relative to world manifold point
	Vec2 rB; ///< Position of body B relative to world manifold point
	float_t normalImpulse; ///< Normal impulse.
	float_t tangentImpulse; ///< Tangent impulse.
	float_t normalMass; ///< Normal mass.
	float_t tangentMass; ///< Tangent mass.
	float_t velocityBias; ///< Velocity bias.
};

/// Contact velocity constraint body data.
/// @note This structure is intentionally toplevel like the ContactPositionConstraintBodyData
///   structure.
struct ContactVelocityConstraintBodyData
{
	using index_t = size_t;

	index_t index; ///< Index within island of body.
	float_t invMass; ///< Inverse mass of body.
	float_t invI; ///< Inverse rotational interia of body.
};

/// Contact velocity constraint.
/// @note A valid contact velocity constraint must have either a point count of either 1 or 2.
class ContactVelocityConstraint
{
public:
	using size_type = std::remove_cv<decltype(MaxManifoldPoints)>::type;
	using index_type = size_t;
	
	/// Gets the count of points added to this object.
	/// @return Value between 0 and MaxManifoldPoints
	/// @sa MaxManifoldPoints.
	/// @sa AddPoint.
	size_type GetPointCount() const noexcept { return pointCount; }
	
	/// Gets the point identified by the given index.
	/// @note Behavior is undefined if the identified point does not exist.
	/// @param index Index of the point to return. This is a value less than returned by GetPointCount().
	/// @return velocity constraint point for the given index.
	/// @sa GetPointCount.
	const VelocityConstraintPoint& GetPoint(size_type index) const
	{
		assert(index < pointCount);
		return points[index];
	}

	/// Gets the point identified by the given index.
	/// @note Behavior is undefined if the identified point does not exist.
	/// @param index Index of the point to return. This is a value less than returned by GetPointCount().
	/// @return velocity constraint point for the given index.
	/// @sa GetPointCount.
	VelocityConstraintPoint& GetPoint(size_type index)
	{
		assert(index < pointCount);
		return points[index];
	}

	void ClearPoints() noexcept { pointCount = 0; }

	/// Adds the given point to this contact velocity constraint object.
	/// @detail Adds up to MaxManifoldPoints points. To find out how many points have already
	///   been added, call GetPointCount().
	/// @param val Velocity constraint point value to add.
	/// @note Behavior is undefined if an attempt is made to add more than MaxManifoldPoints points.
	/// @sa GetPointCount().
	void AddPoint(const VelocityConstraintPoint& val)
	{
		assert(pointCount < MaxManifoldPoints);
		points[pointCount] = val;
		++pointCount;
	}

	void RemovePoint()
	{
		assert(pointCount > 0);
		--pointCount;
	}

	Vec2 normal;
	Mat22 normalMass;
	Mat22 K;
	ContactVelocityConstraintBodyData bodyA; ///< Body A contact velocity constraint data.
	ContactVelocityConstraintBodyData bodyB; ///< Body B contact velocity constraint data.
	float_t friction; ///< Friction coefficient. Usually in the range of [0,1].
	float_t restitution;
	float_t tangentSpeed;
	index_type contactIndex;

private:
	VelocityConstraintPoint points[MaxManifoldPoints];
	size_type pointCount;
};

struct ContactSolverDef
{
	using size_type = size_t;

	TimeStep step;
	Contact** contacts; // pointers to contacts
	size_type count; // count of contacts
	Position* positions; // positions for every body referenced by a contact
	Velocity* velocities; // velocities for every body referenced by a contact
	StackAllocator* allocator;
};

/// Contact Solver.
class ContactSolver
{
public:
	using size_type = size_t;
	
	/// Minimum separation for position constraints.
	static constexpr auto MinSeparationThreshold = -LinearSlop * float_t(3);

	/// Minimum time of impact separation for TOI position constraints.
	static constexpr auto MinToiSeparation = -LinearSlop * float_t(1.5);

	ContactSolver(ContactSolverDef* def);
	~ContactSolver();

	ContactSolver() = delete;
	ContactSolver(const ContactSolver& copy) = delete;

	void InitializeVelocityConstraints();

	void WarmStart();
	void StoreImpulses();

	void SolveVelocityConstraints();

	/// Solves position constraints.
	/// @return true if the minimum separation is above the minimum separation threshold, false otherwise.
	/// @sa MinSeparationThreshold.
	bool SolvePositionConstraints();
	
	/// Sequential position solver for TOI-based position constraints.
	/// @return true if the minimum separation is above the minimum TOI separation value, false otherwise.
	bool SolveTOIPositionConstraints(size_type indexA, size_type indexB);

	const ContactVelocityConstraint* GetVelocityConstraints() const noexcept
	{
		return m_velocityConstraints;
	}

private:
	static void Assign(ContactVelocityConstraint& var, const Contact& val);
	static void Assign(ContactPositionConstraintBodyData& var, const Body& val);
	static void Assign(ContactVelocityConstraintBodyData& var, const Body& val);

	void InitializeVelocityConstraint(ContactVelocityConstraint& vc, const ContactPositionConstraint& pc);

	const TimeStep m_step;
	Position* const m_positions;
	Velocity* const m_velocities;
	StackAllocator* const m_allocator;
	Contact** const m_contacts;
	const size_type m_count;
	ContactPositionConstraint* const m_positionConstraints;
	ContactVelocityConstraint* const m_velocityConstraints;
};

} // namespace box2d

#endif

