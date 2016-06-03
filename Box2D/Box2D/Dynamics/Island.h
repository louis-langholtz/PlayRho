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

#ifndef B2_ISLAND_H
#define B2_ISLAND_H

#include <Box2D/Common/Math.h>
#include <Box2D/Dynamics/TimeStep.h>

namespace box2d {

class Body;
class Contact;
class Joint;
class StackAllocator;
class ContactListener;
struct ContactVelocityConstraint;
struct Profile;

/// Island.
/// @detail A container of bodies contacts and joints relavent to handling world dynamics.
/// @note This is an internal class.
class Island
{
public:
	Island(island_count_t bodyCapacity, island_count_t contactCapacity, island_count_t jointCapacity,
		   StackAllocator& allocator, ContactListener* listener);

	/// Destructor.
	/// @detail Sets all bodies's island indexes to Body::InvalidIslandIndex and then frees allocated memory.
	~Island();

	/// Clears this island.
	/// @detail This undoes the adds of all bodies contacts and joints - removing them.
	///   On return, the get body contact and joint count methods will all return 0.
	///   Additionally all removed bodies will have their island indexes set to Body::InvalidIslandIndex.
	/// @sa void Add(Body* body).
	/// @sa void Add(Contact* contact).
	/// @sa void Add(Joint* joint).
	void Clear() noexcept;

	/// Solves this island.
	/// @detail This:
	///   1. Updates every body's sweep.pos0 to its sweep.pos1.
	///   2. Updates every body's sweep.pos1 to the new "solved" position for it.
	///   3. Updates every body's velocity to the new accelerated, dampened, and "solved" velocity for it.
	///   4. Synchronizes every body's transform (by updating it to transform one of the body's sweep).
	///   5. Reports to the listener.
	void Solve(const TimeStep& step, const Vec2& gravity, bool allowSleep);

	void SolveTOI(const TimeStep& subStep, island_count_t toiIndexA, island_count_t toiIndexB);

	/// Adds the given body to this island.
	/// @param body Body to add to this island.
	void Add(Body* body);

	void Add(Contact* contact);

	void Add(Joint* joint);

	/// Reports the given constraints to the listener.
	/// This calls the listener's PostSolve method for all m_contactCount elements of the given array of constraints.
	/// @param constraints Array of m_contactCount contact velocity constraint elements.
	void Report(const ContactVelocityConstraint* constraints);

	inline island_count_t GetBodyCapacity() const noexcept
	{
		return m_bodyCapacity;
	}

	inline island_count_t GetContactCapacity() const noexcept
	{
		return m_contactCapacity;
	}
	
	inline island_count_t GetJointCapacity() const noexcept
	{
		return m_jointCapacity;
	}

	/// Gets the body count.
	/// @return Count of bodies added to this island.
	inline island_count_t GetBodyCount() const noexcept
	{
		return m_bodyCount;
	}

	inline island_count_t GetContactCount() const noexcept
	{
		return m_contactCount;
	}

	inline island_count_t GetJointCount() const noexcept
	{
		return m_jointCount;
	}

	inline const Body* GetBody(island_count_t i) const
	{
		assert((0 <= i) && (i < m_bodyCount));
		return m_bodies[i];
	}

	inline Body* GetBody(island_count_t i)
	{
		assert((0 <= i) && (i < m_bodyCount));
		return m_bodies[i];
	}

private:
	static void CopyOut(const island_count_t count, const Position* positions, const Velocity* velocities,
						Body** bodies);
	
	void InitJointVelocityConstraints(const SolverData& solverData);
	void SolveJointVelocityConstraints(const SolverData& solverData);
	bool SolveJointPositionConstraints(const SolverData& solverData);

	float_t UpdateSleepTimes(float_t h);

	/// Clears this island of added bodies.
	/// @detail This sets all bodies's island indexes to Body::InvalidIslandIndex and resets
	///   the body count to 0.
	/// @sa Add(Body* body).
	/// @sa Body::InvalidIslandIndex.
	void ClearBodies() noexcept;

	const island_count_t m_bodyCapacity; ///< Body capacity.
	const island_count_t m_contactCapacity; ///< Contact capacity.
	const island_count_t m_jointCapacity; ///< Joint capacity.

	island_count_t m_bodyCount = 0; ///< Count of bodies added to the island. Max of m_bodyCapacity.
	island_count_t m_contactCount = 0; ///< Count of contacts added to the island. Max of m_contactCapacity.
	island_count_t m_jointCount = 0; ///< Count of joints added to the island. Max of m_jointCapacity.
	
	StackAllocator& m_allocator; ///< Stack-style memory allocator set on construction.
	ContactListener* const m_listener;

	Body** const m_bodies;
	Contact** const m_contacts;
	Joint** const m_joints;

	/// Buffer of velocities allocated on construction.
	/// @detail This is a body-capacity-sized array of velocities.
	/// @sa m_bodyCapacity.
	Velocity* const m_velocities;
	
	/// Buffer of positions allocated on construction.
 	/// @detail This is a body-capacity-sized array of positions.
	/// @sa m_bodyCapacity.
	Position* const m_positions;
};

} // namespace box2d

#endif
