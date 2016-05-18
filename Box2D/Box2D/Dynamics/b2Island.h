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

#include <Box2D/Common/b2Math.h>
#include <Box2D/Dynamics/b2TimeStep.h>

namespace box2d {

class b2Body;
class b2Contact;
class b2Joint;
class b2StackAllocator;
class b2ContactListener;
struct b2ContactVelocityConstraint;
struct b2Profile;

/// Island.
/// @detail A container of bodies contacts and joints relavent to handling world dynamics.
/// @note This is an internal class.
class b2Island
{
public:
	b2Island(island_count_t bodyCapacity, island_count_t contactCapacity, island_count_t jointCapacity,
			b2StackAllocator* allocator, b2ContactListener* listener);
	~b2Island();

	/// Clears this island.
	/// @detail This undoes the adds of all bodies contacts and joints - removing them.
	///   On return, the get body contact and joint count methods will all return 0.
	///   Additionally all removed bodies will have their island indexes set to b2Body::InvalidIslandIndex.
	/// @sa void Add(b2Body* body).
	/// @sa void Add(b2Contact* contact).
	/// @sa void Add(b2Joint* joint).
	void Clear() noexcept;

	void Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, bool allowSleep);

	void SolveTOI(const b2TimeStep& subStep, island_count_t toiIndexA, island_count_t toiIndexB);

	void Add(b2Body* body);

	void Add(b2Contact* contact);

	void Add(b2Joint* joint);

	void Report(const b2ContactVelocityConstraint* constraints);

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

	inline const b2Body* GetBody(island_count_t i) const
	{
		assert((0 <= i) && (i < m_bodyCount));
		return m_bodies[i];
	}

	inline b2Body* GetBody(island_count_t i)
	{
		assert((0 <= i) && (i < m_bodyCount));
		return m_bodies[i];
	}

private:
	
	/// Clears this island of added bodies.
	/// @detail This sets all bodies's island indexes to b2Body::InvalidIslandIndex and resets
	///   the body count to 0.
	/// @sa Add(b2Body* body).
	/// @sa b2Body::InvalidIslandIndex.
	void ClearBodies() noexcept;

	island_count_t m_bodyCount = 0;
	island_count_t m_contactCount = 0;
	island_count_t m_jointCount = 0;

	const island_count_t m_bodyCapacity;
	const island_count_t m_contactCapacity;
	const island_count_t m_jointCapacity;

	b2StackAllocator* const m_allocator;
	b2ContactListener* const m_listener;

	b2Body** const m_bodies;
	b2Contact** const m_contacts;
	b2Joint** const m_joints;
	b2Velocity* const m_velocities;
	b2Position* const m_positions;
};

} // namespace box2d

#endif
