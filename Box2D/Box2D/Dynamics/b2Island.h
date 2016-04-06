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

class b2Body;
class b2Contact;
class b2Joint;
class b2StackAllocator;
class b2ContactListener;
struct b2ContactVelocityConstraint;
struct b2Profile;

/// This is an internal class.
class b2Island
{
public:
	using size_type = std::size_t;

	b2Island(size_type bodyCapacity, size_type contactCapacity, size_type jointCapacity,
			b2StackAllocator* allocator, b2ContactListener* listener);
	~b2Island();

	void Clear() noexcept;

	void Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, bool allowSleep);

	void SolveTOI(const b2TimeStep& subStep, size_type toiIndexA, size_type toiIndexB);

	void Add(b2Body* body);

	void Add(b2Contact* contact);

	void Add(b2Joint* joint);

	void Report(const b2ContactVelocityConstraint* constraints);

	inline size_type GetBodyCapacity() const noexcept
	{
		return m_bodyCapacity;
	}

	inline size_type GetContactCapacity() const noexcept
	{
		return m_contactCapacity;
	}
	
	inline size_type GetJointCapacity() const noexcept
	{
		return m_jointCapacity;
	}

	inline size_type GetBodyCount() const noexcept
	{
		return m_bodyCount;
	}

	inline size_type GetContactCount() const noexcept
	{
		return m_contactCount;
	}

	inline size_type GetJointCount() const noexcept
	{
		return m_jointCount;
	}

	inline const b2Body* GetBody(size_type i) const
	{
		b2Assert((0 <= i) && (i < m_bodyCount));
		return m_bodies[i];
	}

	inline b2Body* GetBody(size_type i)
	{
		b2Assert((0 <= i) && (i < m_bodyCount));
		return m_bodies[i];
	}

private:
	void ClearBodies() noexcept;

	size_type m_bodyCount = 0;
	size_type m_contactCount = 0;
	size_type m_jointCount = 0;

	const size_type m_bodyCapacity;
	const size_type m_contactCapacity;
	const size_type m_jointCapacity;

	b2StackAllocator* const m_allocator;
	b2ContactListener* const m_listener;

	b2Body** const m_bodies;
	b2Contact** const m_contacts;
	b2Joint** const m_joints;
	b2Velocity* const m_velocities;
	b2Position* const m_positions;
};

#endif
