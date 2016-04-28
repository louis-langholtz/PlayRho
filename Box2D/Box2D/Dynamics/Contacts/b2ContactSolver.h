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

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Dynamics/b2TimeStep.h>

class b2Contact;
class b2Body;
class b2StackAllocator;
struct b2ContactPositionConstraint;
struct b2ContactPositionConstraintBodyData;

struct b2VelocityConstraintPoint
{
	b2Vec2 rA;
	b2Vec2 rB;
	b2Float normalImpulse;
	b2Float tangentImpulse;
	b2Float normalMass;
	b2Float tangentMass;
	b2Float velocityBias;
};

struct b2ContactVelocityConstraintBodyData
{
	using index_t = b2_size_t;

	index_t index; // index of body within island
	b2Float invMass;
	b2Float invI;
};

class b2ContactVelocityConstraint
{
public:
	using size_type = std::remove_cv<decltype(b2_maxManifoldPoints)>::type;
	using index_type = b2_size_t;
	
	size_type GetPointCount() const noexcept { return pointCount; }
	
	const b2VelocityConstraintPoint& GetPoint(size_type index) const
	{
		b2Assert(index < pointCount);
		return points[index];
	}

	b2VelocityConstraintPoint& GetPoint(size_type index)
	{
		b2Assert(index < pointCount);
		return points[index];
	}

	void ClearPoints() noexcept { pointCount = 0; }

	void AddPoint(const b2VelocityConstraintPoint& val)
	{
		b2Assert(pointCount < b2_maxManifoldPoints);
		points[pointCount] = val;
		++pointCount;
	}

	void RemovePoint()
	{
		b2Assert(pointCount > 0);
		--pointCount;
	}

	b2Vec2 normal;
	b2Mat22 normalMass;
	b2Mat22 K;
	b2ContactVelocityConstraintBodyData bodyA;
	b2ContactVelocityConstraintBodyData bodyB;
	b2Float friction;
	b2Float restitution;
	b2Float tangentSpeed;
	index_type contactIndex;

private:
	b2VelocityConstraintPoint points[b2_maxManifoldPoints];
	size_type pointCount;
};

struct b2ContactSolverDef
{
	using size_type = b2_size_t;

	b2TimeStep step;
	b2Contact** contacts; // pointers to contacts
	size_type count; // count of contacts
	b2Position* positions; // positions for every body referenced by a contact
	b2Velocity* velocities; // velocities for every body referenced by a contact
	b2StackAllocator* allocator;
};

class b2ContactSolver
{
public:
	using size_type = b2_size_t;

	b2ContactSolver(b2ContactSolverDef* def);
	~b2ContactSolver();

	b2ContactSolver() = delete;
	b2ContactSolver(const b2ContactSolver& copy) = delete;

	void InitializeVelocityConstraints();

	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
	bool SolveTOIPositionConstraints(size_type toiIndexA, size_type toiIndexB);

	const b2ContactVelocityConstraint* GetVelocityConstraints() const noexcept
	{
		return m_velocityConstraints;
	}

private:
	static void Assign(b2ContactVelocityConstraint& var, const b2Contact& val);
	static void Assign(b2ContactPositionConstraintBodyData& var, const b2Body& val);
	static void Assign(b2ContactVelocityConstraintBodyData& var, const b2Body& val);

	const b2TimeStep m_step;
	b2Position* const m_positions;
	b2Velocity* const m_velocities;
	b2StackAllocator* const m_allocator;
	b2Contact** const m_contacts;
	const size_type m_count;
	b2ContactPositionConstraint* const m_positionConstraints;
	b2ContactVelocityConstraint* const m_velocityConstraints;
};

#endif

