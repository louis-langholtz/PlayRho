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

struct b2VelocityConstraintPoint
{
	b2Vec2 rA;
	b2Vec2 rB;
	float32 normalImpulse;
	float32 tangentImpulse;
	float32 normalMass;
	float32 tangentMass;
	float32 velocityBias;
};

class b2ContactVelocityConstraint
{
public:
	int32 GetPointCount() const noexcept { return pointCount; }
	
	const b2VelocityConstraintPoint& GetPoint(int32 index) const
	{
		b2Assert(index < pointCount);
		return points[index];
	}

	b2VelocityConstraintPoint& GetPoint(int32 index)
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

	float32 GetFriction() const noexcept { return friction; }
	void SetFriction(float32 val) noexcept { friction = val; }

	float32 GetRestitution() const noexcept { return restitution; }
	void SetRestitution(float32 val) noexcept { restitution = val; }

	float32 GetTangentSpeed() const noexcept { return tangentSpeed; }
	void SetTangentSpeed(float32 val) noexcept { tangentSpeed = val; }

	b2Vec2 GetNormal() const noexcept { return normal; }
	void SetNormal(const b2Vec2& val) noexcept { normal = val; }

	int32 GetIndexA() const noexcept { return indexA; }
	float32 GetInvMassA() const noexcept { return invMassA; }
	float32 GetInvIA() const noexcept { return invIA; }
	void SetIndexA(int32 val) noexcept { indexA = val; }
	void SetInvMassA(float32 val) noexcept { invMassA = val; }
	void SetInvIA(float32 val) noexcept { invIA = val; }

	int32 GetIndexB() const noexcept { return indexB; }
	float32 GetInvMassB() const noexcept { return invMassB; }
	float32 GetInvIB() const noexcept { return invIB; }
	void SetIndexB(int32 val) noexcept { indexB = val; }
	void SetInvMassB(float32 val) noexcept { invMassB = val; }
	void SetInvIB(float32 val) noexcept { invIB = val; }

	int32 GetContactIndex() const noexcept { return contactIndex; }
	void SetContactIndex(int32 val) noexcept { contactIndex = val; }

	const b2Mat22& GetNormalMass() const noexcept { return normalMass; }
	void SetNormalMass(const b2Mat22& val) noexcept { normalMass = val; }

	const b2Mat22& GetK() const noexcept { return K; }
	void SetK(const b2Mat22& val) noexcept { K = val; }

private:
	b2VelocityConstraintPoint points[b2_maxManifoldPoints];
	b2Vec2 normal;
	b2Mat22 normalMass;
	b2Mat22 K;
	int32 indexA;
	int32 indexB;
	float32 invMassA, invMassB;
	float32 invIA, invIB;
	float32 friction;
	float32 restitution;
	float32 tangentSpeed;
	int32 pointCount;
	int32 contactIndex;
};

struct b2ContactSolverDef
{
	b2TimeStep step;
	b2Contact** contacts;
	int32 count;
	b2Position* positions;
	b2Velocity* velocities;
	b2StackAllocator* allocator;
};

class b2ContactSolver
{
public:
	b2ContactSolver(b2ContactSolverDef* def);
	~b2ContactSolver();

	b2ContactSolver() = delete;
	b2ContactSolver(const b2ContactSolver& copy) = delete;

	void InitializeVelocityConstraints();

	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
	bool SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB);

	const b2ContactVelocityConstraint* GetVelocityConstraints() const noexcept
	{
		return m_velocityConstraints;
	}

private:
	const b2TimeStep m_step;
	b2Position* const m_positions;
	b2Velocity* const m_velocities;
	b2StackAllocator* const m_allocator;
	b2Contact** const m_contacts;
	const int m_count;
	b2ContactPositionConstraint* const m_positionConstraints;
	b2ContactVelocityConstraint* const m_velocityConstraints;
};

#endif

