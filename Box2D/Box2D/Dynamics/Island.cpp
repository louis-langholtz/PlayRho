/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Distance.h>
#include <Box2D/Dynamics/Island.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Contacts/ContactSolver.h>
#include <Box2D/Dynamics/Joints/Joint.h>
#include <Box2D/Common/StackAllocator.h>
#include <Box2D/Common/Timer.h>

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than LinearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

using namespace box2d;

Island::Island(
	island_count_t bodyCapacity,
	island_count_t contactCapacity,
	island_count_t jointCapacity,
	StackAllocator* allocator,
	ContactListener* listener):
m_bodyCapacity(bodyCapacity),
m_contactCapacity(contactCapacity),
m_jointCapacity(jointCapacity),
m_allocator(allocator),
m_listener(listener),
m_bodies(static_cast<Body**>(m_allocator->Allocate(bodyCapacity * sizeof(Body*)))),
m_contacts(static_cast<Contact**>(m_allocator->Allocate(contactCapacity * sizeof(Contact*)))),
m_joints(static_cast<Joint**>(m_allocator->Allocate(jointCapacity * sizeof(Joint*)))),
m_velocities(static_cast<Velocity*>(m_allocator->Allocate(bodyCapacity * sizeof(Velocity)))),
m_positions(static_cast<Position*>(m_allocator->Allocate(bodyCapacity * sizeof(Position))))
{
}

Island::~Island()
{
	ClearBodies();

	// Warning: the order should reverse the constructor order.
	m_allocator->Free(m_positions);
	m_allocator->Free(m_velocities);
	m_allocator->Free(m_joints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_bodies);
}

void Island::Clear() noexcept
{
	ClearBodies();
	m_contactCount = 0;
	m_jointCount = 0;
}

void Island::ClearBodies() noexcept
{
	for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
		m_bodies[i]->m_islandIndex = Body::InvalidIslandIndex;	
	m_bodyCount = 0;
}

void Island::Solve(Profile* profile, const TimeStep& step, const Vec2& gravity, bool allowSleep)
{
	const auto h = step.get_dt();

	// Integrate velocities and apply damping. Initialize the body state.
	for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
	{
		auto& b = *m_bodies[i];

		// Store positions for continuous collision.
		b.m_sweep.pos0 = b.m_sweep.pos1;
		m_positions[i] = b.m_sweep.pos1;

		{
			auto velocity = b.m_velocity;
			if (b.m_type == BodyType::Dynamic)
			{
				// Integrate velocities.
				velocity.v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
				velocity.w += h * b.m_invI * b.m_torque;

				// Apply damping.
				// ODE: dv/dt + c * v = 0
				// Solution: v(t) = v0 * exp(-c * t)
				// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
				// v2 = exp(-c * dt) * v1
				// Pade approximation:
				// v2 = v1 * 1 / (1 + c * dt)
				velocity.v *= float_t(1) / (float_t(1) + h * b.m_linearDamping);
				velocity.w *= float_t(1) / (float_t(1) + h * b.m_angularDamping);
			}
			m_velocities[i] = velocity;
		}
	}

	// Solver data
	const auto solverData = SolverData{step, m_positions, m_velocities};

	// Initialize velocity constraints.
	ContactSolverDef contactSolverDef;
	contactSolverDef.step = step;
	contactSolverDef.contacts = m_contacts;
	contactSolverDef.count = m_contactCount;
	contactSolverDef.positions = m_positions;
	contactSolverDef.velocities = m_velocities;
	contactSolverDef.allocator = m_allocator;

	ContactSolver contactSolver(&contactSolverDef);
	contactSolver.InitializeVelocityConstraints();

	if (step.warmStarting)
	{
		contactSolver.WarmStart();
	}
	
	for (auto i = decltype(m_jointCount){0}; i < m_jointCount; ++i)
	{
		m_joints[i]->InitVelocityConstraints(solverData);
	}

	// Solve velocity constraints
	for (auto i = decltype(step.velocityIterations){0}; i < step.velocityIterations; ++i)
	{
		for (auto j = decltype(m_jointCount){0}; j < m_jointCount; ++j)
		{
			m_joints[j]->SolveVelocityConstraints(solverData);
		}

		contactSolver.SolveVelocityConstraints();
	}

	// Store impulses for warm starting
	contactSolver.StoreImpulses();

	// Integrate positions
	for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
	{
		auto velocity = m_velocities[i];
		auto translation = h * velocity.v;
		auto rotation = h * velocity.w;
		
		if (translation.LengthSquared() > Square(MaxTranslation))
		{
			const auto ratio = MaxTranslation / translation.Length();
			velocity.v *= ratio;
			translation = h * velocity.v;
		}
		if (Abs(rotation) > MaxRotation)
		{
			const auto ratio = MaxRotation / Abs(rotation);
			velocity.w *= ratio;
			rotation = h * velocity.w;
		}

		m_velocities[i] = velocity;
		m_positions[i] = Position{m_positions[i].c + translation, m_positions[i].a + rotation};
	}

	// Solve position constraints
	auto positionSolved = false;
	for (auto i = decltype(step.positionIterations){0}; i < step.positionIterations; ++i)
	{
		const auto contactsOkay = contactSolver.SolvePositionConstraints();

		auto jointsOkay = true;
		for (auto j = decltype(m_jointCount){0}; j < m_jointCount; ++j)
		{
			const auto jointOkay = m_joints[j]->SolvePositionConstraints(solverData);
			jointsOkay = jointsOkay && jointOkay;
		}

		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			positionSolved = true;
			break;
		}
	}

	// Copy state buffers back to the bodies
	for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
	{
		auto& body = *m_bodies[i];
		body.m_sweep.pos1 = m_positions[i];
		body.m_velocity = m_velocities[i];
		body.m_xf = GetTransformOne(body.m_sweep);
	}

	Report(contactSolver.GetVelocityConstraints());

	if (allowSleep)
	{
		auto minSleepTime = MaxFloat;

		constexpr auto linTolSqr = Square(LinearSleepTolerance);
		constexpr auto angTolSqr = Square(AngularSleepTolerance);

		for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
		{
			auto& b = *m_bodies[i];
			if (b.GetType() == BodyType::Static)
			{
				continue;
			}

			if ((!b.IsSleepingAllowed()) ||
				(Square(b.m_velocity.w) > angTolSqr) ||
				(b.m_velocity.v.LengthSquared() > linTolSqr))
			{
				b.m_sleepTime = float_t{0};
				minSleepTime = float_t{0};
			}
			else
			{
				b.m_sleepTime += h;
				minSleepTime = Min(minSleepTime, b.m_sleepTime);
			}
		}

		if ((minSleepTime >= TimeToSleep) && positionSolved)
		{
			for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
			{
				m_bodies[i]->UnsetAwake();
			}
		}
	}
}

void Island::SolveTOI(const TimeStep& subStep, island_count_t toiIndexA, island_count_t toiIndexB)
{
	assert(toiIndexA < m_bodyCount);
	assert(toiIndexB < m_bodyCount);

	// Initialize the body state.
	for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
	{
		const auto& b = *m_bodies[i];
		m_positions[i] = b.m_sweep.pos1;
		m_velocities[i] = b.m_velocity;
	}

	ContactSolverDef contactSolverDef;
	contactSolverDef.contacts = m_contacts;
	contactSolverDef.count = m_contactCount;
	contactSolverDef.allocator = m_allocator;
	contactSolverDef.step = subStep;
	contactSolverDef.positions = m_positions;
	contactSolverDef.velocities = m_velocities;
	ContactSolver contactSolver(&contactSolverDef);

	// Solve position constraints.
	for (auto i = decltype(subStep.positionIterations){0}; i < subStep.positionIterations; ++i)
	{
		const auto contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
		if (contactsOkay)
		{
			break;
		}
	}

#if 0
	// Is the new position really safe?
	for (auto i = decltype(m_contactCount){0}; i < m_contactCount; ++i)
	{
		Contact* c = m_contacts[i];
		Fixture* fA = c->GetFixtureA();
		Fixture* fB = c->GetFixtureB();

		Body* bA = fA->GetBody();
		Body* bB = fB->GetBody();

		int32 indexA = c->GetChildIndexA();
		int32 indexB = c->GetChildIndexB();

		DistanceInput input;
		input.proxyA.Set(*fA->GetShape(), indexA);
		input.proxyB.Set(*fB->GetShape(), indexB);
		input.transformA = bA->GetTransform();
		input.transformB = bB->GetTransform();
		input.useRadii = false;

		SimplexCache cache;
		const auto output = Distance(cache, input);
		if (output.distance == 0 || cache.GetCount() == 3)
		{
			;;
		}
	}
#endif

	// Leap of faith to new safe state.
	m_bodies[toiIndexA]->m_sweep.pos0 = m_positions[toiIndexA];
	m_bodies[toiIndexB]->m_sweep.pos0 = m_positions[toiIndexB];

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	contactSolver.InitializeVelocityConstraints();

	// Solve velocity constraints.
	for (auto i = decltype(subStep.velocityIterations){0}; i < subStep.velocityIterations; ++i)
	{
		contactSolver.SolveVelocityConstraints();
	}

	// Don't store the TOI contact forces for warm starting
	// because they can be quite large.

	const auto h = subStep.get_dt();

	// Integrate positions
	for (auto i = decltype(m_bodyCount){0}; i < m_bodyCount; ++i)
	{
		auto velocity = m_velocities[i];
		auto translation = h * velocity.v;
		auto rotation = h * velocity.w;
		
		if (translation.LengthSquared() > Square(MaxTranslation))
		{
			const auto ratio = MaxTranslation / translation.Length();
			velocity.v *= ratio;
			translation = h * velocity.v;
		}

		if (Abs(rotation) > MaxRotation)
		{
			const auto ratio = MaxRotation / Abs(rotation);
			velocity.w *= ratio;
			rotation = h * velocity.w;
		}

		m_velocities[i] = velocity;
		m_positions[i] = Position{m_positions[i].c + translation, m_positions[i].a + rotation};

		// Sync bodies
		auto& body = *m_bodies[i];
		body.m_sweep.pos1 = m_positions[i];
		body.m_velocity = velocity;
		body.m_xf = GetTransformOne(body.m_sweep);
	}

	Report(contactSolver.GetVelocityConstraints());
}

void Island::Add(Body* body)
{
	assert(body->m_islandIndex == Body::InvalidIslandIndex);
	assert(m_bodyCount < m_bodyCapacity);
	body->m_islandIndex = m_bodyCount;
	m_bodies[m_bodyCount] = body;
	++m_bodyCount;
}

void Island::Add(Contact* contact)
{
	assert(m_contactCount < m_contactCapacity);
	m_contacts[m_contactCount] = contact;
	++m_contactCount;
}

void Island::Add(Joint* joint)
{
	assert(m_jointCount < m_jointCapacity);
	m_joints[m_jointCount] = joint;
	++m_jointCount;
}

void Island::Report(const ContactVelocityConstraint* constraints)
{
	if (!m_listener)
		return;

	for (auto i = decltype(m_contactCount){0}; i < m_contactCount; ++i)
	{
		ContactImpulse impulse;
		const auto& vc = constraints[i];
		const auto count = vc.GetPointCount();
		for (auto j = decltype(count){0}; j < count; ++j)
		{
			const auto point = vc.GetPoint(j);
			impulse.AddEntry(point.normalImpulse, point.tangentImpulse);
		}
		m_listener->PostSolve(m_contacts[i], &impulse);
	}
}
