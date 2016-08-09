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

#include <Box2D/Collision/Distance.h>
#include <Box2D/Dynamics/Island.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Contacts/ContactSolver.h>
#include <Box2D/Dynamics/Joints/Joint.h>
#include <Box2D/Common/StackAllocator.h>

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

using VelocityContainer = AllocatedArray<Velocity, StackAllocator&>;
using PositionContainer = AllocatedArray<Position, StackAllocator&>;
using PositionConstraintsContainer = AllocatedArray<ContactPositionConstraint, StackAllocator&>;
using VelocityConstraintsContainer = AllocatedArray<ContactVelocityConstraint, StackAllocator&>;

namespace {

	/// Calculates movement.
	/// @detail Calculate the positional displacement based on the given velocity
	///    that's possibly clamped to the maximum translation and rotation.
	inline Position CalculateMovement(Velocity& velocity, float_t h)
	{
		assert(IsValid(velocity));
		assert(IsValid(h));

		auto translation = h * velocity.v;
		if (LengthSquared(translation) > Square(MaxTranslation))
		{
			const auto ratio = MaxTranslation / Sqrt(LengthSquared(translation));
			velocity.v *= ratio;
			translation = h * velocity.v;
		}
		
		auto rotation = h * velocity.w;
		if (Abs(rotation) > MaxRotation)
		{
			const auto ratio = MaxRotation / Abs(rotation);
			velocity.w *= ratio;
			rotation = h * velocity.w;
		}
		
		return Position{translation, rotation};
	}

	inline void IntegratePositions(PositionContainer& positions, VelocityContainer& velocities, float_t h)
	{
		auto i = size_t{0};
		for (auto&& velocity: velocities)
		{
			positions[i] += CalculateMovement(velocity, h);
			++i;
		}
	}

	inline ContactImpulse GetContactImpulse(const ContactVelocityConstraint& vc)
	{
		ContactImpulse impulse;
		const auto count = vc.GetPointCount();
		for (auto j = decltype(count){0}; j < count; ++j)
		{
			const auto point = vc.GetPoint(j);
			impulse.AddEntry(point.normalImpulse, point.tangentImpulse);
		}
		return impulse;
	}

	/// Reports the given constraints to the listener.
	/// @detail
	/// This calls the listener's PostSolve method for all contacts.size() elements of
	/// the given array of constraints.
	/// @param listener Listener to call.
	/// @param constraints Array of m_contactCount contact velocity constraint elements.
	inline void Report(ContactListener& listener,
					   Island::ContactContainer& contacts,
					   const ContactVelocityConstraint* constraints,
					   TimeStep::iteration_type solved)
	{
		const auto size = contacts.size();
		for (auto i = decltype(size){0}; i < size; ++i)
		{
			listener.PostSolve(*contacts[i], GetContactImpulse(constraints[i]), solved);
		}
	}
	
	inline ContactVelocityConstraint::BodyData GetVelocityConstraintBodyData(const Body& val)
	{
		assert(IsValidIslandIndex(val));
		return ContactVelocityConstraint::BodyData{val.GetIslandIndex(), val.GetInverseMass(), val.GetInverseInertia()};
	}
	
	inline ContactPositionConstraint::BodyData GetPositionConstraintBodyData(const Body& val)
	{
		assert(IsValidIslandIndex(val));
		return ContactPositionConstraint::BodyData{val.GetIslandIndex(), val.GetInverseMass(), val.GetInverseInertia(), val.GetLocalCenter()};
	}

	/// Gets the position-independent velocity constraint for the given contact, index, and time slot values.
	inline ContactVelocityConstraint GetVelocityConstraint(const Contact& contact, ContactVelocityConstraint::index_type index, float_t dtRatio)
	{
		ContactVelocityConstraint constraint(index, contact.GetFriction(), contact.GetRestitution(), contact.GetTangentSpeed());
		
		constraint.normal = Vec2_zero;
		
		constraint.bodyA = GetVelocityConstraintBodyData(*(contact.GetFixtureA()->GetBody()));
		constraint.bodyB = GetVelocityConstraintBodyData(*(contact.GetFixtureB()->GetBody()));
		
		const auto& manifold = contact.GetManifold();
		const auto pointCount = manifold.GetPointCount();
		assert(pointCount > 0);
		for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
		{
			VelocityConstraintPoint vcp;
			
			const auto& mp = manifold.GetPoint(j);
			vcp.normalImpulse = dtRatio * mp.normalImpulse;
			vcp.tangentImpulse = dtRatio * mp.tangentImpulse;
			vcp.rA = Vec2_zero;
			vcp.rB = Vec2_zero;
			vcp.normalMass = float_t{0};
			vcp.tangentMass = float_t{0};
			vcp.velocityBias = float_t{0};
			
			constraint.AddPoint(vcp);
		}
		
		return constraint;
	}

	inline ContactPositionConstraint GetPositionConstraint(const Manifold& manifold, const Fixture& fixtureA, const Fixture& fixtureB)
	{
		return ContactPositionConstraint{manifold,
			GetPositionConstraintBodyData(*(fixtureA.GetBody())), fixtureA.GetShape()->GetRadius(),
			GetPositionConstraintBodyData(*(fixtureB.GetBody())), fixtureB.GetShape()->GetRadius()};
	}

	inline void InitPositionConstraints(ContactPositionConstraint* constraints,
										contact_count_t count, Contact** contacts)
	{
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto& contact = *contacts[i];
			constraints[i] = GetPositionConstraint(contact.GetManifold(), *contact.GetFixtureA(), *contact.GetFixtureB());
		}
	}
	
	inline void InitVelocityConstraints(ContactVelocityConstraint* constraints,
										contact_count_t count, Contact** contacts, float_t dtRatio)
	{
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			constraints[i] = GetVelocityConstraint(*contacts[i], i, dtRatio);
		}
	}
	
	inline void AssignImpulses(Manifold::Point& var, const VelocityConstraintPoint& val)
	{
		var.normalImpulse = val.normalImpulse;
		var.tangentImpulse = val.tangentImpulse;
	}
	
	/// Stores impulses.
	/// @detail Saves the normal and tangent impulses of all the velocity constraint points back to their
	///   associated contacts' manifold points.
	inline void StoreImpulses(size_t count, const ContactVelocityConstraint* velocityConstraints, Contact** contacts)
	{
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto& vc = velocityConstraints[i];
			auto& manifold = contacts[vc.GetContactIndex()]->GetManifold();
			
			const auto point_count = vc.GetPointCount();
			for (auto j = decltype(point_count){0}; j < point_count; ++j)
			{
				AssignImpulses(manifold.GetPoint(j), vc.GetPoint(j));
			}
		}
	}
};

Island::Island(body_count_t bodyCapacity, contact_count_t contactCapacity, island_count_t jointCapacity,
			   StackAllocator& allocator):
	m_bodies{bodyCapacity, allocator.AllocateArray<Body*>(bodyCapacity), allocator},
	m_contacts{contactCapacity, allocator.AllocateArray<Contact*>(contactCapacity), allocator},
	m_joints{jointCapacity, allocator.AllocateArray<Joint*>(jointCapacity), allocator}
{
}

void Island::CopyOut(const Position* positions, const Velocity* velocities, BodyContainer& bodies)
{
	// Copy velocity and position array data back out to the bodies
	auto i = size_t{0};
	for (auto&& body: bodies)
	{
		body->m_velocity = velocities[i];
		body->m_sweep.pos1 = positions[i];
		body->m_xf = GetTransform1(body->m_sweep);
		++i;
	}
}

bool Island::Solve(const TimeStep& step, ContactListener* listener, StackAllocator& allocator)
{
	// Would be nice to actually allocate this data on the actual stack but the running thread may not have nearly enough stack space for this.
	auto positionConstraints = PositionConstraintsContainer{m_contacts.size(), allocator.AllocateArray<ContactPositionConstraint>(m_contacts.size()), allocator};
	auto velocityConstraints = VelocityConstraintsContainer{m_contacts.size(), allocator.AllocateArray<ContactVelocityConstraint>(m_contacts.size()), allocator};
	InitPositionConstraints(positionConstraints.data(), static_cast<contact_count_t>(m_contacts.size()), m_contacts.data());
	InitVelocityConstraints(velocityConstraints.data(), static_cast<contact_count_t>(m_contacts.size()), m_contacts.data(),
							step.warmStarting? step.dtRatio: float_t{0});
	auto velocities = VelocityContainer{m_bodies.size(), allocator.AllocateArray<Velocity>(m_bodies.size()), allocator};
	auto positions = PositionContainer{m_bodies.size(), allocator.AllocateArray<Position>(m_bodies.size()), allocator};

	const auto h = step.get_dt(); ///< Time step (in seconds).

	// Update bodies' pos0 values then copy their pos1 and velocity data into local arrays.
	for (auto&& body: m_bodies)
	{
		body->m_sweep.pos0 = body->m_sweep.pos1; // like Advance0(1) on the sweep.
		positions.push_back(body->m_sweep.pos1);
		const auto new_velocity = GetVelocity(*body, h);
		assert(IsValid(new_velocity));
		velocities.push_back(new_velocity);
	}

	const auto solverData = SolverData{step, positions.data(), velocities.data()};

	ContactSolver contactSolver{positions.data(), velocities.data(),
		static_cast<contact_count_t>(m_contacts.size()),
		positionConstraints.data(), velocityConstraints.data()
	};
	contactSolver.UpdateVelocityConstraints();

	if (step.warmStarting)
	{
		contactSolver.WarmStart();
	}
	
	for (auto&& joint: m_joints)
	{
		joint->InitVelocityConstraints(solverData);
	}

	for (auto i = decltype(step.velocityIterations){0}; i < step.velocityIterations; ++i)
	{
		for (auto&& joint: m_joints)
		{
			joint->SolveVelocityConstraints(solverData);
		}
		contactSolver.SolveVelocityConstraints();
	}

	// updates array of tentative new body positions per the velocities as if there were no obstacles...
	IntegratePositions(positions, velocities, h);

	// Solve position constraints
	auto positionConstraintsSolved = TimeStep::InvalidIteration;
	for (auto i = decltype(step.positionIterations){0}; i < step.positionIterations; ++i)
	{
		const auto contactsOkay = contactSolver.SolvePositionConstraints();
		const auto jointsOkay = [&]()
		{
			auto allOkay = true;
			for (auto&& joint: m_joints)
			{
				const auto okay = joint->SolvePositionConstraints(solverData);
				allOkay = allOkay && okay;
			}
			return allOkay;
		}();

		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			positionConstraintsSolved = i;
			break;
		}
	}

	// Update normal and tangent impulses of contacts' manifold points
	StoreImpulses(m_contacts.size(), velocityConstraints.data(), m_contacts.data());

	// Updates m_bodies[i].m_sweep.pos1 to positions[i]
	CopyOut(positions.data(), velocities.data(), m_bodies);

	if (listener)
	{
		Report(*listener, m_contacts, velocityConstraints.data(), positionConstraintsSolved);
	}

	return positionConstraintsSolved != TimeStep::InvalidIteration;
}

bool Island::SolveTOI(const TimeStep& step, ContactListener* listener, StackAllocator& allocator, island_count_t indexA, island_count_t indexB)
{
	assert(indexA < m_bodies.size());
	assert(indexB < m_bodies.size());

	auto velocities = VelocityContainer{m_bodies.size(), allocator.AllocateArray<Velocity>(m_bodies.size()), allocator};
	auto positions = PositionContainer{m_bodies.size(), allocator.AllocateArray<Position>(m_bodies.size()), allocator};
	auto positionConstraints = PositionConstraintsContainer{m_contacts.size(), allocator.AllocateArray<ContactPositionConstraint>(m_contacts.size()), allocator};
	auto velocityConstraints = VelocityConstraintsContainer{m_contacts.size(), allocator.AllocateArray<ContactVelocityConstraint>(m_contacts.size()), allocator};
	InitPositionConstraints(positionConstraints.data(), static_cast<contact_count_t>(m_contacts.size()), m_contacts.data());
	InitVelocityConstraints(velocityConstraints.data(), static_cast<contact_count_t>(m_contacts.size()), m_contacts.data(),
							step.warmStarting? step.dtRatio: float_t{0});

	// Initialize the body state.
	for (auto&& body: m_bodies)
	{
		positions.push_back(body->m_sweep.pos1);
		velocities.push_back(body->GetVelocity());
	}

	ContactSolver contactSolver{positions.data(), velocities.data(),
		static_cast<contact_count_t>(m_contacts.size()),
		positionConstraints.data(), velocityConstraints.data()
	};

	// Solve TOI-based position constraints.
	auto positionConstraintsSolved = TimeStep::InvalidIteration;
	for (auto i = decltype(step.positionIterations){0}; i < step.positionIterations; ++i)
	{
		if (contactSolver.SolveTOIPositionConstraints(indexA, indexB))
		{
			positionConstraintsSolved = i;
			break;
		}
	}

	// Leap of faith to new safe state.
	m_bodies[indexA]->m_sweep.pos0 = positions[indexA];
	m_bodies[indexB]->m_sweep.pos0 = positions[indexB];	
	
	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	contactSolver.UpdateVelocityConstraints();

	// Solve velocity constraints.
	for (auto i = decltype(step.velocityIterations){0}; i < step.velocityIterations; ++i)
	{
		contactSolver.SolveVelocityConstraints();
	}

	// Don't store TOI contact forces for warm starting because they can be quite large.

	IntegratePositions(positions, velocities, step.get_dt());

	// Update m_bodies[i].m_sweep.pos1 to position[i]
	CopyOut(positions.data(), velocities.data(), m_bodies);

	if (listener)
	{
		Report(*listener, m_contacts, velocityConstraints.data(), positionConstraintsSolved);
	}
	
	return positionConstraintsSolved != TimeStep::InvalidIteration;
}
