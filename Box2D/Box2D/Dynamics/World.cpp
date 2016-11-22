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

#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/TimeStep.h>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Island.h>
#include <Box2D/Dynamics/Joints/PulleyJoint.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Collision/BroadPhase.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/TimeOfImpact.hpp>
#include <Box2D/Collision/RayCastOutput.hpp>
#include <Box2D/Common/Timer.hpp>
#include <Box2D/Common/AllocatedArray.hpp>

#include <Box2D/Dynamics/Contacts/ContactSolver.h>
#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>
#include <Box2D/Dynamics/Contacts/PositionConstraint.hpp>

#include <new>
#include <functional>
#include <type_traits>
#include <memory>

namespace box2d
{

using VelocityContainer = AllocatedArray<Velocity, StackAllocator&>;
using PositionContainer = AllocatedArray<Position, StackAllocator&>;
using PositionConstraintsContainer = AllocatedArray<PositionConstraint, StackAllocator&>;
using VelocityConstraintsContainer = AllocatedArray<VelocityConstraint, StackAllocator&>;

struct MovementConf
{
	float_t maxTranslation;
	Angle maxRotation;
};

template <typename T>
class FlagGuard
{
public:
	FlagGuard(T& flag, T value) : m_flag(flag), m_value(value)
	{
		static_assert(std::is_unsigned<T>::value, "Unsigned integer required");
		m_flag |= m_value;
	}

	~FlagGuard() noexcept
	{
		m_flag &= ~m_value;
	}

	FlagGuard() = delete;

private:
	T& m_flag;
	T m_value;
};

template <class T>
class RaiiWrapper
{
public:
	RaiiWrapper() = delete;
	RaiiWrapper(std::function<void(T&)> on_destruction): m_on_destruction(on_destruction) {}
	~RaiiWrapper() { m_on_destruction(m_wrapped); }
	T m_wrapped;

private:
	std::function<void(T&)> m_on_destruction;
};

World::World(const Def def):
	m_gravity(def.gravity),
	m_linearSlop(def.linearSlop),
	m_angularSlop(def.angularSlop),
	m_maxLinearCorrection(def.maxLinearCorrection),
	m_maxAngularCorrection(def.maxAngularCorrection),
	m_maxTranslation(def.maxTranslation),
	m_maxRotation(def.maxRotation)
{
	memset(&m_profile, 0, sizeof(Profile));
}

World::~World()
{
	// Some shapes allocate using alloc.
	while (!m_bodies.empty())
	{
		auto&& b = m_bodies.front();
		Destroy(&b);
	}
}

void World::SetDestructionListener(DestructionListener* listener) noexcept
{
	m_destructionListener = listener;
}

void World::SetContactFilter(ContactFilter* filter) noexcept
{
	m_contactMgr.m_contactFilter = filter;
}

void World::SetContactListener(ContactListener* listener) noexcept
{
	m_contactMgr.m_contactListener = listener;
}

void World::SetGravity(const Vec2& gravity) noexcept
{
	if (m_gravity != gravity)
	{
		const auto diff = gravity - m_gravity;
		for (auto&& body: m_bodies)
		{
			ApplyLinearAcceleration(body, diff);
		}
		m_gravity = gravity;
	}
}

Body* World::CreateBody(const BodyDef& def)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(Body));
	auto b = new (mem) Body(def, this);
	if (b)
	{
		if (!Add(*b))
		{
			b->~Body();
			m_blockAllocator.Free(b, sizeof(Body));
			return nullptr;
		}		
	}

	b->SetAcceleration(m_gravity, 0_rad);
	return b;
}

bool World::Add(Body& b)
{
	assert(!b.m_prev);
	assert(!b.m_next);

	if (m_bodies.size() >= MaxBodies)
	{
		return false;
	}
	
	// Add to world doubly linked list.
	m_bodies.push_front(&b);
	return true;
}

bool World::Remove(Body& b)
{
	assert(!m_bodies.empty());
	if (m_bodies.empty())
	{
		return false;
	}

	m_bodies.erase(BodyIterator{&b});
	return true;
}

void World::Destroy(Body* b)
{
	assert(b->m_world == this);
	
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	Remove(*b);
	
	b->~Body();
	m_blockAllocator.Free(b, sizeof(*b));
}

Joint* World::CreateJoint(const JointDef& def)
{
	if (m_joints.size() >= m_joints.max_size())
	{
		return nullptr;
	}

	assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}

	// Note: creating a joint doesn't wake the bodies.
	auto j = Joint::Create(def, m_blockAllocator);

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = j->m_bodyA->m_joints.p;
	if (j->m_bodyA->m_joints.p)
	{
		j->m_bodyA->m_joints.p->prev = &j->m_edgeA;
	}
	j->m_bodyA->m_joints.p = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = j->m_bodyB->m_joints.p;
	if (j->m_bodyB->m_joints.p)
	{
		j->m_bodyB->m_joints.p->prev = &j->m_edgeB;
	}
	j->m_bodyB->m_joints.p = &j->m_edgeB;

	auto bodyA = def.bodyA;
	auto bodyB = def.bodyB;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!def.collideConnected)
	{
		for (auto&& edge: bodyB->GetContactEdges())
		{
			if (edge.other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.contact->FlagForFiltering();
			}
		}
	}

	Add(*j);
	
	return j;
}

bool World::Add(Joint& j)
{
	m_joints.push_front(&j);
	return true;
}

bool World::Remove(Joint& j)
{
	const auto it = JointIterator{&j};
	return m_joints.erase(it) != it;
}

void World::Destroy(Joint* j)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	if (!Remove(*j))
	{
		return;
	}

	const auto collideConnected = j->m_collideConnected;
	
	// Disconnect from island graph.
	auto bodyA = j->m_bodyA;
	auto bodyB = j->m_bodyB;

	// Wake up connected bodies.
	bodyA->SetAwake();
	bodyB->SetAwake();

	// Remove from body 1.
	if (j->m_edgeA.prev)
	{
		j->m_edgeA.prev->next = j->m_edgeA.next;
	}

	if (j->m_edgeA.next)
	{
		j->m_edgeA.next->prev = j->m_edgeA.prev;
	}

	if (&j->m_edgeA == bodyA->m_joints.p)
	{
		bodyA->m_joints.p = j->m_edgeA.next;
	}

	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = nullptr;

	// Remove from body 2
	if (j->m_edgeB.prev)
	{
		j->m_edgeB.prev->next = j->m_edgeB.next;
	}

	if (j->m_edgeB.next)
	{
		j->m_edgeB.next->prev = j->m_edgeB.prev;
	}

	if (&j->m_edgeB == bodyB->m_joints.p)
	{
		bodyB->m_joints.p = j->m_edgeB.next;
	}

	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = nullptr;

	Joint::Destroy(j, m_blockAllocator);

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!collideConnected)
	{
		for (auto&& edge: bodyB->GetContactEdges())
		{
			if (edge.other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.contact->FlagForFiltering();
			}
		}
	}
}

//
void World::SetAllowSleeping(bool flag) noexcept
{
	if (flag == GetAllowSleeping())
	{
		return;
	}

	if (flag)
	{
		SetAllowSleeping();
	}
	else // !flag
	{
		UnsetAllowSleeping();
		for (auto&& b: m_bodies)
		{
			b.SetAwake();
		}
	}
}

static inline float_t UpdateSleepTimes(Island::BodyContainer& bodies, float_t h)
{
	auto minSleepTime = MaxFloat;
	for (auto&& b: bodies)
	{
		if (b->IsSpeedable())
		{
			minSleepTime = Min(minSleepTime, b->UpdateSleepTime(h));
		}
	}
	return minSleepTime;
}

static inline void Sleepem(Island::BodyContainer& bodies)
{
	for (auto&& b: bodies)
	{
		b->UnsetAwake();
	}
}
	
body_count_t World::AddToIsland(Island& island, Body& body)
{
	const auto index = static_cast<body_count_t>(island.m_bodies.size());
	body.m_islandIndex = index;
	island.m_bodies.push_back(&body);
	return index;
}

Island World::BuildIsland(Body& seed,
				  BodyList::size_type& remNumBodies,
				  contact_count_t& remNumContacts,
				  JointList::size_type& remNumJoints)
{
	assert(remNumBodies != 0);

	// Size the island for the remaining un-evaluated bodies, contacts, and joints.
	Island island(remNumBodies, remNumContacts, remNumJoints, m_stackAllocator);

	// Perform a depth first search (DFS) on the constraint graph.
	AllocatedArray<Body*, StackAllocator&> stack(remNumBodies, m_stackAllocator.AllocateArray<Body*>(remNumBodies), m_stackAllocator);
	stack.push_back(&seed);
	seed.SetInIsland();
	while (!stack.empty())
	{
		// Grab the next body off the stack and add it to the island.
		const auto b = stack.back();
		stack.pop_back();
		
		assert(b->IsActive());
		AddToIsland(island, *b);
		--remNumBodies;
		
		// Make sure the body is awake.
		b->SetAwake();
		
		// To keep islands smaller, don't propagate islands across bodies that can't have a velocity (static bodies).
		if (!b->IsSpeedable())
		{
			continue;
		}
		
		const auto numContacts = island.m_contacts.size();
		// Adds appropriate contacts of current body and appropriate 'other' bodies of those contacts.
		for (auto&& ce: b->GetContactEdges())
		{
			const auto contact = ce.contact;
			if (!contact->IsInIsland() && contact->IsEnabled() && contact->IsTouching() && !HasSensor(*contact))
			{
				island.m_contacts.push_back(contact);
				contact->SetInIsland();
				const auto other = ce.other;
				if (!other->IsInIsland())
				{				
					stack.push_back(other);
					other->SetInIsland();
				}
			}			
		}
		remNumContacts -= island.m_contacts.size() - numContacts;
		
		const auto numJoints = island.m_joints.size();
		// Adds appropriate joints of current body and appropriate 'other' bodies of those joint.
		for (auto&& je: b->m_joints)
		{
			const auto joint = je.joint;
			const auto other = je.other;
			if (!joint->IsInIsland() && other->IsActive())
			{
				island.m_joints.push_back(joint);
				joint->SetInIsland(true);
				if (!other->IsInIsland())
				{					
					stack.push_back(other);
					other->SetInIsland();
				}
			}
		}
		remNumJoints -= island.m_joints.size() - numJoints;
	}
	
	return island;
}
	
void World::Solve(const TimeStep& step)
{
	// Clear all the island flags.
	for (auto&& b: m_bodies)
	{
		b.UnsetInIsland();
	}
	for (auto&& c: m_contactMgr.GetContacts())
	{
		c.UnsetInIsland();
	}
	for (auto&& j: m_joints)
	{
		j.SetInIsland(false);
	}

	{
		auto remNumBodies = m_bodies.size(); ///< Remaining number of bodies.
		auto remNumContacts = m_contactMgr.GetContacts().size(); ///< Remaining number of contacts.
		auto remNumJoints = m_joints.size(); ///< Remaining number of joints.
		
		// Build and simulate all awake islands.
		for (auto&& body: m_bodies)
		{
			if (!body.IsInIsland() && body.IsSpeedable() && body.IsAwake() && body.IsActive())
			{
				auto island = BuildIsland(body, remNumBodies, remNumContacts, remNumJoints);
				
				// Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
				const auto constraintsSolved = Solve(step, island);
				
				if (GetAllowSleeping())
				{
					const auto minSleepTime = UpdateSleepTimes(island.m_bodies, step.get_dt());
					if ((minSleepTime >= MinStillTimeToSleep) && constraintsSolved)
					{
						Sleepem(island.m_bodies);
					}
				}
				
				for (auto&& b: island.m_bodies)
				{
					// Allow static bodies to participate in other islands.
					if (!b->IsSpeedable())
					{
						b->UnsetInIsland();
						++remNumBodies;
					}
				}
			}		
		}
	}

	for (auto&& b: m_bodies)
	{
		// A non-static body that was in an island may have moved.
		if ((b.m_flags & (Body::e_velocityFlag|Body::e_islandFlag)) == (Body::e_velocityFlag|Body::e_islandFlag))
		{
			// Update fixtures (for broad-phase).
			b.SynchronizeFixtures();
		}
	}

	// Look for new contacts.
	m_contactMgr.FindNewContacts();
}

	// ==== begin
	
	namespace {
		
		/// Calculates movement.
		/// @detail Calculate the positional displacement based on the given velocity
		///    that's possibly clamped to the maximum translation and rotation.
		inline Position CalculateMovement(Velocity& velocity, float_t h, MovementConf conf)
		{
			assert(IsValid(velocity));
			assert(IsValid(h));
			
			auto translation = h * velocity.v;
			if (GetLengthSquared(translation) > Square(conf.maxTranslation))
			{
				const auto ratio = conf.maxTranslation / Sqrt(GetLengthSquared(translation));
				velocity.v *= ratio;
				translation = h * velocity.v;
			}
			
			auto rotation = h * velocity.w;
			if (Abs(rotation) > conf.maxRotation)
			{
				const auto ratio = conf.maxRotation / Abs(rotation);
				velocity.w *= ratio;
				rotation = h * velocity.w;
			}
			
			return Position{translation, rotation};
		}
		
		inline void IntegratePositions(PositionContainer& positions, VelocityContainer& velocities,
									   float_t h, MovementConf conf)
		{
			auto i = size_t{0};
			for (auto&& velocity: velocities)
			{
				positions[i] += CalculateMovement(velocity, h, conf);
				++i;
			}
		}
		
		inline ContactImpulse GetContactImpulse(const VelocityConstraint& vc)
		{
			ContactImpulse impulse;
			const auto count = vc.GetPointCount();
			for (auto j = decltype(count){0}; j < count; ++j)
			{
				impulse.AddEntry(GetNormalImpulseAtPoint(vc, j), GetTangentImpulseAtPoint(vc, j));
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
						   Span<Contact*> contacts,
						   Span<const VelocityConstraint> constraints,
						   TimeStep::iteration_type solved)
		{
			const auto size = contacts.size();
			for (auto i = decltype(size){0}; i < size; ++i)
			{
				listener.PostSolve(*contacts[i], GetContactImpulse(constraints[i]), solved);
			}
		}
		
		inline VelocityConstraint::BodyData GetVelocityConstraintBodyData(const Body& val)
		{
			assert(IsValidIslandIndex(val));
			return VelocityConstraint::BodyData{val.GetIslandIndex(), val.GetInverseMass(), val.GetInverseInertia()};
		}
		
		inline PositionConstraint::BodyData GetPositionConstraintBodyData(const Body& val)
		{
			assert(IsValidIslandIndex(val));
			return PositionConstraint::BodyData{val.GetIslandIndex(), val.GetInverseMass(), val.GetInverseInertia(), val.GetLocalCenter()};
		}
		
		/// Gets the position-independent velocity constraint for the given contact, index, and time slot values.
		inline VelocityConstraint GetVelocityConstraint(const Contact& contact, VelocityConstraint::index_type index, float_t dtRatio)
		{
			VelocityConstraint constraint(index,
										  contact.GetFriction(),
										  contact.GetRestitution(),
										  contact.GetTangentSpeed(),
										  GetVelocityConstraintBodyData(*(contact.GetFixtureA()->GetBody())),
										  GetVelocityConstraintBodyData(*(contact.GetFixtureB()->GetBody())));
		
			const auto& manifold = contact.GetManifold();
			const auto pointCount = manifold.GetPointCount();
			assert(pointCount > 0);
			for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
			{
				const auto& mp = manifold.GetPoint(j);
				constraint.AddPoint(dtRatio * mp.normalImpulse, dtRatio * mp.tangentImpulse);
			}
			
			return constraint;
		}
		
		inline PositionConstraint GetPositionConstraint(const Manifold& manifold, const Fixture& fixtureA, const Fixture& fixtureB)
		{
			return PositionConstraint{manifold,
				GetPositionConstraintBodyData(*(fixtureA.GetBody())), GetVertexRadius(*fixtureA.GetShape()),
				GetPositionConstraintBodyData(*(fixtureB.GetBody())), GetVertexRadius(*fixtureB.GetShape())};
		}
		
		inline void InitPosConstraints(PositionConstraintsContainer& constraints, const Island::ContactContainer& contacts)
		{
			for (auto&& contact: contacts)
			{
				constraints.push_back(GetPositionConstraint(contact->GetManifold(),
															*(contact->GetFixtureA()),
															*(contact->GetFixtureB())));
			}
		}
		
		inline void InitVelConstraints(VelocityConstraintsContainer& constraints,
									   const Island::ContactContainer& contacts, float_t dtRatio)
		{
			auto i = VelocityConstraint::index_type{0};
			for (auto&& contact: contacts)
			{
				constraints.push_back(GetVelocityConstraint(*contact, i, dtRatio));
				++i;
			}
		}
		
		inline void AssignImpulses(Manifold& var, const VelocityConstraint& vc)
		{
			assert(var.GetPointCount() >= vc.GetPointCount());

			const auto count = vc.GetPointCount();
			for (auto i = decltype(count){0}; i < count; ++i)
			{
				auto&& mp = var.GetPoint(i);
				mp.normalImpulse = GetNormalImpulseAtPoint(vc, i);
				mp.tangentImpulse = GetTangentImpulseAtPoint(vc, i);
			}
		}
		
		/// Stores impulses.
		/// @detail Saves the normal and tangent impulses of all the velocity constraint points back to their
		///   associated contacts' manifold points.
		inline void StoreImpulses(Span<const VelocityConstraint> velocityConstraints, Span<Contact*> contacts)
		{
			for (auto&& vc: velocityConstraints)
			{
				auto& manifold = contacts[vc.GetContactIndex()]->GetManifold();
				AssignImpulses(manifold, vc);
			}
		}
		
		struct VelocityPair
		{
			Velocity a;
			Velocity b;
		};
		
		inline VelocityPair CalcWarmStartVelocityDeltas(const VelocityConstraint& vc)
		{
			VelocityPair vp{Velocity{Vec2_zero, 0_rad}, Velocity{Vec2_zero, 0_rad}};
			
			const auto normal = GetNormal(vc);
			const auto tangent = GetTangent(vc);
			if (IsValid(normal) && IsValid(tangent))
			{
				const auto pointCount = vc.GetPointCount();	
				for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
				{
					const auto P = GetNormalImpulseAtPoint(vc, j) * normal + GetTangentImpulseAtPoint(vc, j) * tangent;
					vp.a -= Velocity{
						vc.bodyA.GetInvMass() * P,
						1_rad * vc.bodyA.GetInvRotI() * Cross(GetPointRelPosA(vc, j), P)
					};
					vp.b += Velocity{
						vc.bodyB.GetInvMass() * P,
						1_rad * vc.bodyB.GetInvRotI() * Cross(GetPointRelPosB(vc, j), P)
					};
				}
			}
			return vp;
		}
		
		inline void WarmStartVelocities(Span<const VelocityConstraint> velocityConstraints, Span<Velocity> velocities)
		{
			for (auto&& vc: velocityConstraints)
			{
				const auto vp = CalcWarmStartVelocityDeltas(vc);
				velocities[vc.bodyA.GetIndex()] += vp.a;
				velocities[vc.bodyB.GetIndex()] += vp.b;
			}		
		}
		
		/// Updates the given velocity constraints.
		/// @detail
		/// Updates the position dependent portions of the velocity constraints with the
		/// information from the current position constraints.
		/// @note This MUST be called prior to calling <code>SolveVelocityConstraints</code>.
		/// @post Velocity constraints will have their "normal" field set to the world manifold normal for them.
		/// @post Velocity constraints will have their constraint points updated.
		/// @sa SolveVelocityConstraints.
		inline void UpdateVelocityConstraints(Span<VelocityConstraint> velocityConstraints,
											  Span<const Velocity> velocities,
											  Span<const PositionConstraint> positionConstraints,
											  Span<const Position> positions)
		{
			auto i = Span<VelocityConstraint>::size_type{0};
			for (auto&& pc: positionConstraints)
			{
				const auto posA = positions[pc.bodyA.index];
				const auto posB = positions[pc.bodyB.index];
				const auto worldManifold = GetWorldManifold(pc, posA, posB);
				velocityConstraints[i].Update(worldManifold, posA.c, posB.c, velocities, true);
				++i;
			}
		}
		
		/// "Solves" the velocity constraints.
		/// @detail Updates the velocities and velocity constraint points' normal and tangent impulses.
		/// @pre <code>UpdateVelocityConstraints</code> has been called on the velocity constraints.
		inline void SolveVelocityConstraints(Span<VelocityConstraint> velocityConstraints,
											 Span<Velocity> velocities)
		{
			for (auto&& vc: velocityConstraints)
			{
				SolveVelocityConstraint(vc,
										velocities[vc.bodyA.GetIndex()],
										velocities[vc.bodyB.GetIndex()]);
			}
		}
	};
	
	// ==== end
	
bool World::Solve(const TimeStep& step, Island& island)
{
	const auto contacts_count = static_cast<contact_count_t>(island.m_contacts.size());

	auto positionConstraints = PositionConstraintsContainer{
		contacts_count, m_stackAllocator.AllocateArray<PositionConstraint>(contacts_count), m_stackAllocator
	};
	InitPosConstraints(positionConstraints, island.m_contacts);

	auto velocityConstraints = VelocityConstraintsContainer{
		contacts_count, m_stackAllocator.AllocateArray<VelocityConstraint>(contacts_count), m_stackAllocator
	};
	InitVelConstraints(velocityConstraints, island.m_contacts,
					   step.warmStarting? step.dtRatio: float_t{0});
	
	auto velocities = VelocityContainer{
		island.m_bodies.size(),
		m_stackAllocator.AllocateArray<Velocity>(island.m_bodies.size()),
		m_stackAllocator
	};
	
	auto positions = PositionContainer{
		island.m_bodies.size(),
		m_stackAllocator.AllocateArray<Position>(island.m_bodies.size()),
		m_stackAllocator
	};
	
	const auto h = step.get_dt(); ///< Time step (in seconds).
	
	// Update bodies' pos0 values then copy their pos1 and velocity data into local arrays.
	for (auto&& body: island.m_bodies)
	{
		body->m_sweep.pos0 = body->m_sweep.pos1; // like Advance0(1) on the sweep.
		positions.push_back(body->m_sweep.pos1);
		const auto new_velocity = GetVelocity(*body, h);
		assert(IsValid(new_velocity));
		velocities.push_back(new_velocity);
	}
	
	UpdateVelocityConstraints(velocityConstraints, velocities, positionConstraints, positions);
	
	if (step.warmStarting)
	{
		WarmStartVelocities(velocityConstraints, velocities);
	}

	const auto psConf = ConstraintSolverConf{}
		.UseResolutionRate(Baumgarte)
		.UseLinearSlop(GetLinearSlop())
		.UseAngularSlop(GetAngularSlop())
		.UseMaxLinearCorrection(GetMaxLinearCorrection())
		.UseMaxAngularCorrection(GetMaxAngularCorrection());

	for (auto&& joint: island.m_joints)
	{
		joint->InitVelocityConstraints(velocities, positions, step, psConf);
	}
	
	for (auto i = decltype(step.velocityIterations){0}; i < step.velocityIterations; ++i)
	{
		for (auto&& joint: island.m_joints)
		{
			joint->SolveVelocityConstraints(velocities, step);
		}

		SolveVelocityConstraints(velocityConstraints, velocities);
	}
	
	// updates array of tentative new body positions per the velocities as if there were no obstacles...
	IntegratePositions(positions, velocities, h, MovementConf{m_maxTranslation, m_maxRotation});
	
	// Solve position constraints
	auto iterationSolved = TimeStep::InvalidIteration;
	for (auto i = decltype(step.positionIterations){0}; i < step.positionIterations; ++i)
	{
		const auto minSep = SolvePositionConstraints(positionConstraints, positions, psConf);
		const auto contactsOkay = (minSep >= -psConf.linearSlop * 3);

		const auto jointsOkay = [&]()
		{
			auto allOkay = true;
			for (auto&& joint: island.m_joints)
			{
				if (!joint->SolvePositionConstraints(positions, psConf))
				{
					allOkay = false;
				}
			}
			return allOkay;
		}();
		
		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			iterationSolved = i;
			break;
		}
	}
	
	// Update normal and tangent impulses of contacts' manifold points
	StoreImpulses(velocityConstraints, island.m_contacts);
	
	// Updates m_bodies[i].m_sweep.pos1 to positions[i]
	// Copy velocity and position array data back out to the bodies
	{
		auto i = size_t{0};
		for (auto&& b: island.m_bodies)
		{
			Update(*b, positions[i], velocities[i]);
			++i;
		}
	}

	if (m_contactMgr.m_contactListener)
	{
		Report(*m_contactMgr.m_contactListener, island.m_contacts, velocityConstraints,
			   iterationSolved);
	}
	
	return iterationSolved != TimeStep::InvalidIteration;
}

void World::ResetBodiesForSolveTOI()
{
	for (auto&& b: m_bodies)
	{
		b.UnsetInIsland();
		b.m_sweep.ResetAlpha0();
	}
}

void World::ResetContactsForSolveTOI()
{
	for (auto&& c: m_contactMgr.GetContacts())
	{
		// Invalidate TOI
		c.UnsetInIsland();
		c.UnsetToi();
		c.m_toiCount = 0;
	}	
}

World::ContactToiData World::UpdateContactTOIs()
{
	auto minContact = static_cast<Contact*>(nullptr);
	auto minToi = MaxFloat;
	
	const auto toiConf = ToiConf{
		1,
		GetLinearSlop() * 3, // Targetted depth of impact
		GetLinearSlop() / 4, // Tolerance.
		MaxTOIRootIterCount,
		MaxTOIIterations
	};

	auto count = contact_count_t{0};
	for (auto&& c: m_contactMgr.GetContacts())
	{
		if (c.IsEnabled() && (c.GetToiCount() < MaxSubSteps) &&
			(c.HasValidToi() || c.UpdateTOI(toiConf)))
		{
			const auto toi = c.GetToi();
			if (minToi > toi)
			{
				minToi = toi;
				minContact = &c;
				count = contact_count_t{1};
			}
			else if (minToi == toi)
			{
				// Have multiple contacts at the current minimum time of impact.
				++count;

				// Should the first one found be dealt with first?
				// Does ordering of these contacts even matter?
				//
				//   Presumably if these contacts are independent then ordering won't matter
				//   since they'd be dealt with in separate islands anyway.
				//   OTOH, if these contacts are dependent, then the contact returned will be
				//   first to have its two bodies positions handled for impact before other
				//   bodies which seems like it will matter.
				//
				//   Prioritizing contact with non-accelerable body doesn't prevent
				//   tunneling of bullet objects through static objects in the multi body
				//   collision case however.
#if 0
				if (!c.GetFixtureB()->GetBody()->IsAccelerable() ||
					!c.GetFixtureB()->GetBody()->IsAccelerable())
				{
					minContact = &c;
				}
#endif
			}
		}
	}

	return ContactToiData{count, minContact, minToi};
}

// Find TOI contacts and solve them.
void World::SolveTOI(const TimeStep& step)
{
	if (IsStepComplete())
	{
		ResetBodiesForSolveTOI();
		ResetContactsForSolveTOI();
	}

	// Find TOI events and solve them.
	for (;;)
	{
		// Find the first TOI - the soonest one.
		const auto minContactToi = UpdateContactTOIs();

		if ((!minContactToi.contact) || (minContactToi.toi >= 1))
		{
			// No more TOI events. Done!
			SetStepComplete(true);
			break;
		}

		SolveTOI(step, *minContactToi.contact);
		
		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		m_contactMgr.FindNewContacts();

		if (GetSubStepping())
		{
			SetStepComplete(false);
			break;
		}
	}
}

void World::SolveTOI(const TimeStep& step, Contact& contact)
{
	const auto toi = contact.GetToi();
	auto bA = contact.GetFixtureA()->GetBody();
	auto bB = contact.GetFixtureB()->GetBody();

	const auto backupA = bA->m_sweep;
	const auto backupB = bB->m_sweep;

	// Advance the bodies to the TOI.
	bA->Advance(toi);
	bB->Advance(toi);

	// The TOI contact likely has some new contact points.
	contact.Update(m_contactMgr.m_contactListener);
	contact.UnsetToi();

	++contact.m_toiCount;
	if (contact.m_toiCount >= MaxSubSteps)
	{
		// Note: This contact won't be passed again to this method within the current world step
		// (as the UpdateContactTOIs method won't return it anymore).
		// What are the pros/cons of this?
		// Larger MaxSubSteps slows down the simulation.
		// MaxSubSteps of 44 and higher seems to decrease the occurrance of tunneling of multiple
		// bullet body collisions with static objects.
	}

	// Is contact disabled or separated?
	if (!contact.IsEnabled() || !contact.IsTouching())
	{
		// Restore the sweeps by undoing the body "advance" calls (and anything else done movement-wise)
		contact.UnsetEnabled();
		bA->m_sweep = backupA;
		bA->m_xf = GetTransform1(bA->m_sweep);
		bB->m_sweep = backupB;
		bB->m_xf = GetTransform1(bB->m_sweep);
		return;
	}

	bA->SetAwake();
	bB->SetAwake();

	// Build the island
	Island island(m_bodies.size(), m_contactMgr.GetContacts().size(), 0, m_stackAllocator);

	AddToIsland(island, *bA);
	bA->SetInIsland();
	AddToIsland(island, *bB);
	bB->SetInIsland();
	island.m_contacts.push_back(&contact);
	contact.SetInIsland();

	// Process the contacts of the two bodies, adding appropriate ones to the island,
	// adding appropriate other bodies of added contacts, and advancing those other
	// bodies sweeps and transforms to the minimum contact's TOI.
	if (bA->IsAccelerable())
	{
		ProcessContactsForTOI(island, *bA, toi, m_contactMgr.m_contactListener);
	}
	if (bB->IsAccelerable())
	{
		ProcessContactsForTOI(island, *bB, toi, m_contactMgr.m_contactListener);
	}

	{
		TimeStep subStep;
		subStep.set_dt((float_t{1} - toi) * step.get_dt());
		subStep.dtRatio = float_t{1};
		subStep.positionIterations = step.positionIterations? MaxSubStepPositionIterations: 0;
		subStep.velocityIterations = step.velocityIterations;
		subStep.warmStarting = false;
		SolveTOI(subStep, island);
	}

	// Reset island flags and synchronize broad-phase proxies.
	for (auto&& body: island.m_bodies)
	{
		body->UnsetInIsland();

		if (body->IsAccelerable())
		{
			body->SynchronizeFixtures();
			ResetContactsForSolveTOI(*body);
		}
	}
}

void World::Update(Body& body, const Position pos, const Velocity vel)
{
	body.m_velocity = vel; // sets what Body::GetVelocity returns
	body.m_sweep.pos1 = pos; // sets what Body::GetWorldCenter returns
	body.m_xf = GetTransformation(body.m_sweep.pos1, body.m_sweep.GetLocalCenter()); // sets what Body::GetLocation returns
}

bool World::SolveTOI(const TimeStep& step, Island& island)
{
	assert(island.m_bodies.size() >= 2);
	const auto contacts_count = static_cast<contact_count_t>(island.m_contacts.size());
	
	auto velocities = VelocityContainer{
		island.m_bodies.size(),
		m_stackAllocator.AllocateArray<Velocity>(island.m_bodies.size()),
		m_stackAllocator
	};
	auto positions = PositionContainer{
		island.m_bodies.size(),
		m_stackAllocator.AllocateArray<Position>(island.m_bodies.size()),
		m_stackAllocator
	};
	auto positionConstraints = PositionConstraintsContainer{
		contacts_count,
		m_stackAllocator.AllocateArray<PositionConstraint>(contacts_count),
		m_stackAllocator
	};
	auto velocityConstraints = VelocityConstraintsContainer{
		contacts_count,
		m_stackAllocator.AllocateArray<VelocityConstraint>(contacts_count),
		m_stackAllocator
	};
	InitPosConstraints(positionConstraints, island.m_contacts);
	InitVelConstraints(velocityConstraints, island.m_contacts,
					   step.warmStarting? step.dtRatio: float_t{0});
	
	// Initialize the body state.
	for (auto&& body: island.m_bodies)
	{
		positions.push_back(body->m_sweep.pos1);
		velocities.push_back(body->GetVelocity());
	}
	
	// Solve TOI-based position constraints.
	auto positionConstraintsSolved = TimeStep::InvalidIteration;
	const auto psConf = ConstraintSolverConf{}
		.UseResolutionRate(ToiBaumgarte)
		.UseLinearSlop(GetLinearSlop())
		.UseAngularSlop(GetAngularSlop())
		.UseMaxLinearCorrection(GetMaxLinearCorrection())
		.UseMaxAngularCorrection(GetMaxAngularCorrection());

	for (auto i = decltype(step.positionIterations){0}; i < step.positionIterations; ++i)
	{
		const auto minSeparation = SolvePositionConstraints(positionConstraints, positions,
															0, 1, psConf);
		if (minSeparation >= -psConf.linearSlop * float_t(1.5))
		{
			positionConstraintsSolved = i;
			break;
		}
	}
	
	// Leap of faith to new safe state.
	island.m_bodies[0]->m_sweep.pos0 = positions[0];
	island.m_bodies[1]->m_sweep.pos0 = positions[1];	
	
	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	UpdateVelocityConstraints(velocityConstraints, velocities, positionConstraints, positions);
	
	// Solve velocity constraints.
	for (auto i = decltype(step.velocityIterations){0}; i < step.velocityIterations; ++i)
	{
		SolveVelocityConstraints(velocityConstraints, velocities);
	}
	
	// Don't store TOI contact forces for warm starting because they can be quite large.
	
	IntegratePositions(positions, velocities, step.get_dt(), MovementConf{m_maxTranslation, m_maxRotation});
	
	// Update m_bodies[i].m_sweep.pos1 to position[i]
	// Copy velocity and position array data back out to the bodies
	{
		auto i = size_t{0};
		for (auto&& b: island.m_bodies)
		{
			Update(*b, positions[i], velocities[i]);
			++i;
		}
	}

	if (m_contactMgr.m_contactListener)
	{
		Report(*m_contactMgr.m_contactListener, island.m_contacts, velocityConstraints,
			   positionConstraintsSolved);
	}
	
	return positionConstraintsSolved != TimeStep::InvalidIteration;
}
	
void World::ResetContactsForSolveTOI(Body& body)
{
	// Invalidate all contact TOIs on this displaced body.
	for (auto&& ce: body.GetContactEdges())
	{
		ce.contact->UnsetInIsland();
		ce.contact->UnsetToi();
	}
}

void World::ProcessContactsForTOI(Island& island, Body& body, float_t toi, ContactListener* listener)
{
	assert(body.IsAccelerable());

	for (auto&& ce: body.GetContactEdges())
	{
		auto contact = ce.contact;
		auto other = ce.other;

		if (!contact->IsInIsland() && !HasSensor(*contact) && (other->IsImpenetrable() || body.IsImpenetrable()))
		{
			// Tentatively advance the body to the TOI.
			const auto backup = other->m_sweep;
			if (!other->IsInIsland())
			{
				other->Advance(toi);
			}
			
			// Update the contact points
			contact->Update(listener);
			
			// Revert and skip if contact disabled by user or no contact points anymore.
			if (!contact->IsEnabled() || !contact->IsTouching())
			{
				other->m_sweep = backup;
				other->m_xf = GetTransform1(other->m_sweep);
				continue;
			}
			
			island.m_contacts.push_back(contact);
			contact->SetInIsland();
			
			if (!other->IsInIsland())
			{
				other->SetInIsland();			
				if (other->IsSpeedable())
				{
					other->SetAwake();
				}
				AddToIsland(island, *other);
			}		
		}		
	}
}

void World::Step(float_t dt, unsigned velocityIterations, unsigned positionIterations)
{
	if (HasNewFixtures())
	{
		UnsetNewFixtures();
		
		// New fixtures were added: need to find and create the new contacts.
		m_contactMgr.FindNewContacts();
	}

	assert(!IsLocked());
	FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

	// Update and destroy contacts. No new contacts are created though.
	m_contactMgr.Collide();

	if (dt > 0)
	{
		TimeStep step;
		step.set_dt(dt);
		step.velocityIterations	= velocityIterations;
		step.positionIterations = positionIterations;
		step.dtRatio = dt * m_inv_dt0;
		step.warmStarting = GetWarmStarting();
		m_inv_dt0 = step.get_inv_dt();

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (IsStepComplete())
		{
			Solve(step);
		}

		// Handle TOI events.
		if (GetContinuousPhysics())
		{
			SolveTOI(step);
		}
	}
}

void World::ClearForces() noexcept
{
	for (auto&& body: m_bodies)
	{
		body.SetAcceleration(m_gravity, 0_rad);
	}
}

struct WorldQueryWrapper
{
	using size_type = BroadPhase::size_type;

	bool QueryCallback(size_type proxyId)
	{
		const auto proxy = static_cast<FixtureProxy*>(broadPhase->GetUserData(proxyId));
		return callback->ReportFixture(proxy->fixture);
	}

	const BroadPhase* broadPhase;
	QueryFixtureReporter* callback;
};

void World::QueryAABB(QueryFixtureReporter* callback, const AABB& aabb) const
{
	WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactMgr.m_broadPhase;
	wrapper.callback = callback;
	m_contactMgr.m_broadPhase.Query(&wrapper, aabb);
}

struct WorldRayCastWrapper
{
	using size_type = BroadPhase::size_type;

	float_t RayCastCallback(const RayCastInput& input, size_type proxyId)
	{
		auto userData = broadPhase->GetUserData(proxyId);
		const auto proxy = static_cast<FixtureProxy*>(userData);
		auto fixture = proxy->fixture;
		const auto index = proxy->childIndex;
		const auto output = RayCast(*fixture, input, index);

		if (output.hit)
		{
			const auto fraction = output.fraction;
			assert(fraction >= 0 && fraction <= 1);
			const auto point = (float_t{1} - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(fixture, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	WorldRayCastWrapper() = delete;

	constexpr WorldRayCastWrapper(const BroadPhase* bp, RayCastFixtureReporter* cb): broadPhase(bp), callback(cb) {}

	const BroadPhase* const broadPhase;
	RayCastFixtureReporter* const callback;
};

void World::RayCast(RayCastFixtureReporter* callback, const Vec2& point1, const Vec2& point2) const
{
	WorldRayCastWrapper wrapper(&m_contactMgr.m_broadPhase, callback);
	const auto input = RayCastInput{point1, point2, float_t{1}};
	m_contactMgr.m_broadPhase.RayCast(&wrapper, input);
}

World::size_type World::GetProxyCount() const noexcept
{
	return m_contactMgr.m_broadPhase.GetProxyCount();
}

World::size_type World::GetTreeHeight() const noexcept
{
	return m_contactMgr.m_broadPhase.GetTreeHeight();
}

World::size_type World::GetTreeBalance() const
{
	return m_contactMgr.m_broadPhase.GetTreeBalance();
}

float_t World::GetTreeQuality() const
{
	return m_contactMgr.m_broadPhase.GetTreeQuality();
}

void World::ShiftOrigin(const Vec2& newOrigin)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	for (auto&& b: m_bodies)
	{
		b.m_xf.p -= newOrigin;
		b.m_sweep.pos0.c -= newOrigin;
		b.m_sweep.pos1.c -= newOrigin;
	}

	for (auto&& j: m_joints)
	{
		j.ShiftOrigin(newOrigin);
	}

	m_contactMgr.m_broadPhase.ShiftOrigin(newOrigin);
}

void Dump(const World& world)
{
	const auto gravity = world.GetGravity();
	log("Vec2 g(%.15lef, %.15lef);\n", gravity.x, gravity.y);
	log("m_world->SetGravity(g);\n");

	const auto& bodies = world.GetBodies();
	log("Body** bodies = (Body**)alloc(%d * sizeof(Body*));\n", bodies.size());
	auto i = size_t{0};
	for (auto&& b: bodies)
	{
		Dump(b, i);
		++i;
	}

	const auto& joints = world.GetJoints();
	log("Joint** joints = (Joint**)alloc(%d * sizeof(Joint*));\n", joints.size());
	i = 0;
	for (auto&& j: joints)
	{
		log("{\n");
		Dump(j, i);
		log("}\n");
		++i;
	}

	log("free(joints);\n");
	log("free(bodies);\n");
	log("joints = nullptr;\n");
	log("bodies = nullptr;\n");
}


} // namespace box2d
