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

#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Island.hpp>
#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Collision/BroadPhase.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/TimeOfImpact.hpp>
#include <Box2D/Collision/RayCastOutput.hpp>
#include <Box2D/Common/Timer.hpp>
#include <Box2D/Common/AllocatedArray.hpp>

#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>
#include <Box2D/Dynamics/Contacts/PositionConstraint.hpp>

#include <new>
#include <functional>
#include <type_traits>
#include <memory>
#include <set>

namespace box2d
{

using VelocityContainer = AllocatedArray<Velocity, StackAllocator&>;
using PositionContainer = AllocatedArray<Position, StackAllocator&>;
using PositionConstraintsContainer = AllocatedArray<PositionConstraint, StackAllocator&>;
using VelocityConstraintsContainer = AllocatedArray<VelocityConstraint, StackAllocator&>;

struct MovementConf
{
	RealNum maxTranslation;
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

namespace {
	
	/// Calculates movement.
	/// @detail Calculate the positional displacement based on the given velocity
	///    that's possibly clamped to the maximum translation and rotation.
	inline Position CalculateMovement(Velocity& velocity, RealNum h, MovementConf conf)
	{
		assert(IsValid(velocity));
		assert(IsValid(h));
		
		auto translation = h * velocity.linear;
		if (GetLengthSquared(translation) > Square(conf.maxTranslation))
		{
			const auto ratio = conf.maxTranslation / Sqrt(GetLengthSquared(translation));
			velocity.linear *= ratio;
			translation = h * velocity.linear;
		}
		
		auto rotation = h * velocity.angular;
		if (Abs(rotation) > conf.maxRotation)
		{
			const auto ratio = conf.maxRotation / Abs(rotation);
			velocity.angular *= ratio;
			rotation = h * velocity.angular;
		}
		
		return Position{translation, rotation};
	}
	
	inline void IntegratePositions(PositionContainer& positions, VelocityContainer& velocities,
								   RealNum h, MovementConf conf)
	{
		auto i = size_t{0};
		for (auto&& velocity: velocities)
		{
			positions[i] += CalculateMovement(velocity, h, conf);
			++i;
		}
	}
	
	inline ContactImpulsesList GetContactImpulses(const VelocityConstraint& vc)
	{
		ContactImpulsesList impulse;
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
					   StepConf::iteration_type solved)
	{
		const auto size = contacts.size();
		for (auto i = decltype(size){0}; i < size; ++i)
		{
			listener.PostSolve(*contacts[i], GetContactImpulses(constraints[i]), solved);
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
	inline VelocityConstraint GetVelocityConstraint(const Contact& contact, VelocityConstraint::index_type index, RealNum dtRatio)
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
			const auto ci = manifold.GetContactImpulses(j);
			constraint.AddPoint(dtRatio * ci.m_normal, dtRatio * ci.m_tangent);
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
								   const Island::ContactContainer& contacts, RealNum dtRatio)
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
			var.SetPointImpulses(i, GetNormalImpulseAtPoint(vc, i), GetTangentImpulseAtPoint(vc, i));
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
										  Span<const Position> positions,
										  const VelocityConstraint::UpdateConf& conf)
	{
		auto i = Span<VelocityConstraint>::size_type{0};
		for (auto&& pc: positionConstraints)
		{
			const auto posA = positions[pc.bodyA.index];
			const auto posB = positions[pc.bodyB.index];
			const auto worldManifold = GetWorldManifold(pc, posA, posB);
			velocityConstraints[i].Update(worldManifold, posA.linear, posB.linear, velocities, conf);
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

	inline RealNum UpdateSleepTimes(Island::BodyContainer& bodies, RealNum h)
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
	
	inline size_t Sleepem(Island::BodyContainer& bodies)
	{
		auto unawoken = size_t{0};
		for (auto&& b: bodies)
		{
			if (b->UnsetAwake())
			{
				++unawoken;
			}
		}
		return unawoken;
	}

} // anonymous namespace

const BodyDef& World::GetDefaultBodyDef()
{
	static const BodyDef def = BodyDef{};
	return def;
}

World::World(const Def& def):
	m_gravity(def.gravity),
	m_linearSlop(def.linearSlop),
	m_angularSlop(def.angularSlop),
	m_maxVertexRadius(def.maxVertexRadius)
{
	// Confirm that linearSlop and maxVertexRadius can work together
	assert((def.maxVertexRadius * 2) + (def.linearSlop / 4) > (def.maxVertexRadius * 2));
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

void World::SetGravity(const Vec2 gravity) noexcept
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
	
RegStepStats World::Solve(const StepConf& step)
{
	auto stats = RegStepStats{};

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
				++stats.islandsFound;

				auto island = BuildIsland(body, remNumBodies, remNumContacts, remNumJoints);
				
				// Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
				const auto constraintsSolved = Solve(step, island);
				if (constraintsSolved)
				{
					++stats.islandsSolved;
				}

				if (IsValid(step.minStillTimeToSleep))
				{
					const auto minSleepTime = UpdateSleepTimes(island.m_bodies, step.get_dt());
					if ((minSleepTime >= step.minStillTimeToSleep) && constraintsSolved)
					{
						stats.bodiesSlept += Sleepem(island.m_bodies);
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
	stats.contactsAdded = m_contactMgr.FindNewContacts();
	
	return stats;
}
	
bool World::Solve(const StepConf& step, Island& island)
{
	const auto ncontacts = static_cast<contact_count_t>(island.m_contacts.size());
	const auto nbodies = island.m_bodies.size();

	auto positionConstraints = PositionConstraintsContainer{
		ncontacts, m_stackAllocator.AllocateArray<PositionConstraint>(ncontacts), m_stackAllocator
	};
	InitPosConstraints(positionConstraints, island.m_contacts);

	auto velocityConstraints = VelocityConstraintsContainer{
		ncontacts, m_stackAllocator.AllocateArray<VelocityConstraint>(ncontacts), m_stackAllocator
	};
	InitVelConstraints(velocityConstraints, island.m_contacts, step.doWarmStart? step.dtRatio: 0);
	
	auto velocities = VelocityContainer{
		nbodies, m_stackAllocator.AllocateArray<Velocity>(nbodies), m_stackAllocator
	};
	
	auto positions = PositionContainer{
		nbodies, m_stackAllocator.AllocateArray<Position>(nbodies), m_stackAllocator
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
	
	UpdateVelocityConstraints(velocityConstraints, velocities, positionConstraints, positions,
							  VelocityConstraint::UpdateConf{step.velocityThreshold, true});
	
	if (step.doWarmStart)
	{
		WarmStartVelocities(velocityConstraints, velocities);
	}

	const auto psConf = ConstraintSolverConf{}
		.UseResolutionRate(step.regResolutionRate)
		.UseLinearSlop(GetLinearSlop())
		.UseAngularSlop(GetAngularSlop())
		.UseMaxLinearCorrection(step.maxLinearCorrection)
		.UseMaxAngularCorrection(step.maxAngularCorrection);

	for (auto&& joint: island.m_joints)
	{
		joint->InitVelocityConstraints(velocities, positions, step, psConf);
	}
	
	for (auto i = decltype(step.regVelocityIterations){0}; i < step.regVelocityIterations; ++i)
	{
		for (auto&& joint: island.m_joints)
		{
			joint->SolveVelocityConstraints(velocities, step);
		}

		SolveVelocityConstraints(velocityConstraints, velocities);
	}
	
	// updates array of tentative new body positions per the velocities as if there were no obstacles...
	IntegratePositions(positions, velocities, h, MovementConf{step.maxTranslation, step.maxRotation});
	
	// Solve position constraints
	auto iterationSolved = StepConf::InvalidIteration;
	for (auto i = decltype(step.regPositionIterations){0}; i < step.regPositionIterations; ++i)
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
	
	UpdateBodies(island.m_bodies, positions, velocities);

	if (m_contactMgr.m_contactListener)
	{
		Report(*m_contactMgr.m_contactListener, island.m_contacts, velocityConstraints,
			   iterationSolved);
	}
	
	return iterationSolved != StepConf::InvalidIteration;
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

void World::UpdateContactTOIs(const StepConf& step)
{
	const auto toiConf = ToiConf{}
		.UseTimeMax(1)
		.UseTargetDepth(GetLinearSlop() * 3)
		.UseTolerance(GetLinearSlop() / 4)
		.UseMaxRootIters(step.maxTOIRootIterCount)
		.UseMaxToiIters(step.maxTOIIterations);
	for (auto&& c: m_contactMgr.GetContacts())
	{
		if (c.IsEnabled() && (c.GetToiCount() < step.maxSubSteps) && !c.HasValidToi())
		{
			c.UpdateTOI(toiConf);
		}
	}
}
	
World::ContactToiData World::GetSoonestContact(const StepConf& step)
{
	auto minToi = MaxFloat;
	auto minContact = static_cast<Contact*>(nullptr);
	auto count = contact_count_t{0};
	for (auto&& c: m_contactMgr.GetContacts())
	{
		if (c.IsEnabled() && (c.GetToiCount() < step.maxSubSteps) && c.HasValidToi())
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
#if 1
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
ToiStepStats World::SolveTOI(const StepConf& step)
{
	auto stats = ToiStepStats{};

	if (IsStepComplete())
	{
		ResetBodiesForSolveTOI();
		ResetContactsForSolveTOI();
	}

	// Find TOI events and solve them.
	for (;;)
	{
		UpdateContactTOIs(step);
		
		const auto next = GetSoonestContact(step);
		if ((!next.contact) || (next.toi >= 1))
		{
			// No more TOI events to handle within the current time step. Done!
			SetStepComplete(true);
			break;
		}

		++stats.contactsChecked;
		if (SolveTOI(step, *next.contact))
		{
			++stats.islandsFound;
		}
		
		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		stats.contactsAdded += m_contactMgr.FindNewContacts();

		if (GetSubStepping())
		{
			SetStepComplete(false);
			break;
		}
	}
	return stats;
}

bool World::SolveTOI(const StepConf& step, Contact& contact)
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
	contact.SetEnabled();	
	contact.Update(m_contactMgr.m_contactListener);
	contact.UnsetToi();

	++contact.m_toiCount;
	if (contact.m_toiCount >= step.maxSubSteps)
	{
		// Note: This contact won't be passed again to this method within the current world step
		// (as the UpdateContactTOIs method won't return it anymore).
		// What are the pros/cons of this?
		// Larger m_maxSubSteps slows down the simulation.
		// m_maxSubSteps of 44 and higher seems to decrease the occurrance of tunneling of multiple
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
		return false;
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

	// Now solve for remainder of time step
	SolveTOI(StepConf{step}.use_dt((1 - toi) * step.get_dt()), island);

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
	
	return true;
}

void World::UpdateBodies(Span<Body*> bodies,
						 Span<const Position> positions, Span<const Velocity> velocities)
{
	auto i = size_t{0};
	for (auto&& b: bodies)
	{
		b->m_velocity = velocities[i]; // sets what Body::GetVelocity returns
		b->m_sweep.pos1 = positions[i]; // sets what Body::GetWorldCenter returns
		b->m_xf = GetTransformation(b->m_sweep.pos1, b->m_sweep.GetLocalCenter()); // sets what Body::GetLocation returns
		++i;
	}
}

bool World::SolveTOI(const StepConf& step, Island& island)
{
	const auto ncontacts = static_cast<contact_count_t>(island.m_contacts.size());
	const auto nbodies = island.m_bodies.size();

	assert(nbodies >= 2);
	assert(ncontacts >= 1);

	auto velocities = VelocityContainer{
		nbodies, m_stackAllocator.AllocateArray<Velocity>(nbodies), m_stackAllocator
	};
	auto positions = PositionContainer{
		nbodies, m_stackAllocator.AllocateArray<Position>(nbodies), m_stackAllocator
	};
	auto positionConstraints = PositionConstraintsContainer{
		ncontacts, m_stackAllocator.AllocateArray<PositionConstraint>(ncontacts), m_stackAllocator
	};
	auto velocityConstraints = VelocityConstraintsContainer{
		ncontacts, m_stackAllocator.AllocateArray<VelocityConstraint>(ncontacts), m_stackAllocator
	};
	InitPosConstraints(positionConstraints, island.m_contacts);
	InitVelConstraints(velocityConstraints, island.m_contacts, 0);
	
	// Initialize the body state.
	for (auto&& body: island.m_bodies)
	{
		positions.push_back(body->m_sweep.pos1);
		velocities.push_back(body->GetVelocity());
	}
	
	// Solve TOI-based position constraints.
	auto positionConstraintsSolved = StepConf::InvalidIteration;
	const auto psConf = ConstraintSolverConf{}
		.UseResolutionRate(step.toiResolutionRate)
		.UseLinearSlop(GetLinearSlop())
		.UseAngularSlop(GetAngularSlop())
		.UseMaxLinearCorrection(step.maxLinearCorrection)
		.UseMaxAngularCorrection(step.maxAngularCorrection);

	for (auto i = decltype(step.toiPositionIterations){0}; i < step.toiPositionIterations; ++i)
	{
		//
		// Note: There are two flavors of the SolvePositionConstraints function.
		//   One takes an extra two arguments that are the indexes of two bodies that are okay to
		//   move. The other one does not.
		//   Calling the selective solver (that takes the two additional arguments) appears to
		//   result in phsyics simulations that are more prone to tunneling. Meanwhile, using the
		//   non-selective solver would presumably be slower (since it appears to have more that
		//   it will do). Assuming that slower is preferable to tunnelling, then the non-selective
		//   function is the one to be calling here.
		//
		const auto minSeparation = SolvePositionConstraints(positionConstraints, positions, psConf);
		if (minSeparation >= -psConf.linearSlop * RealNum(1.5))
		{
			positionConstraintsSolved = i;
			break;
		}
	}
	
	{
		// Leap of faith to new safe state.
		// Not doing this results in slower simulations.
		// Originally this update was only done to island.m_bodies 0 and 1.
		// Unclear whether rest of bodies should also be updated. No difference noticed.
		auto i = size_t{0};
		for (auto&& body: island.m_bodies)
		{
			body->m_sweep.pos0 = positions[i];
			++i;
		}
	}
	
	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	UpdateVelocityConstraints(velocityConstraints, velocities, positionConstraints, positions,
							  VelocityConstraint::UpdateConf{step.velocityThreshold, true});
	
	// Solve velocity constraints.
	for (auto i = decltype(step.toiVelocityIterations){0}; i < step.toiVelocityIterations; ++i)
	{
		SolveVelocityConstraints(velocityConstraints, velocities);
	}
	
	// Don't store TOI contact forces for warm starting because they can be quite large.
	
	IntegratePositions(positions, velocities, step.get_dt(), MovementConf{step.maxTranslation, step.maxRotation});
	
	UpdateBodies(island.m_bodies, positions, velocities);

	if (m_contactMgr.m_contactListener)
	{
		Report(*m_contactMgr.m_contactListener, island.m_contacts, velocityConstraints,
			   positionConstraintsSolved);
	}
	
	return positionConstraintsSolved != StepConf::InvalidIteration;
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

void World::ProcessContactsForTOI(Island& island, Body& body, RealNum toi, ContactListener* listener)
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
			contact->SetEnabled();
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

StepStats World::Step(const StepConf& conf)
{
	auto stepStats = StepStats{};

	if (HasNewFixtures())
	{
		UnsetNewFixtures();
		
		// New fixtures were added: need to find and create the new contacts.
		stepStats.pre.added = m_contactMgr.FindNewContacts();
	}

	assert(!IsLocked());
	FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

	// Update and destroy contacts. No new contacts are created though.
	const auto collideStats = m_contactMgr.Collide();
	stepStats.pre.ignored = collideStats.ignored;
	stepStats.pre.destroyed = collideStats.destroyed;
	stepStats.pre.updated = collideStats.updated;

	if (conf.get_dt() > 0)
	{
		m_inv_dt0 = conf.get_inv_dt();

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (IsStepComplete())
		{
			stepStats.reg = Solve(conf);
		}

		// Handle TOI events.
		if (conf.doToi)
		{
			stepStats.toi = SolveTOI(conf);
		}
	}
	return stepStats;
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

	RealNum RayCastCallback(const RayCastInput& input, size_type proxyId)
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
			const auto point = (RealNum{1} - fraction) * input.p1 + fraction * input.p2;
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
	const auto input = RayCastInput{point1, point2, RealNum{1}};
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

RealNum World::GetTreeQuality() const
{
	return m_contactMgr.m_broadPhase.GetTreeQuality();
}

void World::ShiftOrigin(const Vec2 newOrigin)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	for (auto&& b: m_bodies)
	{
		b.m_xf.p -= newOrigin;
		b.m_sweep.pos0.linear -= newOrigin;
		b.m_sweep.pos1.linear -= newOrigin;
	}

	for (auto&& j: m_joints)
	{
		j.ShiftOrigin(newOrigin);
	}

	m_contactMgr.m_broadPhase.ShiftOrigin(newOrigin);
}

StepStats Step(World& world, RealNum dt, World::ts_iters_type velocityIterations, World::ts_iters_type positionIterations)
{
	StepConf step;
	step.set_dt(dt);
	step.regVelocityIterations = velocityIterations;
	step.regPositionIterations = positionIterations;
	step.toiVelocityIterations = velocityIterations;
	if (positionIterations == 0)
	{
		step.toiPositionIterations = 0;
	}
	step.dtRatio = dt * world.GetInvDeltaTime();
	return world.Step(step);
}

size_t GetFixtureCount(const World& world) noexcept
{
	size_t sum = 0;
	for (auto&& b: world.GetBodies())
	{
		sum += GetFixtureCount(b);
	}
	return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
	auto shapes = std::set<const Shape*>();
	for (auto&& b: world.GetBodies())
	{
		for (auto&& f: b.GetFixtures())
		{
			shapes.insert(f.GetShape());
		}
	}
	return shapes.size();
}

size_t GetAwakeCount(const World& world) noexcept
{
	auto count = size_t(0);
	for (auto&& body: world.GetBodies())
	{
		if (body.IsAwake())
		{
			++count;
		}
	}
	return count;
}
	
size_t Awaken(World& world)
{
	auto awoken = size_t{0};
	for (auto&& b: world.GetBodies())
	{
		if (b.SetAwake())
		{
			++awoken;
		}
	}
	return awoken;
}

void ClearForces(World& world) noexcept
{
	const auto g = world.GetGravity();
	for (auto&& body: world.GetBodies())
	{
		body.SetAcceleration(g, 0_rad);
	}
}

} // namespace box2d
