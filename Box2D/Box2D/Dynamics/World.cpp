/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Common/Timer.hpp>

#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>
#include <Box2D/Dynamics/Contacts/PositionConstraint.hpp>

#include <new>
#include <functional>
#include <type_traits>
#include <memory>
#include <set>
#include <vector>
#include <unordered_map>

//#define DO_THREADED
#if defined(DO_THREADED)
#include <future>
#endif

#define BOX2D_MAGIC(x) (x)

namespace box2d
{

using BodyConstraints = std::unordered_map<const Body*, BodyConstraint>;
using PositionConstraints = std::vector<PositionConstraint>;
using VelocityConstraints = std::vector<VelocityConstraint>;
	
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
	
	struct PositionAndVelocity
	{
		Position position;
		Velocity velocity;
	};

	/// Calculates movement.
	/// @detail Calculate the positional displacement based on the given velocity
	///    that's possibly clamped to the maximum translation and rotation.
	inline PositionAndVelocity CalculateMovement(const BodyConstraint& body, RealNum h, MovementConf conf)
	{
		assert(IsValid(h));
		
		auto velocity = body.GetVelocity();
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
		
		return PositionAndVelocity{body.GetPosition() + Position{translation, rotation}, velocity};
	}
	
	inline void IntegratePositions(BodyConstraints& bodies,
								   RealNum h, MovementConf conf)
	{
		for (auto&& body: bodies)
		{
			const auto newPosAndVel = CalculateMovement(body.second, h, conf);
			body.second.SetPosition(newPosAndVel.position);
			body.second.SetVelocity(newPosAndVel.velocity);
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
					   const VelocityConstraints& constraints,
					   StepConf::iteration_type solved)
	{
		const auto size = contacts.size();
		for (auto i = decltype(size){0}; i < size; ++i)
		{
			listener.PostSolve(*contacts[i], GetContactImpulses(constraints[i]), solved);
		}
	}
	
	PositionConstraints GetPositionConstraints(const Island::Contacts& contacts, BodyConstraints& bodies)
	{
		auto constraints = PositionConstraints{};
		constraints.reserve(contacts.size());
		for (auto&& contact: contacts)
		{
			const auto& manifold = contact->GetManifold();
			const auto& fixtureA = *(contact->GetFixtureA());
			const auto& fixtureB = *(contact->GetFixtureB());
			
			const auto bodyA = fixtureA.GetBody();
			const auto shapeA = fixtureA.GetShape();

			const auto bodyB = fixtureB.GetBody();
			const auto shapeB = fixtureB.GetShape();

			auto& bodiesA = bodies.at(bodyA);
			auto& bodiesB = bodies.at(bodyB);

			const auto radiusA = GetVertexRadius(*shapeA);
			const auto radiusB = GetVertexRadius(*shapeB);
			
			constraints.emplace_back(manifold, bodiesA, radiusA, bodiesB, radiusB);
		}
		return constraints;
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
	inline void StoreImpulses(const VelocityConstraints& velocityConstraints, Span<Contact*> contacts)
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
					1_rad * vc.bodyA.GetInvRotInertia() * Cross(GetPointRelPosA(vc, j), P)
				};
				vp.b += Velocity{
					vc.bodyB.GetInvMass() * P,
					1_rad * vc.bodyB.GetInvRotInertia() * Cross(GetPointRelPosB(vc, j), P)
				};
			}
		}
		return vp;
	}
	
	inline void WarmStartVelocities(const VelocityConstraints& velocityConstraints)
	{
		for (auto&& vc: velocityConstraints)
		{
			const auto vp = CalcWarmStartVelocityDeltas(vc);
			vc.bodyA.SetVelocity(vc.bodyA.GetVelocity() + vp.a);
			vc.bodyB.SetVelocity(vc.bodyB.GetVelocity() + vp.b);
		}
	}

	/// Gets the velocity constraints for the given inputs.
	/// @detail
	/// Inializes the velocity constraints with the position dependent portions of the current position constraints.
	/// @post Velocity constraints will have their "normal" field set to the world manifold normal for them.
	/// @post Velocity constraints will have their constraint points set.
	/// @sa SolveVelocityConstraints.
	VelocityConstraints GetVelocityConstraints(const Island::Contacts& contacts,
														BodyConstraints& bodies,
														const VelocityConstraint::Conf conf)
	{
		auto velocityConstraints = VelocityConstraints{};
		const auto numContacts = contacts.size();
		velocityConstraints.reserve(numContacts);

		//auto i = VelocityConstraint::index_type{0};
		for (auto i = decltype(numContacts){0}; i < numContacts; ++i)
		{
			const auto& contact = *contacts[i];

			const auto& manifold = contact.GetManifold();
			const auto fixtureA = contact.GetFixtureA();
			const auto fixtureB = contact.GetFixtureB();
			const auto friction = contact.GetFriction();
			const auto restitution = contact.GetRestitution();
			const auto tangentSpeed = contact.GetTangentSpeed();
			
			const auto bodyA = fixtureA->GetBody();
			const auto shapeA = fixtureA->GetShape();

			const auto bodyB = fixtureB->GetBody();
			const auto shapeB = fixtureB->GetShape();

			auto& bodiesA = bodies.at(bodyA);
			auto& bodiesB = bodies.at(bodyB);

			const auto radiusA = shapeA->GetVertexRadius();
			const auto radiusB = shapeB->GetVertexRadius();
			
			velocityConstraints.emplace_back(i, friction, restitution, tangentSpeed,
											 manifold, bodiesA, radiusA, bodiesB, radiusB,
											 conf);

		}
		return velocityConstraints;
	}

	/// "Solves" the velocity constraints.
	/// @detail Updates the velocities and velocity constraint points' normal and tangent impulses.
	/// @pre <code>UpdateVelocityConstraints</code> has been called on the velocity constraints.
	inline RealNum SolveVelocityConstraints(VelocityConstraints& velocityConstraints)
	{
		auto maxIncImpulse = RealNum{0};
		for (auto&& vc: velocityConstraints)
		{
			maxIncImpulse = std::max(maxIncImpulse, SolveVelocityConstraint(vc));
		}
		return maxIncImpulse;
	}

	inline RealNum UpdateSleepTimes(Island::Bodies& bodies, const StepConf& step)
	{
		auto minSleepTime = std::numeric_limits<RealNum>::infinity();
		for (auto&& b: bodies)
		{
			if (b->IsSpeedable())
			{
				const auto sleepTime = b->UpdateSleepTime(step.get_dt(),
														  step.linearSleepTolerance,
														  step.angularSleepTolerance);
				minSleepTime = Min(minSleepTime, sleepTime);
			}
		}
		return minSleepTime;
	}
	
	inline size_t Sleepem(Island::Bodies& bodies)
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
	
	inline bool IsAllFlagsSet(uint16 value, uint16 flags)
	{
		return (value & flags) == flags;
	}
	
	inline bool IsValidForTime(TOIOutput::State state) noexcept
	{
		return state == TOIOutput::e_touching;
	}
	
	inline bool IsFor(const Contact& contact,
					  const Fixture* fixtureA, child_count_t indexA,
					  const Fixture* fixtureB, child_count_t indexB)
	{
		const auto fA = contact.GetFixtureA();
		const auto fB = contact.GetFixtureB();
		const auto iA = contact.GetChildIndexA();
		const auto iB = contact.GetChildIndexB();
		
		return
			((fA == fixtureA) && (fB == fixtureB) && (iA == indexA) && (iB == indexB)) ||
			((fA == fixtureB) && (fB == fixtureA) && (iA == indexB) && (iB == indexA));
	}

	void FlagContactsForFiltering(Body* bodyA, Body* bodyB)
	{
		for (auto&& contact: bodyB->GetContacts())
		{
			const auto fA = contact->GetFixtureA();
			const auto fB = contact->GetFixtureB();
			const auto bA = fA->GetBody();
			const auto bB = fB->GetBody();
			const auto other = (bA != bodyB)? bA: bB;
			if (other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				contact->FlagForFiltering();
			}
		}
	}
	
} // anonymous namespace

const BodyDef& World::GetDefaultBodyDef()
{
	static const BodyDef def = BodyDef{};
	return def;
}

World::World(const Def& def):
	m_gravity(def.gravity),
	m_minVertexRadius(def.minVertexRadius),
	m_maxVertexRadius(def.maxVertexRadius)
{
	assert(::box2d::IsValid(def.gravity));
	assert(def.minVertexRadius > 0);
	assert(def.minVertexRadius < def.maxVertexRadius);
}

World::~World()
{
	// Gets rid of the associated contacts.
	while (!m_contacts.empty())
	{
		const auto c = m_contacts.front();
		const auto fixtureA = c->GetFixtureA();
		const auto fixtureB = c->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		bodyA->Erase(c);
		bodyB->Erase(c);
		m_contacts.pop_front();
		Contact::Destroy(c, m_blockAllocator);
	}

	// Gets rid of the created joints.
	while (!m_joints.empty())
	{
		const auto j = m_joints.front();
		const auto bodyA = j->GetBodyA();
		const auto bodyB = j->GetBodyB();
		if (bodyA)
		{
			bodyA->Erase(j);
		}
		if (bodyB)
		{
			bodyB->Erase(j);
		}
		m_joints.pop_front();
		Joint::Destroy(j, m_blockAllocator);
	}

	// Gets rid of the created bodies and any associated fixtures.
	while (!m_bodies.empty())
	{
		const auto b = m_bodies.front();
		m_bodies.pop_front();
		while (!b->m_fixtures.empty())
		{
			const auto fixture = b->m_fixtures.front();
			b->m_fixtures.pop_front();
			if (m_destructionListener)
			{
				m_destructionListener->SayGoodbye(*fixture);
			}
			fixture->DestroyProxies(m_blockAllocator, m_broadPhase);
			Delete(fixture, m_blockAllocator);
		}
		b->~Body();
		m_blockAllocator.Free(b, sizeof(Body));
	}
}

void World::SetDestructionListener(DestructionListener* listener) noexcept
{
	m_destructionListener = listener;
}

void World::SetContactFilter(ContactFilter* filter) noexcept
{
	m_contactFilter = filter;
}

void World::SetContactListener(ContactListener* listener) noexcept
{
	m_contactListener = listener;
}

void World::SetGravity(const Vec2 gravity) noexcept
{
	if (m_gravity != gravity)
	{
		const auto diff = gravity - m_gravity;
		for (auto&& body: m_bodies)
		{
			ApplyLinearAcceleration(*body, diff);
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
	for (auto iter = m_bodies.begin(); iter != m_bodies.end(); ++iter)
	{
		if (*iter == &b)
		{
			m_bodies.erase(iter);
			return true;
		}
	}
	return false;
}

void World::Destroy(Body* b)
{
	assert(b);
	assert(b->m_world == this);
	
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	// Delete the attached joints.
	while (!b->m_joints.empty())
	{
		auto iter = b->m_joints.begin();
		const auto joint = *iter;
		b->m_joints.erase(iter);
		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(*joint);
		}
		Destroy(joint);
	}

	// Destroy the attached contacts.
	while (!b->m_contacts.empty())
	{
		auto iter = b->m_contacts.begin();
		const auto contact = *iter;
		b->m_contacts.erase(iter);
		Destroy(contact);
	}

	// Delete the attached fixtures. This destroys broad-phase proxies.
	while (!b->m_fixtures.empty())
	{
		const auto fixture = b->m_fixtures.front();
		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(*fixture);
		}
		fixture->DestroyProxies(m_blockAllocator, m_broadPhase);
		Delete(fixture, m_blockAllocator);
		b->m_fixtures.pop_front();
	}
	
	Remove(*b);
	
	b->~Body();
	m_blockAllocator.Free(b, sizeof(*b));
}

Joint* World::CreateJoint(const JointDef& def)
{
	if (m_joints.size() >= MaxJoints)
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
	if (!j)
	{
		return nullptr;
	}

	// Connect to the bodies' doubly linked lists.
	const auto bodyA = j->GetBodyA();
	const auto bodyB = j->GetBodyB();
	if (bodyA)
	{
		bodyA->m_joints.insert(j);
	}
	if (bodyB)
	{
		bodyB->m_joints.insert(j);
	}

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!def.collideConnected)
	{
		FlagContactsForFiltering(bodyA, bodyB);
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
	for (auto iter = m_joints.begin(); iter != m_joints.end(); ++iter)
	{
		if (*iter == &j)
		{
			m_joints.erase(iter);
			return true;
		}
	}
	return false;
}

void World::Destroy(Joint* j)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	InternalDestroy(j);
}
	
void World::InternalDestroy(Joint* j)
{
	if (!Remove(*j))
	{
		return;
	}

	const auto collideConnected = j->m_collideConnected;
	
	// Disconnect from island graph.
	const auto bodyA = j->GetBodyA();
	const auto bodyB = j->GetBodyB();

	// Wake up connected bodies.
	if (bodyA)
	{
		bodyA->SetAwake();
		bodyA->m_joints.erase(j);
	}
	if (bodyB)
	{
		bodyB->SetAwake();
		bodyB->m_joints.erase(j);
	}

	Joint::Destroy(j, m_blockAllocator);

	// If the joint prevented collisions, then flag any contacts for filtering.
	if (!collideConnected)
	{
		FlagContactsForFiltering(bodyA, bodyB);
	}
}

Island World::BuildIsland(Body& seed,
				  Bodies::size_type& remNumBodies,
				  Contacts::size_type& remNumContacts,
				  Joints::size_type& remNumJoints)
{
	assert(!seed.IsInIsland());
	assert(seed.IsSpeedable());
	assert(seed.IsAwake());
	assert(seed.IsActive());
	assert(remNumBodies != 0);

	// Size the island for the remaining un-evaluated bodies, contacts, and joints.
	Island island(remNumBodies, remNumContacts, remNumJoints);

	// Perform a depth first search (DFS) on the constraint graph.
	auto stack = std::vector<Body*>();
	stack.reserve(remNumBodies);
	stack.push_back(&seed);
	seed.SetInIsland();
	while (!stack.empty())
	{
		// Grab the next body off the stack and add it to the island.
		const auto b = stack.back();
		stack.pop_back();
		
		assert(b->IsActive());
		island.m_bodies.push_back(b);
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
		for (auto&& contact: b->GetContacts())
		{
			const auto fA = contact->GetFixtureA();
			const auto fB = contact->GetFixtureB();
			const auto bA = fA->GetBody();
			const auto bB = fB->GetBody();
			const auto other = (bA != b)? bA: bB;

			if (!contact->IsInIsland() && !HasSensor(*contact) && contact->IsEnabled() && contact->IsTouching())
			{
				island.m_contacts.push_back(contact);
				contact->SetInIsland();

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
		for (auto&& joint: b->m_joints)
		{
			const auto bodyA = joint->GetBodyA();
			const auto bodyB = joint->GetBodyB();
			const auto other = (b != bodyA)? bodyA: bodyB;
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
	
RegStepStats World::SolveReg(const StepConf& step)
{
	auto stats = RegStepStats{};

	// Clear all the island flags.
	// This builds the logical set of bodies, contacts, and joints eligible for resolution.
	// As bodies, contacts, or joints get added to resolution islands, they're essentially
	// removed from this eligible set (and their IsInIsland method thereafter returns true.
	for (auto&& body: m_bodies)
	{
		body->UnsetInIsland();
	}
	for (auto&& contact: m_contacts)
	{
		contact->UnsetInIsland();
	}
	for (auto&& joint: m_joints)
	{
		joint->SetInIsland(false);
	}

	auto remNumBodies = m_bodies.size(); ///< Remaining number of bodies.
	auto remNumContacts = m_contacts.size(); ///< Remaining number of contacts.
	auto remNumJoints = m_joints.size(); ///< Remaining number of joints.

#if defined(DO_THREADED)
	std::vector<std::future<World::IslandSolverResults>> futures;
	futures.reserve(remNumBodies);
#endif
	// Build and simulate all awake islands.
	for (auto&& body: m_bodies)
	{
		if (!body->IsInIsland() && body->IsSpeedable() && body->IsAwake() && body->IsActive())
		{
			++stats.islandsFound;

			auto island = BuildIsland(*body, remNumBodies, remNumContacts, remNumJoints);
			for (auto&& b: island.m_bodies)
			{
				// Allow static bodies to participate in other islands.
				if (!b->IsSpeedable())
				{
					b->UnsetInIsland();
					++remNumBodies;
				}
			}

#if defined(DO_THREADED)
			// Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
			futures.push_back(std::async(&World::SolveRegIsland, this, step, island));
#else
			const auto solverResults = SolveRegIsland(step, island);
			stats.maxIncImpulse = Max(stats.maxIncImpulse, solverResults.maxIncImpulse);
			stats.minSeparation = Min(stats.minSeparation, solverResults.minSeparation);
			if (solverResults.solved)
			{
				++stats.islandsSolved;
			}
			stats.sumPosIters += solverResults.positionIterations;
			stats.sumVelIters += solverResults.velocityIterations;
			stats.bodiesSlept += solverResults.bodiesSlept;
#endif
		}
	}

#if defined(DO_THREADED)
	for (auto&& future: futures)
	{
		const auto solverResults = future.get();
		stats.maxIncImpulse = Max(stats.maxIncImpulse, solverResults.maxIncImpulse);
		stats.minSeparation = Min(stats.minSeparation, solverResults.minSeparation);
		if (solverResults.solved)
		{
			++stats.islandsSolved;
		}
		stats.sumPosIters += solverResults.positionIterations;
		stats.sumVelIters += solverResults.velocityIterations;
		stats.bodiesSlept += solverResults.bodiesSlept;
	}
#endif

	for (auto&& body: m_bodies)
	{
		// A non-static body that was in an island may have moved.
		if ((body->m_flags & (Body::e_velocityFlag|Body::e_islandFlag)) == (Body::e_velocityFlag|Body::e_islandFlag))
		{
			// Update fixtures (for broad-phase).
			stats.proxiesMoved += SynchronizeFixtures(*body, step.displaceMultiplier, step.aabbExtension);
		}
	}

	// Look for new contacts.
	stats.contactsAdded = FindNewContacts();
	
	return stats;
}

World::IslandSolverResults World::SolveRegIsland(const StepConf& step, Island island)
{
	auto finMinSeparation = std::numeric_limits<RealNum>::infinity();
	auto solved = false;
	auto positionIterations = step.regPositionIterations;
	const auto h = step.get_dt(); ///< Time step (in seconds).

	auto bodyConstraints = BodyConstraints{};
	bodyConstraints.reserve(island.m_bodies.size());

	// Update bodies' pos0 values then copy their pos1 and velocity data into local arrays.
	for (auto&& body: island.m_bodies)
	{
		body->m_sweep.pos0 = body->m_sweep.pos1; // like Advance0(1) on the sweep.
		bodyConstraints[body] = GetBodyConstraint(*body, h);
	}
	auto positionConstraints = GetPositionConstraints(island.m_contacts, bodyConstraints);

	auto velocityConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraints,
													  VelocityConstraint::Conf{step.doWarmStart? step.dtRatio: 0, step.velocityThreshold, true});
	
	if (step.doWarmStart)
	{
		WarmStartVelocities(velocityConstraints);
	}

	const auto psConf = ConstraintSolverConf{}
		.UseResolutionRate(step.regResolutionRate)
		.UseLinearSlop(step.linearSlop)
		.UseAngularSlop(step.angularSlop)
		.UseMaxLinearCorrection(step.maxLinearCorrection)
		.UseMaxAngularCorrection(step.maxAngularCorrection);

	for (auto&& joint: island.m_joints)
	{
		joint->InitVelocityConstraints(bodyConstraints, step, psConf);
	}
	
	auto velocityIterations = step.regVelocityIterations;
	auto maxIncImpulse = RealNum{0};
	for (auto i = decltype(step.regVelocityIterations){0}; i < step.regVelocityIterations; ++i)
	{
		for (auto&& joint: island.m_joints)
		{
			joint->SolveVelocityConstraints(bodyConstraints, step);
		}

		const auto newIncImpulse = SolveVelocityConstraints(velocityConstraints);
		maxIncImpulse = std::max(maxIncImpulse, newIncImpulse);
	}
	
	// updates array of tentative new body positions per the velocities as if there were no obstacles...
	IntegratePositions(bodyConstraints, h, MovementConf{step.maxTranslation, step.maxRotation});
	
	// Solve position constraints
	for (auto i = decltype(step.regPositionIterations){0}; i < step.regPositionIterations; ++i)
	{
		const auto minSeparation = SolvePositionConstraints(positionConstraints, psConf);
		finMinSeparation = Min(finMinSeparation, minSeparation);
		const auto contactsOkay = (minSeparation >= step.regMinSeparation);

		const auto jointsOkay = [&]()
		{
			auto allOkay = true;
			for (auto&& joint: island.m_joints)
			{
				if (!joint->SolvePositionConstraints(bodyConstraints, psConf))
				{
					allOkay = false;
				}
			}
			return allOkay;
		}();
		
		if (contactsOkay && jointsOkay)
		{
			// Reached tolerance, early out...
			positionIterations = i + 1;
			solved = true;
			break;
		}
	}
	
	// Update normal and tangent impulses of contacts' manifold points
	StoreImpulses(velocityConstraints, island.m_contacts);
	
	for (auto&& body: island.m_bodies)
	{
		const auto& constraint = bodyConstraints[body];
		UpdateBody(*body, constraint.GetPosition(), constraint.GetVelocity());
	}
	
	if (m_contactListener)
	{
		Report(*m_contactListener, island.m_contacts, velocityConstraints,
			   solved? positionIterations - 1: StepConf::InvalidIteration);
	}
	
	uint16 bodiesSlept = 0;
	if (::box2d::IsValid(step.minStillTimeToSleep))
	{
		const auto minSleepTime = UpdateSleepTimes(island.m_bodies, step);
		if ((minSleepTime >= step.minStillTimeToSleep) && solved)
		{
			bodiesSlept = static_cast<decltype(bodiesSlept)>(Sleepem(island.m_bodies));
		}
	}

	return IslandSolverResults{finMinSeparation, maxIncImpulse, solved,
		positionIterations, velocityIterations, bodiesSlept};
}

void World::ResetBodiesForSolveTOI()
{
	for (auto&& b: m_bodies)
	{
		b->UnsetInIsland();
		b->m_sweep.ResetAlpha0();
	}
}

void World::ResetContactsForSolveTOI()
{
	for (auto&& c: m_contacts)
	{
		// Invalidate TOI
		c->UnsetInIsland();
		c->UnsetToi();
		c->ResetToiCount();
	}	
}
	
World::UpdateContactsData World::UpdateContactTOIs(const StepConf& step)
{
	auto results = UpdateContactsData{};

	const auto toiConf = ToiConf{}
		.UseTimeMax(1)
		.UseTargetDepth(step.targetDepth)
		.UseTolerance(step.tolerance)
		.UseMaxRootIters(step.maxToiRootIters)
		.UseMaxToiIters(step.maxToiIters)
		.UseMaxDistIters(step.maxDistanceIters);
	
	for (auto&& c: m_contacts)
	{
		if (c->HasValidToi())
		{
			++results.numValidTOI;
			continue;
		}
		if (!c->IsEnabled() || HasSensor(*c) || !IsActive(*c) || !IsImpenetrable(*c))
		{
			continue;
		}
		if (c->GetToiCount() >= step.maxSubSteps)
		{
			// What are the pros/cons of this?
			// Larger m_maxSubSteps slows down the simulation.
			// m_maxSubSteps of 44 and higher seems to decrease the occurrance of tunneling of multiple
			// bullet body collisions with static objects.
			++results.numAtMaxSubSteps;
			continue;
		}
		
		const auto fA = c->GetFixtureA();
		const auto fB = c->GetFixtureB();
		
		const auto bA = fA->GetBody();
		const auto bB = fB->GetBody();
				
		/*
		 * Put the sweeps onto the same time interval.
		 * Presumably no unresolved collisions happen before the maximum of the bodies' alpha-0 times.
		 * So long as the least TOI of the contacts is always the first collision that gets dealt with,
		 * this presumption is safe.
		 */
		const auto alpha0 = Max(bA->m_sweep.GetAlpha0(), bB->m_sweep.GetAlpha0());
		assert(alpha0 >= 0 && alpha0 < 1);
		bA->m_sweep.Advance0(alpha0);
		bB->m_sweep.Advance0(alpha0);
		
		// Compute the TOI for this contact (one or both bodies are active and impenetrable).
		// Computes the time of impact in interval [0, 1]
		// Large rotations can make the root finder of TimeOfImpact fail, so normalize the sweep angles.
		const auto output = TimeOfImpact(GetDistanceProxy(*fA->GetShape(), c->GetChildIndexA()),
										 GetAnglesNormalized(bA->m_sweep),
										 GetDistanceProxy(*fB->GetShape(), c->GetChildIndexB()),
										 GetAnglesNormalized(bB->m_sweep),
										 toiConf);
		
		
		// Use Min function to handle floating point imprecision which possibly otherwise
		// could provide a TOI that's greater than 1.
		const auto toi = IsValidForTime(output.get_state())?
			Min(alpha0 + (1 - alpha0) * output.get_t(), RealNum{1}): RealNum{1};
		assert(toi >= alpha0);
		c->SetToi(toi);
		
		results.maxDistIters = Max(results.maxDistIters, output.get_max_dist_iters());
		results.maxToiIters = Max(results.maxToiIters, output.get_toi_iters());
		results.maxRootIters = Max(results.maxRootIters, output.get_max_root_iters());
		++results.numUpdatedTOI;
	}

	return results;
}
	
World::ContactToiData World::GetSoonestContacts(const size_t reserveSize) const
{
	auto minToi = std::nextafter(RealNum{1}, RealNum{0});
	auto minContacts = std::vector<Contact*>();
	minContacts.reserve(reserveSize);
	for (auto&& c: m_contacts)
	{
		if (c->HasValidToi())
		{
			const auto toi = c->GetToi();
			if (minToi > toi)
			{
				minToi = toi;
				minContacts.clear();
				minContacts.push_back(c);
			}
			else if (minToi == toi)
			{
				// Have multiple contacts at the current minimum time of impact.
				minContacts.push_back(c);
			}
		}
	}
	return ContactToiData{minContacts, minToi};
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
		const auto updateData = UpdateContactTOIs(step);
		stats.contactsAtMaxSubSteps += updateData.numAtMaxSubSteps;
		stats.contactsUpdatedToi += updateData.numUpdatedTOI;
		stats.maxDistIters = Max(stats.maxDistIters, updateData.maxDistIters);
		stats.maxRootIters = Max(stats.maxRootIters, updateData.maxRootIters);
		stats.maxToiIters = Max(stats.maxToiIters, updateData.maxToiIters);
		
		const auto next = GetSoonestContacts(updateData.numValidTOI + updateData.numUpdatedTOI);
		const auto ncount = next.contacts.size();
		if (ncount == 0)
		{
			// No more TOI events to handle within the current time step. Done!
			SetStepComplete(true);
			break;
		}

		stats.maxSimulContacts = std::max(stats.maxSimulContacts, static_cast<decltype(stats.maxSimulContacts)>(ncount));
		stats.contactsFound += ncount;
		auto islandsFound = 0u;
		for (auto&& contact: next.contacts)
		{
			if (!contact->IsInIsland())
			{
				const auto solverResults = SolveTOI(step, *contact);
				stats.minSeparation = Min(stats.minSeparation, solverResults.minSeparation);
				stats.maxIncImpulse = Max(stats.maxIncImpulse, solverResults.maxIncImpulse);
				if (solverResults.solved)
				{
					++stats.islandsSolved;
				}
				if ((solverResults.positionIterations > 0) || (solverResults.velocityIterations > 0))
				{
					++islandsFound;
					stats.sumPosIters += solverResults.positionIterations;
					stats.sumVelIters += solverResults.velocityIterations;
				}
				break; // TODO: get working without breaking.
			}
		}
		stats.islandsFound += islandsFound;

		// Reset island flags and synchronize broad-phase proxies.
		for (auto&& body: m_bodies)
		{
			if (body->IsInIsland())
			{
				body->UnsetInIsland();
				if (body->IsAccelerable())
				{
					stats.proxiesMoved += SynchronizeFixtures(*body, step.displaceMultiplier, step.aabbExtension);
					ResetContactsForSolveTOI(*body);
				}
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		stats.contactsAdded += FindNewContacts();

		if (GetSubStepping())
		{
			SetStepComplete(false);
			break;
		}
	}
	return stats;
}

World::IslandSolverResults World::SolveTOI(const StepConf& step, Contact& contact)
{
	assert(!contact.IsInIsland());
	
	const auto toi = contact.GetToi();
	const auto bA = contact.GetFixtureA()->GetBody();
	const auto bB = contact.GetFixtureB()->GetBody();

	{
		const auto backupA = bA->m_sweep;
		const auto backupB = bB->m_sweep;

		// Advance the bodies to the TOI.
		bA->Advance(toi);
		bB->Advance(toi);

		// The TOI contact likely has some new contact points.
		contact.SetEnabled();	
		contact.Update(m_contactListener);
		contact.UnsetToi();

		++contact.m_toiCount;

		// Is contact disabled or separated?
		if (!contact.IsEnabled() || !contact.IsTouching())
		{
			// Restore the sweeps by undoing the body "advance" calls (and anything else done movement-wise)
			contact.UnsetEnabled();
			bA->m_sweep = backupA;
			bA->m_xf = GetTransform1(bA->m_sweep);
			bB->m_sweep = backupB;
			bB->m_xf = GetTransform1(bB->m_sweep);
			return IslandSolverResults{};
		}
	}

	bA->SetAwake();
	bB->SetAwake();

	// Build the island
	Island island(m_bodies.size(), m_contacts.size(), 0);

	assert(!bA->IsInIsland());
	assert(!bB->IsInIsland());
	
	island.m_bodies.push_back(bA);
	bA->SetInIsland();
	island.m_bodies.push_back(bB);
	bB->SetInIsland();
	island.m_contacts.push_back(&contact);
	contact.SetInIsland();

	// Process the contacts of the two bodies, adding appropriate ones to the island,
	// adding appropriate other bodies of added contacts, and advancing those other
	// bodies sweeps and transforms to the minimum contact's TOI.
	if (bA->IsAccelerable())
	{
		ProcessContactsForTOI(island, *bA, toi, m_contactListener);
	}
	if (bB->IsAccelerable())
	{
		ProcessContactsForTOI(island, *bB, toi, m_contactListener);
	}
	
	for (auto&& b: island.m_bodies)
	{
		// Allow static bodies to participate in other islands.
		if (!b->IsSpeedable())
		{
			b->UnsetInIsland();
		}
	}

	// Now solve for remainder of time step
	return SolveTOI(StepConf{step}.set_dt((1 - toi) * step.get_dt()), island);
}

void World::UpdateBody(Body& body, const Position& pos, const Velocity& vel)
{
	body.m_velocity = vel; // sets what Body::GetVelocity returns
	body.m_sweep.pos1 = pos; // sets what Body::GetWorldCenter returns
	body.m_xf = GetTransformation(body.m_sweep.pos1, body.m_sweep.GetLocalCenter()); // sets what Body::GetLocation returns
}

World::IslandSolverResults World::SolveTOI(const StepConf& step, Island& island)
{
	auto bodyConstraints = BodyConstraints{};
	bodyConstraints.reserve(island.m_bodies.size());

	// Initialize the body state.
#if 0
	for (auto&& contact: island.m_contacts)
	{
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();

		bodyConstraints[bodyA] = GetBodyConstraint(*bodyA);
		bodyConstraints[bodyB] = GetBodyConstraint(*bodyB);
	}
#else
	for (auto&& body: island.m_bodies)
	{
		/*
		 * Presumably the regular phase resolution has already taken care of updating the
		 * body's velocity w.r.t. acceleration and damping such that this call here to get
		 * the body constraint doesn't need to pass an elapsed time (and doesn't need to
		 * update the velocity from what it already is).
		 */
		bodyConstraints[body] = GetBodyConstraint(*body);
	}
#endif
	
	auto positionConstraints = GetPositionConstraints(island.m_contacts, bodyConstraints);
	
	// Solve TOI-based position constraints.
	auto finMinSeparation = std::numeric_limits<RealNum>::infinity();
	auto solved = false;
	auto positionIterations = step.toiPositionIterations;
	
	{
		const auto psConf = ConstraintSolverConf{}
			.UseResolutionRate(step.toiResolutionRate)
			.UseLinearSlop(step.linearSlop)
			.UseAngularSlop(step.angularSlop)
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
			const auto minSeparation = SolvePositionConstraints(positionConstraints, psConf);
			finMinSeparation = Min(finMinSeparation, minSeparation);
			if (minSeparation >= step.toiMinSeparation)
			{
				// Reached tolerance, early out...
				positionIterations = i + 1;
				solved = true;
				break;
			}
		}
	}
	
	// Leap of faith to new safe state.
	// Not doing this results in slower simulations.
	// Originally this update was only done to island.m_bodies 0 and 1.
	// Unclear whether rest of bodies should also be updated. No difference noticed.
#if 0
	for (auto&& contact: island.m_contacts)
	{
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		
		bodyA->m_sweep.pos0 = bodyConstraints[bodyA].GetPosition();
		bodyB->m_sweep.pos0 = bodyConstraints[bodyB].GetPosition();
	}
#else
	for (auto&& body: island.m_bodies)
	{
		body->m_sweep.pos0 = bodyConstraints[body].GetPosition();
	}
#endif
	
	auto velocityConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraints,
													  VelocityConstraint::Conf{0, step.velocityThreshold, true});

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.

	// Solve velocity constraints.
	auto maxIncImpulse = RealNum{0};
	auto velocityIterations = step.toiVelocityIterations;
	for (auto i = decltype(step.toiVelocityIterations){0}; i < step.toiVelocityIterations; ++i)
	{
		const auto newIncImpulse = SolveVelocityConstraints(velocityConstraints);
		maxIncImpulse = std::max(maxIncImpulse, newIncImpulse);
	}
	
	// Don't store TOI contact forces for warm starting because they can be quite large.
	
	IntegratePositions(bodyConstraints, step.get_dt(), MovementConf{step.maxTranslation, step.maxRotation});
	
	for (auto&& body: island.m_bodies)
	{
		const auto& constraint = bodyConstraints[body];
		UpdateBody(*body, constraint.GetPosition(), constraint.GetVelocity());
	}

	if (m_contactListener)
	{
		Report(*m_contactListener, island.m_contacts, velocityConstraints, positionIterations);
	}
	
	return IslandSolverResults{finMinSeparation, maxIncImpulse, solved, positionIterations, velocityIterations};
}
	
void World::ResetContactsForSolveTOI(Body& body)
{
	// Invalidate all contact TOIs on this displaced body.
	for (auto&& contact: body.GetContacts())
	{
		contact->UnsetInIsland();
		contact->UnsetToi();
	}
}

void World::ProcessContactsForTOI(Island& island, Body& body, RealNum toi, ContactListener* listener)
{
	assert(body.IsInIsland());
	assert(body.IsAccelerable());
	assert(toi >= 0 && toi <= 1);

	for (auto&& contact: body.GetContacts())
	{
		const auto fA = contact->GetFixtureA();
		const auto fB = contact->GetFixtureB();
		const auto bA = fA->GetBody();
		const auto bB = fB->GetBody();
		const auto other = (bA != &body)? bA: bB;

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
				if (other->IsSpeedable())
				{
					other->SetAwake();
				}
				island.m_bodies.push_back(other);
				other->SetInIsland();
#if 0
				if (other->IsSpeedable())
				{
					ProcessContactsForTOI(island, *other, toi, listener);
				}
#endif
			}
		}		
	}
}

StepStats World::Step(const StepConf& conf)
{
	assert((m_maxVertexRadius * 2) + (conf.linearSlop / 4) > (m_maxVertexRadius * 2));

	auto stepStats = StepStats{};

	if (HasNewFixtures())
	{
		UnsetNewFixtures();
		
		// New fixtures were added: need to find and create the new contacts.
		stepStats.pre.added = FindNewContacts();
	}

	assert(!IsLocked());
	FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

	// Update and destroy contacts. No new contacts are created though.
	const auto collideStats = Collide();
	stepStats.pre.ignored = collideStats.ignored;
	stepStats.pre.destroyed = collideStats.destroyed;
	stepStats.pre.updated = collideStats.updated;

	if (conf.get_dt() > 0)
	{
		m_inv_dt0 = conf.get_inv_dt();

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (IsStepComplete())
		{
			stepStats.reg = SolveReg(conf);
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
	wrapper.broadPhase = &m_broadPhase;
	wrapper.callback = callback;
	m_broadPhase.Query(aabb, [&](BroadPhase::size_type nodeId) {
		return wrapper.QueryCallback(nodeId);
	});
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
	WorldRayCastWrapper wrapper(&m_broadPhase, callback);
	const auto input = RayCastInput{point1, point2, RealNum{1}};
	m_broadPhase.RayCast(input, [&](const RayCastInput& rci, BroadPhase::size_type proxyId) {
		return wrapper.RayCastCallback(rci, proxyId);
	});
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
		b->m_xf.p -= newOrigin;
		b->m_sweep.pos0.linear -= newOrigin;
		b->m_sweep.pos1.linear -= newOrigin;
	}

	for (auto&& j: m_joints)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_broadPhase.ShiftOrigin(newOrigin);
}

bool World::IsActive(const Contact& contact) noexcept
{
	const auto bA = contact.GetFixtureA()->GetBody();
	const auto bB = contact.GetFixtureB()->GetBody();
	
	const auto activeA = IsAllFlagsSet(bA->m_flags, Body::e_awakeFlag|Body::e_velocityFlag);
	const auto activeB = IsAllFlagsSet(bB->m_flags, Body::e_awakeFlag|Body::e_velocityFlag);
	
	// Is at least one body active (awake and dynamic or kinematic)?
	return activeA || activeB;
}

void World::Erase(Contact* c)
{
	assert(c);
	
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
	{
		if (*iter == c)
		{
			m_contacts.erase(iter);
			break;
		}
	}
}

void World::EraseFromBodies(Contact* c)
{
	const auto fixtureA = c->GetFixtureA();
	const auto fixtureB = c->GetFixtureB();
	const auto bodyA = fixtureA->GetBody();
	const auto bodyB = fixtureB->GetBody();
	
	bodyA->Erase(c);
	bodyB->Erase(c);
}

void World::InternalDestroy(Contact* c)
{
	if (m_contactListener && c->IsTouching())
	{
		// EndContact hadn't been called in Collide() since is-touching, so call it now
		m_contactListener->EndContact(*c);
	}
	
	EraseFromBodies(c);
	
	Contact::Destroy(c, m_blockAllocator);
}

void World::Destroy(Contact* c)
{
	InternalDestroy(c);
	Erase(c);
}

void World::Destroy(Contacts::iterator iter)
{
	InternalDestroy(*iter);
	m_contacts.erase(iter);
}

World::CollideStats World::Collide()
{
	auto stats = CollideStats{};
	
	// Update awake contacts.
	auto next = m_contacts.begin();
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); iter = next)
	{
		const auto contact = *iter;
		next = std::next(iter);
		
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		
		// Is this contact flagged for filtering?
		if (contact->NeedsFiltering())
		{
			// Can these bodies collide?
			if (!(bodyB->ShouldCollide(bodyA)))
			{
				Destroy(iter);
				++stats.destroyed;
				continue;
			}
			
			// Check user filtering.
			if (m_contactFilter && !(m_contactFilter->ShouldCollide(fixtureA, fixtureB)))
			{
				Destroy(iter);
				++stats.destroyed;
				continue;
			}
			
			// Clear the filtering flag.
			contact->UnflagForFiltering();
		}
		
		// collidable means is-awake && is-speedable (dynamic or kinematic)
		auto is_collidable = [&](Body* body) {
			constexpr auto awake_and_speedable = Body::e_awakeFlag|Body::e_velocityFlag;
			return (body->m_flags & awake_and_speedable) == awake_and_speedable;
		};
		
		// At least one body must be collidable
		if (!is_collidable(bodyA) && !is_collidable(bodyB))
		{
			++stats.ignored;
			continue;
		}
		
		const auto overlap = [&]() {
			const auto indexA = contact->GetChildIndexA();
			const auto indexB = contact->GetChildIndexB();
			const auto proxyIdA = fixtureA->m_proxies[indexA].proxyId;
			const auto proxyIdB = fixtureB->m_proxies[indexB].proxyId;
			return TestOverlap(m_broadPhase, proxyIdA, proxyIdB);
		}();
		
		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (!overlap)
		{
			Destroy(iter);
			++stats.destroyed;
			continue;
		}
		
		// The contact persists.
		
		// Update the contact manifold and notify the listener.
		contact->SetEnabled();
		contact->Update(m_contactListener);
		++stats.updated;
	}
	
	return stats;
}

contact_count_t World::FindNewContacts()
{
	return m_broadPhase.UpdatePairs([&](void* a, void* b) {
		return Add(*static_cast<FixtureProxy*>(a), *static_cast<FixtureProxy*>(b));
	});
}

bool World::Add(const FixtureProxy& proxyA, const FixtureProxy& proxyB)
{
	const auto pidA = proxyA.proxyId;
	const auto fixtureA = proxyA.fixture; ///< Fixture of proxyA (but may get switched with fixtureB).
	const auto pidB = proxyB.proxyId;
	const auto fixtureB = proxyB.fixture; ///< Fixture of proxyB (but may get switched with fixtureA).
	
	assert(pidA != pidB);
	assert(sizeof(pidA) + sizeof(pidB) == sizeof(size_t));
	
	const auto bodyA = fixtureA->GetBody();
	const auto bodyB = fixtureB->GetBody();
	
	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return false;
	}
	
	const auto childIndexA = proxyA.childIndex;
	const auto childIndexB = proxyB.childIndex;
	
	// TODO: use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	for (auto&& contact: bodyB->m_contacts)
	{
		if (IsFor(*contact, fixtureA, childIndexA, fixtureB, childIndexB))
		{
			// Already have a contact for proxyA with proxyB, bail!
			return false;
		}
	}
	
	// Does a joint override collision? Is at least one body dynamic?
	if (!bodyB->ShouldCollide(bodyA))
	{
		return false;
	}
	
	// Check user filtering.
	if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
	{
		return false;
	}
	
	assert(m_contacts.size() < MaxContacts);
	
	// Call the contact factory create method.
	const auto contact = Contact::Create(*fixtureA, childIndexA, *fixtureB, childIndexB, m_blockAllocator);
	assert(contact);
	if (!contact)
	{
		return false;
	}
	
	bodyA->Insert(contact);
	bodyB->Insert(contact);
	
	// Wake up the bodies
	if (!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		bodyA->SetAwake();
		bodyB->SetAwake();
	}
	
	// Insert into the world.
	m_contacts.push_front(contact);
	
	return true;
}

void World::SetActive(Body& body, bool flag, const RealNum aabbExtension)
{
	if (body.GetWorld() != this)
	{
		return;
	}
	if (body.IsActive() == flag)
	{
		return;
	}

	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	if (flag)
	{
		body.m_flags |= Body::e_activeFlag;
		
		// Create all proxies.
		const auto xf = body.GetTransformation();
		for (auto&& fixture: body.GetFixtures())
		{
			fixture->CreateProxies(m_blockAllocator, m_broadPhase, xf, aabbExtension);
		}
		
		// Contacts are created the next time step.
	}
	else
	{
		body.m_flags &= ~Body::e_activeFlag;
		
		// Destroy all proxies.
		for (auto&& fixture: body.GetFixtures())
		{
			fixture->DestroyProxies(m_blockAllocator, m_broadPhase);
		}
		
		// Destroy the attached contacts.
		while (!body.m_contacts.empty())
		{
			auto iter = body.m_contacts.begin();
			const auto contact = *iter;
			body.m_contacts.erase(iter);
			Destroy(contact);
		}
	}
}

void World::Refilter(Fixture& fixture)
{
	const auto body = fixture.GetBody();
	if (body)
	{
		if (body->GetWorld() != this)
		{
			return;
		}

		// Flag associated contacts for filtering.
		for (auto&& contact: body->GetContacts())
		{
			const auto fixtureA = contact->GetFixtureA();
			const auto fixtureB = contact->GetFixtureB();
			if ((fixtureA == &fixture) || (fixtureB == &fixture))
			{
				contact->FlagForFiltering();
			}
		}
	
		for (auto i = decltype(fixture.m_proxyCount){0}; i < fixture.m_proxyCount; ++i)
		{
			m_broadPhase.TouchProxy(fixture.m_proxies[i].proxyId);
		}
	}
}

void World::DestroyFixtures(Body& body)
{
	if (body.GetWorld() != this)
	{
		return;
	}

	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	// Delete the attached fixtures. This destroys broad-phase proxies.
	while (!body.m_fixtures.empty())
	{
		const auto fixture = body.m_fixtures.front();
		body.m_fixtures.pop_front();
		
		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(*fixture);
		}
		
		fixture->DestroyProxies(m_blockAllocator, m_broadPhase);
		Delete(fixture, m_blockAllocator);
	}
	body.ResetMassData();
}

void World::SetType(Body& body, BodyType type, const RealNum aabbExtension)
{
	if (body.GetWorld() != this)
	{
		return;
	}
	if (body.GetType() == type)
	{
		return;
	}

	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	body.m_flags &= ~(Body::e_impenetrableFlag|Body::e_velocityFlag|Body::e_accelerationFlag);
	body.m_flags |= Body::GetFlags(type);	
	body.ResetMassData();
	
	if (type == BodyType::Static)
	{
		body.m_velocity = Velocity{Vec2_zero, 0_rad};
		body.m_sweep.pos0 = body.m_sweep.pos1;
		
		// Note: displacement multiplier has no effect here since no displacement.
		SynchronizeFixtures(body, 0, aabbExtension);
	}
	
	body.SetAwake();
	
	body.m_linearAcceleration = Vec2_zero;
	body.m_angularAcceleration = 0_rad;
	if (body.IsAccelerable())
	{
		body.m_linearAcceleration += GetGravity();
	}
	
	// Destroy the attached contacts.
	while (!body.m_contacts.empty())
	{
		auto iter = body.m_contacts.begin();
		const auto contact = *iter;
		body.m_contacts.erase(iter);
		Destroy(contact);
	}
	
	for (auto&& fixture: body.GetFixtures())
	{
		fixture->TouchProxies(m_broadPhase);
	}
}

bool World::IsValid(std::shared_ptr<const Shape> shape) const noexcept
{
	if (!shape)
	{
		return false;
	}
	const auto vr = GetVertexRadius(*shape);
	if (!(vr >= GetMinVertexRadius()))
	{
		return false;
	}
	if (!(vr <= GetMaxVertexRadius()))
	{
		return false;
	}
	return true;
}

Fixture* World::CreateFixture(Body& body, std::shared_ptr<const Shape> shape,
					   const FixtureDef& def, bool resetMassData)
{
	if (body.GetWorld() != this)
	{
		return nullptr;
	}
	if (!IsValid(shape) || !Body::IsValid(def))
	{
		return nullptr;
	}
	
	assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}
	
	const auto memory = m_blockAllocator.Allocate(sizeof(Fixture));
	const auto fixture = new (memory) Fixture{&body, def, shape};
	
	if (body.IsActive())
	{
		fixture->CreateProxies(m_blockAllocator, m_broadPhase, body.GetTransformation(), def.aabbExtension);
	}
	
	body.m_fixtures.push_front(fixture);
	
	// Adjust mass properties if needed.
	if (fixture->GetDensity() > 0)
	{
		body.SetMassDataDirty();
		if (resetMassData)
		{
			body.ResetMassData();
		}
	}
	
	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	SetNewFixtures();
	
	return fixture;
}

bool World::DestroyFixture(Fixture* fixture, bool resetMassData)
{
	if (!fixture)
	{
		return false;
	}
	auto& body = *fixture->GetBody();
	if (body.GetWorld() != this)
	{
		return false;
	}
	if (IsLocked())
	{
		return false;
	}
	
	// Remove the fixture from this body's singly linked list.
	const auto found = body.Erase(fixture);
	if (!found)
	{
		// Fixture probably destroyed already.
		return false;
	}
	
	// Destroy any contacts associated with the fixture.
	for (auto&& contact: body.m_contacts)
	{
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		if ((fixture == fixtureA) || (fixture == fixtureB))
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			Destroy(contact);
		}
	}
	
	fixture->DestroyProxies(m_blockAllocator, m_broadPhase);
	
	Delete(fixture, m_blockAllocator);
	
	body.SetMassDataDirty();
	if (resetMassData)
	{
		body.ResetMassData();
	}
	
	return true;
}

contact_count_t World::SynchronizeFixtures(Body& body,
										   const Transformation& t1, const Transformation& t2,
										  const RealNum multiplier, const RealNum aabbExtension)
{
	auto movedCount = contact_count_t{0};
	for (auto&& fixture: body.GetFixtures())
	{
		movedCount += fixture->Synchronize(m_broadPhase, t1, t2, multiplier, aabbExtension);
	}
	return movedCount;
}

contact_count_t World::SynchronizeFixtures(Body& body, const RealNum multiplier, const RealNum aabbExtension)
{
	return SynchronizeFixtures(body, GetTransform0(body.m_sweep), body.GetTransformation(),
							   multiplier, aabbExtension);
}

void World::SetTransform(Body& body, const Vec2 position, Angle angle, const RealNum aabbExtension)
{
	assert(::box2d::IsValid(position));
	assert(::box2d::IsValid(angle));
	
	if (body.GetWorld() != this)
	{
		return;
	}

	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	const auto xf = Transformation{position, UnitVec2{angle}};
	body.m_xf = xf;
	body.m_sweep = Sweep{Position{Transform(body.GetLocalCenter(), xf), angle}, body.GetLocalCenter()};
	
	// Note that distanceMultiplier parameter has no effect here since no displacement.
	SynchronizeFixtures(body, xf, xf, 0, aabbExtension);
}

// Free functions...

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
	for (auto&& body: world.GetBodies())
	{
		sum += GetFixtureCount(*body);
	}
	return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
	auto shapes = std::set<const Shape*>();
	for (auto&& body: world.GetBodies())
	{
		for (auto&& fixture: body->GetFixtures())
		{
			shapes.insert(fixture->GetShape());
		}
	}
	return shapes.size();
}

size_t GetAwakeCount(const World& world) noexcept
{
	auto count = size_t(0);
	for (auto&& body: world.GetBodies())
	{
		if (body->IsAwake())
		{
			++count;
		}
	}
	return count;
}
	
size_t Awaken(World& world)
{
	auto awoken = size_t{0};
	for (auto&& body: world.GetBodies())
	{
		if (body->SetAwake())
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
		body->SetAcceleration(g, 0_rad);
	}
}

} // namespace box2d
