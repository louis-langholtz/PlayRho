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

	inline MovementConf GetMovementConf(const StepConf& conf)
	{
		return MovementConf{conf.maxTranslation, conf.maxRotation};
	}

	inline ConstraintSolverConf GetRegConstraintSolverConf(const StepConf& conf)
	{
		return ConstraintSolverConf{}
			.UseResolutionRate(conf.regResolutionRate)
			.UseLinearSlop(conf.linearSlop)
			.UseAngularSlop(conf.angularSlop)
			.UseMaxLinearCorrection(conf.maxLinearCorrection)
			.UseMaxAngularCorrection(conf.maxAngularCorrection);
	}
	
	inline ConstraintSolverConf GetToiConstraintSolverConf(const StepConf& conf)
	{
		return ConstraintSolverConf{}
			.UseResolutionRate(conf.toiResolutionRate)
			.UseLinearSlop(conf.linearSlop)
			.UseAngularSlop(conf.angularSlop)
			.UseMaxLinearCorrection(conf.maxLinearCorrection)
			.UseMaxAngularCorrection(conf.maxAngularCorrection);
	}
	
	inline ToiConf GetToiConf(const StepConf& conf)
	{
		return ToiConf{}
			.UseTimeMax(1)
			.UseTargetDepth(conf.targetDepth)
			.UseTolerance(conf.tolerance)
			.UseMaxRootIters(conf.maxToiRootIters)
			.UseMaxToiIters(conf.maxToiIters)
			.UseMaxDistIters(conf.maxDistanceIters);
	}
	
	/// Calculates movement.
	/// @detail Calculate the positional displacement based on the given velocity
	///    that's possibly clamped to the maximum translation and rotation.
	inline PositionAndVelocity CalculateMovement(const BodyConstraint& body, TimeSpan h, MovementConf conf)
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
								   TimeSpan h, MovementConf conf)
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
	
	inline RealNum GetUnderActiveTime(const Body& b, const StepConf& conf) noexcept
	{
		const auto underactive = IsUnderActive(b.GetVelocity(), conf.linearSleepTolerance, conf.angularSleepTolerance);
		const auto sleepable = b.IsSleepingAllowed();
		return (sleepable && underactive)? b.GetUnderActiveTime() + conf.get_dt(): RealNum{0};
	}

	inline TimeSpan UpdateUnderActiveTimes(Island::Bodies& bodies, const StepConf& conf)
	{
		auto minUnderActiveTime = std::numeric_limits<TimeSpan>::infinity();
		for (auto&& b: bodies)
		{
			if (b->IsSpeedable())
			{
				const auto underActiveTime = GetUnderActiveTime(*b, conf);
				b->SetUnderActiveTime(underActiveTime);
				minUnderActiveTime = Min(minUnderActiveTime, underActiveTime);
			}
		}
		return minUnderActiveTime;
	}
	
	inline size_t Sleepem(Island::Bodies& bodies)
	{
		auto unawoken = size_t{0};
		for (auto&& b: bodies)
		{
			if (Unawaken(*b))
			{
				++unawoken;
			}
		}
		return unawoken;
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
	
	inline bool TestOverlap(const BroadPhase& bp,
							const Fixture* fixtureA, child_count_t indexA,
							const Fixture* fixtureB, child_count_t indexB)
	{
		const auto proxyIdA = fixtureA->GetProxy(indexA)->proxyId;
		const auto proxyIdB = fixtureB->GetProxy(indexB)->proxyId;
		return TestOverlap(bp, proxyIdA, proxyIdB);
	}

} // anonymous namespace


/// Fixture Attorney.
///
/// @detail This class uses the "attorney-client" idiom to control the granularity of
///   friend-based access to the Fixture class. This is meant to help preserve and enforce
///   the invariants of the Fixture class.
///
/// @sa https://en.wikibooks.org/wiki/More_C++_Idioms/Friendship_and_the_Attorney-Client
///
class FixtureAtty
{
private:
	static Span<FixtureProxy> GetProxies(const Fixture& fixture)
	{
		return fixture.GetProxies();
	}
	
	static void SetProxies(Fixture& fixture, Span<FixtureProxy> value)
	{
		fixture.SetProxies(value);
	}
	
	static Fixture* Create(Body* body, const FixtureDef& def, std::shared_ptr<const Shape> shape)
	{
		return new Fixture{body, def, shape};
	}
	
	friend class World;
};

class ContactAtty
{
private:
	static Contact* Create(Fixture& fixtureA, child_count_t indexA,
						   Fixture& fixtureB, child_count_t indexB)
	{
		return Contact::Create(fixtureA, indexA, fixtureB, indexB);
	}

	static void Destroy(Contact* c)
	{
		Contact::Destroy(c);
	}
	
	static void SetToi(Contact& c, RealNum value) noexcept
	{
		c.SetToi(value);
	}

	static void UnsetToi(Contact& c) noexcept
	{
		c.UnsetToi();
	}

	static void IncrementToiCount(Contact& c) noexcept
	{
		++c.m_toiCount;
	}
	
	static void ResetToiCount(Contact& c) noexcept
	{
		c.ResetToiCount();
	}
	
	static void UnflagForFiltering(Contact& c) noexcept
	{
		c.UnflagForFiltering();
	}
	
	static void Update(Contact& c, ContactListener* listener)
	{
		c.Update(listener);
	}
	
	friend class World;
};

class JointAtty
{
private:
	static Joint* Create(const box2d::JointDef &def)
	{
		return Joint::Create(def);
	}
	
	static void Destroy(Joint* j)
	{
		Joint::Destroy(j);
	}
	
	static void InitVelocityConstraints(Joint& j, BodyConstraints &bodies,
										const box2d::StepConf &step, const ConstraintSolverConf &conf)
	{
		j.InitVelocityConstraints(bodies, step, conf);
	}
	
	static RealNum SolveVelocityConstraints(Joint& j, BodyConstraints &bodies, const box2d::StepConf &conf)
	{
		return j.SolveVelocityConstraints(bodies, conf);
	}
	
	static bool SolvePositionConstraints(Joint& j, BodyConstraints &bodies, const ConstraintSolverConf &conf)
	{
		return j.SolvePositionConstraints(bodies, conf);
	}
	
	friend class World;
};

class BodyAtty
{
private:
	static Body* Create(World* world, const BodyDef& def)
	{
		return new Body(def, world);
	}

	static void Destruct(Body* b)
	{
		delete b;
	}
	
	static void SetTypeFlags(Body& b, BodyType type) noexcept
	{
		b.m_flags &= ~(Body::e_impenetrableFlag|Body::e_velocityFlag|Body::e_accelerationFlag);
		b.m_flags |= Body::GetFlags(type);
		
		switch (type)
		{
			case BodyType::Dynamic:
				break;
			case BodyType::Kinematic:
				break;
			case BodyType::Static:
				b.UnsetAwakeFlag();
				b.m_underActiveTime = 0;
				b.m_velocity = Velocity{Vec2_zero, 0_rad};
				b.m_sweep.pos0 = b.GetSweep().pos1;
				break;
		}
	}
	
	static void SetMassDataDirty(Body& b) noexcept
	{
		b.SetMassDataDirty();
	}

	static bool Erase(Body& b, Fixture* value)
	{
		return b.Erase(value);
	}
	
	static bool Erase(Body& b, Contact* value)
	{
		return b.Erase(value);
	}

	static bool Erase(Body& b, Joint* value)
	{
		return b.Erase(value);
	}

	static bool Insert(Body& b, Joint* value)
	{
		return b.Insert(value);
	}
	
	static bool Insert(Body& b, Contact* value)
	{
		return b.Insert(value);
	}
	
	static bool Insert(Body& b, Fixture* value)
	{
		b.m_fixtures.push_front(value);
		return true;
	}

	static void SetPosition0(Body& b, Position value) noexcept
	{
		b.m_sweep.pos0 = value;
	}

	/// Sets the body sweep's position 1 value.
	/// @note This sets what Body::GetWorldCenter returns.
	static void SetPosition1(Body& b, Position value) noexcept
	{
		b.m_sweep.pos1 = value;
	}

	static void ResetAlpha0(Body& b)
	{
		b.m_sweep.ResetAlpha0();
	}
	
	static void SetSweep(Body& b, const Sweep value) noexcept
	{
		b.m_sweep = value;
	}
	
	/// Sets the body's transformation.
	/// @note This sets what Body::GetLocation returns.
	static void SetTransformation(Body& b, const Transformation value) noexcept
	{
		b.SetTransformation(value);
	}
	
	/// Sets the body's velocity.
	/// @note This sets what Body::GetVelocity returns.
	static void SetVelocity(Body& b, Velocity value) noexcept
	{
		b.m_velocity = value;
	}
	
	static void Advance0(Body& b, RealNum value) noexcept
	{
		b.m_sweep.Advance0(value);
	}
	
	static void Advance(Body& b, RealNum toi) noexcept
	{
		b.Advance(toi);
	}

	static void Restore(Body& b, const Sweep value) noexcept
	{
		BodyAtty::SetSweep(b, value);
		BodyAtty::SetTransformation(b, GetTransform1(value));
	}

	static void ClearFixtures(Body& b, std::function<void(Fixture&)> callback)
	{
		while (!b.m_fixtures.empty())
		{
			const auto fixture = b.m_fixtures.front();
			callback(*fixture);
			delete fixture;
			b.m_fixtures.pop_front();
		}
	}
	
	static void ClearJoints(Body& b, std::function<void(Joint&)> callback)
	{
		while (!b.m_joints.empty())
		{
			auto iter = b.m_joints.begin();
			const auto joint = *iter;
			b.m_joints.erase(iter);
			callback(*joint);
		}
	}
	
	static void EraseContacts(Body& b, std::function<bool(Contact&)> callback)
	{
		const auto end = b.m_contacts.end();
		auto iter = b.m_contacts.begin();
		while (iter != end)
		{
			const auto contact = *iter;
			if (callback(*contact))
			{
				const auto next = std::next(iter);
				b.m_contacts.erase(iter);
				iter = next;
			}
			else
			{
				iter = std::next(iter);
			}
		}
	}
	
	friend class World;
};

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
		BodyAtty::Erase(*bodyA, c);
		BodyAtty::Erase(*bodyB, c);
		m_contacts.pop_front();
		ContactAtty::Destroy(c);
	}

	// Gets rid of the created joints.
	while (!m_joints.empty())
	{
		const auto j = m_joints.front();
		const auto bodyA = j->GetBodyA();
		const auto bodyB = j->GetBodyB();
		if (bodyA)
		{
			BodyAtty::Erase(*bodyA, j);
		}
		if (bodyB)
		{
			BodyAtty::Erase(*bodyB, j);
		}
		m_joints.pop_front();
		JointAtty::Destroy(j);
	}

	// Gets rid of the created bodies and any associated fixtures.
	while (!m_bodies.empty())
	{
		const auto b = m_bodies.front();
		m_bodies.pop_front();
		BodyAtty::ClearFixtures(*b, [&](Fixture& fixture) {
			if (m_destructionListener)
			{
				m_destructionListener->SayGoodbye(fixture);
			}
			DestroyProxies(fixture);
		});
		assert(b->GetJoints().empty());
		assert(b->GetContacts().empty());
		BodyAtty::Destruct(b);
	}
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

	auto b = BodyAtty::Create(this, def);
	if (b)
	{
		if (!Add(*b))
		{
			BodyAtty::Destruct(b);
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
	assert(b->GetWorld() == this);
	
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	// Delete the attached joints.
	BodyAtty::ClearJoints(*b, [&](Joint& joint) {
		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(joint);
		}
		InternalDestroy(&joint);
	});
	
	// Destroy the attached contacts.
	BodyAtty::EraseContacts(*b, [&](Contact& contact) {
		Destroy(&contact, b);
		return true;
	});
	
	// Delete the attached fixtures. This destroys broad-phase proxies.
	BodyAtty::ClearFixtures(*b, [&](Fixture& fixture) {
		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(fixture);
		}
		DestroyProxies(fixture);
	});
	
	Remove(*b);
	
	BodyAtty::Destruct(b);
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
	auto j = JointAtty::Create(def);
	if (!j)
	{
		return nullptr;
	}

	// Connect to the bodies' doubly linked lists.
	const auto bodyA = j->GetBodyA();
	const auto bodyB = j->GetBodyB();
	if (bodyA)
	{
		BodyAtty::Insert(*bodyA, j);
	}
	if (bodyB)
	{
		BodyAtty::Insert(*bodyB, j);
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
	if (!j)
	{
		return;
	}
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
	
	// Disconnect from island graph.
	const auto bodyA = j->GetBodyA();
	const auto bodyB = j->GetBodyB();

	// Wake up connected bodies.
	if (bodyA)
	{
		bodyA->SetAwake();
		BodyAtty::Erase(*bodyA, j);
	}
	if (bodyB)
	{
		bodyB->SetAwake();
		BodyAtty::Erase(*bodyB, j);
	}

	const auto collideConnected = j->GetCollideConnected();

	JointAtty::Destroy(j);

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
	assert(!m_bodiesIslanded.count(&seed));
	assert(seed.IsSpeedable());
	assert(seed.IsAwake());
	assert(seed.IsEnabled());
	assert(remNumBodies != 0);
	assert(remNumBodies < MaxBodies);

	// Size the island for the remaining un-evaluated bodies, contacts, and joints.
	Island island(remNumBodies, remNumContacts, remNumJoints);

	// Perform a depth first search (DFS) on the constraint graph.

	// Create a stack for bodies to be islanded that aren't already islanded.
	auto stack = std::vector<Body*>();
	stack.reserve(remNumBodies);

	stack.push_back(&seed);
	m_bodiesIslanded.insert(&seed);
	
	while (!stack.empty())
	{
		// Grab the next body off the stack and add it to the island.
		const auto b = stack.back();
		stack.pop_back();
		
		assert(b->IsEnabled());
		island.m_bodies.push_back(b);
		assert(remNumBodies > 0);
		--remNumBodies;
		
		// Don't propagate islands across bodies that can't have a velocity (static bodies).
		// This keeps islands smaller and helps with isolating separable collision clusters.
		if (!b->IsSpeedable())
		{
			continue;
		}

		// Make sure the body is awake.
		b->SetAwake();

		const auto oldNumContacts = island.m_contacts.size();
		// Adds appropriate contacts of current body and appropriate 'other' bodies of those contacts.
		for (auto&& contact: b->GetContacts())
		{
			const auto fA = contact->GetFixtureA();
			const auto fB = contact->GetFixtureB();
			const auto bA = fA->GetBody();
			const auto bB = fB->GetBody();
			const auto other = (bA != b)? bA: bB;

			if (!m_contactsIslanded.count(contact) && !HasSensor(*contact) && contact->IsEnabled() && contact->IsTouching())
			{
				island.m_contacts.push_back(contact);
				m_contactsIslanded.insert(contact);

				if (!m_bodiesIslanded.count(other))
				{				
					stack.push_back(other);
					m_bodiesIslanded.insert(other);
				}
			}			
		}
		
		const auto newNumContacts = island.m_contacts.size();
		assert(newNumContacts >= oldNumContacts);
		const auto netNumContacts = newNumContacts - oldNumContacts;
		assert(remNumContacts >= netNumContacts);
		remNumContacts -= netNumContacts;
		
		const auto numJoints = island.m_joints.size();
		// Adds appropriate joints of current body and appropriate 'other' bodies of those joint.
		for (auto&& joint: b->GetJoints())
		{
			const auto bodyA = joint->GetBodyA();
			const auto bodyB = joint->GetBodyB();
			const auto other = (b != bodyA)? bodyA: bodyB;
			if (!m_jointsIslanded.count(joint) && other->IsEnabled())
			{
				island.m_joints.push_back(joint);
				m_jointsIslanded.insert(joint);
				if (!m_bodiesIslanded.count(other))
				{					
					stack.push_back(other);
					m_bodiesIslanded.insert(other);
				}
			}
		}
		remNumJoints -= island.m_joints.size() - numJoints;
	}
	
	return island;
}
	
RegStepStats World::SolveReg(const StepConf& conf)
{
	auto stats = RegStepStats{};

	auto remNumBodies = m_bodies.size(); ///< Remaining number of bodies.
	auto remNumContacts = m_contacts.size(); ///< Remaining number of contacts.
	auto remNumJoints = m_joints.size(); ///< Remaining number of joints.

	// Clear all the island flags.
	// This builds the logical set of bodies, contacts, and joints eligible for resolution.
	// As bodies, contacts, or joints get added to resolution islands, they're essentially
	// removed from this eligible set.
	m_bodiesIslanded.clear();
	m_bodiesIslanded.reserve(remNumBodies);
	m_contactsIslanded.clear();
	m_contactsIslanded.reserve(remNumContacts);
	m_jointsIslanded.clear();
	m_jointsIslanded.reserve(remNumJoints);

#if defined(DO_THREADED)
	std::vector<std::future<World::IslandSolverResults>> futures;
	futures.reserve(remNumBodies);
#endif
	// Build and simulate all awake islands.
	for (auto&& body: m_bodies)
	{
		assert(!body->IsAwake() || body->IsSpeedable());
		if (!m_bodiesIslanded.count(body) && body->IsAwake() && body->IsEnabled())
		{
			++stats.islandsFound;

			auto island = BuildIsland(*body, remNumBodies, remNumContacts, remNumJoints);
			for (auto&& b: island.m_bodies)
			{
				// Allow static bodies to participate in other islands.
				if (!b->IsSpeedable())
				{
					m_bodiesIslanded.erase(b);
					++remNumBodies;
				}
			}

#if defined(DO_THREADED)
			// Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
			futures.push_back(std::async(&World::SolveRegIsland, this, conf, island));
#else
			const auto solverResults = SolveRegIsland(conf, island);
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
		if (body->IsSpeedable() && m_bodiesIslanded.count(body))
		{
			// Update fixtures (for broad-phase).
			stats.proxiesMoved += Synchronize(*body, GetTransform0(body->GetSweep()), body->GetTransformation(),
						conf.displaceMultiplier, conf.aabbExtension);
		}
	}

	// Look for new contacts.
	stats.contactsAdded = FindNewContacts();
	
	return stats;
}

World::IslandSolverResults World::SolveRegIsland(const StepConf& conf, Island island)
{
	auto finMinSeparation = std::numeric_limits<RealNum>::infinity();
	auto solved = false;
	auto positionIterations = conf.regPositionIterations;
	const auto h = conf.get_dt(); ///< Time step.

	auto bodyConstraints = BodyConstraints{};
	bodyConstraints.reserve(island.m_bodies.size());

	// Update bodies' pos0 values then copy their pos1 and velocity data into local arrays.
	for (auto&& body: island.m_bodies)
	{
		BodyAtty::SetPosition0(*body, GetPosition1(*body)); // like Advance0(1) on the sweep.
		bodyConstraints[body] = GetBodyConstraint(*body, h); // new velocity = acceleration * h
	}
	auto positionConstraints = GetPositionConstraints(island.m_contacts, bodyConstraints);
	auto velocityConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraints,
													  VelocityConstraint::Conf{conf.doWarmStart? conf.dtRatio: 0, conf.velocityThreshold, true});
	
	if (conf.doWarmStart)
	{
		WarmStartVelocities(velocityConstraints);
	}

	const auto psConf = GetRegConstraintSolverConf(conf);

	for (auto&& joint: island.m_joints)
	{
		JointAtty::InitVelocityConstraints(*joint, bodyConstraints, conf, psConf);
	}
	
	auto velocityIterations = conf.regVelocityIterations;
	auto maxIncImpulse = RealNum{0};
	for (auto i = decltype(conf.regVelocityIterations){0}; i < conf.regVelocityIterations; ++i)
	{
		for (auto&& joint: island.m_joints)
		{
			JointAtty::SolveVelocityConstraints(*joint, bodyConstraints, conf);
		}
		const auto newIncImpulse = SolveVelocityConstraints(velocityConstraints);
		maxIncImpulse = std::max(maxIncImpulse, newIncImpulse);
	}
	
	// updates array of tentative new body positions per the velocities as if there were no obstacles...
	IntegratePositions(bodyConstraints, h, GetMovementConf(conf));
	
	// Solve position constraints
	for (auto i = decltype(conf.regPositionIterations){0}; i < conf.regPositionIterations; ++i)
	{
		const auto minSeparation = SolvePositionConstraints(positionConstraints, psConf);
		finMinSeparation = Min(finMinSeparation, minSeparation);
		const auto contactsOkay = (minSeparation >= conf.regMinSeparation);

		const auto jointsOkay = [&]()
		{
			auto allOkay = true;
			for (auto&& joint: island.m_joints)
			{
				if (!JointAtty::SolvePositionConstraints(*joint, bodyConstraints, psConf))
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
	
	auto bodiesSlept = body_count_t{0};
	if (::box2d::IsValid(conf.minStillTimeToSleep))
	{
		const auto minUnderActiveTime = UpdateUnderActiveTimes(island.m_bodies, conf);
		if ((minUnderActiveTime >= conf.minStillTimeToSleep) && solved)
		{
			bodiesSlept = static_cast<decltype(bodiesSlept)>(Sleepem(island.m_bodies));
		}
	}

	return IslandSolverResults{finMinSeparation, maxIncImpulse, solved,
		positionIterations, velocityIterations, bodiesSlept};
}

void World::ResetBodiesForSolveTOI()
{
	m_bodiesIslanded.clear();
	for (auto&& b: m_bodies)
	{
		BodyAtty::ResetAlpha0(*b);
	}
}

void World::ResetContactsForSolveTOI()
{
	m_contactsIslanded.clear();
	for (auto&& c: m_contacts)
	{
		// Invalidate TOI
		ContactAtty::UnsetToi(*c);
		ContactAtty::ResetToiCount(*c);
	}
}

static Sweep Transform(Sweep sweep, Transformation xfm)
{
	return Sweep{
		Position{Transform(sweep.pos0.linear, xfm), sweep.pos0.angular},
		Position{Transform(sweep.pos1.linear, xfm), sweep.pos1.angular},
		sweep.GetLocalCenter(), sweep.GetAlpha0()
	};
}

World::UpdateContactsData World::UpdateContactTOIs(const StepConf& conf)
{
	auto results = UpdateContactsData{};

	const auto toiConf = GetToiConf(conf);
	
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
		if (c->GetToiCount() >= conf.maxSubSteps)
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
		const auto alpha0 = Max(bA->GetSweep().GetAlpha0(), bB->GetSweep().GetAlpha0());
		assert(alpha0 >= 0 && alpha0 < 1);
		BodyAtty::Advance0(*bA, alpha0);
		BodyAtty::Advance0(*bB, alpha0);
		
		const auto proxyA = GetDistanceProxy(*fA->GetShape(), c->GetChildIndexA());
		const auto sweepA = Transform(GetAnglesNormalized(bA->GetSweep()), fA->GetTransformation());
		const auto proxyB = GetDistanceProxy(*fB->GetShape(), c->GetChildIndexB());
		const auto sweepB = Transform(GetAnglesNormalized(bB->GetSweep()), fB->GetTransformation());

		// Compute the TOI for this contact (one or both bodies are active and impenetrable).
		// Computes the time of impact in interval [0, 1]
		// Large rotations can make the root finder of TimeOfImpact fail, so normalize the sweep angles.
		const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, toiConf);
		
		
		// Use Min function to handle floating point imprecision which possibly otherwise
		// could provide a TOI that's greater than 1.
		const auto toi = IsValidForTime(output.get_state())?
			Min(alpha0 + (1 - alpha0) * output.get_t(), RealNum{1}): RealNum{1};
		assert(toi >= alpha0);
		ContactAtty::SetToi(*c, toi);
		
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

ToiStepStats World::SolveTOI(const StepConf& conf)
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
		const auto updateData = UpdateContactTOIs(conf);
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

		stats.maxSimulContacts = std::max(stats.maxSimulContacts,
										  static_cast<decltype(stats.maxSimulContacts)>(ncount));
		stats.contactsFound += ncount;
		auto islandsFound = 0u;
		for (auto&& contact: next.contacts)
		{
			if (!m_contactsIslanded.count(contact))
			{
				const auto solverResults = SolveTOI(conf, *contact);
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
			if (m_bodiesIslanded.count(body))
			{
				m_bodiesIslanded.erase(body);
				if (body->IsAccelerable())
				{
					stats.proxiesMoved += Synchronize(*body, GetTransform0(body->GetSweep()), body->GetTransformation(),
													  conf.displaceMultiplier, conf.aabbExtension);
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

World::IslandSolverResults World::SolveTOI(const StepConf& conf, Contact& contact)
{
	assert(!m_contactsIslanded.count(&contact));
	
	const auto toi = contact.GetToi();
	const auto bA = contact.GetFixtureA()->GetBody();
	const auto bB = contact.GetFixtureB()->GetBody();

	{
		const auto backupA = bA->GetSweep();
		const auto backupB = bB->GetSweep();

		// Advance the bodies to the TOI.
		BodyAtty::Advance(*bA, toi);
		BodyAtty::Advance(*bB, toi);

		// The TOI contact likely has some new contact points.
		contact.SetEnabled();	
		ContactAtty::Update(contact, m_contactListener);
		ContactAtty::UnsetToi(contact);
		ContactAtty::IncrementToiCount(contact);

		// Is contact disabled or separated?
		//
		// XXX: Not often, but sometimes, contact.IsTouching() is false now.
		//      Seems like this is a bug, or at least suboptimal, condition.
		//      This method shouldn't be getting called unless contact has an
		//      impact indeed at the given TOI. Seen this happen in an edge-polygon
		//      contact situation where the polygon had a larger than default
		//      vertex radius. CollideShapes had called GetManifoldFaceB which
		//      was failing to see 2 clip points after GetClipPoints was called.
		if (!contact.IsEnabled() || !contact.IsTouching())
		{
			contact.UnsetEnabled();
			BodyAtty::Restore(*bA, backupA);
			BodyAtty::Restore(*bB, backupB);
			return IslandSolverResults{};
		}
	}

	bA->SetAwake();
	bB->SetAwake();

	// Build the island
	Island island(m_bodies.size(), m_contacts.size(), 0);

	assert(!m_bodiesIslanded.count(bA));
	assert(!m_bodiesIslanded.count(bB));
	
	island.m_bodies.push_back(bA);
	m_bodiesIslanded.insert(bA);
	island.m_bodies.push_back(bB);
	m_bodiesIslanded.insert(bB);
	island.m_contacts.push_back(&contact);
	m_contactsIslanded.insert(&contact);

	// Process the contacts of the two bodies, adding appropriate ones to the island,
	// adding appropriate other bodies of added contacts, and advancing those other
	// bodies sweeps and transforms to the minimum contact's TOI.
	if (bA->IsAccelerable())
	{
		ProcessContactsForTOI(island, *bA, toi);
	}
	if (bB->IsAccelerable())
	{
		ProcessContactsForTOI(island, *bB, toi);
	}
	
	for (auto&& b: island.m_bodies)
	{
		// Allow static bodies to participate in other islands.
		if (!b->IsSpeedable())
		{
			m_bodiesIslanded.erase(b);
		}
	}

	// Now solve for remainder of time step
	return SolveTOI(StepConf{conf}.set_dt((1 - toi) * conf.get_dt()), island);
}

void World::UpdateBody(Body& body, const Position& pos, const Velocity& vel)
{
	BodyAtty::SetVelocity(body, vel);
	BodyAtty::SetPosition1(body, pos);
	BodyAtty::SetTransformation(body, GetTransformation(GetPosition1(body), body.GetLocalCenter()));
}

World::IslandSolverResults World::SolveTOI(const StepConf& conf, Island& island)
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
	auto positionIterations = conf.toiPositionIterations;
	
	{
		const auto psConf = GetToiConstraintSolverConf(conf);

		for (auto i = decltype(conf.toiPositionIterations){0}; i < conf.toiPositionIterations; ++i)
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
			if (minSeparation >= conf.toiMinSeparation)
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
		BodyAtty::SetPosition0(*body, bodyConstraints[body].GetPosition());
	}
#endif
	
	auto velocityConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraints,
													  VelocityConstraint::Conf{0, conf.velocityThreshold, true});

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.

	// Solve velocity constraints.
	auto maxIncImpulse = RealNum{0};
	auto velocityIterations = conf.toiVelocityIterations;
	for (auto i = decltype(conf.toiVelocityIterations){0}; i < conf.toiVelocityIterations; ++i)
	{
		const auto newIncImpulse = SolveVelocityConstraints(velocityConstraints);
		maxIncImpulse = std::max(maxIncImpulse, newIncImpulse);
	}
	
	// Don't store TOI contact forces for warm starting because they can be quite large.
	
	IntegratePositions(bodyConstraints, conf.get_dt(), GetMovementConf(conf));
	
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
		m_contactsIslanded.erase(contact);
		ContactAtty::UnsetToi(*contact);
	}
}

void World::ProcessContactsForTOI(Island& island, Body& body, RealNum toi)
{
	assert(m_bodiesIslanded.count(&body));
	assert(body.IsAccelerable());
	assert(toi >= 0 && toi <= 1);

	// Note: the original contact (for body of which this method was called) already islanded.
	for (auto&& contact: body.GetContacts())
	{
		const auto fA = contact->GetFixtureA();
		const auto fB = contact->GetFixtureB();
		const auto bA = fA->GetBody();
		const auto bB = fB->GetBody();
		const auto other = (bA != &body)? bA: bB;

		if (!m_contactsIslanded.count(contact) && !HasSensor(*contact) && (other->IsImpenetrable() || body.IsImpenetrable()))
		{
			// Tentatively advance the body to the TOI.
			const auto backup = other->GetSweep();
			if (!m_bodiesIslanded.count(other))
			{
				BodyAtty::Advance(*other, toi);
			}
			
			// Update the contact points
			contact->SetEnabled();
			ContactAtty::Update(*contact, m_contactListener);
			
			// Revert and skip if contact disabled by user or not touching anymore (very possible).
			if (!contact->IsEnabled() || !contact->IsTouching())
			{
				BodyAtty::Restore(*other, backup);
				continue;
			}
			
			island.m_contacts.push_back(contact);
			m_contactsIslanded.insert(contact);
			
			if (!m_bodiesIslanded.count(other))
			{
				if (other->IsSpeedable())
				{
					other->SetAwake();
				}
				island.m_bodies.push_back(other);
				m_bodiesIslanded.insert(other);
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
	assert(!IsLocked());

	auto stepStats = StepStats{};
	{
		FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

		CreateAndDestroyProxies(conf);
		SynchronizeProxies(conf);

		// Note: this may update bodies (in addition to the contacts container).
		const auto destroyStats = DestroyContacts(m_contacts);
		if (HasNewFixtures())
		{
			UnsetNewFixtures();
			
			// New fixtures were added: need to find and create the new contacts.
			// Note: this may update bodies (in addition to the contacts container).
			stepStats.pre.added = FindNewContacts();
		}

		if (conf.get_dt() != 0)
		{
			m_inv_dt0 = conf.get_inv_dt();

			// Could potentially run UpdateContacts multithreaded over split lists...
			const auto updateStats = UpdateContacts(m_contacts);
			
			stepStats.pre.ignored = updateStats.ignored;
			stepStats.pre.destroyed = destroyStats.filteredOut + destroyStats.notOverlapping;
			stepStats.pre.updated = updateStats.updated;

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
		auto transformation = b->GetTransformation();
		transformation.p -= newOrigin;
		BodyAtty::SetTransformation(*b, transformation);
		
		auto sweep = b->GetSweep();
		sweep.pos0.linear -= newOrigin;
		sweep.pos1.linear -= newOrigin;
		BodyAtty::SetSweep(*b, sweep);
	}

	for (auto&& j: m_joints)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_broadPhase.ShiftOrigin(newOrigin);
}

bool World::Erase(Contact* c)
{
	assert(c);
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
	{
		if (*iter == c)
		{
			m_contacts.erase(iter);
			return true;
		}
	}
	return false;
}

void World::InternalDestroy(Contact* c, Body* from)
{
	if (m_contactListener && c->IsTouching())
	{
		// EndContact hadn't been called in DestroyOrUpdateContacts() since is-touching, so call it now
		m_contactListener->EndContact(*c);
	}
	
	{
		const auto fixtureA = c->GetFixtureA();
		const auto fixtureB = c->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		
		if (bodyA != from)
		{
			BodyAtty::Erase(*bodyA, c);
		}
		if (bodyB != from)
		{
			BodyAtty::Erase(*bodyB, c);
		}
	}
	
	ContactAtty::Destroy(c);
}

void World::Destroy(Contact* c, Body* from)
{
	InternalDestroy(c, from);
	Erase(c);
}

World::DestroyContactsStats World::DestroyContacts(Contacts& contacts)
{
	auto stats = DestroyContactsStats{};
	
	Contacts::iterator next;
	for (auto iter = contacts.begin(); iter != contacts.end(); iter = next)
	{
		next = std::next(iter);

		const auto contact = *iter;
		const auto indexA = contact->GetChildIndexA();
		const auto indexB = contact->GetChildIndexB();
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		
		// Is this contact flagged for filtering?
		if (contact->NeedsFiltering())
		{
			if (!::box2d::ShouldCollide(*bodyB, *bodyA) || !ShouldCollide(fixtureA, fixtureB))
			{
				InternalDestroy(*iter);
				contacts.erase(iter);
				++stats.filteredOut;
				continue;
			}
			ContactAtty::UnflagForFiltering(*contact);
		}
		
		if (!TestOverlap(m_broadPhase, fixtureA, indexA, fixtureB, indexB))
		{
			// Destroy contacts that cease to overlap in the broad-phase.
			InternalDestroy(*iter);
			contacts.erase(iter);
			++stats.notOverlapping;
			continue;
		}

		++stats.ignored;
	}
	
	return stats;
}

World::UpdateContactsStats World::UpdateContacts(Contacts& contacts)
{
	auto stats = UpdateContactsStats{};
	
	// Update awake contacts.
	for (auto iter = contacts.begin(); iter != contacts.end(); ++iter)
	{
		const auto contact = *iter;
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		
		// Awake && speedable (dynamic or kinematic) means collidable.
		// At least one body must be collidable
		assert(!bodyA->IsAwake() || bodyA->IsSpeedable());
		assert(!bodyB->IsAwake() || bodyB->IsSpeedable());
		if (!bodyA->IsAwake() && !bodyB->IsAwake())
		{
			++stats.ignored;
			continue;
		}
		
		// Update the contact manifold and notify the listener.
		contact->SetEnabled();
		
		// The following may call listener but is otherwise thread-safe.
		ContactAtty::Update(*contact, m_contactListener);
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
	const auto fixtureA = proxyA.fixture; ///< Fixture of proxyA (but may get switched with fixtureB).
	const auto fixtureB = proxyB.fixture; ///< Fixture of proxyB (but may get switched with fixtureA).
	
#ifndef NDEBUG
	const auto pidA = proxyA.proxyId;
	const auto pidB = proxyB.proxyId;
	assert(pidA != pidB);
	assert(sizeof(pidA) + sizeof(pidB) == sizeof(size_t));
#endif
	
	const auto bodyA = fixtureA->GetBody();
	const auto bodyB = fixtureB->GetBody();
	
	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return false;
	}
	
	// Does a joint override collision? Is at least one body dynamic?
	if (!::box2d::ShouldCollide(*bodyB, *bodyA))
	{
		return false;
	}
	
	// Check user filtering.
	if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
	{
		return false;
	}

	const auto childIndexA = proxyA.childIndex;
	const auto childIndexB = proxyB.childIndex;
	
#ifndef NO_RACING
	// Code herein may be racey in a multithreaded context...

	// TODO: use hash table to remove potential bottleneck when both bodies have many contacts?
	// Does a contact already exist?
	const auto searchBody = (bodyA->GetContacts().size() < bodyB->GetContacts().size())? bodyA: bodyB;
	for (auto&& contact: searchBody->GetContacts())
	{
		if (IsFor(*contact, fixtureA, childIndexA, fixtureB, childIndexB))
		{
			// Already have a contact for proxyA with proxyB, bail!
			return false;
		}
	}
	
	assert(m_contacts.size() < MaxContacts);
	
	// Call the contact factory create method.
	const auto contact = ContactAtty::Create(*fixtureA, childIndexA, *fixtureB, childIndexB);
	assert(contact);
	if (!contact)
	{
		return false;
	}
	
	// Insert into the contacts container.
	//
	// Should the new contact be added at front or back?
	//
	// Original strategy added to the front. Since processing done front to back, front
	// adding means container more a LIFO container, while back adding means more a FIFO.
	//
	// Does it matter statistically?
	//
	// Tiles push_front #s:
	// Reg sums: 7927 isl-found, 7395 isl-solved, 8991 pos-iters, 63416 vel-iters, 3231 moved
	// TOI sums: 3026 isl-found, 3026 isl-solved, 7373 pos-iters, 24208 vel-iters, 0 moved, 29825 upd
	//   Total iters: 72407 reg, 31581 TOI, 103988 sum.
	//
	// Tiles push_back #s:
	// Reg sums: 7930 isl-found, 7397 isl-solved, 8997 pos-iters, 63440 vel-iters, 3259 moved
	// TOI sums: 2960 isl-found, 2960 isl-solved, 7189 pos-iters, 23680 vel-iters, 0 moved, 29701 upd
	//   Total iters: 72437 reg, 30869 TOI, 103306 sum.
	//
	m_contacts.push_back(contact);

	BodyAtty::Insert(*bodyA, contact);
	BodyAtty::Insert(*bodyB, contact);
	
	// Wake up the bodies
	if (!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		if (bodyA->IsSpeedable())
		{
			bodyA->SetAwake();
		}
		if (bodyB->IsSpeedable())
		{
			bodyB->SetAwake();
		}
	}
#endif
	
	return true;
}

bool World::RegisterForProxies(Fixture* fixture)
{
	if (fixture)
	{
		const auto body = fixture->GetBody();
		if (body)
		{
			const auto world = body->GetWorld();
			if (world == this)
			{
				m_fixturesForProxies.push_back(fixture);
				return true;
			}
		}
	}
	return false;
}

bool World::RegisterForProxies(Body* body)
{
	if (body)
	{
		const auto world = body->GetWorld();
		if (world == this)
		{
			m_bodiesForProxies.push_back(body);
			return true;
		}
	}
	return false;
}

void World::CreateAndDestroyProxies(const StepConf& conf)
{
	for (auto&& fixture: m_fixturesForProxies)
	{
		CreateAndDestroyProxies(*fixture, conf);
	}
	m_fixturesForProxies.clear();
}

void World::CreateAndDestroyProxies(Fixture& fixture, const StepConf& conf)
{
	const auto body = fixture.GetBody();
	const auto enabled = body->IsEnabled();

	const auto proxies = FixtureAtty::GetProxies(fixture);
	if (proxies.size() == 0)
	{
		if (enabled)
		{
			CreateProxies(fixture, conf.aabbExtension);
		}
	}
	else
	{
		if (!enabled)
		{
			DestroyProxies(fixture);

			// Destroy any contacts associated with the fixture.
			BodyAtty::EraseContacts(*body, [&](Contact& contact) {
				const auto fixtureA = contact.GetFixtureA();
				const auto fixtureB = contact.GetFixtureB();
				if ((fixtureA == &fixture) || (fixtureB == &fixture))
				{
					Destroy(&contact, body);
					return true;
				}
				return false;
			});
		}
	}
}

void World::SynchronizeProxies(const StepConf& conf)
{
	for (auto&& body: m_bodiesForProxies)
	{
		SynchronizeProxies(*body, conf);
	}
	m_bodiesForProxies.clear();
}

void World::SynchronizeProxies(Body& body, const StepConf& conf)
{
	const auto xfm = body.GetTransformation();
	Synchronize(body, xfm, xfm, conf.displaceMultiplier, conf.aabbExtension);
}

void World::SetType(Body& body, BodyType type)
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
	
	BodyAtty::SetTypeFlags(body, type);
	body.ResetMassData();
	
	// Destroy the attached contacts.
	BodyAtty::EraseContacts(body, [&](Contact& contact) {
		Destroy(&contact, &body);
		return true;
	});

	if (type == BodyType::Static)
	{
#ifndef NDEBUG
		const auto xfm1 = GetTransform0(body.GetSweep());
		const auto xfm2 = body.GetTransformation();
		assert(xfm1 == xfm2);
#endif
		RegisterForProxies(&body);
	}
	else
	{
		body.SetAwake();
		body.SetAcceleration(body.IsAccelerable()? GetGravity(): Vec2_zero, 0_rad);
		
		for (auto&& fixture: body.GetFixtures())
		{
			InternalTouchProxies(*fixture);
		}
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
	if (!IsValid(shape) || !Body::IsValid(*shape))
	{
		return nullptr;
	}
	
	assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}
	
	const auto fixture = FixtureAtty::Create(&body, def, shape);
	BodyAtty::Insert(body, fixture);

	if (body.IsEnabled())
	{
		RegisterForProxies(fixture);
	}
	
	// Adjust mass properties if needed.
	if (fixture->GetDensity() > 0)
	{
		BodyAtty::SetMassDataDirty(body);
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
	const auto found = BodyAtty::Erase(body, fixture);
	if (!found)
	{
		// Fixture probably destroyed already.
		return false;
	}
	
#if 0
	/*
	 * XXX: Should the destruction listener be called when the user requested that
	 *   the fixture be destroyed or only when the fixture is destroyed indirectly?
	 */
	if (m_destructionListener)
	{
		m_destructionListener->SayGoodbye(*fixture);
	}
#endif

	// Destroy any contacts associated with the fixture.
	BodyAtty::EraseContacts(body, [&](Contact& contact) {
		const auto fixtureA = contact.GetFixtureA();
		const auto fixtureB = contact.GetFixtureB();
		if ((fixtureA == fixture) || (fixtureB == fixture))
		{
			Destroy(&contact, &body);
			return true;
		}
		return false;
	});
	
	DestroyProxies(*fixture);
	delete fixture;
	
	BodyAtty::SetMassDataDirty(body);
	if (resetMassData)
	{
		body.ResetMassData();
	}
	
	return true;
}

void World::CreateProxies(Fixture& fixture, const RealNum aabbExtension)
{
	const auto body = fixture.GetBody();
	const auto bodyXfm = body->GetTransformation();
	assert(fixture.GetProxyCount() == 0);
	
	const auto shape = fixture.GetShape();
	const auto fixtureXfm = fixture.GetTransformation();
	
	// Reserve proxy space and create proxies in the broad-phase.
	const auto childCount = GetChildCount(*shape);
	const auto proxies = static_cast<FixtureProxy*>(alloc(sizeof(FixtureProxy) * childCount));
	
	const auto xf = Mul(bodyXfm, fixtureXfm);
	for (auto childIndex = decltype(childCount){0}; childIndex < childCount; ++childIndex)
	{
		const auto aabb = ComputeAABB(*shape, xf, childIndex);
		const auto proxyPtr = proxies + childIndex;
		const auto proxyId = m_broadPhase.CreateProxy(GetFattenedAABB(aabb, aabbExtension), proxyPtr);
		new (proxyPtr) FixtureProxy{aabb, proxyId, &fixture, childIndex};
	}

	FixtureAtty::SetProxies(fixture, Span<FixtureProxy>(proxies, childCount));
}

void World::DestroyProxies(Fixture& fixture)
{
	const auto proxies = FixtureAtty::GetProxies(fixture);

	// Destroy proxies in reverse order from what they were created in.
	for (auto i = proxies.size() - 1; i < proxies.size(); --i)
	{
		m_broadPhase.DestroyProxy(proxies[i].proxyId);
		proxies[i].~FixtureProxy();
	}
	free(proxies.begin());

	FixtureAtty::SetProxies(fixture, Span<FixtureProxy>(static_cast<FixtureProxy*>(nullptr), child_count_t{0}));
}

bool World::TouchProxies(Fixture& fixture) noexcept
{
	const auto body = fixture.GetBody();
	if (body)
	{
		const auto world = body->GetWorld();
		if (world == this)
		{
			InternalTouchProxies(fixture);
			return true;
		}
	}
	return false;
}

void World::InternalTouchProxies(Fixture& fixture) noexcept
{
	const auto proxyCount = fixture.GetProxyCount();
	for (auto i = decltype(proxyCount){0}; i < proxyCount; ++i)
	{
		m_broadPhase.TouchProxy(fixture.GetProxy(i)->proxyId);
	}
}

child_count_t World::Synchronize(Fixture& fixture,
								 Transformation xfm1, Transformation xfm2,
								 const RealNum multiplier, const RealNum extension)
{
	assert(::box2d::IsValid(xfm1));
	assert(::box2d::IsValid(xfm2));
	
	const auto shape = fixture.GetShape();
	const auto fixtureXfm = fixture.GetTransformation();
	
	xfm1 = Mul(xfm1, fixtureXfm);
	xfm2 = Mul(xfm2, fixtureXfm);

	auto updatedCount = child_count_t{0};
	const auto displacement = xfm2.p - xfm1.p;
	const auto proxies = FixtureAtty::GetProxies(fixture);
	for (auto&& proxy: proxies)
	{
		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		const auto aabb1 = ComputeAABB(*shape, xfm1, proxy.childIndex);
		const auto aabb2 = ComputeAABB(*shape, xfm2, proxy.childIndex);
		proxy.aabb = GetEnclosingAABB(aabb1, aabb2);
		
		if (m_broadPhase.UpdateProxy(proxy.proxyId, proxy.aabb, displacement, multiplier, extension))
		{
			++updatedCount;
		}
	}
	return updatedCount;
}

contact_count_t World::Synchronize(Body& body,
								   const Transformation& xfm1, const Transformation& xfm2,
								   const RealNum multiplier, const RealNum aabbExtension)
{
	auto updatedCount = contact_count_t{0};
	for (auto&& fixture: body.GetFixtures())
	{
		updatedCount += Synchronize(*fixture, xfm1, xfm2, multiplier, aabbExtension);
	}
	return updatedCount;
}

// Free functions...

StepStats Step(World& world, RealNum dt, World::ts_iters_type velocityIterations, World::ts_iters_type positionIterations)
{
	StepConf conf;
	conf.set_dt(dt);
	conf.regVelocityIterations = velocityIterations;
	conf.regPositionIterations = positionIterations;
	conf.toiVelocityIterations = velocityIterations;
	if (positionIterations == 0)
	{
		conf.toiPositionIterations = 0;
	}
	conf.dtRatio = dt * world.GetInvDeltaTime();
	return world.Step(conf);
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
		if (Awaken(*body))
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

bool IsActive(const Contact& contact) noexcept
{
	const auto bA = contact.GetFixtureA()->GetBody();
	const auto bB = contact.GetFixtureB()->GetBody();
	
	assert(!bA->IsAwake() || bA->IsSpeedable());
	assert(!bB->IsAwake() || bB->IsSpeedable());

	const auto activeA = bA->IsAwake();
	const auto activeB = bB->IsAwake();
	
	// Is at least one body active (awake and dynamic or kinematic)?
	return activeA || activeB;
}

} // namespace box2d
