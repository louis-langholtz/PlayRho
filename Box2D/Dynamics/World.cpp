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
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/BodyAtty.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureAtty.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Island.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>
#include <Box2D/Dynamics/JointAtty.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Dynamics/ContactAtty.hpp>
#include <Box2D/Collision/BroadPhase.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/TimeOfImpact.hpp>
#include <Box2D/Collision/RayCastOutput.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>

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
    Length maxTranslation;
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
    /// @details Calculate the positional displacement based on the given velocity
    ///    that's possibly clamped to the maximum translation and rotation.
    inline PositionAndVelocity CalculateMovement(const BodyConstraint& body, Time h, MovementConf conf)
    {
        assert(IsValid(h));
        
        auto velocity = body.GetVelocity();
        auto translation = h * velocity.linear;
        const auto lsquared = GetLengthSquared(translation);
        if (lsquared > Square(conf.maxTranslation))
        {
            const auto ratio = conf.maxTranslation / Sqrt(lsquared);
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
                                   Time h, MovementConf conf)
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
    /// @details
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
            const auto& manifold = static_cast<const Contact*>(contact)->GetManifold();

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
    
    struct VelocityPair
    {
        Velocity a;
        Velocity b;
    };
    
    inline VelocityPair CalcWarmStartVelocityDeltas(const VelocityConstraint& vc)
    {
        auto vp = VelocityPair{
            Velocity{Vec2_zero * MeterPerSecond, AngularVelocity{0}},
            Velocity{Vec2_zero * MeterPerSecond, AngularVelocity{0}}
        };
        
        const auto normal = GetNormal(vc);
        const auto tangent = GetTangent(vc);
        if (IsValid(normal) && IsValid(tangent))
        {
            // inverse moment of inertia : L^-2 M^-1 QP^2
            const auto invRotInertiaA = vc.bodyA.GetInvRotInertia();
            const auto invRotInertiaB = vc.bodyB.GetInvRotInertia();

            const auto pointCount = vc.GetPointCount();
            for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
            {
                // P is M L T^-2
                // GetPointRelPosA() is Length2D
                // Cross(Length2D, P) is: M L^2 T^-2
                // L^-2 M^-1 QP^2 M L^2 T^-2 is: QP^2 T^-2
                const auto P = (GetNormalImpulseAtPoint(vc, j) * normal + GetTangentImpulseAtPoint(vc, j) * tangent);
                const auto LA = Cross(GetPointRelPosA(vc, j), P) / Radian;
                const auto LB = Cross(GetPointRelPosB(vc, j), P) / Radian;
                vp.a -= Velocity{vc.bodyA.GetInvMass() * P, invRotInertiaA * LA};
                vp.b += Velocity{vc.bodyB.GetInvMass() * P, invRotInertiaB * LB};
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
    /// @details
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
    /// @details Updates the velocities and velocity constraint points' normal and tangent impulses.
    /// @pre <code>UpdateVelocityConstraints</code> has been called on the velocity constraints.
    /// @return Maximum momentum used for solving both the tangential and normal portions of
    ///   the velocity constraints.
    inline Momentum SolveVelocityConstraints(VelocityConstraints& velocityConstraints)
    {
        auto maxIncImpulse = Momentum{0};
        for (auto&& vc: velocityConstraints)
        {
            maxIncImpulse = std::max(maxIncImpulse, SolveVelocityConstraint(vc));
        }
        return maxIncImpulse;
    }
    
    inline Time GetUnderActiveTime(const Body& b, const StepConf& conf) noexcept
    {
        const auto underactive = IsUnderActive(b.GetVelocity(), conf.linearSleepTolerance, conf.angularSleepTolerance);
        const auto sleepable = b.IsSleepingAllowed();
        return (sleepable && underactive)? b.GetUnderActiveTime() + conf.GetTime(): Second * RealNum{0};
    }

    inline Time UpdateUnderActiveTimes(Island::Bodies& bodies, const StepConf& conf)
    {
        auto minUnderActiveTime = Second * std::numeric_limits<RealNum>::infinity();
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
    
    void FlagContactsForFiltering(Body* bodyA, Body* bodyB)
    {
        for (auto&& ci: bodyB->GetContacts())
        {
            const auto contact = GetContactPtr(ci);
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

    inline VelocityConstraint::Conf GetRegVelocityConstraintConf(const StepConf& conf)
    {
        return VelocityConstraint::Conf{
            conf.doWarmStart? conf.dtRatio: 0,
            conf.velocityThreshold,
            conf.doBlocksolve
        };
    }
    
    inline VelocityConstraint::Conf GetToiVelocityConstraintConf(const StepConf& conf)
    {
        return VelocityConstraint::Conf{0, conf.velocityThreshold, conf.doBlocksolve};
    }
    
} // anonymous namespace

World::World(const WorldDef& def):
    m_gravity{def.gravity},
    m_minVertexRadius{def.minVertexRadius},
    m_maxVertexRadius{def.maxVertexRadius}
{
    assert(::box2d::IsValid(def.gravity.x) && ::box2d::IsValid(def.gravity.y));
    assert(def.minVertexRadius > Length{0});
    assert(def.minVertexRadius < def.maxVertexRadius);
}

World::~World()
{
    // Gets rid of the associated contacts.
    while (!m_contacts.empty())
    {
        auto&& contact = m_contacts.front();
        const auto c = GetContactPtr(contact);
        const auto fixtureA = c->GetFixtureA();
        const auto fixtureB = c->GetFixtureB();
        const auto bodyA = fixtureA->GetBody();
        const auto bodyB = fixtureB->GetBody();
        BodyAtty::Erase(*bodyA, c);
        BodyAtty::Erase(*bodyB, c);
        if ((c->GetManifold().GetPointCount() > 0) &&
            !fixtureA->IsSensor() && !fixtureB->IsSensor())
        {
            // Contact may have been keeping accelerable bodies of fixture A or B from moving.
            // Need to awaken those bodies now in case they are again movable.
            bodyA->SetAwake();
            bodyB->SetAwake();
        }
        m_contacts.pop_front();
    }

    // Gets rid of the created bodies and any associated fixtures.
    // Do this before getting rid of joints so that joints can be removed from
    // bodies in their listed order.
    while (!m_bodies.empty())
    {
        auto& body = m_bodies.front();
        const auto b = GetBodyPtr(body);
        BodyAtty::ClearJoints(*b, [&](Joint&) {
            // Intentionally empty.
        });
        BodyAtty::ClearFixtures(*b, [&](Fixture& fixture) {
            if (m_destructionListener)
            {
                m_destructionListener->SayGoodbye(fixture);
            }
            DestroyProxies(fixture);
        });
        assert(b->GetJoints().empty());
        assert(b->GetContacts().empty());
        m_bodies.pop_front();
    }
    
    // Gets rid of the created joints.
    while (!m_joints.empty())
    {
        const auto j = m_joints.front();
        m_joints.pop_front();
        JointAtty::Destroy(j);
    }
}

void World::SetGravity(const LinearAcceleration2D gravity) noexcept
{
    if (m_gravity != gravity)
    {
        const auto diff = gravity - m_gravity;
        for (auto&& b: m_bodies)
        {
            const auto body = GetBodyPtr(b);
            ApplyLinearAcceleration(*body, diff);
        }
        m_gravity = gravity;
    }
}

Body* World::CreateBody(const BodyDef& def)
{
    if (IsLocked())
    {
        throw LockedError();
    }

    if (m_bodies.size() >= MaxBodies)
    {
        return nullptr;
    }
    
    // Add to world doubly linked list.
    m_bodies.emplace_front(def, this);
    auto& b = m_bodies.front();
    b.SetAcceleration(m_gravity, AngularAcceleration{0});
    return &b;
}

bool World::Remove(Body& b)
{
    for (auto iter = m_bodies.begin(); iter != m_bodies.end(); ++iter)
    {
        if (&*iter == &b)
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
    
    if (IsLocked())
    {
        throw LockedError();
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
}

Joint* World::CreateJoint(const JointDef& def)
{
    if (m_joints.size() >= MaxJoints)
    {
        return nullptr;
    }

    if (IsLocked())
    {
        throw LockedError();
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
    if (IsLocked())
    {
        throw LockedError();
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

void World::AddToIsland(Island& island, Body& seed,
                  Bodies::size_type& remNumBodies,
                  Contacts::size_type& remNumContacts,
                  Joints::size_type& remNumJoints)
{
    assert(!IsIslanded(&seed));
    assert(seed.IsSpeedable());
    assert(seed.IsAwake());
    assert(seed.IsEnabled());
    assert(remNumBodies != 0);
    assert(remNumBodies < MaxBodies);
    
    // Perform a depth first search (DFS) on the constraint graph.

    // Create a stack for bodies to be islanded that aren't already islanded.
    auto stack = std::vector<Body*>();
    stack.reserve(remNumBodies);

    stack.push_back(&seed);
    SetIslanded(&seed);
    
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

        // Make sure the body is awake (without resetting sleep timer).
        BodyAtty::SetAwakeFlag(*b);

        const auto oldNumContacts = island.m_contacts.size();
        // Adds appropriate contacts of current body and appropriate 'other' bodies of those contacts.
        for (auto&& ci: b->GetContacts())
        {
            const auto contact = GetContactPtr(ci);
            if (!IsIslanded(contact) && contact->IsEnabled() && contact->IsTouching())
            {
                const auto fA = contact->GetFixtureA();
                const auto fB = contact->GetFixtureB();
                if (!fA->IsSensor() && !fB->IsSensor())
                {
                    const auto bA = fA->GetBody();
                    const auto bB = fB->GetBody();
                    const auto other = (bA != b)? bA: bB;
                    island.m_contacts.push_back(contact);
                    SetIslanded(contact);
                    if (!IsIslanded(other))
                    {                
                        stack.push_back(other);
                        SetIslanded(other);
                    }
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
        for (auto&& ji: b->GetJoints())
        {
            // Use data of ji before dereferencing its pointers.
            const auto other = ji.first;
            const auto joint = ji.second;
            if (!IsIslanded(joint) && other->IsEnabled())
            {
                island.m_joints.push_back(joint);
                SetIslanded(joint);
                if (!IsIslanded(other))
                {
                    // Only now dereference ji's pointers.
                    const auto bodyA = joint->GetBodyA();
                    const auto bodyB = joint->GetBodyB();
                    const auto rwOther = bodyA != b? bodyA: bodyB;
                    stack.push_back(rwOther);
                    SetIslanded(rwOther);
                }
            }
        }
        remNumJoints -= island.m_joints.size() - numJoints;
    }
}

World::Bodies::size_type World::RemoveUnspeedablesFromIslanded(const std::vector<Body*>& bodies)
{
    auto numRemoved = Bodies::size_type{0};
    for (auto&& body: bodies)
    {
        // Allow static bodies to participate in other islands.
        if (!body->IsSpeedable())
        {
            UnsetIslanded(body);
            ++numRemoved;
        }
    }
    return numRemoved;
}

RegStepStats World::SolveReg(const StepConf& conf)
{
    auto stats = RegStepStats{};
    assert(stats.islandsFound == 0);
    assert(stats.islandsSolved == 0);

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
    for (auto&& b: m_bodies)
    {
        const auto body = GetBodyPtr(b);
        assert(!body->IsAwake() || body->IsSpeedable());
        if (!IsIslanded(body) && body->IsAwake() && body->IsEnabled())
        {
            ++stats.islandsFound;

            // Size the island for the remaining un-evaluated bodies, contacts, and joints.
            Island island(remNumBodies, remNumContacts, remNumJoints);

            AddToIsland(island, *body, remNumBodies, remNumContacts, remNumJoints);
            remNumBodies += RemoveUnspeedablesFromIslanded(island.m_bodies);

#if defined(DO_THREADED)
            // Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
            futures.push_back(std::async(std::launch::async, &World::SolveRegIsland, this, conf, island));
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

    for (auto&& b: m_bodies)
    {
        const auto body = GetBodyPtr(b);
        // A non-static body that was in an island may have moved.
        if (IsIslanded(body) && body->IsSpeedable())
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
    auto results = IslandSolverResults{};
    results.positionIterations = conf.regPositionIterations;
    const auto h = conf.GetTime(); ///< Time step.

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
                                                      GetRegVelocityConstraintConf(conf));
    
    if (conf.doWarmStart)
    {
        WarmStartVelocities(velocityConstraints);
    }

    const auto psConf = GetRegConstraintSolverConf(conf);

    for (auto&& joint: island.m_joints)
    {
        JointAtty::InitVelocityConstraints(*joint, bodyConstraints, conf, psConf);
    }
    
    results.velocityIterations = conf.regVelocityIterations;
    for (auto i = decltype(conf.regVelocityIterations){0}; i < conf.regVelocityIterations; ++i)
    {
        auto jointsOkay = true;
        for (auto&& joint: island.m_joints)
        {
            jointsOkay &= JointAtty::SolveVelocityConstraints(*joint, bodyConstraints, conf);
        }

        // Note that the new incremental impulse can potentially be orders of magnitude
        // greater than the last incremental impulse used in this loop.
        const auto newIncImpulse = SolveVelocityConstraints(velocityConstraints);
        results.maxIncImpulse = std::max(results.maxIncImpulse, newIncImpulse);

        if (jointsOkay && (newIncImpulse == Momentum{0}))
        {
            // No joint related velocity constraints were out of tolerance.
            // No body related velocity constraints were out of tolerance.
            // There does not appear to be any benefit to doing more loops now.
            // XXX: Is it really safe to bail now? Not certain of that.
            // Bail now assuming that this is helpful to do...
            results.velocityIterations = i + 1;
            break;
        }
    }
    
    // updates array of tentative new body positions per the velocities as if there were no obstacles...
    IntegratePositions(bodyConstraints, h, GetMovementConf(conf));
    
    // Solve position constraints
    for (auto i = decltype(conf.regPositionIterations){0}; i < conf.regPositionIterations; ++i)
    {
        const auto minSeparation = SolvePositionConstraints(positionConstraints, psConf);
        results.minSeparation = Min(results.minSeparation, minSeparation);
        const auto contactsOkay = (minSeparation >= conf.regMinSeparation);

        auto jointsOkay = true;
        for (auto&& joint: island.m_joints)
        {
            jointsOkay &= JointAtty::SolvePositionConstraints(*joint, bodyConstraints, psConf);
        }

        if (contactsOkay && jointsOkay)
        {
            // Reached tolerance, early out...
            results.positionIterations = i + 1;
            results.solved = true;
            break;
        }
    }
    
    // Update normal and tangent impulses of contacts' manifold points
    for (auto&& vc: velocityConstraints)
    {
        auto& manifold = ContactAtty::GetMutableManifold(*island.m_contacts[vc.GetContactIndex()]);
        AssignImpulses(manifold, vc);
    }
    
    for (auto&& body: island.m_bodies)
    {
        const auto& constraint = bodyConstraints[body];
        UpdateBody(*body, constraint.GetPosition(), constraint.GetVelocity());
    }
    
    // XXX: Should contacts needing updating be updated now??

    if (m_contactListener)
    {
        Report(*m_contactListener, island.m_contacts, velocityConstraints,
               results.solved? results.positionIterations - 1: StepConf::InvalidIteration);
    }
    
    results.bodiesSlept = body_count_t{0};
    if (::box2d::IsValid(RealNum{conf.minStillTimeToSleep / Second}))
    {
        const auto minUnderActiveTime = UpdateUnderActiveTimes(island.m_bodies, conf);
        if ((minUnderActiveTime >= conf.minStillTimeToSleep) && results.solved)
        {
            results.bodiesSlept = static_cast<decltype(results.bodiesSlept)>(Sleepem(island.m_bodies));
        }
    }

    return results;
}

void World::ResetBodiesForSolveTOI()
{
    m_bodiesIslanded.clear();
    for (auto&& body: m_bodies)
    {
        const auto b = GetBodyPtr(body);
        BodyAtty::ResetAlpha0(*b);
    }
}

void World::ResetContactsForSolveTOI()
{
    m_contactsIslanded.clear();
    for (auto&& contact: m_contacts)
    {
        const auto c = GetContactPtr(contact);
        ContactAtty::UnsetToi(*c);
        ContactAtty::ResetToiCount(*c);
    }
}

World::UpdateContactsData World::UpdateContactTOIs(const StepConf& conf)
{
    auto results = UpdateContactsData{};

    const auto toiConf = GetToiConf(conf);
    
    for (auto&& contact: m_contacts)
    {
        const auto c = GetContactPtr(contact);
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
        
        // Compute the TOI for this contact (one or both bodies are active and impenetrable).
        // Computes the time of impact in interval [0, 1]
        const auto output = CalcToi(*c, toiConf);
        
        // Use Min function to handle floating point imprecision which possibly otherwise
        // could provide a TOI that's greater than 1.
        const auto toi = IsValidForTime(output.get_state())?
            Min(alpha0 + (1 - alpha0) * output.get_t(), RealNum{1}): RealNum{1};
        assert(toi >= alpha0 && toi <= 1);
        ContactAtty::SetToi(*c, toi);
        
        results.maxDistIters = Max(results.maxDistIters, output.get_max_dist_iters());
        results.maxToiIters = Max(results.maxToiIters, output.get_toi_iters());
        results.maxRootIters = Max(results.maxRootIters, output.get_max_root_iters());
        ++results.numUpdatedTOI;
    }

    return results;
}
    
World::ContactToiData World::GetSoonestContacts(const size_t reserveSize)
{
    auto minToi = std::nextafter(RealNum{1}, RealNum{0});
    auto minContacts = std::vector<Contact*>();
    minContacts.reserve(reserveSize);
    for (auto&& contact: m_contacts)
    {
        const auto c = GetContactPtr(contact);
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
        stats.contactsFound += static_cast<contact_count_t>(ncount);
        auto islandsFound = 0u;
        for (auto&& contact: next.contacts)
        {
            if (!IsIslanded(contact))
            {
                /*
                 * Confirm that contact is as it's supposed to be according to contract of the
                 * GetSoonestContacts method from which this contact was obtained.
                 */
                assert(contact->IsEnabled());
                assert(!HasSensor(*contact));
                assert(IsActive(*contact));
                assert(IsImpenetrable(*contact));

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
                stats.contactsUpdatedTouching += solverResults.contactsUpdated;
                stats.contactsSkippedTouching += solverResults.contactsSkipped;
                break; // TODO: get working without breaking.
            }
        }
        stats.islandsFound += islandsFound;

        // Reset island flags and synchronize broad-phase proxies.
        for (auto&& b: m_bodies)
        {
            const auto body = GetBodyPtr(b);
            if (IsIslanded(body))
            {
                UnsetIslanded(body);
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
    // Note:
    //   This method is what used to be b2World::SolveTOI(const b2TimeStep& step).
    //   It also differs internally from Erin's implementation.
    //
    //   Here's some specific behavioral differences:
    //   1. Bodies don't get their under-active times reset (like they do in Erin's code).

    auto contactsUpdated = contact_count_t{0};
    auto contactsSkipped = contact_count_t{0};

    /*
     * Confirm that contact is as it's supposed to be according to contract of the
     * GetSoonestContacts method from which this contact should have been obtained.
     */
    assert(contact.IsEnabled());
    assert(!HasSensor(contact));
    assert(IsActive(contact));
    assert(IsImpenetrable(contact));
    assert(!IsIslanded(&contact));
    
    const auto toi = contact.GetToi();
    const auto bA = contact.GetFixtureA()->GetBody();
    const auto bB = contact.GetFixtureB()->GetBody();

    /* XXX: if (toi != 0)? */
    /* if (bA->GetSweep().GetAlpha0() != toi || bB->GetSweep().GetAlpha0() != toi) */
    // Seems contact manifold needs updating regardless.
    {
        const auto backupA = bA->GetSweep();
        const auto backupB = bB->GetSweep();

        // Advance the bodies to the TOI.
        assert(toi != 0 || (bA->GetSweep().GetAlpha0() == 0 && bB->GetSweep().GetAlpha0() == 0));
        BodyAtty::Advance(*bA, toi);
        BodyAtty::Advance(*bB, toi);

        // The TOI contact likely has some new contact points.
        contact.SetEnabled();
        if (contact.NeedsUpdating())
        {
	        ContactAtty::Update(contact, conf, m_contactListener);
            ++contactsUpdated;
        }
        else
        {
            ++contactsSkipped;
        }
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
            // assert(!contact.IsEnabled() || contact.IsTouching());
            contact.UnsetEnabled();
            BodyAtty::Restore(*bA, backupA);
            BodyAtty::Restore(*bB, backupB);
            auto results = IslandSolverResults{};
            results.contactsUpdated += contactsUpdated;
            results.contactsSkipped += contactsSkipped;
            return results;
        }
    }
#if 0
    else if (!contact.IsTouching())
    {
        const auto newManifold = contact.Evaluate();
        assert(contact.IsTouching());
        return IslandSolverResults{};
    }
#endif
    
    if (bA->IsSpeedable())
    {
        BodyAtty::SetAwakeFlag(*bA);
        // XXX should the body's under-active time be reset here?
        //   Erin's code does for here but not in b2World::Solve(const b2TimeStep& step).
        //   Calling Body::ResetUnderActiveTime() has performance implications.
    }

    if (bB->IsSpeedable())
    {
        BodyAtty::SetAwakeFlag(*bB);
        // XXX should the body's under-active time be reset here?
        //   Erin's code does for here but not in b2World::Solve(const b2TimeStep& step).
        //   Calling Body::ResetUnderActiveTime() has performance implications.
    }

    // Build the island
    Island island(m_bodies.size(), m_contacts.size(), 0);

     // These asserts get triggered sometimes if contacts within TOI are iterated over.
    assert(!IsIslanded(bA));
    assert(!IsIslanded(bB));
    
    island.m_bodies.push_back(bA);
    SetIslanded(bA);
    island.m_bodies.push_back(bB);
    SetIslanded(bB);
    island.m_contacts.push_back(&contact);
    SetIslanded(&contact);

    // Process the contacts of the two bodies, adding appropriate ones to the island,
    // adding appropriate other bodies of added contacts, and advancing those other
    // bodies sweeps and transforms to the minimum contact's TOI.
    if (bA->IsAccelerable())
    {
        const auto procOut = ProcessContactsForTOI(island, *bA, toi, conf);
        contactsUpdated += procOut.contactsUpdated;
        contactsSkipped += procOut.contactsSkipped;
    }
    if (bB->IsAccelerable())
    {
        const auto procOut = ProcessContactsForTOI(island, *bB, toi, conf);
        contactsUpdated += procOut.contactsUpdated;
        contactsSkipped += procOut.contactsSkipped;
    }
    
    RemoveUnspeedablesFromIslanded(island.m_bodies);

    // Now solve for remainder of time step.
    //
    // Note: subConf is written the way it is because MSVS2017 emitted errors when
    //   written as:
    //     SolveTOI(StepConf{conf}.SetTime((1 - toi) * conf.GetTime()), island);
    //
    StepConf subConf{conf};
    auto results = SolveTOI(subConf.SetTime((1 - toi) * conf.GetTime()), island);
    results.contactsUpdated += contactsUpdated;
    results.contactsSkipped += contactsSkipped;
    return results;
}

void World::UpdateBody(Body& body, const Position& pos, const Velocity& vel)
{
    BodyAtty::SetVelocity(body, vel);
    BodyAtty::SetPosition1(body, pos);
    BodyAtty::SetTransformation(body, GetTransformation(GetPosition1(body), body.GetLocalCenter()));
}

World::IslandSolverResults World::SolveTOI(const StepConf& conf, Island& island)
{
    auto results = IslandSolverResults{};
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
    // XXX: When processing multiple contacts within same TOI,
    //   island.m_bodies *sometimes* doesn't contain all the needed bodies.
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
    assert(results.minSeparation == std::numeric_limits<RealNum>::infinity() * Meter);
    assert(results.solved == false);
    results.positionIterations = conf.toiPositionIterations;
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
            results.minSeparation = Min(results.minSeparation, minSeparation);
            if (minSeparation >= conf.toiMinSeparation)
            {
                // Reached tolerance, early out...
                results.positionIterations = i + 1;
                results.solved = true;
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
        
        BodyAtty::SetPosition0(*bodyA, bodyConstraints[bodyA].GetPosition());
        BodyAtty::SetPosition0(*bodyB, bodyConstraints[bodyB].GetPosition());
    }
#else
    for (auto&& body: island.m_bodies)
    {
        BodyAtty::SetPosition0(*body, bodyConstraints[body].GetPosition());
    }
#endif
    
    auto velocityConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraints,
                                                      GetToiVelocityConstraintConf(conf));

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.

    // Solve velocity constraints.
    assert(results.maxIncImpulse == Momentum{0});
    results.velocityIterations = conf.toiVelocityIterations;
    for (auto i = decltype(conf.toiVelocityIterations){0}; i < conf.toiVelocityIterations; ++i)
    {
        const auto newIncImpulse = SolveVelocityConstraints(velocityConstraints);
        if (newIncImpulse == Momentum(0))
        {
            // No body related velocity constraints were out of tolerance.
            // There does not appear to be any benefit to doing more loops now.
            // XXX: Is it really safe to bail now? Not certain of that.
            // Bail now assuming that this is helpful to do...
            results.velocityIterations = i + 1;
            break;
        }
        results.maxIncImpulse = std::max(results.maxIncImpulse, newIncImpulse);
    }
    
    // Don't store TOI contact forces for warm starting because they can be quite large.
    
    IntegratePositions(bodyConstraints, conf.GetTime(), GetMovementConf(conf));
    
    for (auto&& body: island.m_bodies)
    {
        const auto& constraint = bodyConstraints[body];
        UpdateBody(*body, constraint.GetPosition(), constraint.GetVelocity());
    }

    if (m_contactListener)
    {
        Report(*m_contactListener, island.m_contacts, velocityConstraints, results.positionIterations);
    }
    
    return results;
}
    
void World::ResetContactsForSolveTOI(Body& body)
{
    // Invalidate all contact TOIs on this displaced body.
    for (auto&& ci: body.GetContacts())
    {
        const auto contact = GetContactPtr(ci);
        UnsetIslanded(contact);
        ContactAtty::UnsetToi(*contact);
    }
}

World::ProcessContactsOutput
World::ProcessContactsForTOI(Island& island, Body& body, RealNum toi,
                             const StepConf& conf)
{
    assert(IsIslanded(&body));
    assert(body.IsAccelerable());
    assert(toi >= 0 && toi <= 1);

    auto results = ProcessContactsOutput{};
    assert(results.contactsUpdated == 0);
    assert(results.contactsSkipped == 0);
    
    auto fn = [&](Contact* contact, Body* other)
    {
        const auto otherIslanded = IsIslanded(other);
        {
            const auto backup = other->GetSweep();
            if (!otherIslanded /* && other->GetSweep().GetAlpha0() != toi */)
            {
                BodyAtty::Advance(*other, toi);
            }
            
            // Update the contact points
            contact->SetEnabled();
            if (contact->NeedsUpdating())
            {
                ContactAtty::Update(*contact, conf, m_contactListener);
                ++results.contactsUpdated;
            }
            else
            {
                ++results.contactsSkipped;
            }
            
            // Revert and skip if contact disabled by user or not touching anymore (very possible).
            if (!contact->IsEnabled() || !contact->IsTouching())
            {
                BodyAtty::Restore(*other, backup);
                return;
            }
        }
        island.m_contacts.push_back(contact);
        SetIslanded(contact);
        if (!otherIslanded)
        {
            if (other->IsSpeedable())
            {
                BodyAtty::SetAwakeFlag(*other);
            }
            island.m_bodies.push_back(other);
            SetIslanded(other);
#if 0
            if (other->IsAccelerable())
            {
                contactsUpdated += ProcessContactsForTOI(island, *other, toi);
            }
#endif
        }
#ifndef NDEBUG
        else
        {
            /*
             * If other is islanded but not in current island, then something's gone wrong.
             * Other needs to be in current island but was already islanded.
             * A previous contact island didn't grow to include all the bodies it needed or
             * perhaps the current contact is-touching while another one wasn't and the
             * inconsistency is throwing things off.
             */
            assert(Count(island, other) > 0);
        }
#endif
    };

    // Note: the original contact (for body of which this method was called) already islanded.
    if (body.IsImpenetrable())
    {
        for (auto&& ci: body.GetContacts())
        {
            const auto contact = GetContactPtr(ci);
            if (!IsIslanded(contact))
            {
                const auto fA = contact->GetFixtureA();
                const auto fB = contact->GetFixtureB();
                if (!fA->IsSensor() && !fB->IsSensor())
                {
                    const auto bA = fA->GetBody();
                    const auto bB = fB->GetBody();
                    const auto other = (bA != &body)? bA: bB;
                    fn(contact, other);
                }
            }
        }
    }
    else
    {
        for (auto&& ci: body.GetContacts())
        {
            const auto contact = GetContactPtr(ci);
            if (!IsIslanded(contact))
            {
                const auto fA = contact->GetFixtureA();
                const auto fB = contact->GetFixtureB();
                if (!fA->IsSensor() && !fB->IsSensor())
                {
                    const auto bA = fA->GetBody();
                    const auto bB = fB->GetBody();
                    const auto other = (bA != &body)? bA: bB;
                    if (other->IsImpenetrable())
                    {
                        fn(contact, other);
                    }
                }
            }
        }
    }

    return results;
}

StepStats World::Step(const StepConf& conf)
{
    assert((m_maxVertexRadius * RealNum{2}) +
           (Length{conf.linearSlop} / RealNum{4}) > (m_maxVertexRadius * RealNum{2}));
    
    if (IsLocked())
    {
        throw LockedError();
    }

    auto stepStats = StepStats{};
    {
        FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

        CreateAndDestroyProxies(conf);
        stepStats.pre.proxiesMoved = SynchronizeProxies(conf);
        // pre.proxiesMoved is usually zero but sometimes isn't.

        {
            // Note: this may update bodies (in addition to the contacts container).
            const auto destroyStats = DestroyContacts(m_contacts);
            stepStats.pre.destroyed = destroyStats.filteredOut + destroyStats.notOverlapping;
        }

        if (HasNewFixtures())
        {
            UnsetNewFixtures();
            
            // New fixtures were added: need to find and create the new contacts.
            // Note: this may update bodies (in addition to the contacts container).
            stepStats.pre.added = FindNewContacts();
        }

        if (conf.GetTime() != Second * RealNum{0})
        {
            m_inv_dt0 = conf.GetInvTime();

            // Could potentially run UpdateContacts multithreaded over split lists...
            const auto updateStats = UpdateContacts(m_contacts, conf);
            stepStats.pre.ignored = updateStats.ignored;
            stepStats.pre.updated = updateStats.updated;
            stepStats.pre.skipped = updateStats.skipped;

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

void World::QueryAABB(const AABB& aabb, QueryFixtureCallback callback)
{
    m_broadPhase.Query(aabb, [&](BroadPhase::size_type proxyId) {
        const auto proxy = static_cast<FixtureProxy*>(m_broadPhase.GetUserData(proxyId));
        return callback(proxy->fixture);
    });
}

void World::RayCast(const Length2D& point1, const Length2D& point2, RayCastCallback callback)
{
    m_broadPhase.RayCast(RayCastInput{point1, point2, RealNum{1}},
                         [&](const RayCastInput& input, BroadPhase::size_type proxyId)
    {
        const auto userData = m_broadPhase.GetUserData(proxyId);
        const auto proxy = static_cast<FixtureProxy*>(userData);
        const auto fixture = proxy->fixture;
        const auto index = proxy->childIndex;
        const auto shape = fixture->GetShape();
        const auto body = fixture->GetBody();
        const auto child = shape->GetChild(index);
        const auto transformation = body->GetTransformation();
        const auto output = box2d::RayCast(child, input, transformation);
        if (output.hit)
        {
            const auto fraction = output.fraction;
            assert(fraction >= 0 && fraction <= 1);
         
            // Here point can be calculated these two ways:
            //   (1) point = p1 * (1 - fraction) + p2 * fraction
            //   (2) point = p1 + (p2 - p1) * fraction.
            //
            // The first way however suffers from the fact that:
            //     a * (1 - fraction) + a * fraction != a
            // for all values of a and fraction between 0 and 1 when a and fraction are
            // floating point types.
            // This leads to the posibility that (p1 == p2) && (point != p1 || point != p2),
            // which may be pretty surprising to the callback. So this way SHOULD NOT be used.
            //
            // The second way, does not have this problem.
            //
            const auto point = input.p1 + (input.p2 - input.p1) * fraction;
            
            // Callback return states: terminate (0), ignore (< 0), clip (< 1), reset (1).
            const auto opcode = callback(fixture, point, output.normal);
            switch (opcode)
            {
                case RayCastOpcode::Terminate: return RealNum(0);
                case RayCastOpcode::IgnoreFixture: return RealNum(-1);
                case RayCastOpcode::ClipRay: return fraction;
                case RayCastOpcode::ResetRay: return input.maxFraction;
            }
        }
        return input.maxFraction;
    });
}

void World::ShiftOrigin(const Length2D newOrigin)
{
    if (IsLocked())
    {
        throw LockedError();
    }

    for (auto&& body: GetBodies())
    {
        const auto b = GetBodyPtr(body);

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
        if (&(*iter) == c)
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
    
    if ((c->GetManifold().GetPointCount() > 0) &&
        !fixtureA->IsSensor() && !fixtureB->IsSensor())
    {
        // Contact may have been keeping accelerable bodies of fixture A or B from moving.
        // Need to awaken those bodies now in case they are again movable.
        bodyA->SetAwake();
        bodyB->SetAwake();
    }
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

        const auto contact = GetContactPtr(*iter);
        const auto indexA = contact->GetChildIndexA();
        const auto indexB = contact->GetChildIndexB();
        const auto fixtureA = contact->GetFixtureA();
        const auto fixtureB = contact->GetFixtureB();
        
        if (!TestOverlap(m_broadPhase, fixtureA, indexA, fixtureB, indexB))
        {
            // Destroy contacts that cease to overlap in the broad-phase.
            InternalDestroy(&*iter);
            contacts.erase(iter);
            ++stats.notOverlapping;
            continue;
        }
        
        // Is this contact flagged for filtering?
        if (contact->NeedsFiltering())
        {
            const auto bodyA = fixtureA->GetBody();
            const auto bodyB = fixtureB->GetBody();

            if (!::box2d::ShouldCollide(*bodyB, *bodyA) || !ShouldCollide(fixtureA, fixtureB))
            {
                InternalDestroy(&*iter);
                contacts.erase(iter);
                ++stats.filteredOut;
                continue;
            }
            ContactAtty::UnflagForFiltering(*contact);
        }

        ++stats.ignored;
    }
    
    return stats;
}

World::UpdateContactsStats World::UpdateContacts(Contacts& contacts, const StepConf& conf)
{
    auto stats = UpdateContactsStats{};
    
#if defined(DO_THREADED)
    std::vector<Contact*> contactsNeedingUpdate;
    contactsNeedingUpdate.reserve(contacts.size());
    std::vector<std::future<void>> futures;
    futures.reserve(contacts.size());
#endif

    // Update awake contacts.
    for (auto iter = contacts.begin(); iter != contacts.end(); ++iter)
    {
        const auto contact = GetContactPtr(*iter);

#if 0
        ContactAtty::Update(*contact, conf, m_contactListener);
        ++stats.updated;
#endif
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
            assert(!contact->HasValidToi());
            ++stats.ignored;
            continue;
        }
        
        // Possible that bodyA->GetSweep().GetAlpha0() != 0
        // Possible that bodyB->GetSweep().GetAlpha0() != 0

        // Update the contact manifold and notify the listener.
        contact->SetEnabled();

        // Note: ideally contacts are only updated if there was a change to:
        //   - The fixtures' sensor states.
        //   - The fixtures bodies' transformations.
        //   - The "maxCirclesRatio" per-step configuration state if contact IS NOT for sensor.
        //   - The "maxDistanceIters" per-step configuration state if contact IS for sensor.
        //
        if (contact->NeedsUpdating())
        {
            // The following may call listener but is otherwise thread-safe.
#if defined(DO_THREADED)
            contactsNeedingUpdate.push_back(contact);
            //futures.push_back(std::async(&ContactAtty::Update, *contact, conf, m_contactListener)));
            //futures.push_back(std::async(std::launch::async, [=]{ ContactAtty::Update(*contact, conf, m_contactListener); }));
#else
            ContactAtty::Update(*contact, conf, m_contactListener);
#endif
        	++stats.updated;
        }
        else
        {
            ++stats.skipped;
        }
    }
    
#if defined(DO_THREADED)
    auto numJobs = contactsNeedingUpdate.size();
    const auto jobsPerCore = numJobs / 4;
    for (auto i = decltype(numJobs){0}; numJobs > 0 && i < 3; ++i)
    {
        futures.push_back(std::async(std::launch::async, [=]{
            const auto offset = jobsPerCore * i;
            for (auto j = decltype(jobsPerCore){0}; j < jobsPerCore; ++j)
            {
	            ContactAtty::Update(*contactsNeedingUpdate[offset + j], conf, m_contactListener);
            }
        }));
        numJobs -= jobsPerCore;
    }
    if (numJobs > 0)
    {
        futures.push_back(std::async(std::launch::async, [=]{
            const auto offset = jobsPerCore * 3;
            for (auto j = decltype(numJobs){0}; j < numJobs; ++j)
            {
                ContactAtty::Update(*contactsNeedingUpdate[offset + j], conf, m_contactListener);
            }
        }));
    }
    for (auto&& future: futures)
    {
        future.get();
    }
#endif
    
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
    
    const auto bodyA = fixtureA->GetBody();
    const auto bodyB = fixtureB->GetBody();
    
    // Are the fixtures on the same body? They can be, and they often are.
    // Don't need nor want a contact for these fixtures if they are on the same body.
    if (bodyA == bodyB)
    {
        return false;
    }
    
    // Does a joint override collision? Is at least one body dynamic?
    if (!::box2d::ShouldCollide(*bodyB, *bodyA) || !ShouldCollide(fixtureA, fixtureB))
    {
        return false;
    }

#ifndef NDEBUG
    const auto pidA = proxyA.proxyId;
    const auto pidB = proxyB.proxyId;
    assert(pidA != pidB);
    
    // The following assert fails on Windows
    // assert(sizeof(pidA) + sizeof(pidB) == sizeof(size_t));
#endif
   
#ifndef NO_RACING
    // Code herein may be racey in a multithreaded context...
    // Would need a lock on bodyA, bodyB, and m_contacts.
    // A global lock on the world instance should work but then would it have so much
    // contention as to make multi-threaded handing of adding new connections senseless?

    // Have to quickly figure out if there's a contact already added for the current
    // fixture-childindex pair that this method's been called for.
    //
    // In cases where there's a bigger bullet-enabled object that's colliding with lots of
    // smaller objects packed tightly together and overlapping like in the Add Pair Stress
    // Test demo that has some 400 smaller objects, the bigger object could have 387 contacts
    // while the smaller object has 369 or more, and the total world contact count can be over
    // 30,495. While searching linearly through the object with less contacts should help,
    // that may still be a lot of contacts to be going through in the context this method
    // is being called. OTOH, speed seems to be dominated by cache hit-ratio...
    //
    // With compiler optimization enabled and 400 small bodies and RealNum=double...
    // For world:
    //   World::std::set<Contact*> shows up as .524 seconds max step
    //   World::std::list<Contact> shows up as .482 seconds max step.
    // For body:
    //    using contact map w/ proxy ID keys shows up as .561
    // W/ unordered_map: .529 seconds max step (step 15).
    // W/ World::std::list<Contact> and Body::std::list<ContactKey,Contact*>   .444s@step15, 1.063s-sumstep20
    // W/ World::std::list<Contact> and Body::std::list<ContactKey,Contact*>   .393s@step15, 1.063s-sumstep20
    // W/ World::std::list<Contact> and Body::std::list<ContactKey,Contact*>   .412s@step15, 1.012s-sumstep20
    // W/ World::std::list<Contact> and Body::std::vector<ContactKey,Contact*> .219s@step15, 0.659s-sumstep20

    // Does a contact already exist?
    // Identify body with least contacts and search it.
    // NOTE: Time trial testing found the following rough ordering of data structures, to be
    // fastest to slowest: std::vector, std::list, std::unorderered_set, std::unordered_map,
    //     std::set, std::map.
    const auto key = ContactKey::Get(proxyA, proxyB);
    const auto searchBody = (bodyA->GetContacts().size() < bodyB->GetContacts().size())?
        bodyA: bodyB;
    for (auto&& ci: searchBody->GetContacts())
    {
        if (ci.first == key)
        {
            return false;
        }
    }
    
    assert(m_contacts.size() < MaxContacts);
    if (m_contacts.size() >= MaxContacts)
    {
        // New contact was needed, but denied due to MaxContacts count being reached.
        return false;
    }

    // Insert into the contacts container.
    //
    // Should the new contact be added at front or back?
    //
    // Original strategy added to the front. Since processing done front to back, front
    // adding means container more a LIFO container, while back adding means more a FIFO.
    //
    m_contacts.emplace_back(fixtureA, proxyA.childIndex, fixtureB, proxyB.childIndex);
    const auto contact = GetContactPtr(m_contacts.back());
    assert(contact);
    if (!contact)
    {
        return false;
    }

    BodyAtty::Insert(*bodyA, contact);
    BodyAtty::Insert(*bodyB, contact);

    // Wake up the bodies
    if (!fixtureA->IsSensor() && !fixtureB->IsSensor())
    {
        if (bodyA->IsSpeedable())
        {
            BodyAtty::SetAwakeFlag(*bodyA);
        }
        if (bodyB->IsSpeedable())
        {
            BodyAtty::SetAwakeFlag(*bodyB);
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

PreStepStats::counter_type World::SynchronizeProxies(const StepConf& conf)
{
    auto proxiesMoved = PreStepStats::counter_type{0};
    for (auto&& body: m_bodiesForProxies)
    {
        const auto xfm = body->GetTransformation();
        proxiesMoved += Synchronize(*body, xfm, xfm, conf.displaceMultiplier, conf.aabbExtension);
    }
    m_bodiesForProxies.clear();
    return proxiesMoved;
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

    if (IsLocked())
    {
        throw LockedError();
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
        body.SetAcceleration(body.IsAccelerable()? GetGravity(): Vec2_zero * MeterPerSquareSecond,
                             AngularAcceleration{0});
        for (auto&& fixture: body.GetFixtures())
        {
            InternalTouchProxies(fixture);
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
    if (!IsValid(shape))
    {
        return nullptr;
    }
    
    if (IsLocked())
    {
        throw LockedError();
    }
    
    const auto fixture = BodyAtty::CreateFixture(body, shape, def);

    if (body.IsEnabled())
    {
        RegisterForProxies(fixture);
    }
    
    // Adjust mass properties if needed.
    if (fixture->GetDensity() > Density{0})
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
        throw LockedError();
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

    const auto found = BodyAtty::DestroyFixture(body, fixture);
    if (!found)
    {
        // Fixture probably destroyed already.
        return false;
    }
    
    BodyAtty::SetMassDataDirty(body);
    if (resetMassData)
    {
        body.ResetMassData();
    }
    
    return true;
}

void World::CreateProxies(Fixture& fixture, const Length aabbExtension)
{
    assert(fixture.GetProxyCount() == 0);
    
    const auto shape = fixture.GetShape();
    const auto xfm = GetTransformation(fixture);
    
    // Reserve proxy space and create proxies in the broad-phase.
    const auto childCount = shape->GetChildCount();
    const auto proxies = static_cast<FixtureProxy*>(alloc(sizeof(FixtureProxy) * childCount));
    for (auto childIndex = decltype(childCount){0}; childIndex < childCount; ++childIndex)
    {
        const auto dp = shape->GetChild(childIndex);
        const auto aabb = ComputeAABB(dp, xfm);
        const auto proxyPtr = proxies + childIndex;

        // Note: proxyId from CreateProxy can be higher than the number of fixture proxies.
        const auto fattenedAABB = GetFattenedAABB(aabb, aabbExtension);
        const auto proxyId = m_broadPhase.CreateProxy(fattenedAABB, proxyPtr);
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

    const auto emptyArray = static_cast<FixtureProxy*>(nullptr);
    FixtureAtty::SetProxies(fixture, Span<FixtureProxy>(emptyArray, size_t{0}));
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
                                 const Transformation xfm1, const Transformation xfm2,
                                 const RealNum multiplier, const Length extension)
{
    assert(::box2d::IsValid(xfm1));
    assert(::box2d::IsValid(xfm2));
    
    auto updatedCount = child_count_t{0};
    const auto shape = fixture.GetShape();
    const auto displacement = xfm2.p - xfm1.p;
    const auto proxies = FixtureAtty::GetProxies(fixture);
    for (auto&& proxy: proxies)
    {
        const auto dp = shape->GetChild(proxy.childIndex);

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        const auto aabb1 = ComputeAABB(dp, xfm1);
        const auto aabb2 = ComputeAABB(dp, xfm2);
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
                                   const RealNum multiplier, const Length aabbExtension)
{
    auto updatedCount = contact_count_t{0};
    for (auto&& fixture: body.GetFixtures())
    {
        updatedCount += Synchronize(fixture, xfm1, xfm2, multiplier, aabbExtension);
    }
    return updatedCount;
}

// Free functions...

StepStats Step(World& world, Time dt, World::ts_iters_type velocityIterations,
               World::ts_iters_type positionIterations)
{
    StepConf conf;
    conf.SetTime(dt);
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

contact_count_t GetTouchingCount(const World& world) noexcept
{
    auto count = contact_count_t{0};
    for (auto&& c: world.GetContacts())
    {
        const auto contact = GetContactPtr(c);
        if (contact->IsTouching())
        {
            ++count;
        }
    }
    return count;
}

size_t GetFixtureCount(const World& world) noexcept
{
    auto sum = size_t{0};
    for (auto&& b: world.GetBodies())
    {
        const auto body = GetBodyPtr(b);
        sum += GetFixtureCount(*body);
    }
    return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
    auto shapes = std::set<const Shape*>();
    for (auto&& b: world.GetBodies())
    {
        const auto body = GetBodyPtr(b);
        for (auto&& fixture: body->GetFixtures())
        {
            shapes.insert(fixture.GetShape());
        }
    }
    return shapes.size();
}

size_t GetAwakeCount(const World& world) noexcept
{
    auto count = size_t(0);
    for (auto&& b: world.GetBodies())
    {
        const auto body = GetBodyPtr(b);
        if (body->IsAwake())
        {
            ++count;
        }
    }
    return count;
}
    
size_t Awaken(World& world) noexcept
{
    auto awoken = size_t{0};
    for (auto&& b: world.GetBodies())
    {
        const auto body = GetBodyPtr(b);
        if (box2d::Awaken(*body))
        {
            ++awoken;
        }
    }
    return awoken;
}

void ClearForces(World& world) noexcept
{
    const auto g = world.GetGravity();
    for (auto&& b: world.GetBodies())
    {
        const auto body = GetBodyPtr(b);
        body->SetAcceleration(g, AngularAcceleration{0});
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
