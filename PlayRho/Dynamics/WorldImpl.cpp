/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <PlayRho/Dynamics/WorldImpl.hpp>

#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>
#include <PlayRho/Dynamics/Island.hpp>
#include <PlayRho/Dynamics/MovementConf.hpp>
#include <PlayRho/Dynamics/ContactImpulsesList.hpp>

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/JointVisitor.hpp>
#include <PlayRho/Dynamics/Joints/FunctionalJointVisitor.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Joints/TargetJoint.hpp>
#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/Joints/WheelJoint.hpp>
#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Joints/RopeJoint.hpp>
#include <PlayRho/Dynamics/Joints/MotorJoint.hpp>

#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>
#include <PlayRho/Dynamics/Contacts/PositionConstraint.hpp>

#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/TimeOfImpact.hpp>
#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>

#include <PlayRho/Common/LengthError.hpp>
#include <PlayRho/Common/DynamicMemory.hpp>
#include <PlayRho/Common/FlagGuard.hpp>
#include <PlayRho/Common/WrongState.hpp>

#include <algorithm>
#include <new>
#include <functional>
#include <type_traits>
#include <memory>
#include <set>
#include <vector>
#include <unordered_map>

#ifdef DO_PAR_UNSEQ
#include <atomic>
#endif

//#define DO_THREADED
#if defined(DO_THREADED)
#include <future>
#endif

using std::for_each;
using std::remove;
using std::sort;
using std::transform;
using std::unique;

namespace playrho {
namespace d2 {

static_assert(std::is_default_constructible<WorldImpl>::value,
              "WorldImpl must be default constructible!");
static_assert(std::is_copy_constructible<WorldImpl>::value,
              "WorldImpl must be copy constructible!");
static_assert(std::is_copy_assignable<WorldImpl>::value,
              "WorldImpl must be copy assignable!");
static_assert(std::is_nothrow_destructible<WorldImpl>::value,
              "WorldImpl must be nothrow destructible!");

using playrho::size;

/// @brief Body pointer alias.
using BodyPtr = Body*;

/// @brief A body pointer and body constraint pointer pair.
using BodyConstraintsPair = std::pair<const Body* const, BodyConstraint*>;

/// @brief Collection of body constraints.
using BodyConstraints = std::vector<BodyConstraint>;

/// @brief Collection of position constraints.
using PositionConstraints = std::vector<PositionConstraint>;

/// @brief Collection of velocity constraints.
using VelocityConstraints = std::vector<VelocityConstraint>;

/// @brief Contact pointer type.
using ContactPtr = Contact*;

struct WorldImpl::ContactUpdateConf
{
    DistanceConf distance; ///< Distance configuration data.
    Manifold::Conf manifold; ///< Manifold configuration data.
};

namespace {

inline void IntegratePositions(BodyConstraints& bodies, Time h)
{
    assert(IsValid(h));
    for_each(begin(bodies), end(bodies), [&](BodyConstraint& bc) {
        const auto velocity = bc.GetVelocity();
        const auto translation = h * velocity.linear;
        const auto rotation = h * velocity.angular;
        bc.SetPosition(bc.GetPosition() + Position{translation, rotation});
    });
}

/// Reports the given constraints to the listener.
/// @details
/// This calls the listener's PostSolve method for all size(contacts) elements of
/// the given array of constraints.
/// @param listener Listener to call.
/// @param constraints Array of m_contactCount contact velocity constraint elements.
inline void Report(const WorldImpl::ImpulsesContactListener& listener,
                   const std::vector<ContactID>& contacts,
                   const VelocityConstraints& constraints,
                   StepConf::iteration_type solved)
{
    const auto numContacts = size(contacts);
    for (auto i = decltype(numContacts){0}; i < numContacts; ++i)
    {
        listener(contacts[i], GetContactImpulses(constraints[i]), solved);
    }
}

inline void AssignImpulses(Manifold& var, const VelocityConstraint& vc)
{
    assert(var.GetPointCount() >= vc.GetPointCount());
    
    auto assignProc = [&](VelocityConstraint::size_type i) {
        var.SetPointImpulses(i, GetNormalImpulseAtPoint(vc, i), GetTangentImpulseAtPoint(vc, i));
    };
#if 0
    // Branch free assignment causes problems in TilesComeToRest test.
    assignProc(1);
    assignProc(0);
#else
    const auto count = vc.GetPointCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        assignProc(i);
    }
#endif
}

inline void WarmStartVelocities(const VelocityConstraints& velConstraints)
{
    for_each(cbegin(velConstraints), cend(velConstraints),
                  [&](const VelocityConstraint& vc) {
        const auto vp = CalcWarmStartVelocityDeltas(vc);
        const auto bodyA = vc.GetBodyA();
        const auto bodyB = vc.GetBodyB();
        bodyA->SetVelocity(bodyA->GetVelocity() + std::get<0>(vp));
        bodyB->SetVelocity(bodyB->GetVelocity() + std::get<1>(vp));
    });
}

BodyConstraintsMap GetBodyConstraintsMap(const Island::Bodies& bodies,
                                         BodyConstraints &bodyConstraints)
{
    auto map = BodyConstraintsMap{};
    map.reserve(size(bodies));
    for_each(cbegin(bodies), cend(bodies), [&](const auto& body) {
        const auto i = static_cast<size_t>(&body - data(bodies));
        assert(i < size(bodies));
#ifdef USE_VECTOR_MAP
        map.push_back(BodyConstraintPair{body, &bodyConstraints[i]});
#else
        map[body] = &bodyConstraints[i];
#endif
    });
#ifdef USE_VECTOR_MAP
    sort(begin(map), end(map), [](BodyConstraintPair a, BodyConstraintPair b) {
        return std::get<const Body*>(a) < std::get<const Body*>(b);
    });
#endif
    return map;
}

BodyConstraints GetBodyConstraints(const Island::Bodies& bodies,
                                   const ArrayAllocator<Body>& bodyBuffer,
                                   Time h, MovementConf conf)
{
    auto constraints = BodyConstraints{};
    constraints.reserve(size(bodies));
    transform(cbegin(bodies), cend(bodies), back_inserter(constraints), [&](const auto &b) {
        return GetBodyConstraint(bodyBuffer[UnderlyingValue(b)], h, conf);
    });
    return constraints;
}

PositionConstraints GetPositionConstraints(const ArrayAllocator<Fixture>& fixtureBuffer,
                                           const ArrayAllocator<Contact>& contactBuffer,
                                           const Island::Contacts& contacts,
                                           BodyConstraintsMap& bodies)
{
    auto constraints = PositionConstraints{};
    constraints.reserve(size(contacts));
    transform(cbegin(contacts), cend(contacts), back_inserter(constraints),
              [&](const auto& contactID) {
        const auto& contact = contactBuffer[UnderlyingValue(contactID)];
        const auto& manifold = contact.GetManifold();
        const auto fixtureA = GetFixtureA(contact);
        const auto fixtureB = GetFixtureB(contact);
        const auto indexA = GetChildIndexA(contact);
        const auto indexB = GetChildIndexB(contact);
        const auto bodyA = GetBodyA(contact);
        const auto bodyB = GetBodyB(contact);
        const auto shapeA = fixtureBuffer[UnderlyingValue(fixtureA)].GetShape();
        const auto shapeB = fixtureBuffer[UnderlyingValue(fixtureB)].GetShape();
        const auto bodyConstraintA = At(bodies, bodyA);
        const auto bodyConstraintB = At(bodies, bodyB);
        const auto radiusA = GetVertexRadius(shapeA, indexA);
        const auto radiusB = GetVertexRadius(shapeB, indexB);
        return PositionConstraint{
            manifold, *bodyConstraintA, radiusA, *bodyConstraintB, radiusB
        };
    });
    return constraints;
}

/// @brief Gets the velocity constraints for the given inputs.
/// @details Inializes the velocity constraints with the position dependent portions of
///   the current position constraints.
/// @post Velocity constraints will have their "normal" field set to the world manifold
///   normal for them.
/// @post Velocity constraints will have their constraint points set.
/// @see SolveVelocityConstraints.
VelocityConstraints GetVelocityConstraints(const ArrayAllocator<Fixture>& fixtureBuffer,
                                           const ArrayAllocator<Contact>& contactBuffer,
                                           const Island::Contacts& contacts,
                                           BodyConstraintsMap& bodies,
                                           const VelocityConstraint::Conf conf)
{
    auto velConstraints = VelocityConstraints{};
    velConstraints.reserve(size(contacts));
    transform(cbegin(contacts), cend(contacts), back_inserter(velConstraints),
              [&](const auto& contactID) {
        const auto& contact = contactBuffer[UnderlyingValue(contactID)];
        const auto& manifold = contact.GetManifold();
        const auto fixtureA = contact.GetFixtureA();
        const auto fixtureB = contact.GetFixtureB();
        const auto friction = contact.GetFriction();
        const auto restitution = contact.GetRestitution();
        const auto tangentSpeed = contact.GetTangentSpeed();
        const auto indexA = GetChildIndexA(contact);
        const auto indexB = GetChildIndexB(contact);
        const auto bodyA = fixtureBuffer[UnderlyingValue(fixtureA)].GetBody();
        const auto shapeA = fixtureBuffer[UnderlyingValue(fixtureA)].GetShape();
        const auto bodyB = fixtureBuffer[UnderlyingValue(fixtureB)].GetBody();
        const auto shapeB = fixtureBuffer[UnderlyingValue(fixtureB)].GetShape();
        const auto bodyConstraintA = At(bodies, bodyA);
        const auto bodyConstraintB = At(bodies, bodyB);
        const auto radiusA = GetVertexRadius(shapeA, indexA);
        const auto radiusB = GetVertexRadius(shapeB, indexB);
        const auto xfA = GetTransformation(bodyConstraintA->GetPosition(),
                                           bodyConstraintA->GetLocalCenter());
        const auto xfB = GetTransformation(bodyConstraintB->GetPosition(),
                                           bodyConstraintB->GetLocalCenter());
        const auto worldManifold = GetWorldManifold(manifold, xfA, radiusA, xfB, radiusB);
        return VelocityConstraint{friction, restitution, tangentSpeed, worldManifold,
            *bodyConstraintA, *bodyConstraintB, conf};
    });
    return velConstraints;
}

/// "Solves" the velocity constraints.
/// @details Updates the velocities and velocity constraint points' normal and tangent impulses.
/// @pre <code>UpdateVelocityConstraints</code> has been called on the velocity constraints.
/// @return Maximum momentum used for solving both the tangential and normal portions of
///   the velocity constraints.
Momentum SolveVelocityConstraintsViaGS(VelocityConstraints& velConstraints)
{
    auto maxIncImpulse = 0_Ns;
    for_each(begin(velConstraints), end(velConstraints), [&](VelocityConstraint& vc)
    {
        maxIncImpulse = std::max(maxIncImpulse, GaussSeidel::SolveVelocityConstraint(vc));
    });
    return maxIncImpulse;
}

/// Solves the given position constraints.
/// @details This updates positions (and nothing else) by calling the position constraint solving function.
/// @note Can't expect the returned minimum separation to be greater than or equal to
///  <code>-conf.linearSlop</code> because code won't push the separation above this
///   amount to begin with.
/// @return Minimum separation.
Length SolvePositionConstraintsViaGS(PositionConstraints& posConstraints,
                                     ConstraintSolverConf conf)
{
    auto minSeparation = std::numeric_limits<Length>::infinity();
    
    for_each(begin(posConstraints), end(posConstraints), [&](PositionConstraint &pc) {
        assert(pc.GetBodyA() != pc.GetBodyB()); // Confirms ContactManager::Add() did its job.
        const auto res = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
        pc.GetBodyA()->SetPosition(res.pos_a);
        pc.GetBodyB()->SetPosition(res.pos_b);
        minSeparation = std::min(minSeparation, res.min_separation);
    });
    
    return minSeparation;
}

#if 0
/// Solves the given position constraints.
///
/// @details This updates positions (and nothing else) for the two bodies identified by the
///   given indexes by calling the position constraint solving function.
///
/// @note Can't expect the returned minimum separation to be greater than or equal to
///  <code>ConstraintSolverConf.max_separation</code> because code won't push the separation
///   above this amount to begin with.
///
/// @param positionConstraints Positions constraints.
/// @param bodyConstraintA Pointer to body constraint for body A.
/// @param bodyConstraintB Pointer to body constraint for body B.
/// @param conf Configuration for solving the constraint.
///
/// @return Minimum separation (which is the same as the max amount of penetration/overlap).
///
Length SolvePositionConstraints(PositionConstraints& posConstraints,
                                const BodyConstraint* bodyConstraintA,
                                const BodyConstraint* bodyConstraintB,
                                ConstraintSolverConf conf)
{
    auto minSeparation = std::numeric_limits<Length>::infinity();

    for_each(begin(posConstraints), end(posConstraints), [&](PositionConstraint &pc) {
        const auto moveA = (pc.GetBodyA() == bodyConstraintA) || (pc.GetBodyA() == bodyConstraintB);
        const auto moveB = (pc.GetBodyB() == bodyConstraintA) || (pc.GetBodyB() == bodyConstraintB);
        const auto res = SolvePositionConstraint(pc, moveA, moveB, conf);
        pc.GetBodyA()->SetPosition(res.pos_a);
        pc.GetBodyB()->SetPosition(res.pos_b);
        minSeparation = std::min(minSeparation, res.min_separation);
    });

    return minSeparation;
}
#endif

inline Time GetUnderActiveTime(const Body& b, const StepConf& conf) noexcept
{
    const auto underactive = IsUnderActive(b.GetVelocity(), conf.linearSleepTolerance,
                                           conf.angularSleepTolerance);
    const auto sleepable = b.IsSleepingAllowed();
    return (sleepable && underactive)? b.GetUnderActiveTime() + conf.GetTime(): 0_s;
}

inline Time UpdateUnderActiveTimes(const Island::Bodies& bodies,
                                   ArrayAllocator<Body>& bodyBuffer,
                                   const StepConf& conf)
{
    auto minUnderActiveTime = std::numeric_limits<Time>::infinity();
    for_each(cbegin(bodies), cend(bodies), [&](const auto& bodyID)
    {
        auto& b = bodyBuffer[UnderlyingValue(bodyID)];
        if (b.IsSpeedable())
        {
            const auto underActiveTime = GetUnderActiveTime(b, conf);
            b.SetUnderActiveTime(underActiveTime);
            minUnderActiveTime = std::min(minUnderActiveTime, underActiveTime);
        }
    });
    return minUnderActiveTime;
}

inline BodyCounter Sleepem(const Island::Bodies& bodies,
                           ArrayAllocator<Body>& bodyBuffer)
{
    auto unawoken = BodyCounter{0};
    for_each(cbegin(bodies), cend(bodies), [&](const auto& bodyID)
    {
        if (Unawaken(bodyBuffer[UnderlyingValue(bodyID)]))
        {
            ++unawoken;
        }
    });
    return unawoken;
}

inline bool IsValidForTime(TOIOutput::State state) noexcept
{
    return state == TOIOutput::e_touching;
}

void FlagContactsForFiltering(ArrayAllocator<Contact>& contactBuffer, BodyID bodyA,
                              const SizedRange<Body::Contacts::const_iterator>& contactsBodyB,
                              BodyID bodyB) noexcept
{
    for (auto& ci: contactsBodyB)
    {
        auto& contact = contactBuffer[UnderlyingValue(GetContactPtr(ci))];
        const auto bA = contact.GetBodyA();
        const auto bB = contact.GetBodyB();
        const auto other = (bA != bodyB)? bA: bB;
        if (other == bodyA)
        {
            // Flag the contact for filtering at the next time step (where either
            // body is awake).
            contact.FlagForFiltering();
        }
    }
}

/// @brief Gets the update configuration from the given step configuration data.
WorldImpl::ContactUpdateConf GetUpdateConf(const StepConf& conf) noexcept
{
    return WorldImpl::ContactUpdateConf{GetDistanceConf(conf), GetManifoldConf(conf)};
}

[[maybe_unused]]
bool HasSensor(const ArrayAllocator<Fixture>& fixtures, const Contact& c)
{
    return fixtures[UnderlyingValue(c.GetFixtureA())].IsSensor() || fixtures[UnderlyingValue(c.GetFixtureB())].IsSensor();
}

void FlagForUpdating(ArrayAllocator<Contact>& contactsBuffer,
                     SizedRange<Body::Contacts::const_iterator> contacts)
{
    std::for_each(begin(contacts), end(contacts), [&](const auto& ci) {
        contactsBuffer[UnderlyingValue(std::get<ContactID>(ci))].FlagForUpdating();
    });
}

bool ShouldCollide(const Body& lhs, const Body& rhs, BodyID rhsID)
{
    // At least one body should be accelerable/dynamic.
    if (!lhs.IsAccelerable() && !rhs.IsAccelerable())
    {
        return false;
    }

    // Does a joint prevent collision?
    const auto joints = lhs.GetJoints();
    const auto it = std::find_if(cbegin(joints), cend(joints), [&](const auto& ji) {
        return (std::get<BodyID>(ji) == rhsID) &&
            !(static_cast<Joint*>(UnderlyingValue(std::get<JointID>(ji)))->GetCollideConnected());
    });
    return it == end(joints);
}

/// @brief Executes function for all the fixtures of the given body.
void ForallFixtures(Body& b, std::function<void(FixtureID)> callback)
{
    const auto fixtures = b.GetFixtures();
    std::for_each(begin(fixtures), end(fixtures), [&](const auto& f) {
        callback(f);
    });
}

/// @brief Clears the given body's joints list.
void ClearJoints(Body& b, std::function<void(JointID)> callback)
{
    const auto joints = b.GetJoints();
    std::for_each(cbegin(joints), cend(joints), [&](const auto& j) {
        callback(std::get<JointID>(j));
    });
    b.ClearJoints();
    assert(empty(b.GetJoints()));
}

} // anonymous namespace

WorldImpl::WorldImpl(const WorldConf& def):
    m_tree{def.initialTreeSize},
    m_minVertexRadius{def.minVertexRadius},
    m_maxVertexRadius{def.maxVertexRadius}
{
    if (def.minVertexRadius > def.maxVertexRadius)
    {
        throw InvalidArgument("max vertex radius must be >= min vertex radius");
    }
    m_proxyKeys.reserve(1024);
    m_proxies.reserve(1024);
}

WorldImpl::WorldImpl(const WorldImpl& other):
    m_bodyBuffer{other.m_bodyBuffer},
    m_fixtureBuffer{other.m_fixtureBuffer},
    m_contactBuffer{other.m_contactBuffer},
    m_tree{other.m_tree},
    m_flags{other.m_flags},
    m_inv_dt0{other.m_inv_dt0},
    m_minVertexRadius{other.m_minVertexRadius},
    m_maxVertexRadius{other.m_maxVertexRadius}
{
    auto bodyMap = std::map<const Body*, Body*>();
    auto fixtureMap = std::map<const Fixture*, Fixture*>();
    CopyJoints(bodyMap, other.GetJoints());
}

// WorldImpl::WorldImpl(WorldImpl&& other) = delete;

WorldImpl& WorldImpl::copy(const WorldImpl& other)
{
    Clear();

    m_flags = other.m_flags;
    m_inv_dt0 = other.m_inv_dt0;
    m_minVertexRadius = other.m_minVertexRadius;
    m_maxVertexRadius = other.m_maxVertexRadius;
    m_tree = other.m_tree;

    auto bodyMap = std::map<const Body*, Body*>();
    auto fixtureMap = std::map<const Fixture*, Fixture*>();
    CopyJoints(bodyMap, other.GetJoints());

    return *this;
}

// WorldImpl& WorldImpl::operator= (WorldImpl&& other) = delete;

WorldImpl::~WorldImpl() noexcept
{
    InternalClear();
}

void WorldImpl::Clear()
{
    if (IsLocked())
    {
        throw WrongState("Clear: world is locked");
    }
    InternalClear();
}

void WorldImpl::InternalClear() noexcept
{
    m_proxyKeys.clear();
    m_proxies.clear();
    m_fixturesForProxies.clear();
    m_bodiesForProxies.clear();

    for_each(cbegin(m_joints), cend(m_joints), [&](const auto& j) {
        if (m_jointDestructionListener)
        {
            m_jointDestructionListener(j);
        }
        Joint::Destroy(static_cast<Joint*>(UnderlyingValue(j)));
    });
    for_each(begin(m_bodies), end(m_bodies), [&](const auto& body) {
        auto& b = m_bodyBuffer[UnderlyingValue(body)];
        b.ClearContacts();
        b.ClearJoints();
        ForallFixtures(b, [&](FixtureID id) {
            if (m_fixtureDestructionListener)
            {
                m_fixtureDestructionListener(id);
            }
            DestroyProxies(m_proxies, m_tree, m_fixtureBuffer[UnderlyingValue(id)]);
        });
        b.ClearFixtures();
    });

    for_each(cbegin(m_bodies), cend(m_bodies), [&](const auto& body) {
        m_bodyBuffer.Free(UnderlyingValue(body));
    });
    for_each(cbegin(m_contacts), cend(m_contacts), [&](const Contacts::value_type& c){
        m_contactBuffer.Free(UnderlyingValue(std::get<ContactID>(c)));
    });

    m_bodies.clear();
    m_joints.clear();
    m_contacts.clear();
}

void WorldImpl::CopyJoints(const std::map<const Body*, Body*>& bodyMap,
                           SizedRange<Joints::const_iterator> range)
{
#if 0
    class JointCopier: public ConstJointVisitor
    {
    public:
        
        JointCopier(WorldImpl& w, std::map<const Body*, Body*> bodies):
            world{w}, bodyMap{std::move(bodies)}
        {
            // Intentionally empty.
        }

        void Visit(const RevoluteJoint& oldJoint) override
        {
            auto def = GetRevoluteJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }
        
        void Visit(const PrismaticJoint& oldJoint) override
        {
            auto def = GetPrismaticJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }

        void Visit(const DistanceJoint& oldJoint) override
        {
            auto def = GetDistanceJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }
        
        void Visit(const PulleyJoint& oldJoint) override
        {
            auto def = GetPulleyJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }
        
        void Visit(const TargetJoint& oldJoint) override
        {
            auto def = GetTargetJointConf(oldJoint);
            def.bodyA = (def.bodyA)? bodyMap.at(def.bodyA): nullptr;
            def.bodyB = (def.bodyB)? bodyMap.at(def.bodyB): nullptr;
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }
        
        void Visit(const GearJoint& oldJoint) override
        {
            auto def = GetGearJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            def.joint1 = jointMap.at(def.joint1);
            def.joint2 = jointMap.at(def.joint2);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }

        void Visit(const WheelJoint& oldJoint) override
        {
            auto def = GetWheelJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }

        void Visit(const WeldJoint& oldJoint) override
        {
            auto def = GetWeldJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }

        void Visit(const FrictionJoint& oldJoint) override
        {
            auto def = GetFrictionJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }

        void Visit(const RopeJoint& oldJoint) override
        {
            auto def = GetRopeJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }

        void Visit(const MotorJoint& oldJoint) override
        {
            auto def = GetMotorJointConf(oldJoint);
            def.bodyA = bodyMap.at(def.bodyA);
            def.bodyB = bodyMap.at(def.bodyB);
            jointMap[&oldJoint] = Add(Joint::Create(def));
        }
        
        Joint* Add(Joint* newJoint)
        {
            world.Add(newJoint);
            return newJoint;
        }

        WorldImpl& world;
        const std::map<const Body*, Body*> bodyMap;
        std::map<const Joint*, Joint*> jointMap;
    };

    auto copier = JointCopier{*this, bodyMap};
    auto jointMap = std::map<const Joint*, Joint*>();
    for (const auto& otherJoint: range)
    {
        otherJoint->Accept(copier);
    }
#endif
}

BodyID WorldImpl::CreateBody(const BodyConf& def)
{
    if (IsLocked())
    {
        throw WrongState("CreateBody: world is locked");
    }
    if (size(m_bodies) >= MaxBodies)
    {
        throw LengthError("CreateBody: operation would exceed MaxBodies");
    }
    const auto id = static_cast<BodyID>(
        static_cast<BodyID::underlying_type>(m_bodyBuffer.Allocate(def)));
    m_bodies.push_back(id);
    return id;
}

void WorldImpl::Remove(BodyID id) noexcept
{
    UnregisterForProxies(id);
    const auto it = find(cbegin(m_bodies), cend(m_bodies), id);
    if (it != cend(m_bodies))
    {
        m_bodies.erase(it);
        m_bodyBuffer.Free(UnderlyingValue(id));
    }
}

void WorldImpl::Destroy(BodyID id)
{
    if (IsLocked())
    {
        throw WrongState("Destroy: world is locked");
    }

    const auto body = &GetBody(id);

    // Delete the attached joints.
    ClearJoints(*body, [&](JointID jointID) {
        if (m_jointDestructionListener)
        {
            m_jointDestructionListener(jointID);
        }
        Remove(jointID);
        Joint::Destroy(static_cast<Joint*>(UnderlyingValue(jointID)));
    });

    // Destroy the attached contacts.
    body->Erase([&](ContactID contactID) {
        Destroy(m_contacts, m_endContactListener, contactID, body);
        return true;
    });

    // Delete the attached fixtures. This destroys broad-phase proxies.
    ForallFixtures(*body, [&](FixtureID fixtureID) {
        if (m_fixtureDestructionListener)
        {
            m_fixtureDestructionListener(fixtureID);
        }
        EraseAll(m_fixturesForProxies, fixtureID);
        DestroyProxies(m_proxies, m_tree, m_fixtureBuffer[UnderlyingValue(fixtureID)]);
        m_fixtureBuffer.Free(UnderlyingValue(fixtureID));
    });
    body->ClearFixtures();

    Remove(id);
}

JointID WorldImpl::CreateJoint(const JointConf& def)
{
    if (IsLocked())
    {
        throw WrongState("CreateJoint: world is locked");
    }
    
    if (size(m_joints) >= MaxJoints)
    {
        throw LengthError("CreateJoint: operation would exceed MaxJoints");
    }
    
    // Note: creating a joint doesn't wake the bodies.
    const auto id = static_cast<JointID>(Joint::Create(def));
    Add(id, !def.collideConnected);
    return id;
}

bool WorldImpl::Add(JointID id, bool flagForFiltering)
{
    m_joints.push_back(id);

    const auto joint = static_cast<Joint*>(UnderlyingValue(id));
    const auto bodyA = joint->GetBodyA();
    const auto bodyB = joint->GetBodyB();
    if (bodyA != InvalidBodyID)
    {
        m_bodyBuffer[UnderlyingValue(bodyA)].Insert(id, bodyB);
    }
    if (bodyB != InvalidBodyID)
    {
        m_bodyBuffer[UnderlyingValue(bodyB)].Insert(id, bodyA);
    }
    if (flagForFiltering && (bodyA != InvalidBodyID) && (bodyB != InvalidBodyID))
    {
        FlagContactsForFiltering(m_contactBuffer, bodyA, m_bodyBuffer[UnderlyingValue(bodyB)].GetContacts(), bodyB);
    }

    return true;
}

bool WorldImpl::Remove(JointID id) noexcept
{
    const auto endIter = cend(m_joints);
    const auto iter = find(cbegin(m_joints), endIter, id);
    if (iter == endIter)
    {
        return false;
    }

    // Disconnect from island graph.
    const auto& joint = *static_cast<Joint*>(UnderlyingValue(id));
    const auto bodyIdA = joint.GetBodyA();
    const auto bodyIdB = joint.GetBodyB();

    const auto collideConnected = joint.GetCollideConnected();
    // If the joint prevented collisions, then flag any contacts for filtering.
    if ((!collideConnected) && (bodyIdA != InvalidBodyID) && (bodyIdB != InvalidBodyID))
    {
        FlagContactsForFiltering(m_contactBuffer, bodyIdA, m_bodyBuffer[UnderlyingValue(bodyIdB)].GetContacts(), bodyIdB);
    }

    // Wake up connected bodies.
    if (bodyIdA != InvalidBodyID)
    {
        auto& bodyA = m_bodyBuffer[UnderlyingValue(bodyIdA)];
        bodyA.SetAwake();
        bodyA.Erase(id);
    }
    if (bodyIdB != InvalidBodyID)
    {
        auto& bodyB = m_bodyBuffer[UnderlyingValue(bodyIdB)];
        bodyB.SetAwake();
        bodyB.Erase(id);
    }

    m_joints.erase(iter);
    return true;
}

void WorldImpl::Destroy(JointID joint)
{
    if (joint != InvalidJointID)
    {
        if (IsLocked())
        {
            throw WrongState("Destroy: world is locked");
        }
        if (Remove(joint))
        {
            Joint::Destroy(static_cast<Joint*>(UnderlyingValue(joint)));
        }
    }
}

void WorldImpl::AddToIsland(Island& island, BodyID seedID,
                            Bodies::size_type& remNumBodies,
                            Contacts::size_type& remNumContacts,
                            Joints::size_type& remNumJoints)
{
    auto& seed = m_bodyBuffer[UnderlyingValue(seedID)];

    assert(!seed.IsIslanded());
    assert(seed.IsSpeedable());
    assert(seed.IsAwake());
    assert(seed.IsEnabled());
    assert(remNumBodies != 0);
    assert(remNumBodies < MaxBodies);

    // Perform a depth first search (DFS) on the constraint graph.

    // Create a stack for bodies to be is-in-island that aren't already in the island.
    auto bodies = std::vector<BodyID>{};
    bodies.reserve(remNumBodies);
    bodies.push_back(seedID);
    auto stack = BodyStack{std::move(bodies)};
    seed.SetIslandedFlag();
    AddToIsland(island, stack, remNumBodies, remNumContacts, remNumJoints);
}

void WorldImpl::AddToIsland(Island& island, BodyStack& stack,
                            Bodies::size_type& remNumBodies,
                            Contacts::size_type& remNumContacts,
                            Joints::size_type& remNumJoints)
{
    while (!empty(stack))
    {
        // Grab the next body off the stack and add it to the island.
        const auto bodyID = stack.top();
        stack.pop();

        auto& body = m_bodyBuffer[UnderlyingValue(bodyID)];

        assert(body.IsEnabled());
        island.m_bodies.push_back(bodyID);
        assert(remNumBodies > 0);
        --remNumBodies;

        // Don't propagate islands across bodies that can't have a velocity (static bodies).
        // This keeps islands smaller and helps with isolating separable collision clusters.
        if (!body.IsSpeedable())
        {
            continue;
        }

        // Make sure the body is awake (without resetting sleep timer).
        body.SetAwakeFlag();

        const auto oldNumContacts = size(island.m_contacts);
        // Adds appropriate contacts of current body and appropriate 'other' bodies of those contacts.
        AddContactsToIsland(island, stack, &body);

        const auto newNumContacts = size(island.m_contacts);
        assert(newNumContacts >= oldNumContacts);
        const auto netNumContacts = newNumContacts - oldNumContacts;
        assert(remNumContacts >= netNumContacts);
        remNumContacts -= netNumContacts;
        
        const auto numJoints = size(island.m_joints);
        // Adds appropriate joints of current body and appropriate 'other' bodies of those joint.
        AddJointsToIsland(island, stack, &body);

        remNumJoints -= size(island.m_joints) - numJoints;
    }
}

void WorldImpl::AddContactsToIsland(Island& island, BodyStack& stack, const Body* b)
{
    const auto contacts = b->GetContacts();
    for_each(cbegin(contacts), cend(contacts), [&](const KeyedContactPtr& ci) {
        const auto contactID = std::get<ContactID>(ci);
        auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
        if (!contact.IsIslanded() && contact.IsEnabled() && contact.IsTouching())
        {
            const auto& fA = m_fixtureBuffer[UnderlyingValue(contact.GetFixtureA())];
            const auto& fB = m_fixtureBuffer[UnderlyingValue(contact.GetFixtureB())];
            if (!fA.IsSensor() && !fB.IsSensor())
            {
                const auto bA = &m_bodyBuffer[UnderlyingValue(fA.GetBody())];
                const auto bB = &m_bodyBuffer[UnderlyingValue(fB.GetBody())];
                const auto other = (bA != b)? bA: bB;
                island.m_contacts.push_back(contactID);
                contact.SetIslanded();
                if (!other->IsIslanded())
                {
                    stack.push(static_cast<BodyID>(static_cast<BodyID::underlying_type>(m_bodyBuffer.GetIndex(other))));
                    other->SetIslandedFlag();
                }
            }
        }
    });
}

void WorldImpl::AddJointsToIsland(Island& island, BodyStack& stack, const Body* b)
{
    const auto joints = b->GetJoints();
    for_each(cbegin(joints), cend(joints), [&](const Body::KeyedJointPtr& ji) {
        // Use data of ji before dereferencing its pointers.
        const auto otherID = std::get<BodyID>(ji);
        const auto jointID = std::get<JointID>(ji);
        const auto other = (otherID == InvalidBodyID)? static_cast<Body*>(nullptr): &m_bodyBuffer[UnderlyingValue(otherID)];
        assert(!other || other->IsEnabled() || !other->IsAwake());
        const auto joint = static_cast<Joint*>((jointID == InvalidJointID)? nullptr: UnderlyingValue(jointID));
        assert(joint);
        if (!joint->IsIslanded() && (!other || other->IsEnabled()))
        {
            island.m_joints.push_back(jointID);
            joint->SetIslanded();
            if (other && !other->IsIslanded())
            {
                stack.push(otherID);
                other->SetIslandedFlag();
            }
        }
    });
}

WorldImpl::Bodies::size_type
WorldImpl::RemoveUnspeedablesFromIslanded(const std::vector<BodyID>& bodies,
                                          ArrayAllocator<Body>& buffer)
{
    // Allow static bodies to participate in other islands.
    auto numRemoved = Bodies::size_type{0};
    for_each(begin(bodies), end(bodies), [&](BodyID id) {
        auto& body = buffer[UnderlyingValue(id)];
        if (!body.IsSpeedable())
        {
            body.UnsetIslandedFlag();
            ++numRemoved;
        }
    });
    return numRemoved;
}

RegStepStats WorldImpl::SolveReg(const StepConf& conf)
{
    auto stats = RegStepStats{};
    auto remNumBodies = size(m_bodies); ///< Remaining number of bodies.
    auto remNumContacts = size(m_contacts); ///< Remaining number of contacts.
    auto remNumJoints = size(m_joints); ///< Remaining number of joints.

    // Clear all the island flags.
    // This builds the logical set of bodies, contacts, and joints eligible for resolution.
    // As bodies, contacts, or joints get added to resolution islands, they're essentially
    // removed from this eligible set.
    for_each(begin(m_bodies), end(m_bodies), [this](const auto& b) {
        m_bodyBuffer[UnderlyingValue(b)].UnsetIslandedFlag();
    });
    for_each(begin(m_contacts), end(m_contacts), [this](const auto& c) {
        m_contactBuffer[UnderlyingValue(std::get<ContactID>(c))].UnsetIslanded();
    });
    for_each(begin(m_joints), end(m_joints), [](const auto& j) {
        static_cast<Joint*>(UnderlyingValue(j))->UnsetIslanded();
    });

#if defined(DO_THREADED)
    std::vector<std::future<IslandStats>> futures;
    futures.reserve(remNumBodies);
#endif
    // Build and simulate all awake islands.
    for (auto&& b: m_bodies)
    {
        auto& body = m_bodyBuffer[UnderlyingValue(b)];
        assert(!body.IsAwake() || body.IsSpeedable());
        if (!body.IsIslanded() && body.IsAwake() && body.IsEnabled())
        {
            ++stats.islandsFound;

            // Size the island for the remaining un-evaluated bodies, contacts, and joints.
            Island island(remNumBodies, remNumContacts, remNumJoints);

            AddToIsland(island, b, remNumBodies, remNumContacts, remNumJoints);
            remNumBodies += RemoveUnspeedablesFromIslanded(island.m_bodies, m_bodyBuffer);

#if defined(DO_THREADED)
            // Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
            futures.push_back(std::async(std::launch::async, &WorldImpl::SolveRegIslandViaGS,
                                         m_bodyBuffer, m_contactBuffer, m_fixtureBuffer,
                                         conf, island, m_postSolveContactListener));
#else
            const auto solverResults = SolveRegIslandViaGS(m_bodyBuffer, m_contactBuffer, m_fixtureBuffer,
                                                           conf, island, m_postSolveContactListener);
            ::playrho::Update(stats, solverResults);
#endif
        }
    }

#if defined(DO_THREADED)
    for (auto&& future: futures)
    {
        const auto solverResults = future.get();
        Update(stats, solverResults);
    }
#endif

    for (auto&& b: m_bodies)
    {
        auto& body = m_bodyBuffer[UnderlyingValue(b)];
        // A non-static body that was in an island may have moved.
        if (body.IsIslanded() && body.IsSpeedable())
        {
            // Update fixtures (for broad-phase).
            stats.proxiesMoved += Synchronize(body, GetTransform0(body.GetSweep()), body.GetTransformation(),
                                              conf.displaceMultiplier, conf.aabbExtension);
        }
    }

    // Look for new contacts.
    stats.contactsAdded = FindNewContacts();
    
    return stats;
}

IslandStats WorldImpl::SolveRegIslandViaGS(ArrayAllocator<Body>& bodyBuffer,
                                           ArrayAllocator<Contact>& contacts,
                                           const ArrayAllocator<Fixture>& fixtures,
                                           const StepConf& conf, Island island,
                                           const ImpulsesContactListener& contactListener)
{
    assert(!empty(island.m_bodies) || !empty(island.m_contacts) || !empty(island.m_joints));
    
    auto results = IslandStats{};
    results.positionIterations = conf.regPositionIterations;
    const auto h = conf.GetTime(); ///< Time step.

    // Update bodies' pos0 values.
    for_each(cbegin(island.m_bodies), cend(island.m_bodies), [&](const auto& bodyID) {
        auto& body = bodyBuffer[UnderlyingValue(bodyID)];
        body.SetPosition0(GetPosition1(body)); // like Advance0(1) on the sweep.
    });

    // Copy bodies' pos1 and velocity data into local arrays.
    auto bodyConstraints = GetBodyConstraints(island.m_bodies, bodyBuffer, h, GetMovementConf(conf));
    auto bodyConstraintsMap = GetBodyConstraintsMap(island.m_bodies, bodyConstraints);
    auto posConstraints = GetPositionConstraints(fixtures, contacts,
                                                 island.m_contacts, bodyConstraintsMap);
    auto velConstraints = GetVelocityConstraints(fixtures, contacts,
                                                 island.m_contacts, bodyConstraintsMap,
                                                 GetRegVelocityConstraintConf(conf));
    
    if (conf.doWarmStart)
    {
        WarmStartVelocities(velConstraints);
    }

    const auto psConf = GetRegConstraintSolverConf(conf);

    for_each(cbegin(island.m_joints), cend(island.m_joints), [&](const auto& joint) {
        static_cast<Joint*>(UnderlyingValue(joint))->InitVelocityConstraints(bodyConstraintsMap,
                                           conf, psConf);
    });
    
    results.velocityIterations = conf.regVelocityIterations;
    for (auto i = decltype(conf.regVelocityIterations){0}; i < conf.regVelocityIterations; ++i)
    {
        auto jointsOkay = true;
        for_each(cbegin(island.m_joints), cend(island.m_joints), [&](const auto& jointID) {
            auto& j = *static_cast<Joint*>(static_cast<JointID::underlying_type>(jointID));
            jointsOkay &= j.SolveVelocityConstraints(bodyConstraintsMap, conf);
        });

        // Note that the new incremental impulse can potentially be orders of magnitude
        // greater than the last incremental impulse used in this loop.
        const auto newIncImpulse = SolveVelocityConstraintsViaGS(velConstraints);
        results.maxIncImpulse = std::max(results.maxIncImpulse, newIncImpulse);

        if (jointsOkay && (newIncImpulse <= conf.regMinMomentum))
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
    IntegratePositions(bodyConstraints, h);
    
    // Solve position constraints
    for (auto i = decltype(conf.regPositionIterations){0}; i < conf.regPositionIterations; ++i)
    {
        const auto minSeparation = SolvePositionConstraintsViaGS(posConstraints, psConf);
        results.minSeparation = std::min(results.minSeparation, minSeparation);
        const auto contactsOkay = (minSeparation >= conf.regMinSeparation);

        auto jointsOkay = true;
        for_each(cbegin(island.m_joints), cend(island.m_joints), [&](const auto& jointID) {
            auto& j = *static_cast<Joint*>(static_cast<JointID::underlying_type>(jointID));
            jointsOkay &= j.SolvePositionConstraints(bodyConstraintsMap, psConf);
        });

        if (contactsOkay && jointsOkay)
        {
            // Reached tolerance, early out...
            results.positionIterations = i + 1;
            results.solved = true;
            break;
        }
    }
    
    // Update normal and tangent impulses of contacts' manifold points
    for_each(cbegin(velConstraints), cend(velConstraints), [&](const VelocityConstraint& vc) {
        const auto i = static_cast<VelocityConstraints::size_type>(&vc - data(velConstraints));
        auto& manifold = contacts[UnderlyingValue(island.m_contacts[i])].GetMutableManifold();
        AssignImpulses(manifold, vc);
    });
    
    for_each(cbegin(bodyConstraints), cend(bodyConstraints), [&](const auto& bc) {
        const auto i = static_cast<size_t>(&bc - data(bodyConstraints));
        assert(i < size(bodyConstraints));
        // Could normalize position here to avoid unbounded angles but angular
        // normalization isn't handled correctly by joints that constrain rotation.
        auto& body = bodyBuffer[UnderlyingValue(island.m_bodies[i])];
        body.JustSetVelocity(bc.GetVelocity());
        if (UpdateBody(body, bc.GetPosition()))
        {
            FlagForUpdating(contacts, body.GetContacts());
        }
    });
    
    // XXX: Should contacts needing updating be updated now??

    if (contactListener)
    {
        Report(contactListener, island.m_contacts, velConstraints,
               results.solved? results.positionIterations - 1: StepConf::InvalidIteration);
    }
    
    results.bodiesSlept = BodyCounter{0};
    const auto minUnderActiveTime = UpdateUnderActiveTimes(island.m_bodies, bodyBuffer, conf);
    if ((minUnderActiveTime >= conf.minStillTimeToSleep) && results.solved)
    {
        results.bodiesSlept = static_cast<decltype(results.bodiesSlept)>(Sleepem(island.m_bodies,
                                                                                 bodyBuffer));
    }

    return results;
}

void WorldImpl::ResetBodiesForSolveTOI(Bodies& bodies, ArrayAllocator<Body>& buffer) noexcept
{
    for_each(begin(bodies), end(bodies), [&](const auto& body) {
        auto& b = buffer[UnderlyingValue(body)];
        b.UnsetIslandedFlag();
        b.ResetAlpha0();
    });
}

void WorldImpl::ResetContactsForSolveTOI(ArrayAllocator<Contact>& buffer, const Contacts& contacts) noexcept
{
    for_each(begin(contacts), end(contacts), [&buffer](const auto& c) {
        auto& contact = buffer[UnderlyingValue(std::get<ContactID>(c))];
        contact.UnsetIslanded();
        contact.UnsetToi();
        contact.SetToiCount(0);
    });
}

WorldImpl::UpdateContactsData WorldImpl::UpdateContactTOIs(ArrayAllocator<Contact>& contactBuffer,
                                                           ArrayAllocator<Body>& bodyBuffer,
                                                           const ArrayAllocator<Fixture>& fixtureBuffer,
                                                           const Contacts& contacts, const StepConf& conf)
{
    auto results = UpdateContactsData{};

    const auto toiConf = GetToiConf(conf);
    for (const auto& contact: contacts)
    {
        auto& c = contactBuffer[UnderlyingValue(std::get<ContactID>(contact))];
        if (c.HasValidToi())
        {
            ++results.numValidTOI;
            continue;
        }
        if (!IsEnabled(c) || IsSensor(c) || !IsActive(c) || !IsImpenetrable(c))
        {
            continue;
        }
        if (c.GetToiCount() >= conf.maxSubSteps)
        {
            // What are the pros/cons of this?
            // Larger m_maxSubSteps slows down the simulation.
            // m_maxSubSteps of 44 and higher seems to decrease the occurrance of tunneling
            // of multiple bullet body collisions with static objects.
            ++results.numAtMaxSubSteps;
            continue;
        }

        auto& bA = bodyBuffer[UnderlyingValue(c.GetBodyA())];
        auto& bB = bodyBuffer[UnderlyingValue(c.GetBodyB())];

        /*
         * Put the sweeps onto the same time interval.
         * Presumably no unresolved collisions happen before the maximum of the bodies'
         * alpha-0 times. So long as the least TOI of the contacts is always the first
         * collision that gets dealt with, this presumption is safe.
         */
        const auto alpha0 = std::max(bA.GetSweep().GetAlpha0(), bB.GetSweep().GetAlpha0());
        assert(alpha0 >= 0 && alpha0 < 1);
        bA.Advance0(alpha0);
        bB.Advance0(alpha0);

        // Compute the TOI for this contact (one or both bodies are active and impenetrable).
        // Computes the time of impact in interval [0, 1]
        const auto proxyA = GetChild(fixtureBuffer[UnderlyingValue(c.GetFixtureA())].GetShape(),
                                     c.GetChildIndexA());
        const auto proxyB = GetChild(fixtureBuffer[UnderlyingValue(c.GetFixtureB())].GetShape(),
                                     c.GetChildIndexB());

        // Large rotations can make the root finder of TimeOfImpact fail, so normalize sweep angles.
        const auto sweepA = GetNormalized(bA.GetSweep());
        const auto sweepB = GetNormalized(bB.GetSweep());

        // Compute the TOI for this contact (one or both bodies are active and impenetrable).
        // Computes the time of impact in interval [0, 1]
        // Large rotations can make the root finder of TimeOfImpact fail, so normalize the sweep angles.
        const auto output = GetToiViaSat(proxyA, sweepA, proxyB, sweepB, toiConf);

        // Use Min function to handle floating point imprecision which possibly otherwise
        // could provide a TOI that's greater than 1.
        const auto toi = IsValidForTime(output.state)?
            std::min(alpha0 + (1 - alpha0) * output.time, Real{1}): Real{1};
        assert(toi >= alpha0 && toi <= 1);
        c.SetToi(toi);
        
        results.maxDistIters = std::max(results.maxDistIters, output.stats.max_dist_iters);
        results.maxToiIters = std::max(results.maxToiIters, output.stats.toi_iters);
        results.maxRootIters = std::max(results.maxRootIters, output.stats.max_root_iters);
        ++results.numUpdatedTOI;
    }

    return results;
}

WorldImpl::ContactToiData WorldImpl::GetSoonestContact(const Contacts& contacts,
                                                       const ArrayAllocator<Contact>& buffer) noexcept
{
    auto minToi = nextafter(Real{1}, Real{0});
    auto found = InvalidContactID;
    auto count = ContactCounter{0};
    for (const auto& contact: contacts)
    {
        const auto contactID = std::get<ContactID>(contact);
        const auto& c = buffer[UnderlyingValue(contactID)];
        if (c.HasValidToi())
        {
            const auto toi = c.GetToi();
            if (minToi > toi)
            {
                minToi = toi;
                found = contactID;
                count = 1;
            }
            else if (minToi == toi)
            {
                // Have multiple contacts at the current minimum time of impact.
                ++count;
            }
        }
    }
    return ContactToiData{found, minToi, count};
}

ToiStepStats WorldImpl::SolveToi(const StepConf& conf)
{
    auto stats = ToiStepStats{};

    if (IsStepComplete())
    {
        ResetBodiesForSolveTOI(m_bodies, m_bodyBuffer);
        ResetContactsForSolveTOI(m_contactBuffer, m_contacts);
    }

    const auto subStepping = GetSubStepping();

    // Find TOI events and solve them.
    for (;;)
    {
        const auto updateData = UpdateContactTOIs(m_contactBuffer, m_bodyBuffer, m_fixtureBuffer,
                                                  m_contacts, conf);
        stats.contactsAtMaxSubSteps += updateData.numAtMaxSubSteps;
        stats.contactsUpdatedToi += updateData.numUpdatedTOI;
        stats.maxDistIters = std::max(stats.maxDistIters, updateData.maxDistIters);
        stats.maxRootIters = std::max(stats.maxRootIters, updateData.maxRootIters);
        stats.maxToiIters = std::max(stats.maxToiIters, updateData.maxToiIters);
        
        const auto next = GetSoonestContact(m_contacts, m_contactBuffer);
        const auto contactID = next.contact;
        const auto ncount = next.simultaneous;
        if (contactID == InvalidContactID)
        {
            // No more TOI events to handle within the current time step. Done!
            SetStepComplete(true);
            break;
        }

        stats.maxSimulContacts = std::max(stats.maxSimulContacts,
                                          static_cast<decltype(stats.maxSimulContacts)>(ncount));
        stats.contactsFound += ncount;
        auto islandsFound = 0u;
        auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
        if (!contact.IsIslanded())
        {
            /*
             * Confirm that contact is as it's supposed to be according to contract of the
             * GetSoonestContact method from which this contact was obtained.
             */
            assert(contact.IsEnabled());
            assert(!HasSensor(m_fixtureBuffer, contact));
            assert(IsActive(contact));
            assert(IsImpenetrable(contact));

            const auto solverResults = SolveToi(contactID, conf);
            stats.minSeparation = std::min(stats.minSeparation, solverResults.minSeparation);
            stats.maxIncImpulse = std::max(stats.maxIncImpulse, solverResults.maxIncImpulse);
            stats.islandsSolved += solverResults.solved;
            stats.sumPosIters += solverResults.positionIterations;
            stats.sumVelIters += solverResults.velocityIterations;
            if ((solverResults.positionIterations > 0) || (solverResults.velocityIterations > 0))
            {
                ++islandsFound;
            }
            stats.contactsUpdatedTouching += solverResults.contactsUpdated;
            stats.contactsSkippedTouching += solverResults.contactsSkipped;
        }
        stats.islandsFound += islandsFound;

        // Reset island flags and synchronize broad-phase proxies.
        for (auto&& b: m_bodies)
        {
            auto& body = m_bodyBuffer[UnderlyingValue(b)];
            if (body.IsIslanded())
            {
                body.UnsetIslandedFlag();
                if (body.IsAccelerable())
                {
                    const auto xfm0 = GetTransform0(body.GetSweep());
                    const auto xfm1 = body.GetTransformation();
                    stats.proxiesMoved += Synchronize(body, xfm0, xfm1,
                                                      conf.displaceMultiplier, conf.aabbExtension);
                    ResetContactsForSolveTOI(m_contactBuffer, body);
                }
            }
        }

        // Commit fixture proxy movements to the broad-phase so that new contacts are created.
        // Also, some contacts can be destroyed.
        stats.contactsAdded += FindNewContacts();

        if (subStepping)
        {
            SetStepComplete(false);
            break;
        }
    }
    return stats;
}

IslandStats WorldImpl::SolveToi(ContactID contactID, const StepConf& conf)
{
    // Note:
    //   This method is what used to be b2World::SolveToi(const b2TimeStep& step).
    //   It also differs internally from Erin's implementation.
    //
    //   Here's some specific behavioral differences:
    //   1. Bodies don't get their under-active times reset (like they do in Erin's code).

    auto contactsUpdated = ContactCounter{0};
    auto contactsSkipped = ContactCounter{0};

    auto& contact = m_contactBuffer[UnderlyingValue(contactID)];

    /*
     * Confirm that contact is as it's supposed to be according to contract of the
     * GetSoonestContacts method from which this contact should have been obtained.
     */
    assert(contact.IsEnabled());
    assert(!HasSensor(m_fixtureBuffer, contact));
    assert(IsActive(contact));
    assert(IsImpenetrable(contact));
    assert(!contact.IsIslanded());
    
    const auto toi = contact.GetToi();
    const auto bodyIdA = contact.GetBodyA();
    const auto bodyIdB = contact.GetBodyB();
    auto& bA = m_bodyBuffer[UnderlyingValue(bodyIdA)];
    auto& bB = m_bodyBuffer[UnderlyingValue(bodyIdB)];

    /* XXX: if (toi != 0)? */
    /* if (bA->GetSweep().GetAlpha0() != toi || bB->GetSweep().GetAlpha0() != toi) */
    // Seems contact manifold needs updating regardless.
    {
        const auto backupA = bA.GetSweep();
        const auto backupB = bB.GetSweep();

        // Advance the bodies to the TOI.
        assert(toi != 0 || (bA.GetSweep().GetAlpha0() == 0 && bB.GetSweep().GetAlpha0() == 0));
        bA.Advance(toi);
        FlagForUpdating(m_contactBuffer, bA.GetContacts());
        bB.Advance(toi);
        FlagForUpdating(m_contactBuffer, bB.GetContacts());

        // The TOI contact likely has some new contact points.
        contact.SetEnabled();
        if (contact.NeedsUpdating())
        {
            Update(contactID, GetUpdateConf(conf));
            ++contactsUpdated;
        }
        else
        {
            ++contactsSkipped;
        }
        contact.UnsetToi();
        contact.IncrementToiCount();

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
            bA.Restore(backupA);
            bB.Restore(backupB);
            auto results = IslandStats{};
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
    
    if (bA.IsSpeedable())
    {
        bA.SetAwakeFlag();
        // XXX should the body's under-active time be reset here?
        //   Erin's code does for here but not in b2World::Solve(const b2TimeStep& step).
        //   Calling Body::ResetUnderActiveTime() has performance implications.
    }

    if (bB.IsSpeedable())
    {
        bB.SetAwakeFlag();
        // XXX should the body's under-active time be reset here?
        //   Erin's code does for here but not in b2World::Solve(const b2TimeStep& step).
        //   Calling Body::ResetUnderActiveTime() has performance implications.
    }

    // Build the island
    Island island(used(m_bodyBuffer), used(m_contactBuffer), 0);

     // These asserts get triggered sometimes if contacts within TOI are iterated over.
    assert(!bA.IsIslanded());
    assert(!bB.IsIslanded());

    island.m_bodies.push_back(bodyIdA);
    bA.SetIslandedFlag();
    island.m_bodies.push_back(bodyIdB);
    bB.SetIslandedFlag();
    island.m_contacts.push_back(contactID);
    contact.SetIslanded();

    // Process the contacts of the two bodies, adding appropriate ones to the island,
    // adding appropriate other bodies of added contacts, and advancing those other
    // bodies sweeps and transforms to the minimum contact's TOI.
    if (bA.IsAccelerable())
    {
        const auto procOut = ProcessContactsForTOI(bodyIdA, island, toi, conf);
        contactsUpdated += procOut.contactsUpdated;
        contactsSkipped += procOut.contactsSkipped;
    }
    if (bB.IsAccelerable())
    {
        const auto procOut = ProcessContactsForTOI(bodyIdB, island, toi, conf);
        contactsUpdated += procOut.contactsUpdated;
        contactsSkipped += procOut.contactsSkipped;
    }

    RemoveUnspeedablesFromIslanded(island.m_bodies, m_bodyBuffer);

    // Now solve for remainder of time step.
    //
    // Note: subConf is written the way it is because MSVS2017 emitted errors when
    //   written as:
    //     SolveToi(StepConf{conf}.SetTime((1 - toi) * conf.GetTime()), island);
    //
    auto subConf = StepConf{conf};
    auto results = SolveToiViaGS(island, subConf.SetTime((1 - toi) * conf.GetTime()));
    results.contactsUpdated += contactsUpdated;
    results.contactsSkipped += contactsSkipped;
    return results;
}

bool WorldImpl::UpdateBody(Body& body, const Position& pos)
{
    assert(IsValid(pos));
    body.SetPosition1(pos);
    const auto oldXfm = body.GetTransformation();
    const auto newXfm = GetTransformation(GetPosition1(body), body.GetLocalCenter());
    if (newXfm != oldXfm)
    {
        body.SetTransformation(newXfm);
        return true;
    }
    return false;
}

IslandStats WorldImpl::SolveToiViaGS(const Island& island, const StepConf& conf)
{
    auto results = IslandStats{};

    /*
     * Presumably the regular phase resolution has already taken care of updating the
     * body's velocity w.r.t. acceleration and damping such that this call here to get
     * the body constraint doesn't need to pass an elapsed time (and doesn't need to
     * update the velocity from what it already is).
     */
    auto bodyConstraints = GetBodyConstraints(island.m_bodies, m_bodyBuffer, 0_s,
                                              GetMovementConf(conf));
    auto bodyConstraintsMap = GetBodyConstraintsMap(island.m_bodies, bodyConstraints);

    // Initialize the body state.
#if 0
    for (auto&& contact: island.m_contacts)
    {
        const auto fixtureA = contact->GetFixtureA();
        const auto fixtureB = contact->GetFixtureB();
        const auto bodyA = fixtureA->GetBody();
        const auto bodyB = fixtureB->GetBody();

        bodyConstraintsMap[bodyA] = GetBodyConstraint(*bodyA);
        bodyConstraintsMap[bodyB] = GetBodyConstraint(*bodyB);
    }
#endif

    auto posConstraints = GetPositionConstraints(m_fixtureBuffer, m_contactBuffer,
                                                 island.m_contacts, bodyConstraintsMap);

    // Solve TOI-based position constraints.
    assert(results.minSeparation == std::numeric_limits<Length>::infinity());
    assert(results.solved == false);
    results.positionIterations = conf.toiPositionIterations;
    {
        const auto psConf = GetToiConstraintSolverConf(conf);

        for (auto i = decltype(conf.toiPositionIterations){0}; i < conf.toiPositionIterations; ++i)
        {
            //
            // Note: There are two flavors of the SolvePositionConstraints function.
            //   One takes an extra two arguments that are the indexes of two bodies that are
            //   okay tomove. The other one does not.
            //   Calling the selective solver (that takes the two additional arguments) appears
            //   to result in phsyics simulations that are more prone to tunneling. Meanwhile,
            //   using the non-selective solver would presumably be slower (since it appears to
            //   have more that it will do). Assuming that slower is preferable to tunnelling,
            //   then the non-selective function is the one to be calling here.
            //
            const auto minSeparation = SolvePositionConstraintsViaGS(posConstraints, psConf);
            results.minSeparation = std::min(results.minSeparation, minSeparation);
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
        bodyA->SetPosition0(bodyConstraintsMap.at(bodyA).GetPosition());
        bodyB->SetPosition0(bodyConstraintsMap.at(bodyB).GetPosition());
    }
#else
    for_each(cbegin(bodyConstraints), cend(bodyConstraints), [&](const BodyConstraint& bc) {
        const auto i = static_cast<size_t>(&bc - data(bodyConstraints));
        assert(i < size(bodyConstraints));
        m_bodyBuffer[UnderlyingValue(island.m_bodies[i])].SetPosition0(bc.GetPosition());
    });
#endif

    auto velConstraints = GetVelocityConstraints(m_fixtureBuffer, m_contactBuffer,
                                                 island.m_contacts, bodyConstraintsMap,
                                                 GetToiVelocityConstraintConf(conf));

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.

    // Solve velocity constraints.
    assert(results.maxIncImpulse == 0_Ns);
    results.velocityIterations = conf.toiVelocityIterations;
    for (auto i = decltype(conf.toiVelocityIterations){0}; i < conf.toiVelocityIterations; ++i)
    {
        const auto newIncImpulse = SolveVelocityConstraintsViaGS(velConstraints);
        if (newIncImpulse <= conf.toiMinMomentum)
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

    IntegratePositions(bodyConstraints, conf.GetTime());

    for_each(cbegin(bodyConstraints), cend(bodyConstraints), [&](const BodyConstraint& bc) {
        const auto i = static_cast<size_t>(&bc - data(bodyConstraints));
        assert(i < size(bodyConstraints));
        auto& body = m_bodyBuffer[UnderlyingValue(island.m_bodies[i])];
        body.JustSetVelocity(bc.GetVelocity());
        if (UpdateBody(body, bc.GetPosition()))
        {
            FlagForUpdating(m_contactBuffer, body.GetContacts());
        }
    });

    if (m_postSolveContactListener)
    {
        Report(m_postSolveContactListener, island.m_contacts, velConstraints, results.positionIterations);
    }

    return results;
}
    
void WorldImpl::ResetContactsForSolveTOI(ArrayAllocator<Contact>& buffer, const Body& body) noexcept
{
    // Invalidate all contact TOIs on this displaced body.
    const auto contacts = body.GetContacts();
    for_each(cbegin(contacts), cend(contacts), [&buffer](const auto& ci) {
        auto& contact = buffer[UnderlyingValue(std::get<ContactID>(ci))];
        contact.UnsetIslanded();
        contact.UnsetToi();
    });
}

WorldImpl::ProcessContactsOutput
WorldImpl::ProcessContactsForTOI(BodyID id, Island& island, Real toi, const StepConf& conf)
{
    const auto& body = m_bodyBuffer[UnderlyingValue(id)];

    assert(body.IsIslanded());
    assert(body.IsAccelerable());
    assert(toi >= 0 && toi <= 1);

    auto results = ProcessContactsOutput{};
    assert(results.contactsUpdated == 0);
    assert(results.contactsSkipped == 0);
    
    const auto updateConf = GetUpdateConf(conf);

    // Note: the original contact (for body of which this method was called) already is-in-island.
    const auto bodyImpenetrable = body.IsImpenetrable();
    for (const auto& ci: body.GetContacts())
    {
        const auto contactID = std::get<ContactID>(ci);
        auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
        if (!contact.IsIslanded())
        {
            if (!contact.IsSensor())
            {
                const auto bodyIdA = contact.GetBodyA();
                const auto bodyIdB = contact.GetBodyB();
                const auto otherId = (bodyIdA != id)? bodyIdA: bodyIdB;
                auto& other = m_bodyBuffer[UnderlyingValue(otherId)];
                if (bodyImpenetrable || other.IsImpenetrable())
                {
                    const auto otherIslanded = other.IsIslanded();
                    {
                        const auto backup = other.GetSweep();
                        if (!otherIslanded /* && other->GetSweep().GetAlpha0() != toi */)
                        {
                            other.Advance(toi);
                            FlagForUpdating(m_contactBuffer, other.GetContacts());
                        }

                        // Update the contact points
                        contact.SetEnabled();
                        if (contact.NeedsUpdating())
                        {
                            Update(contactID, updateConf);
                            ++results.contactsUpdated;
                        }
                        else
                        {
                            ++results.contactsSkipped;
                        }

                        // Revert and skip if contact disabled by user or not touching anymore (very possible).
                        if (!contact.IsEnabled() || !contact.IsTouching())
                        {
                            other.Restore(backup);
                            continue;
                        }
                    }
                    island.m_contacts.push_back(contactID);
                    contact.SetIslanded();
                    if (!otherIslanded)
                    {
                        if (other.IsSpeedable())
                        {
                            other.SetAwakeFlag();
                        }
                        island.m_bodies.push_back(otherId);
                        other.SetIslandedFlag();
#if 0
                        if (other.IsAccelerable())
                        {
                            contactsUpdated += ProcessContactsForTOI(island, other, toi);
                        }
#endif
                    }
#ifndef NDEBUG
                    else
                    {
                        /*
                         * If other is-in-island but not in current island, then something's gone wrong.
                         * Other needs to be in current island but was already in the island.
                         * A previous contact island didn't grow to include all the bodies it needed or
                         * perhaps the current contact is-touching while another one wasn't and the
                         * inconsistency is throwing things off.
                         */
                        assert(Count(island, otherId) > 0);
                    }
#endif
                }
            }
        }
    }
    return results;
}

StepStats WorldImpl::Step(const StepConf& conf)
{
    assert((Length{m_maxVertexRadius} * Real{2}) +
           (Length{conf.linearSlop} / Real{4}) > (Length{m_maxVertexRadius} * Real{2}));
    
    if (IsLocked())
    {
        throw WrongState("Step: world is locked");
    }

    // "Named return value optimization" (NRVO) will make returning this more efficient.
    auto stepStats = StepStats{};
    {
        FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

        CreateAndDestroyProxies(conf.aabbExtension);
        m_fixturesForProxies.clear();

        stepStats.pre.proxiesMoved = SynchronizeProxies(conf);
        // pre.proxiesMoved is usually zero but sometimes isn't.

        {
            // Note: this may update bodies (in addition to the contacts container).
            const auto destroyStats = DestroyContacts(m_contacts, m_contactBuffer, m_bodyBuffer,
                                                      m_fixtureBuffer, m_tree, m_endContactListener);
            stepStats.pre.destroyed = destroyStats.erased;
        }

        if (HasNewFixtures())
        {
            UnsetNewFixtures();
            
            // New fixtures were added: need to find and create the new contacts.
            // Note: this may update bodies (in addition to the contacts container).
            stepStats.pre.added = FindNewContacts();
        }

        if (conf.GetTime() != 0_s)
        {
            m_inv_dt0 = conf.GetInvTime();

            // Could potentially run UpdateContacts multithreaded over split lists...
            const auto updateStats = UpdateContacts(conf);
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
                stepStats.toi = SolveToi(conf);
            }
        }
    }
    return stepStats;
}

void WorldImpl::ShiftOrigin(Length2 newOrigin)
{
    if (IsLocked())
    {
        throw WrongState("ShiftOrigin: world is locked");
    }

    // Optimize for newOrigin being different than current...
    const auto bodies = GetBodies();
    for (const auto& body: bodies)
    {
        auto& b = m_bodyBuffer[UnderlyingValue(body)];
        auto transformation = b.GetTransformation();
        transformation.p -= newOrigin;
        b.SetTransformation(transformation);
        FlagForUpdating(m_contactBuffer, b.GetContacts());
        auto sweep = b.GetSweep();
        sweep.pos0.linear -= newOrigin;
        sweep.pos1.linear -= newOrigin;
        b.SetSweep(sweep);
    }

    for_each(begin(m_joints), end(m_joints), [&](Joints::value_type& joint) {
        const auto j = static_cast<Joint*>(UnderlyingValue(joint));
        GetRef(j).ShiftOrigin(newOrigin);
    });

    m_tree.ShiftOrigin(newOrigin);
}

void WorldImpl::InternalDestroy(ContactID contactID,
                                ArrayAllocator<Body>& bodyBuffer,
                                ArrayAllocator<Contact>& contactBuffer,
                                ContactListener listener, Body* from)
{
    assert(contactID != InvalidContactID);
    auto& contact = contactBuffer[UnderlyingValue(contactID)];
    if (listener && contact.IsTouching())
    {
        // EndContact hadn't been called in DestroyOrUpdateContacts() since is-touching, so call it now
        listener(contactID);
    }
    const auto bodyIdA = contact.GetBodyA();
    const auto bodyIdB = contact.GetBodyB();
    const auto bodyA = &bodyBuffer[UnderlyingValue(bodyIdA)];
    const auto bodyB = &bodyBuffer[UnderlyingValue(bodyIdB)];
    if (bodyA != from)
    {
        bodyA->Erase(contactID);
    }
    if (bodyB != from)
    {
        bodyB->Erase(contactID);
    }
    if ((contact.GetManifold().GetPointCount() > 0) && !contact.IsSensor())
    {
        // Contact may have been keeping accelerable bodies of fixture A or B from moving.
        // Need to awaken those bodies now in case they are again movable.
        bodyA->SetAwake();
        bodyB->SetAwake();
    }
    contactBuffer.Free(UnderlyingValue(contactID));
}

void WorldImpl::Destroy(Contacts& contacts, ContactListener listener, ContactID contactID, Body* from)
{
    assert(contactID != InvalidContactID);
    const auto it = find_if(cbegin(contacts), cend(contacts),
                            [&](const Contacts::value_type& c) {
        return std::get<ContactID>(c) == contactID;
    });
    if (it != cend(contacts))
    {
        contacts.erase(it);
    }
    InternalDestroy(contactID, m_bodyBuffer, m_contactBuffer, listener, from);
}

WorldImpl::DestroyContactsStats WorldImpl::DestroyContacts(Contacts& contacts,
                                                           ArrayAllocator<Contact>& contactBuffer,
                                                           ArrayAllocator<Body>& bodyBuffer,
                                                           const ArrayAllocator<Fixture>& fixtureBuffer,
                                                           const DynamicTree& tree,
                                                           ContactListener listener)
{
    const auto beforeSize = size(contacts);
    contacts.erase(std::remove_if(begin(contacts), end(contacts), [&](const auto& c)
    {
        const auto key = std::get<ContactKey>(c);
        const auto contactID = std::get<ContactID>(c);
        
        if (!TestOverlap(tree, key.GetMin(), key.GetMax()))
        {
            // Destroy contacts that cease to overlap in the broad-phase.
            InternalDestroy(contactID, bodyBuffer, contactBuffer, listener);
            return true;
        }
        
        // Is this contact flagged for filtering?
        auto& contact = contactBuffer[UnderlyingValue(contactID)];
        if (contact.NeedsFiltering())
        {
            const auto bodyIdA = contact.GetBodyA();
            const auto& bodyA = bodyBuffer[UnderlyingValue(bodyIdA)];
            const auto& bodyB = bodyBuffer[UnderlyingValue(contact.GetBodyB())];
            const auto& fixtureA = fixtureBuffer[UnderlyingValue(contact.GetFixtureA())];
            const auto& fixtureB = fixtureBuffer[UnderlyingValue(contact.GetFixtureB())];
            if (!ShouldCollide(bodyB, bodyA, bodyIdA) || !ShouldCollide(fixtureA, fixtureB))
            {
                InternalDestroy(contactID, bodyBuffer, contactBuffer, listener);
                return true;
            }
            contact.UnflagForFiltering();
        }

        return false;
    }), end(contacts));
    const auto afterSize = size(contacts);

    auto stats = DestroyContactsStats{};
    stats.ignored = static_cast<ContactCounter>(afterSize);
    stats.erased = static_cast<ContactCounter>(beforeSize - afterSize);
    return stats;
}

WorldImpl::UpdateContactsStats WorldImpl::UpdateContacts(const StepConf& conf)
{
#ifdef DO_PAR_UNSEQ
    atomic<uint32_t> ignored;
    atomic<uint32_t> updated;
    atomic<uint32_t> skipped;
#else
    auto ignored = uint32_t{0};
    auto updated = uint32_t{0};
    auto skipped = uint32_t{0};
#endif

    const auto updateConf = GetUpdateConf(conf);
    
#if defined(DO_THREADED)
    std::vector<Contact*> contactsNeedingUpdate;
    contactsNeedingUpdate.reserve(size(m_contacts));
    std::vector<std::future<void>> futures;
    futures.reserve(size(m_contacts));
#endif

    // Update awake contacts.
    for_each(/*execution::par_unseq,*/ begin(m_contacts), end(m_contacts), [&](const auto& c) {
        const auto contactID = std::get<ContactID>(c);
        auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
#if 0
        Update(contact, updateConf);
        ++updated;
#else
        const auto& bodyA = m_bodyBuffer[UnderlyingValue(contact.GetBodyA())];
        const auto& bodyB = m_bodyBuffer[UnderlyingValue(contact.GetBodyB())];

        // Awake && speedable (dynamic or kinematic) means collidable.
        // At least one body must be collidable
        assert(!bodyA.IsAwake() || bodyA.IsSpeedable());
        assert(!bodyB.IsAwake() || bodyB.IsSpeedable());
        if (!bodyA.IsAwake() && !bodyB.IsAwake())
        {
            // This sometimes fails... is it important?
            //assert(!contact.HasValidToi());
            ++ignored;
            return;
        }

        // Possible that bodyA->GetSweep().GetAlpha0() != 0
        // Possible that bodyB->GetSweep().GetAlpha0() != 0

        // Update the contact manifold and notify the listener.
        contact.SetEnabled();

        // Note: ideally contacts are only updated if there was a change to:
        //   - The fixtures' sensor states.
        //   - The fixtures bodies' transformations.
        //   - The "maxCirclesRatio" per-step configuration state if contact IS NOT for sensor.
        //   - The "maxDistanceIters" per-step configuration state if contact IS for sensor.
        //
        if (contact.NeedsUpdating())
        {
            // The following may call listener but is otherwise thread-safe.
#if defined(DO_THREADED)
            contactsNeedingUpdate.push_back(&contact);
            //futures.push_back(async(&Update, this, *contact, conf)));
            //futures.push_back(async(launch::async, [=]{ Update(*contact, conf); }));
#else
            Update(contactID, updateConf);
#endif
        	++updated;
        }
        else
        {
            ++skipped;
        }
#endif
    });
    
#if defined(DO_THREADED)
    auto numJobs = size(contactsNeedingUpdate);
    const auto jobsPerCore = numJobs / 4;
    for (auto i = decltype(numJobs){0}; numJobs > 0 && i < 3; ++i)
    {
        futures.push_back(std::async(std::launch::async, [=]{
            const auto offset = jobsPerCore * i;
            for (auto j = decltype(jobsPerCore){0}; j < jobsPerCore; ++j)
            {
	            Update(*contactsNeedingUpdate[offset + j], updateConf);
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
                Update(*contactsNeedingUpdate[offset + j], updateConf);
            }
        }));
    }
    for (auto&& future: futures)
    {
        future.get();
    }
#endif
    
    return UpdateContactsStats{
        static_cast<ContactCounter>(ignored),
        static_cast<ContactCounter>(updated),
        static_cast<ContactCounter>(skipped)
    };
}

ContactCounter WorldImpl::FindNewContacts()
{
    m_proxyKeys.clear();

    // Accumalate contact keys for pairs of nodes that are overlapping and aren't identical.
    // Note that if the dynamic tree node provides the body pointer, it's assumed to be faster
    // to eliminate any node pairs that have the same body here before the key pairs are
    // sorted.
    for_each(cbegin(m_proxies), cend(m_proxies), [&](ProxyId pid) {
        const auto body0 = m_tree.GetLeafData(pid).body;
        const auto aabb = m_tree.GetAABB(pid);
        Query(m_tree, aabb, [&](ProxyId nodeId) {
            const auto body1 = m_tree.GetLeafData(nodeId).body;
            // A proxy cannot form a pair with itself.
            if ((nodeId != pid) && (body0 != body1))
            {
                m_proxyKeys.push_back(ContactKey{nodeId, pid});
            }
            return DynamicTreeOpcode::Continue;
        });
    });
    m_proxies.clear();

    // Sort and eliminate any duplicate contact keys.
    sort(begin(m_proxyKeys), end(m_proxyKeys));
    m_proxyKeys.erase(unique(begin(m_proxyKeys), end(m_proxyKeys)), end(m_proxyKeys));

    const auto numContactsBefore = size(m_contacts);
    for_each(cbegin(m_proxyKeys), cend(m_proxyKeys), [&](ContactKey key)
    {
        Add(key);
    });
    const auto numContactsAfter = size(m_contacts);
    return static_cast<ContactCounter>(numContactsAfter - numContactsBefore);
}

bool WorldImpl::Add(ContactKey key)
{
    const auto minKeyLeafData = m_tree.GetLeafData(key.GetMin());
    const auto maxKeyLeafData = m_tree.GetLeafData(key.GetMax());

    const auto bodyIdA = minKeyLeafData.body; // fixtureA->GetBody();
    const auto fixtureIdA = minKeyLeafData.fixture;
    const auto indexA = minKeyLeafData.childIndex;
    const auto bodyIdB = maxKeyLeafData.body; // fixtureB->GetBody();
    const auto fixtureIdB = maxKeyLeafData.fixture;
    const auto indexB = maxKeyLeafData.childIndex;

#if 0
    // Are the fixtures on the same body? They can be, and they often are.
    // Don't need nor want a contact for these fixtures if they are on the same body.
    if (bodyIdA == bodyIdB)
    {
        return false;
    }
#endif
    assert(bodyIdA != bodyIdB);

    auto& bodyA = m_bodyBuffer[UnderlyingValue(bodyIdA)];
    auto& bodyB = m_bodyBuffer[UnderlyingValue(bodyIdB)];
    auto& fixtureA = m_fixtureBuffer[UnderlyingValue(fixtureIdA)];
    auto& fixtureB = m_fixtureBuffer[UnderlyingValue(fixtureIdB)];

    // Does a joint override collision? Is at least one body dynamic?
    if (!ShouldCollide(bodyB, bodyA, bodyIdA) || !ShouldCollide(fixtureA, fixtureB))
    {
        return false;
    }
   
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
    // With compiler optimization enabled and 400 small bodies and Real=double...
    // For world:
    //   World::set<Contact*> shows up as .524 seconds max step
    //   World::list<Contact> shows up as .482 seconds max step.
    // For body:
    //    using contact map w/ proxy ID keys shows up as .561
    // W/ unordered_map: .529 seconds max step (step 15).
    // W/ World::list<Contact> and Body::list<ContactKey,Contact*>   .444s@step15, 1.063s-sumstep20
    // W/ World::list<Contact> and Body::list<ContactKey,Contact*>   .393s@step15, 1.063s-sumstep20
    // W/ World::list<Contact> and Body::list<ContactKey,Contact*>   .412s@step15, 1.012s-sumstep20
    // W/ World::list<Contact> and Body::vector<ContactKey,Contact*> .219s@step15, 0.659s-sumstep20

    // Does a contact already exist?
    // Identify body with least contacts and search it.
    // NOTE: Time trial testing found the following rough ordering of data structures, to be
    // fastest to slowest: vector, list, unorderered_set, unordered_map,
    //     set, map.
    const auto contactsA = bodyA.GetContacts();
    const auto contactsB = bodyB.GetContacts();
    const auto& bodyContacts = (size(contactsA) < size(contactsB))? contactsA: contactsB;
    const auto it = find_if(cbegin(bodyContacts), cend(bodyContacts), [&](KeyedContactPtr ci) {
        return std::get<ContactKey>(ci) == key;
    });
    if (it != cend(bodyContacts))
    {
        return false;
    }
    
    if (size(m_contacts) >= MaxContacts)
    {
        // New contact was needed, but denied due to MaxContacts count being reached.
        return false;
    }

    const auto contactID = static_cast<ContactID>(static_cast<ContactID::underlying_type>(
        m_contactBuffer.Allocate(bodyIdA, fixtureIdA, indexA, bodyIdB, fixtureIdB, indexB)));
    auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
    if (bodyA.IsImpenetrable() || bodyB.IsImpenetrable())
    {
        contact.SetImpenetrable();
    }
    if (bodyA.IsAwake() || bodyB.IsAwake())
    {
        contact.SetIsActive();
    }
    if (fixtureA.IsSensor() || fixtureB.IsSensor())
    {
        contact.SetIsSensor();
    }
    contact.SetFriction(GetDefaultFriction(fixtureA, fixtureB));
    contact.SetRestitution(GetDefaultRestitution(fixtureA, fixtureB));

    // Insert into the contacts container.
    //
    // Should the new contact be added at front or back?
    //
    // Original strategy added to the front. Since processing done front to back, front
    // adding means container more a LIFO container, while back adding means more a FIFO.
    //
    m_contacts.push_back(KeyedContactPtr{key, contactID});

    bodyA.Insert(key, contactID);
    bodyB.Insert(key, contactID);

    // Wake up the bodies
    if (!fixtureA.IsSensor() && !fixtureB.IsSensor())
    {
        if (bodyA.IsSpeedable())
        {
            bodyA.SetAwakeFlag();
        }
        if (bodyB.IsSpeedable())
        {
            bodyB.SetAwakeFlag();
        }
    }
#endif

    return true;
}

void WorldImpl::SetSensor(FixtureID id, bool value)
{
    auto& fixture = GetFixture(id);
    if (fixture.IsSensor() != value)
    {
        // sensor state is changing...
        fixture.SetSensor(value);
        auto& body = m_bodyBuffer[UnderlyingValue(fixture.GetBody())];
        body.SetAwake();
        FlagForUpdating(m_contactBuffer, body.GetContacts());
    }
}

void WorldImpl::RegisterForProxies(FixtureID id)
{
    m_fixturesForProxies.push_back(id);
}

void WorldImpl::RegisterForProxies(BodyID id)
{
    m_bodiesForProxies.push_back(id);
}

void WorldImpl::UnregisterForProxies(BodyID id)
{
    const auto first = remove(begin(m_bodiesForProxies), end(m_bodiesForProxies), id);
    m_bodiesForProxies.erase(first, end(m_bodiesForProxies));
}

void WorldImpl::CreateAndDestroyProxies(Length extension)
{
    for_each(begin(m_fixturesForProxies), end(m_fixturesForProxies), [&](const auto& fixtureID) {
        auto& fixture = m_fixtureBuffer[UnderlyingValue(fixtureID)];
        auto& body = m_bodyBuffer[UnderlyingValue(fixture.GetBody())];
        const auto enabled = body.IsEnabled();

        if (fixture.GetProxies().empty())
        {
            if (enabled)
            {
                CreateProxies(fixtureID, fixture, body.GetTransformation(), m_proxies, m_tree, extension);
            }
        }
        else
        {
            if (!enabled)
            {
                DestroyProxies(m_proxies, m_tree, fixture);

                // Destroy any contacts associated with the fixture.
                body.Erase([&](ContactID contactID) {
                    const auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
                    const auto fixtureA = contact.GetFixtureA();
                    const auto fixtureB = contact.GetFixtureB();
                    if ((fixtureA == fixtureID) || (fixtureB == fixtureID))
                    {
                        Destroy(m_contacts, m_endContactListener, contactID, &body);
                        return true;
                    }
                    return false;
                });
            }
        }
    });
}

PreStepStats::counter_type WorldImpl::SynchronizeProxies(const StepConf& conf)
{
    auto proxiesMoved = PreStepStats::counter_type{0};
    for_each(begin(m_bodiesForProxies), end(m_bodiesForProxies), [&](const auto& bodyID) {
        const auto& b = m_bodyBuffer[UnderlyingValue(bodyID)];
        const auto xfm = b.GetTransformation();
        // Not always true: assert(GetTransform0(b->GetSweep()) == xfm);
        proxiesMoved += Synchronize(b, xfm, xfm, conf.displaceMultiplier, conf.aabbExtension);
    });
    m_bodiesForProxies.clear();
    return proxiesMoved;
}

void WorldImpl::SetType(BodyID bodyID, playrho::BodyType type)
{
    auto& body = GetBody(bodyID);
    if (body.GetType() == type)
    {
        return;
    }

    if (IsLocked())
    {
        throw WrongState("SetType: world is locked");
    }

    body.SetType(type);
    SetMassData(bodyID, ComputeMassData(bodyID));

    // Destroy the attached contacts.
    body.Erase([&](ContactID contactID) {
        Destroy(m_contacts, m_endContactListener, contactID, &body);
        return true;
    });

    if (type == BodyType::Static)
    {
#ifndef NDEBUG
        const auto xfm1 = GetTransform0(body.GetSweep());
        const auto xfm2 = body.GetTransformation();
        assert(xfm1 == xfm2);
#endif
        RegisterForProxies(bodyID);
    }
    else
    {
        body.SetAwake();
        const auto fixtures = body.GetFixtures();
        for_each(begin(fixtures), end(fixtures), [&](const auto& fixtureID) {
            InternalTouchProxies(m_proxies, m_fixtureBuffer[UnderlyingValue(fixtureID)]);
        });
    }
}

FixtureID WorldImpl::CreateFixture(BodyID bodyID, const Shape& shape,
                                   const FixtureConf& def, bool resetMassData)
{
    {
        const auto childCount = GetChildCount(shape);
        const auto minVertexRadius = GetMinVertexRadius();
        const auto maxVertexRadius = GetMaxVertexRadius();
        for (auto i = ChildCounter{0}; i < childCount; ++i)
        {
            const auto vr = GetVertexRadius(shape, i);
            if (!(vr >= minVertexRadius))
            {
                throw InvalidArgument("CreateFixture: vertex radius < min");
            }
            if (!(vr <= maxVertexRadius))
            {
                throw InvalidArgument("CreateFixture: vertex radius > max");
            }
        }
    }

    if (IsLocked())
    {
        throw WrongState("CreateFixture: world is locked");
    }

    if (size(m_fixtureBuffer) >= MaxFixtures)
    {
        throw LengthError("CreateFixture: operation would exceed MaxFixtures");
    }

    auto& body = GetBody(bodyID); // must be called before any mutating actions to validate bodyID!

    const auto fixtureID = static_cast<FixtureID>(
        static_cast<FixtureID::underlying_type>(m_fixtureBuffer.Allocate(bodyID, shape, def)));
    body.AddFixture(fixtureID);

    if (body.IsEnabled())
    {
        RegisterForProxies(fixtureID);
    }

    // Adjust mass properties if needed.
    if (m_fixtureBuffer[UnderlyingValue(fixtureID)].GetDensity() > 0_kgpm2)
    {
        body.SetMassDataDirty();
        if (resetMassData)
        {
            SetMassData(bodyID, ComputeMassData(bodyID));
        }
    }

    // Let the world know we have a new fixture. This will cause new contacts
    // to be created at the beginning of the next time step.
    SetNewFixtures();

    return fixtureID;
}

bool WorldImpl::Destroy(FixtureID id, bool resetMassData)
{
    if (IsLocked())
    {
        throw WrongState("Destroy: world is locked");
    }

#if 0
    /*
     * XXX: Should the destruction listener be called when the user requested that
     *   the fixture be destroyed or only when the fixture is destroyed indirectly?
     */
    if (m_fixtureDestructionListener)
    {
        m_fixtureDestructionListener(id);
    }
#endif

    auto& fixture = GetFixture(id);
    const auto bodyID = fixture.GetBody();
    auto& body = m_bodyBuffer[UnderlyingValue(bodyID)];

    // Destroy any contacts associated with the fixture.
    body.Erase([&](ContactID contactID) {
        auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
        const auto fixtureA = contact.GetFixtureA();
        const auto fixtureB = contact.GetFixtureB();
        if ((fixtureA == id) || (fixtureB == id))
        {
            Destroy(m_contacts, m_endContactListener, contactID, &body);
            return true;
        }
        return false;
    });

    EraseAll(m_fixturesForProxies, id);
    DestroyProxies(m_proxies, m_tree, fixture);

    if (!body.RemoveFixture(id))
    {
        // Fixture probably destroyed already.
        return false;
    }
    m_fixtureBuffer.Free(UnderlyingValue(id));

    body.SetMassDataDirty();
    if (resetMassData)
    {
        SetMassData(bodyID, ComputeMassData(bodyID));
    }
    return true;
}

void WorldImpl::DestroyFixtures(BodyID id)
{
    auto& body = GetBody(id);
    while (!empty(body.GetFixtures()))
    {
        const auto fixtureID = *body.GetFixtures().begin();
        Destroy(fixtureID, false);
    }
    SetMassData(id, ComputeMassData(id));
}

void WorldImpl::CreateProxies(FixtureID fixtureID, Fixture& fixture, const Transformation& xfm,
                              ProxyQueue& proxies, DynamicTree& tree, Length aabbExtension)
{
    assert(fixture.GetProxies().empty());

    const auto bodyID = fixture.GetBody();
    const auto shape = fixture.GetShape();

    // Reserve proxy space and create proxies in the broad-phase.
    const auto childCount = GetChildCount(shape);
    auto fixtureProxies = std::vector<FixtureProxy>{};
    fixtureProxies.reserve(childCount);
    for (auto childIndex = decltype(childCount){0}; childIndex < childCount; ++childIndex)
    {
        const auto dp = GetChild(shape, childIndex);
        const auto aabb = playrho::d2::ComputeAABB(dp, xfm);

        // Note: treeId from CreateLeaf can be higher than the number of fixture proxies.
        const auto fattenedAABB = GetFattenedAABB(aabb, aabbExtension);
        const auto treeId = tree.CreateLeaf(fattenedAABB, DynamicTree::LeafData{
            bodyID, fixtureID, childIndex});
        proxies.push_back(treeId);
        fixtureProxies.push_back(FixtureProxy{treeId});
    }

    fixture.SetProxies(fixtureProxies);
}

void WorldImpl::DestroyProxies(ProxyQueue& proxies, DynamicTree& tree, Fixture& fixture) noexcept
{
    const auto fixtureProxies = fixture.GetProxies();
    const auto childCount = size(fixtureProxies);
    if (childCount > 0)
    {
        // Destroy proxies in reverse order from what they were created in.
        for (auto i = childCount - 1; i < childCount; --i)
        {
            const auto treeId = fixtureProxies[i].treeId;
            EraseFirst(proxies, treeId);
            tree.DestroyLeaf(treeId);
        }
    }
    fixture.SetProxies(std::vector<FixtureProxy>{});
}

void WorldImpl::TouchProxies(const Fixture& fixture) noexcept
{
    InternalTouchProxies(m_proxies, fixture);
}

void WorldImpl::InternalTouchProxies(ProxyQueue& proxies, const Fixture& fixture) noexcept
{
    for (const auto& proxy: fixture.GetProxies()) {
        proxies.push_back(proxy.treeId);
    }
}

ContactCounter WorldImpl::Synchronize(const Body& body,
                                        const Transformation& xfm1, const Transformation& xfm2,
                                        Real multiplier, Length extension)
{
    assert(::playrho::IsValid(xfm1));
    assert(::playrho::IsValid(xfm2));

    auto updatedCount = ContactCounter{0};
    const auto displacement = multiplier * (xfm2.p - xfm1.p);
    const auto fixtures = body.GetFixtures();
    for_each(cbegin(fixtures), cend(fixtures), [&](const auto& fixtureID) {
        updatedCount += Synchronize(m_fixtureBuffer[UnderlyingValue(fixtureID)],
                                    xfm1, xfm2, displacement, extension);
    });
    return updatedCount;
}

ContactCounter WorldImpl::Synchronize(const Fixture& fixture,
                                        const Transformation& xfm1, const Transformation& xfm2,
                                        Length2 displacement, Length extension)
{
    assert(::playrho::IsValid(xfm1));
    assert(::playrho::IsValid(xfm2));
    
    auto updatedCount = ContactCounter{0};
    const auto shape = fixture.GetShape();
    const auto proxies = fixture.GetProxies();
    auto childIndex = ChildCounter{0};
    for (auto& proxy: proxies)
    {
        const auto treeId = proxy.treeId;
        
        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        const auto aabb = ComputeAABB(GetChild(shape, childIndex), xfm1, xfm2);
        if (!Contains(m_tree.GetAABB(treeId), aabb))
        {
            const auto newAabb = GetDisplacedAABB(GetFattenedAABB(aabb, extension),
                                                  displacement);
            m_tree.UpdateLeaf(treeId, newAabb);
            m_proxies.push_back(treeId);
            ++updatedCount;
        }
        ++childIndex;
    }
    return updatedCount;
}

void WorldImpl::Refilter(FixtureID id)
{
    const auto& fixture = GetFixture(id);
    const auto bodyID = fixture.GetBody();
    const auto& body = m_bodyBuffer[UnderlyingValue(bodyID)];

    // Flag associated contacts for filtering.
    const auto contacts = body.GetContacts();
    std::for_each(cbegin(contacts), cend(contacts), [&](KeyedContactPtr ci) {
        const auto contactID = std::get<ContactID>(ci);
        auto& contact = m_contactBuffer[UnderlyingValue(contactID)];
        const auto fixtureIdA = contact.GetFixtureA();
        const auto fixtureIdB = contact.GetFixtureB();
        if ((fixtureIdA == id) || (fixtureIdB == id))
        {
            contact.FlagForFiltering();
        }
    });

    TouchProxies(fixture);
}

void WorldImpl::SetFilterData(FixtureID id, Filter filter)
{
    GetFixture(id).SetFilterData(filter);
    Refilter(id);
}

void WorldImpl::SetEnabled(BodyID id, bool flag)
{
    auto& body = GetBody(id);
    if (body.IsEnabled() == flag)
    {
        return;
    }

    if (IsLocked())
    {
        throw WrongState("Body::SetEnabled: world is locked");
    }

    if (flag)
    {
        body.SetEnabledFlag();
    }
    else
    {
        body.UnsetEnabledFlag();
    }

    // Register for proxies so contacts created or destroyed the next time step.
    ForallFixtures(body, [this](const auto& fixtureID) {
        RegisterForProxies(fixtureID);
    });
}

MassData WorldImpl::ComputeMassData(BodyID id) const
{
    auto mass = 0_kg;
    auto I = RotInertia{0};
    auto center = Length2{};
    const auto& body = GetBody(id);
    for (const auto& f: body.GetFixtures())
    {
        const auto& fixture = m_fixtureBuffer[UnderlyingValue(f)];
        if (fixture.GetDensity() > 0_kgpm2)
        {
            const auto massData = GetMassData(fixture.GetShape());
            mass += Mass{massData.mass};
            center += Real{Mass{massData.mass} / Kilogram} * massData.center;
            I += RotInertia{massData.I};
        }
    }
    return MassData{center, mass, I};
}

void WorldImpl::SetMassData(BodyID id, const MassData& massData)
{
    if (IsLocked())
    {
        throw WrongState("SetMassData: world is locked");
    }

    auto& body = GetBody(id);
    if (!body.IsAccelerable())
    {
        body.SetInvMass(Real{0});
        body.SetInvRotI(Real{0});
        body.SetSweep(Sweep{Position{body.GetLocation(), body.GetAngle()}});
        body.UnsetMassDataDirty();
        return;
    }

    const auto mass = (massData.mass > 0_kg)? Mass{massData.mass}: 1_kg;
    body.SetInvMass(Real{1} / mass);

    if ((massData.I > RotInertia{0}) && (!body.IsFixedRotation()))
    {
        const auto lengthSquared = GetMagnitudeSquared(massData.center);
        // L^2 M QP^-2
        const auto I = RotInertia{massData.I} - RotInertia{(mass * lengthSquared) / SquareRadian};
        assert(I > RotInertia{0});
        body.SetInvRotI(Real{1} / I);
    }
    else
    {
        body.SetInvRotI(0);
    }

    // Move center of mass.
    const auto oldCenter = body.GetWorldCenter();
    body.SetSweep(Sweep{
        Position{Transform(massData.center, body.GetTransformation()), body.GetAngle()},
        massData.center
    });

    // Update center of mass velocity.
    const auto newCenter = body.GetWorldCenter();
    const auto deltaCenter = newCenter - oldCenter;
    auto newVelocity = body.GetVelocity();
    newVelocity.linear += GetRevPerpendicular(deltaCenter) * (newVelocity.angular / Radian);
    body.JustSetVelocity(newVelocity);
    body.UnsetMassDataDirty();
}

void WorldImpl::SetTransformation(BodyID id, Transformation xfm)
{
    assert(IsValid(xfm));
    if (IsLocked())
    {
        throw WrongState("SetTransformation: world is locked");
    }
    auto& body = GetBody(id);
    if (body.GetTransformation() != xfm)
    {
        FlagForUpdating(m_contactBuffer, body.GetContacts());
        body.SetTransformation(xfm);
        body.SetSweep(Sweep{
            Position{Transform(body.GetLocalCenter(), xfm), GetAngle(xfm.q)}, body.GetLocalCenter()
        });
        m_bodiesForProxies.push_back(id);
    }
}

FixtureCounter WorldImpl::GetFixtureCount(BodyID id) const
{
    return ::playrho::d2::GetFixtureCount(GetBody(id));
}

std::size_t WorldImpl::GetShapeCount() const noexcept
{
    auto shapes = std::set<const void*>();
    for_each(cbegin(m_bodies), cend(m_bodies), [&](const auto& b) {
        const auto fixtures = m_bodyBuffer[UnderlyingValue(b)].GetFixtures();
        for_each(cbegin(fixtures), cend(fixtures), [&](const auto& f) {
            const auto& fixture = m_fixtureBuffer[UnderlyingValue(f)];
            shapes.insert(GetData(fixture.GetShape()));
        });
    });
    return size(shapes);
}

void WorldImpl::Accept(JointID id, JointVisitor& visitor) const
{
    if (id == InvalidJointID)
    {
        throw std::out_of_range("invalid JointID");
    }
    const auto& joint = *static_cast<const Joint*>(UnderlyingValue(id));
    joint.Accept(visitor);
}

void WorldImpl::Accept(JointID id, JointVisitor& visitor)
{
    if (id == InvalidJointID)
    {
        throw std::out_of_range("invalid JointID");
    }
    auto& joint = *static_cast<Joint*>(UnderlyingValue(id));
    joint.Accept(visitor);
}

void WorldImpl::Update(ContactID contactID, const ContactUpdateConf& conf)
{
    auto& c = m_contactBuffer[UnderlyingValue(contactID)];
    auto& manifold = c.GetMutableManifold();
    const auto oldManifold = manifold;

    // Note: do not assume the fixture AABBs are overlapping or are valid.
    const auto oldTouching = c.IsTouching();
    auto newTouching = false;

    const auto bodyIdA = c.GetBodyA();
    const auto fixtureIdA = c.GetFixtureA();
    const auto indexA = c.GetChildIndexA();
    const auto bodyIdB = c.GetBodyB();
    const auto fixtureIdB = c.GetFixtureB();
    const auto indexB = c.GetChildIndexB();
    const auto& fixtureA = m_fixtureBuffer[UnderlyingValue(fixtureIdA)];
    const auto& fixtureB = m_fixtureBuffer[UnderlyingValue(fixtureIdB)];
    const auto shapeA = fixtureA.GetShape();
    const auto& bodyA = m_bodyBuffer[UnderlyingValue(bodyIdA)];
    const auto& bodyB = m_bodyBuffer[UnderlyingValue(bodyIdB)];
    const auto xfA = bodyA.GetTransformation();
    const auto shapeB = fixtureB.GetShape();
    const auto xfB = bodyB.GetTransformation();
    const auto childA = GetChild(shapeA, indexA);
    const auto childB = GetChild(shapeB, indexB);

    // NOTE: Ideally, the touching state returned by the TestOverlap function
    //   agrees 100% of the time with that returned from the CollideShapes function.
    //   This is not always the case however especially as the separation or overlap
    //   approaches zero.
#define OVERLAP_TOLERANCE (SquareMeter / Real(20))

    const auto sensor = fixtureA.IsSensor() || fixtureB.IsSensor();
    if (sensor)
    {
        const auto overlapping = TestOverlap(childA, xfA, childB, xfB, conf.distance);
        newTouching = (overlapping >= 0_m2);

#ifdef OVERLAP_TOLERANCE
#ifndef NDEBUG
        const auto tolerance = OVERLAP_TOLERANCE;
        const auto newManifold = CollideShapes(childA, xfA, childB, xfB, conf.manifold);
        assert(newTouching == (newManifold.GetPointCount() > 0) ||
               abs(overlapping) < tolerance);
#endif
#endif

        // Sensors don't generate manifolds.
        manifold = Manifold{};
    }
    else
    {
        auto newManifold = CollideShapes(childA, xfA, childB, xfB, conf.manifold);

        const auto old_point_count = oldManifold.GetPointCount();
        const auto new_point_count = newManifold.GetPointCount();

        newTouching = new_point_count > 0;

#ifdef OVERLAP_TOLERANCE
#ifndef NDEBUG
        const auto tolerance = OVERLAP_TOLERANCE;
        const auto overlapping = TestOverlap(childA, xfA, childB, xfB, conf.distance);
        assert(newTouching == (overlapping >= 0_m2) ||
               abs(overlapping) < tolerance);
#endif
#endif
        // Match old contact ids to new contact ids and copy the stored impulses to warm
        // start the solver. Note: missing any opportunities to warm start the solver
        // results in squishier stacking and less stable simulations.
        bool found[2] = {false, new_point_count < 2};
        for (auto i = decltype(new_point_count){0}; i < new_point_count; ++i)
        {
            const auto new_cf = newManifold.GetContactFeature(i);
            for (auto j = decltype(old_point_count){0}; j < old_point_count; ++j)
            {
                if (new_cf == oldManifold.GetContactFeature(j))
                {
                    found[i] = true;
                    newManifold.SetContactImpulses(i, oldManifold.GetContactImpulses(j));
                    break;
                }
            }
        }
        // If warm starting data wasn't found for a manifold point via contact feature
        // matching, it's better to just set the data to whatever old point is closest
        // to the new one.
        for (auto i = decltype(new_point_count){0}; i < new_point_count; ++i)
        {
            if (!found[i])
            {
                auto leastSquareDiff = std::numeric_limits<Area>::infinity();
                const auto newPt = newManifold.GetPoint(i);
                for (auto j = decltype(old_point_count){0}; j < old_point_count; ++j)
                {
                    const auto oldPt = oldManifold.GetPoint(j);
                    const auto squareDiff = GetMagnitudeSquared(oldPt.localPoint - newPt.localPoint);
                    if (leastSquareDiff > squareDiff)
                    {
                        leastSquareDiff = squareDiff;
                        newManifold.SetContactImpulses(i, oldManifold.GetContactImpulses(j));
                    }
                }
            }
        }

        // Ideally this method is **NEVER** called unless a dependency changed such
        // that the following assertion is **ALWAYS** valid.
        //assert(newManifold != oldManifold);

        manifold = newManifold;

#ifdef MAKE_CONTACT_PROCESSING_ORDER_DEPENDENT
        /*
         * The following code creates an ordering dependency in terms of update processing
         * over a container of contacts. It also puts this method into the situation of
         * modifying bodies which adds race potential in a multi-threaded mode of operation.
         * Lastly, without this code, the step-statistics show a world getting to sleep in
         * less TOI position iterations.
         */
        if (newTouching != oldTouching)
        {
            bodyA.SetAwake();
            bodyB.SetAwake();
        }
#endif
    }

    c.UnflagForUpdating();

    if (!oldTouching && newTouching)
    {
        c.SetTouching();
        if (m_beginContactListener)
        {
            m_beginContactListener(contactID);
        }
    }
    else if (oldTouching && !newTouching)
    {
        c.UnsetTouching();
        if (m_endContactListener)
        {
            m_endContactListener(contactID);
        }
    }

    if (!sensor && newTouching)
    {
        if (m_preSolveContactListener)
        {
            m_preSolveContactListener(contactID, oldManifold);
        }
    }
}

const Fixture& WorldImpl::GetFixture(FixtureID id) const
{
    return m_fixtureBuffer.at(UnderlyingValue(id));
}

Fixture& WorldImpl::GetFixture(FixtureID id)
{
    return m_fixtureBuffer.at(UnderlyingValue(id));
}

const Body& WorldImpl::GetBody(BodyID id) const
{
    return m_bodyBuffer.at(UnderlyingValue(id));
}

Body& WorldImpl::GetBody(BodyID id)
{
    return m_bodyBuffer.at(UnderlyingValue(id));
}

const Joint& WorldImpl::GetJoint(JointID id) const
{
    if (id == InvalidJointID)
    {
        throw std::out_of_range("invalid JointID");
    }
    return *static_cast<Joint*>(UnderlyingValue(id));
}

Joint& WorldImpl::GetJoint(JointID id)
{
    if (id == InvalidJointID)
    {
        throw std::out_of_range("invalid JointID");
    }
    return *static_cast<Joint*>(UnderlyingValue(id));
}

const Contact& WorldImpl::GetContact(ContactID id) const
{
    return m_contactBuffer.at(UnderlyingValue(id));
}

Contact& WorldImpl::GetContact(ContactID id)
{
    return m_contactBuffer.at(UnderlyingValue(id));
}

} // namespace d2
} // namespace playrho
