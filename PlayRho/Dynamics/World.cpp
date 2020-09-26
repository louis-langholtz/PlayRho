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

#include <PlayRho/Dynamics/World.hpp>

#include <PlayRho/Dynamics/WorldImpl.hpp> // for std::unique_ptr<WorldImpl> destruction
#include <PlayRho/Dynamics/WorldImplBody.hpp>
#include <PlayRho/Dynamics/WorldImplContact.hpp>
#include <PlayRho/Dynamics/WorldImplFixture.hpp>
#include <PlayRho/Dynamics/WorldImplJoint.hpp>
#include <PlayRho/Dynamics/WorldImplMisc.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>
#include <PlayRho/Dynamics/Island.hpp>
#include <PlayRho/Dynamics/MovementConf.hpp>
#include <PlayRho/Dynamics/ContactImpulsesList.hpp>

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/JointVisitor.hpp>
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

using std::for_each;
using std::remove;
using std::sort;
using std::transform;
using std::unique;

namespace playrho {
namespace d2 {

using playrho::size;

static_assert(std::is_default_constructible<World>::value, "World must be default constructible!");
static_assert(std::is_copy_constructible<World>::value, "World must be copy constructible!");
static_assert(std::is_copy_assignable<World>::value, "World must be copy assignable!");
static_assert(std::is_nothrow_destructible<World>::value, "World must be nothrow destructible!");

World::World(const WorldConf& def): m_impl{CreateWorldImpl(def)}
{
}

World::World(const World& other): m_impl{CreateWorldImpl(*other.m_impl)}
{
}

World& World::operator= (const World& other)
{
    *m_impl = *other.m_impl;
    return *this;
}

World::~World() noexcept {};

void World::Clear()
{
    ::playrho::d2::Clear(*m_impl);
}

void World::SetFixtureDestructionListener(const FixtureListener& listener) noexcept
{
    ::playrho::d2::SetFixtureDestructionListener(*m_impl, listener);
}

void World::SetJointDestructionListener(JointListener listener) noexcept
{
    ::playrho::d2::SetJointDestructionListener(*m_impl, listener);
}

void World::SetBeginContactListener(ContactListener listener) noexcept
{
    ::playrho::d2::SetBeginContactListener(*m_impl, listener);
}

void World::SetEndContactListener(ContactListener listener) noexcept
{
    ::playrho::d2::SetEndContactListener(*m_impl, listener);
}

void World::SetPreSolveContactListener(ManifoldContactListener listener) noexcept
{
    ::playrho::d2::SetPreSolveContactListener(*m_impl, listener);
}

void World::SetPostSolveContactListener(ImpulsesContactListener listener) noexcept
{
    ::playrho::d2::SetPostSolveContactListener(*m_impl, listener);
}

BodyID World::CreateBody(const BodyConf& def)
{
    return ::playrho::d2::CreateBody(*m_impl, def);
}

void World::Destroy(BodyID id)
{
    ::playrho::d2::Destroy(*m_impl, id);
}

JointID World::CreateJoint(const JointConf& def)
{
    return ::playrho::d2::CreateJoint(*m_impl, def);
}

void World::Destroy(JointID id)
{
    ::playrho::d2::Destroy(*m_impl, id);
}
    
StepStats World::Step(const StepConf& conf)
{
    return ::playrho::d2::Step(*m_impl, conf);
}

void World::ShiftOrigin(Length2 newOrigin)
{
    ::playrho::d2::ShiftOrigin(*m_impl, newOrigin);
}

SizedRange<World::Bodies::const_iterator> World::GetBodies() const noexcept
{
    return ::playrho::d2::GetBodies(*m_impl);
}

SizedRange<World::Bodies::const_iterator> World::GetBodiesForProxies() const noexcept
{
    return ::playrho::d2::GetBodiesForProxies(*m_impl);
}

SizedRange<World::Fixtures::const_iterator> World::GetFixturesForProxies() const noexcept
{
    return ::playrho::d2::GetFixturesForProxies(*m_impl);
}

SizedRange<World::Joints::const_iterator> World::GetJoints() const noexcept
{
    return ::playrho::d2::GetJoints(*m_impl);
}

SizedRange<World::BodyJoints::const_iterator> World::GetJoints(BodyID id) const
{
    return ::playrho::d2::GetJoints(*m_impl, id);
}

bool World::IsSpeedable(BodyID id) const
{
    return ::playrho::d2::IsSpeedable(*m_impl, id);
}

bool World::IsAccelerable(BodyID id) const
{
    return ::playrho::d2::IsAccelerable(*m_impl, id);
}

bool World::IsImpenetrable(BodyID id) const
{
    return ::playrho::d2::IsImpenetrable(*m_impl, id);
}

SizedRange<World::Contacts::const_iterator> World::GetContacts(BodyID id) const
{
    return ::playrho::d2::GetContacts(*m_impl, id);
}

void* World::GetUserData(BodyID id) const
{
    return ::playrho::d2::GetUserData(*m_impl, id);
}

SizedRange<World::Contacts::const_iterator> World::GetContacts() const noexcept
{
    return ::playrho::d2::GetContacts(*m_impl);
}

bool World::IsLocked() const noexcept
{
    return m_impl && ::playrho::d2::IsLocked(*m_impl);
}

bool World::IsStepComplete() const noexcept
{
    return ::playrho::d2::IsStepComplete(*m_impl);
}

bool World::GetSubStepping() const noexcept
{
    return ::playrho::d2::GetSubStepping(*m_impl);
}

void World::SetSubStepping(bool flag) noexcept
{
    ::playrho::d2::SetSubStepping(*m_impl, flag);
}

Length World::GetMinVertexRadius() const noexcept
{
    return ::playrho::d2::GetMinVertexRadius(*m_impl);
}

Length World::GetMaxVertexRadius() const noexcept
{
    return ::playrho::d2::GetMaxVertexRadius(*m_impl);
}

Frequency World::GetInvDeltaTime() const noexcept
{
    return ::playrho::d2::GetInvDeltaTime(*m_impl);
}

const DynamicTree& World::GetTree() const noexcept
{
    return ::playrho::d2::GetTree(*m_impl);
}

void World::Refilter(FixtureID id)
{
    ::playrho::d2::Refilter(*m_impl, id);
}

void World::SetFilterData(FixtureID id, const Filter& filter)
{
    ::playrho::d2::SetFilterData(*m_impl, id, filter);
}

void World::SetType(BodyID id, BodyType type)
{
    ::playrho::d2::SetType(*m_impl, id, type);
}

FixtureID World::CreateFixture(BodyID body, const Shape& shape, const FixtureConf& def,
                              bool resetMassData)
{
    return ::playrho::d2::CreateFixture(*m_impl, body, shape, def, resetMassData);
}

bool World::Destroy(FixtureID id, bool resetMassData)
{
    return ::playrho::d2::Destroy(*m_impl, id, resetMassData);
}

void World::DestroyFixtures(BodyID id)
{
    ::playrho::d2::DestroyFixtures(*m_impl, id);
}

bool World::IsEnabled(BodyID id) const
{
    return ::playrho::d2::IsEnabled(*m_impl, id);
}

void World::SetEnabled(BodyID id, bool flag)
{
    ::playrho::d2::SetEnabled(*m_impl, id, flag);
}

MassData World::ComputeMassData(BodyID id) const
{
    return ::playrho::d2::ComputeMassData(*m_impl, id);
}

void World::SetMassData(BodyID id, const MassData& massData)
{
    ::playrho::d2::SetMassData(*m_impl, id, massData);
}

SizedRange<World::Fixtures::const_iterator> World::GetFixtures(BodyID id) const
{
    return ::playrho::d2::GetFixtures(*m_impl, id);
}

std::size_t World::GetShapeCount() const noexcept
{
    return ::playrho::d2::GetShapeCount(*m_impl);
}

FixtureCounter World::GetFixtureCount(BodyID id) const
{
    return ::playrho::d2::GetFixtureCount(*m_impl, id);
}

BodyConf World::GetBodyConf(BodyID id) const
{
    return ::playrho::d2::GetBodyConf(*m_impl, id);
}

BodyID World::GetBody(FixtureID id) const
{
    return ::playrho::d2::GetBody(*m_impl, id);
}

void* World::GetUserData(FixtureID id) const
{
    return ::playrho::d2::GetUserData(*m_impl, id);
}

Shape World::GetShape(FixtureID id) const
{
    return ::playrho::d2::GetShape(*m_impl, id);
}

void World::SetSensor(FixtureID id, bool value)
{
    ::playrho::d2::SetSensor(*m_impl, id, value);
}

bool World::IsSensor(FixtureID id) const
{
    return ::playrho::d2::IsSensor(*m_impl, id);
}

AreaDensity World::GetDensity(FixtureID id) const
{
    return ::playrho::d2::GetDensity(*m_impl, id);
}

const World::FixtureProxies& World::GetProxies(FixtureID id) const
{
    return ::playrho::d2::GetProxies(*m_impl, id);
}

Angle World::GetAngle(BodyID id) const
{
    return ::playrho::d2::GetAngle(*m_impl, id);
}

Transformation World::GetTransformation(BodyID id) const
{
    return ::playrho::d2::GetTransformation(*m_impl, id);
}

void World::SetTransformation(BodyID id, Transformation xfm)
{
    return ::playrho::d2::SetTransformation(*m_impl, id, xfm);
}

Length2 World::GetLocalCenter(BodyID id) const
{
    return ::playrho::d2::GetLocalCenter(*m_impl, id);
}

Length2 World::GetWorldCenter(BodyID id) const
{
    return ::playrho::d2::GetWorldCenter(*m_impl, id);
}

Velocity World::GetVelocity(BodyID id) const
{
    return ::playrho::d2::GetVelocity(*m_impl, id);
}

void World::SetVelocity(BodyID id, const Velocity& value)
{
    ::playrho::d2::SetVelocity(*m_impl, id, value);
}

void World::UnsetAwake(BodyID id)
{
    ::playrho::d2::UnsetAwake(*m_impl, id);
}

void World::SetAwake(BodyID id)
{
    ::playrho::d2::SetAwake(*m_impl, id);
}

bool World::IsMotorEnabled(JointID id) const
{
    return ::playrho::d2::IsMotorEnabled(*m_impl, id);
}

void World::EnableMotor(JointID id, bool value)
{
    ::playrho::d2::EnableMotor(*m_impl, id, value);
}

void World::SetAwake(JointID id)
{
    ::playrho::d2::SetAwake(*m_impl, id);
}

bool World::IsAwake(ContactID id) const
{
    return ::playrho::d2::IsAwake(*m_impl, id);
}

void World::SetAwake(ContactID id)
{
    ::playrho::d2::SetAwake(*m_impl, id);
}

bool World::IsMassDataDirty(BodyID id) const
{
    return ::playrho::d2::IsMassDataDirty(*m_impl, id);
}

bool World::IsFixedRotation(BodyID id) const
{
    return ::playrho::d2::IsFixedRotation(*m_impl, id);
}

void World::SetFixedRotation(BodyID id, bool value)
{
    ::playrho::d2::SetFixedRotation(*m_impl, id, value);
}

BodyType World::GetType(BodyID id) const
{
    return ::playrho::d2::GetType(*m_impl, id);
}

JointType World::GetType(JointID id) const
{
    return ::playrho::d2::GetType(*m_impl, id);
}

bool World::IsAwake(BodyID id) const
{
    return ::playrho::d2::IsAwake(*m_impl, id);
}

LinearAcceleration2 World::GetLinearAcceleration(BodyID id) const
{
    return ::playrho::d2::GetLinearAcceleration(*m_impl, id);
}

AngularAcceleration World::GetAngularAcceleration(BodyID id) const
{
    return ::playrho::d2::GetAngularAcceleration(*m_impl, id);
}

void World::SetAcceleration(BodyID id, LinearAcceleration2 linear, AngularAcceleration angular)
{
    ::playrho::d2::SetAcceleration(*m_impl, id, linear, angular);
}

InvMass World::GetInvMass(BodyID id) const
{
    return ::playrho::d2::GetInvMass(*m_impl, id);
}

InvRotInertia World::GetInvRotInertia(BodyID id) const
{
    return ::playrho::d2::GetInvRotInertia(*m_impl, id);
}

bool World::GetCollideConnected(JointID id) const
{
    return ::playrho::d2::GetCollideConnected(*m_impl, id);
}

void* World::GetUserData(JointID id) const
{
    return ::playrho::d2::GetUserData(*m_impl, id);
}

BodyID World::GetBodyA(JointID id) const
{
    return ::playrho::d2::GetBodyA(*m_impl, id);
}

BodyID World::GetBodyB(JointID id) const
{
    return ::playrho::d2::GetBodyB(*m_impl, id);
}

Length2 World::GetLocalAnchorA(JointID id) const
{
    return ::playrho::d2::GetLocalAnchorA(*m_impl, id);
}

Length2 World::GetLocalAnchorB(JointID id) const
{
    return ::playrho::d2::GetLocalAnchorB(*m_impl, id);
}

Momentum2 World::GetLinearReaction(JointID id) const
{
    return ::playrho::d2::GetLinearReaction(*m_impl, id);
}

AngularMomentum World::GetAngularReaction(JointID id) const
{
    return ::playrho::d2::GetAngularReaction(*m_impl, id);
}

Angle World::GetReferenceAngle(JointID id) const
{
    return ::playrho::d2::GetReferenceAngle(*m_impl, id);
}

UnitVec World::GetLocalAxisA(JointID id) const
{
    return ::playrho::d2::GetLocalAxisA(*m_impl, id);
}

AngularVelocity World::GetMotorSpeed(JointID id) const
{
    return ::playrho::d2::GetMotorSpeed(*m_impl, id);
}

void World::SetMotorSpeed(JointID id, AngularVelocity value)
{
    ::playrho::d2::SetMotorSpeed(*m_impl, id, value);
}

Torque World::GetMaxMotorTorque(JointID id) const
{
    return ::playrho::d2::GetMaxMotorTorque(*m_impl, id);
}

void World::SetMaxMotorTorque(JointID id, Torque value)
{
    ::playrho::d2::SetMaxMotorTorque(*m_impl, id, value);
}

AngularMomentum World::GetAngularMotorImpulse(JointID id) const
{
    return ::playrho::d2::GetAngularMotorImpulse(*m_impl, id);
}

bool World::IsTouching(ContactID id) const
{
    return ::playrho::d2::IsTouching(*m_impl, id);
}

bool World::NeedsFiltering(ContactID id) const
{
    return ::playrho::d2::NeedsFiltering(*m_impl, id);
}

bool World::NeedsUpdating(ContactID id) const
{
    return ::playrho::d2::NeedsUpdating(*m_impl, id);
}

FixtureID World::GetFixtureA(ContactID id) const
{
    return ::playrho::d2::GetFixtureA(*m_impl, id);
}

FixtureID World::GetFixtureB(ContactID id) const
{
    return ::playrho::d2::GetFixtureB(*m_impl, id);
}

Real World::GetDefaultFriction(ContactID id) const
{
    return ::playrho::d2::GetDefaultFriction(*m_impl, id);
}

Real World::GetDefaultRestitution(ContactID id) const
{
    return ::playrho::d2::GetDefaultRestitution(*m_impl, id);
}

Real World::GetFriction(ContactID id) const
{
    return ::playrho::d2::GetFriction(*m_impl, id);
}

Real World::GetRestitution(ContactID id) const
{
    return ::playrho::d2::GetRestitution(*m_impl, id);
}

void World::SetFriction(ContactID id, Real value)
{
    ::playrho::d2::SetFriction(*m_impl, id, value);
}

void World::SetRestitution(ContactID id, Real value)
{
    ::playrho::d2::SetRestitution(*m_impl, id, value);
}

const Manifold& World::GetManifold(ContactID id) const
{
    return ::playrho::d2::GetManifold(*m_impl, id);
}

// Free functions...

StepStats Step(World& world, Time delta, TimestepIters velocityIterations,
               TimestepIters positionIterations)
{
    StepConf conf;
    conf.SetTime(delta);
    conf.regVelocityIterations = velocityIterations;
    conf.regPositionIterations = positionIterations;
    conf.toiVelocityIterations = velocityIterations;
    if (positionIterations == 0)
    {
        conf.toiPositionIterations = 0;
    }
    conf.dtRatio = delta * world.GetInvDeltaTime();
    return world.Step(conf);
}

ContactCounter GetTouchingCount(const World& world) noexcept
{
    const auto contacts = world.GetContacts();
    return static_cast<ContactCounter>(count_if(cbegin(contacts), cend(contacts),
                                                [&](const auto &c) {
        return world.IsTouching(std::get<ContactID>(c));
    }));
}

FixtureCounter GetFixtureCount(const World& world) noexcept
{
    auto sum = FixtureCounter{0};
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world,&sum](const auto &b) {
        sum += GetFixtureCount(world, b);
    });
    return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
    return world.GetShapeCount();
}

BodyCounter GetAwakeCount(const World& world) noexcept
{
    const auto bodies = world.GetBodies();
    return static_cast<BodyCounter>(count_if(cbegin(bodies), cend(bodies),
                                             [&](const auto &b) {
                                                 return IsAwake(world, b); }));
}
    
BodyCounter Awaken(World& world) noexcept
{
    // Can't use count_if since body gets modified.
    auto awoken = BodyCounter{0};
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world,&awoken](const auto &b) {
        if (::playrho::d2::Awaken(world, b))
        {
            ++awoken;
        }
    });
    return awoken;
}

void SetAccelerations(World& world, Acceleration acceleration) noexcept
{
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world, acceleration](const auto &b) {
        SetAcceleration(world, b, acceleration);
    });
}

void SetAccelerations(World& world, LinearAcceleration2 acceleration) noexcept
{
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world, acceleration](const auto &b) {
        SetAcceleration(world, b, acceleration);
    });
}

BodyID FindClosestBody(const World& world, Length2 location) noexcept
{
    const auto bodies = world.GetBodies();
    auto found = InvalidBodyID;
    auto minLengthSquared = std::numeric_limits<Area>::infinity();
    for (const auto& body: bodies)
    {
        const auto bodyLoc = GetLocation(world, body);
        const auto lengthSquared = GetMagnitudeSquared(bodyLoc - location);
        if (minLengthSquared > lengthSquared)
        {
            minLengthSquared = lengthSquared;
            found = body;
        }
    }
    return found;
}

void SetLocation(World& world, BodyID body, Length2 value)
{
    SetTransform(world, body, value, GetAngle(world, body));
}

void SetAngle(World& world, BodyID body, Angle value)
{
    SetTransform(world, body, GetLocation(world, body), value);
}

void RotateAboutWorldPoint(World& world, BodyID body, Angle amount, Length2 worldPoint)
{
    const auto xfm = GetTransformation(world, body);
    const auto p = xfm.p - worldPoint;
    const auto c = cos(amount);
    const auto s = sin(amount);
    const auto x = GetX(p) * c - GetY(p) * s;
    const auto y = GetX(p) * s + GetY(p) * c;
    const auto pos = Length2{x, y} + worldPoint;
    const auto angle = GetAngle(xfm.q) + amount;
    SetTransform(world, body, pos, angle);
}

void RotateAboutLocalPoint(World& world, BodyID body, Angle amount, Length2 localPoint)
{
    RotateAboutWorldPoint(world, body, amount, GetWorldPoint(world, body, localPoint));
}

Acceleration CalcGravitationalAcceleration(const World& world, BodyID body)
{
    const auto m1 = GetMass(world, body);
    if (m1 != 0_kg)
    {
        auto sumForce = Force2{};
        const auto loc1 = GetLocation(world, body);
        for (const auto& b2: world.GetBodies())
        {
            if (b2 == body)
            {
                continue;
            }
            const auto m2 = GetMass(world, b2);
            const auto delta = GetLocation(world, b2) - loc1;
            const auto dir = GetUnitVector(delta);
            const auto rr = GetMagnitudeSquared(delta);

            // Uses Newton's law of universal gravitation: F = G * m1 * m2 / rr.
            // See: https://en.wikipedia.org/wiki/Newton%27s_law_of_universal_gravitation
            // Note that BigG is typically very small numerically compared to either mass
            // or the square of the radius between the masses. That's important to recognize
            // in order to avoid operational underflows or overflows especially when
            // playrho::Real has less exponential range like when it's defined to be float
            // instead of double. The operational ordering is deliberately established here
            // to help with this.
            const auto orderedMass = std::minmax(m1, m2);
            const auto f = (BigG * std::get<0>(orderedMass)) * (std::get<1>(orderedMass) / rr);
            sumForce += f * dir;
        }
        // F = m a... i.e.  a = F / m.
        return Acceleration{sumForce / m1, 0 * RadianPerSquareSecond};
    }
    return Acceleration{};
}

BodyCounter GetWorldIndex(const World& world, BodyID id) noexcept
{
    const auto elems = world.GetBodies();
    const auto it = std::find(cbegin(elems), cend(elems), id);
    if (it != cend(elems))
    {
        return static_cast<BodyCounter>(std::distance(cbegin(elems), it));
    }
    return BodyCounter(-1);
}

ChildCounter GetProxyCount(const World& world, FixtureID id)
{
    return static_cast<ChildCounter>(size(GetProxies(world, id)));
}

const FixtureProxy& GetProxy(const World& world, FixtureID id, ChildCounter child)
{
    return GetProxies(world, id).at(child);
}

JointCounter GetWorldIndex(const World& world, JointID id) noexcept
{
    const auto elems = world.GetJoints();
    const auto it = std::find(cbegin(elems), cend(elems), id);
    if (it != cend(elems))
    {
        return static_cast<JointCounter>(std::distance(cbegin(elems), it));
    }
    return JointCounter(-1);
}

bool ShouldCollide(const World& world, BodyID lhs, BodyID rhs)
{
    // At least one body should be accelerable/dynamic.
    if (!IsAccelerable(GetType(world, lhs)) && !IsAccelerable(GetType(world, rhs)))
    {
        return false;
    }

    // Does a joint prevent collision?
    const auto joints = GetJoints(world, lhs);
    const auto it = std::find_if(cbegin(joints), cend(joints), [&](Body::KeyedJointPtr ji) {
        return (std::get<0>(ji) == rhs) && !world.GetCollideConnected(std::get<JointID>(ji));
    });
    return it == end(joints);
}

bool TestPoint(const World& world, FixtureID id, Length2 p)
{
    return TestPoint(GetShape(world, id), InverseTransform(p, GetTransformation(world, id)));
}

Force2 GetCentripetalForce(const World& world, BodyID id, Length2 axis)
{
    // For background on centripetal force, see:
    //   https://en.wikipedia.org/wiki/Centripetal_force

    // Force is M L T^-2.
    const auto velocity = GetLinearVelocity(world, id);
    const auto magnitudeOfVelocity = GetMagnitude(GetVec2(velocity)) * MeterPerSecond;
    const auto location = GetLocation(world, id);
    const auto mass = GetMass(world, id);
    const auto delta = axis - location;
    const auto invRadius = Real{1} / GetMagnitude(delta);
    const auto dir = delta * invRadius;
    return Force2{dir * mass * Square(magnitudeOfVelocity) * invRadius};
}

} // namespace d2
} // namespace playrho
