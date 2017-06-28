/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>

#include <iterator>
#include <utility>

using namespace box2d;

using std::begin;
using std::end;

Body::FlagsType Body::GetFlags(const BodyDef& bd) noexcept
{
    // @invariant Only bodies that allow sleeping, can be put to sleep.
    // @invariant Only "speedable" bodies can be awake.
    // @invariant Only "speedable" bodies can have non-zero velocities.
    // @invariant Only "accelerable" bodies can have non-zero accelerations.
    // @invariant Only "accelerable" bodies can have non-zero "under-active" times.

    auto flags = GetFlags(bd.type);
    if (bd.bullet)
    {
        flags |= e_impenetrableFlag;
    }
    if (bd.fixedRotation)
    {
        flags |= e_fixedRotationFlag;
    }
    if (bd.allowSleep)
    {
        flags |= e_autoSleepFlag;
    }
    if (bd.awake)
    {
        if (flags & e_velocityFlag)
        {
            flags |= e_awakeFlag;
        }
    }
    else
    {
        if (!bd.allowSleep && (flags & e_velocityFlag))
        {
            flags |= e_awakeFlag;
        }
    }
    if (bd.enabled)
    {
        flags |= e_enabledFlag;
    }
    return flags;
}

Body::Body(const BodyDef& bd, World* world):
    m_flags{GetFlags(bd)},
    m_xf{bd.position, UnitVec2{bd.angle}},
    m_world{world},
    m_sweep{Position{bd.position, bd.angle}},
    m_invMass{(bd.type == BodyType::Dynamic)? InvMass{RealNum{1} / Kilogram}: InvMass{0}},
    m_linearDamping{bd.linearDamping},
    m_angularDamping{bd.angularDamping},
    m_userData{bd.userData}
{
    assert(::IsValid(bd.position));
    assert(::IsValid(bd.linearVelocity.x) && ::IsValid(bd.linearVelocity.y));
    assert(::IsValid(bd.angle));
    assert(::IsValid(bd.angularVelocity));

    SetVelocity(Velocity{bd.linearVelocity, bd.angularVelocity});
    SetAcceleration(bd.linearAcceleration, bd.angularAcceleration);
    SetUnderActiveTime(bd.underActiveTime);
}

Body::~Body()
{
    assert(m_joints.empty());
    assert(m_contacts.empty());
    assert(m_fixtures.empty());
}

void Body::SetType(BodyType type)
{
    m_world->SetType(*this, type);
}

Fixture* Body::CreateFixture(std::shared_ptr<const Shape> shape, const FixtureDef& def, bool resetMassData)
{
    return m_world->CreateFixture(*this, shape, def, resetMassData);
}

bool Body::DestroyFixture(Fixture* fixture, bool resetMassData)
{
    if (fixture->GetBody() != this)
    {
        return false;
    }
    return m_world->DestroyFixture(fixture, resetMassData);
}

void Body::DestroyFixtures()
{
    while (!m_fixtures.empty())
    {
        auto& fixture = m_fixtures.front();
        DestroyFixture(&fixture, false);
    }
    ResetMassData();
}

void Body::ResetMassData()
{
    // Compute mass data from shapes. Each shape has its own density.

    // Non-dynamic bodies (Static and kinematic ones) have zero mass.
    if (!IsAccelerable())
    {
        m_invMass = 0;
        m_invRotI = 0;
        m_sweep = Sweep{Position{GetLocation(), GetAngle()}};
        UnsetMassDataDirty();
        return;
    }

    const auto massData = ComputeMassData(*this);

    // Force all dynamic bodies to have a positive mass.
    const auto mass = (massData.mass > Mass{0})? Mass{massData.mass}: Kilogram;
    m_invMass = RealNum{1} / mass;

    // Compute center of mass.
    const auto localCenter = massData.center * RealNum{m_invMass * Kilogram};

    if ((massData.I > RotInertia{0}) && (!IsFixedRotation()))
    {
        // Center the inertia about the center of mass.
        const auto lengthSquared = GetLengthSquared(localCenter);
        const auto I = RotInertia{massData.I} - RotInertia{(mass * lengthSquared / SquareRadian)};
        //assert((massData.I - mass * lengthSquared) > 0);
        m_invRotI = RealNum{1} / I;
    }
    else
    {
        m_invRotI = 0;
    }

    // Move center of mass.
    const auto oldCenter = GetWorldCenter();
    m_sweep = Sweep{Position{Transform(localCenter, GetTransformation()), GetAngle()}, localCenter};
    const auto newCenter = GetWorldCenter();

    // Update center of mass velocity.
    const auto deltaCenter = newCenter - oldCenter;
    m_velocity.linear += GetRevPerpendicular(deltaCenter) * m_velocity.angular / Radian;

    UnsetMassDataDirty();
}

void Body::SetMassData(const MassData& massData)
{
    if (m_world->IsLocked())
    {
        throw World::LockedError();
    }

    if (!IsAccelerable())
    {
        return;
    }

    const auto mass = (massData.mass > Mass{0})? Mass{massData.mass}: Kilogram;
    m_invMass = RealNum{1} / mass;

    if ((massData.I > RotInertia{0}) && (!IsFixedRotation()))
    {
        const auto lengthSquared = GetLengthSquared(massData.center);
        // L^2 M QP^-2
        const auto I = RotInertia{massData.I} - RotInertia{(mass * lengthSquared) / SquareRadian};
        assert(I > RotInertia{0});
        m_invRotI = RealNum{1} / I;
    }
    else
    {
        m_invRotI = 0;
    }

    // Move center of mass.
    const auto oldCenter = GetWorldCenter();
    m_sweep = Sweep{
        Position{Transform(massData.center, GetTransformation()), GetAngle()},
        massData.center
    };

    // Update center of mass velocity.
    const auto newCenter = GetWorldCenter();
    const auto deltaCenter = newCenter - oldCenter;
    m_velocity.linear += GetRevPerpendicular(deltaCenter) * m_velocity.angular / Radian;

    UnsetMassDataDirty();
}

void Body::SetVelocity(const Velocity& velocity) noexcept
{
    if ((velocity.linear != Vec2_zero * MeterPerSecond) || (velocity.angular != AngularVelocity{0}))
    {
        if (!IsSpeedable())
        {
            return;
        }
        SetAwakeFlag();
        ResetUnderActiveTime();
    }
    m_velocity = velocity;
}

void Body::SetAcceleration(const LinearAcceleration2D linear, const AngularAcceleration angular) noexcept
{
    assert(::IsValid(linear.x) && ::IsValid(linear.y));
    assert(::IsValid(angular));

    if ((linear != Vec2_zero * MeterPerSquareSecond) || (angular != AngularAcceleration{0}))
    {
        if (!IsAccelerable())
        {
            return;
        }
        SetAwakeFlag();
        ResetUnderActiveTime();
    }
    m_linearAcceleration = linear;
    m_angularAcceleration = angular;
}

void Body::SetTransformation(const Transformation value) noexcept
{
    if (m_xf != value)
    {
        m_xf = value;
        std::for_each(begin(m_contacts), end(m_contacts), [&](KeyedContactPtr ci) {
            ci.second->FlagForUpdating();
        });
    }
}

void Body::SetTransform(const Length2D position, Angle angle)
{
    assert(::IsValid(position));
    assert(::IsValid(angle));

    if (GetWorld()->IsLocked())
    {
        throw World::LockedError();
    }

    const auto xfm = Transformation{position, UnitVec2{angle}};
    SetTransformation(xfm);

    const auto sweep = Sweep{Position{Transform(GetLocalCenter(), xfm), angle}, GetLocalCenter()};
    m_sweep = sweep;

    GetWorld()->RegisterForProxies(this);
}

void Body::SetEnabled(bool flag)
{
    if (IsEnabled() == flag)
    {
        return;
    }

    if (m_world->IsLocked())
    {
        throw World::LockedError();
    }

    if (flag)
    {
        SetEnabledFlag();
    }
    else
    {
        UnsetEnabledFlag();
    }

    // Register for proxies so contacts created or destroyed the next time step.
    std::for_each(begin(m_fixtures), end(m_fixtures), [&](Fixture &f) {
        m_world->RegisterForProxies(&f);
    });
}

void Body::SetFixedRotation(bool flag)
{
    const auto status = IsFixedRotation();
    if (status == flag)
    {
        return;
    }

    if (flag)
    {
        m_flags |= e_fixedRotationFlag;
    }
    else
    {
        m_flags &= ~e_fixedRotationFlag;
    }

    m_velocity.angular = AngularVelocity{0};

    ResetMassData();
}

bool Body::Insert(Joint* j)
{
    const auto bodyA = j->GetBodyA();
    const auto bodyB = j->GetBodyB();
    
    const auto other = (this == bodyA)? bodyB: (this == bodyB)? bodyA: nullptr;
    if (!other)
    {
        return false;
    }
    m_joints.push_back(std::make_pair(other, j));
    return true;
}

bool Body::Insert(Contact* contact)
{
#ifndef NDEBUG
    // Prevent the same contact from being added more than once...
    const auto it = std::find_if(begin(m_contacts), end(m_contacts), [&](KeyedContactPtr ci) {
        return ci.second == contact;
    });
    assert(it == end(m_contacts));
    if (it != end(m_contacts))
    {
        return false;
    }
#endif

    m_contacts.emplace_back(GetContactKey(*contact), contact);
    return true;
}

bool Body::Erase(const Joint* const joint)
{
    const auto it = std::find_if(begin(m_joints), end(m_joints), [&](KeyedJointPtr ji) {
        return ji.second == joint;
    });
    if (it != end(m_joints))
    {
        m_joints.erase(it);
        return true;
    }
    return false;
}

bool Body::Erase(const Contact* const contact)
{
    const auto it = std::find_if(begin(m_contacts), end(m_contacts), [&](KeyedContactPtr ci) {
        return ci.second == contact;
    });
    if (it != end(m_contacts))
    {
        m_contacts.erase(it);
        return true;
    }
    return false;
}

void Body::ClearContacts()
{
    m_contacts.clear();
}

void Body::ClearJoints()
{
    m_joints.clear();
}

// Free functions...

bool box2d::ShouldCollide(const Body& lhs, const Body& rhs) noexcept
{
    // At least one body should be accelerable/dynamic.
    if (!lhs.IsAccelerable() && !rhs.IsAccelerable())
    {
        return false;
    }

    // Does a joint prevent collision?
    const auto joints = lhs.GetJoints();
    const auto it = std::find_if(begin(joints), end(joints), [&](Body::KeyedJointPtr ji) {
        return (ji.first == &rhs) && !(ji.second->GetCollideConnected());
    });
    return it == end(joints);
}

BodyCounter box2d::GetWorldIndex(const Body* body)
{
    if (body)
    {
        const auto world = body->GetWorld();
        const auto bodies = world->GetBodies();
        auto i = BodyCounter{0};
        const auto it = std::find_if(begin(bodies), end(bodies), [&](const Body &b) {
            return &b == body || (++i, false);
        });
        if (it != end(bodies))
        {
            return i;
        }
    }
    return BodyCounter(-1);
}

Velocity box2d::GetVelocity(const Body& body, const Time h) noexcept
{
    // Integrate velocity and apply damping.
    auto velocity = body.GetVelocity();
    if (body.IsAccelerable())
    {
        // Integrate velocities.
        velocity.linear += h * body.GetLinearAcceleration();
        velocity.angular += AngularVelocity{h * body.GetAngularAcceleration()};

        // Apply damping.
        // Ordinary differential equation: dv/dt + c * v = 0
        //                       Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation (see https://en.wikipedia.org/wiki/Pad%C3%A9_approximant ):
        // v2 = v1 * 1 / (1 + c * dt)
        velocity.linear  /= RealNum{1 + h * body.GetLinearDamping()};
        velocity.angular /= RealNum{1 + h * body.GetAngularDamping()};
    }
    return velocity;
}

std::size_t box2d::GetFixtureCount(const Body& body)
{
    const auto& fixtures = body.GetFixtures();
    return fixtures.size();
}

void box2d::RotateAboutWorldPoint(Body& body, Angle amount, Length2D worldPoint)
{
    const auto xfm = body.GetTransformation();
    const auto p = xfm.p - worldPoint;
    const auto c = RealNum{std::cos(amount / Radian)};
    const auto s = RealNum{std::sin(amount / Radian)};
    const auto x = p.x * c - p.y * s;
    const auto y = p.x * s + p.y * c;
    const auto pos = Length2D{x, y} + worldPoint;
    const auto angle = GetAngle(xfm.q) + amount;
    body.SetTransform(pos, angle);
}

void box2d::RotateAboutLocalPoint(Body& body, Angle amount, Length2D localPoint)
{
    RotateAboutWorldPoint(body, amount, GetWorldPoint(body, localPoint));
}

Force2D box2d::GetCentripetalForce(const Body& body, const Length2D axis)
{
    // For background on centripetal force, see:
    //   https://en.wikipedia.org/wiki/Centripetal_force

    // Force is M L T^-2.
    const auto velocity = GetLinearVelocity(body);
    const auto magnitude = GetLength(StripUnits(velocity)) * MeterPerSecond;
    const auto location = body.GetLocation();
    const auto mass = GetMass(body);
    const auto delta = Length2D{axis - location};
    const auto radius = GetLength(delta);
    const auto dir = delta / radius;
    return Force2D{Vec2{dir.x, dir.y} * mass * Square(magnitude) / radius};
}
