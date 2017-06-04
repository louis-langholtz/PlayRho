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
    assert(::box2d::IsValid(bd.position));
    assert(::box2d::IsValid(bd.linearVelocity.x) && ::box2d::IsValid(bd.linearVelocity.y));
    assert(::box2d::IsValid(bd.angle));
    assert(::box2d::IsValid(bd.angularVelocity));

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
        DestroyFixture(&fixture);
    }
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
    assert(!m_world->IsLocked());
    if (m_world->IsLocked())
    {
        return;
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
    const auto newCenter = GetWorldCenter();

    // Update center of mass velocity.
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
    assert(::box2d::IsValid(linear.x) && ::box2d::IsValid(linear.y));
    assert(::box2d::IsValid(angular));

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
        for (auto&& ci: GetContacts())
        {
            const auto contact = GetContactPtr(ci);
            contact->FlagForUpdating();
        }
    }
}

void Body::SetTransform(const Length2D position, Angle angle)
{
    assert(::box2d::IsValid(position));
    assert(::box2d::IsValid(angle));
    assert(!GetWorld()->IsLocked());

    if (GetWorld()->IsLocked())
    {
        return;
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

    assert(!m_world->IsLocked());
    if (m_world->IsLocked())
    {
        return;
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
    for (auto&& fixture: m_fixtures)
    {
        m_world->RegisterForProxies(&fixture);
    }
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

bool Body::Erase(Joint* const joint)
{
    for (auto iter = m_joints.begin(); iter != m_joints.end(); ++iter)
    {
        if (iter->second == joint)
        {
            m_joints.erase(iter);
            return true;
        }
    }
    return false;
}

bool Body::Insert(Contact* contact)
{
#ifndef NDEBUG
    // Prevent the same contact from being added more than once...
    for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
    {
        assert(iter->second != contact);
        if (iter->second == contact)
        {
            return false;
        }
    }
#endif

    m_contacts.emplace_back(GetContactKey(*contact), contact);
    return true;
}

bool Body::Erase(Contact* const contact)
{
    for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
    {
        if (iter->second == contact)
        {
            m_contacts.erase(iter);
            return true;
        }
    }
    return false;
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
    for (auto&& ji: lhs.GetJoints())
    {
        if (ji.first == &rhs)
        {
            if (!(ji.second->GetCollideConnected()))
            {
                return false;
            }
        }
    }

    return true;
}

box2d::size_t box2d::GetWorldIndex(const Body* body)
{
    if (body)
    {
        const auto world = body->GetWorld();

        auto i = size_t{0};
        for (auto&& b: world->GetBodies())
        {
            if (&b == body)
            {
                return i;
            }
            ++i;
        }
    }
    return size_t(-1);
}

Velocity box2d::GetVelocity(const Body& body, Time h) noexcept
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

size_t box2d::GetFixtureCount(const Body& body)
{
    const auto& fixtures = body.GetFixtures();
    return static_cast<size_t>(fixtures.size());
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
