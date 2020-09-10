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
#include <PlayRho/Dynamics/WorldImpl.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/BodyAtty.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/FixtureAtty.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>
#include <PlayRho/Dynamics/Island.hpp>
#include <PlayRho/Dynamics/JointAtty.hpp>
#include <PlayRho/Dynamics/ContactAtty.hpp>
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

#ifdef DO_PAR_UNSEQ
#include <atomic>
#endif

//#define DO_THREADED
#if defined(DO_THREADED)
#include <future>
#endif

#define PLAYRHO_MAGIC(x) (x)

using std::for_each;
using std::remove;
using std::sort;
using std::transform;
using std::unique;

namespace playrho {
namespace d2 {

using playrho::size;

World::World(const WorldConf& def): m_impl{std::make_unique<WorldImpl>(*this, def)}
{
}

World::World(const World& other): m_impl{std::make_unique<WorldImpl>(*this, *other.m_impl)}
{
}

World& World::operator= (const World& other)
{
    m_impl->copy(*other.m_impl);
    return *this;
}

World::~World() noexcept = default;

void World::Clear()
{
    m_impl->Clear();
}

Body* World::CreateBody(const BodyConf& def)
{
    return m_impl->CreateBody(def);
}

void World::Destroy(Body* body)
{
    m_impl->Destroy(body);
}

Joint* World::CreateJoint(const JointConf& def)
{
    return m_impl->CreateJoint(def);
}

void World::Destroy(Joint* joint)
{
    m_impl->Destroy(joint);
}
    
StepStats World::Step(const StepConf& conf)
{
    return m_impl->Step(conf);
}

void World::ShiftOrigin(Length2 newOrigin)
{
    m_impl->ShiftOrigin(newOrigin);
}

SizedRange<World::Bodies::iterator> World::GetBodies() noexcept
{
    return m_impl->GetBodies();
}

SizedRange<World::Bodies::const_iterator> World::GetBodies() const noexcept
{
    return m_impl->GetBodies();
}

SizedRange<World::Bodies::const_iterator> World::GetBodiesForProxies() const noexcept
{
    return m_impl->GetBodiesForProxies();
}

SizedRange<World::Fixtures::const_iterator> World::GetFixturesForProxies() const noexcept
{
    return m_impl->GetFixturesForProxies();
}

SizedRange<World::Joints::const_iterator> World::GetJoints() const noexcept
{
    return m_impl->GetJoints();
}

SizedRange<World::Joints::iterator> World::GetJoints() noexcept
{
    return m_impl->GetJoints();
}

SizedRange<World::Contacts::const_iterator> World::GetContacts() const noexcept
{
    return m_impl->GetContacts();
}

bool World::IsLocked() const noexcept
{
    return m_impl && m_impl->IsLocked();
}

bool World::IsStepComplete() const noexcept
{
    return m_impl->IsStepComplete();
}

bool World::GetSubStepping() const noexcept
{
    return m_impl->GetSubStepping();
}

void World::SetSubStepping(bool flag) noexcept
{
    m_impl->SetSubStepping(flag);
}

Length World::GetMinVertexRadius() const noexcept
{
    return m_impl->GetMinVertexRadius();
}

Length World::GetMaxVertexRadius() const noexcept
{
    return m_impl->GetMaxVertexRadius();
}

Frequency World::GetInvDeltaTime() const noexcept
{
    return m_impl->GetInvDeltaTime();
}

const DynamicTree& World::GetTree() const noexcept
{
    return m_impl->GetTree();
}

void World::SetDestructionListener(DestructionListener* listener) noexcept
{
    m_impl->SetDestructionListener(listener);
}

void World::SetContactListener(ContactListener* listener) noexcept
{
    m_impl->SetContactListener(listener);
}

void World::Refilter(Fixture& fixture)
{
    m_impl->Refilter(fixture);
}

void World::SetFilterData(Fixture& fixture, const Filter& filter)
{
    m_impl->SetFilterData(fixture, filter);
}

void World::SetType(Body& body, BodyType type)
{
    m_impl->SetType(body, type);
}

Fixture* World::CreateFixture(Body& body, const Shape& shape, const FixtureConf& def, bool resetMassData)
{
    return m_impl->CreateFixture(body, shape, def, resetMassData);
}

bool World::Destroy(Fixture& fixture, bool resetMassData)
{
    return m_impl->Destroy(fixture, resetMassData);
}

void World::DestroyFixtures(Body& body)
{
    m_impl->DestroyFixtures(body);
}

void World::SetEnabled(Body& body, bool flag)
{
    m_impl->SetEnabled(body, flag);
}

void World::SetMassData(Body& body, const MassData& massData)
{
    m_impl->SetMassData(body, massData);
}

void World::SetTransform(Body& body, Length2 location, Angle angle)
{
    m_impl->SetTransform(body, location, angle);
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
                                                [&](const World::Contacts::value_type &c) {
        return GetRef(std::get<Contact*>(c)).IsTouching();
    }));
}

size_t GetFixtureCount(const World& world) noexcept
{
    auto sum = size_t{0};
    const auto bodies = world.GetBodies();
    for_each(cbegin(bodies), cend(bodies),
             [&](const World::Bodies::value_type &body) {
        sum += GetFixtureCount(GetRef(body));
    });
    return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
    auto shapes = std::set<const void*>();
    const auto bodies = world.GetBodies();
    for_each(cbegin(bodies), cend(bodies), [&](const World::Bodies::value_type &b) {
        const auto fixtures = GetRef(b).GetFixtures();
        for_each(cbegin(fixtures), cend(fixtures), [&](const Body::Fixtures::value_type& f) {
            shapes.insert(GetData(GetRef(f).GetShape()));
        });
    });
    return size(shapes);
}

BodyCounter GetAwakeCount(const World& world) noexcept
{
    const auto bodies = world.GetBodies();
    return static_cast<BodyCounter>(count_if(cbegin(bodies), cend(bodies),
                                             [&](const World::Bodies::value_type &b) {
                                                 return GetRef(b).IsAwake(); }));
}
    
BodyCounter Awaken(World& world) noexcept
{
    // Can't use count_if since body gets modified.
    auto awoken = BodyCounter{0};
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&](World::Bodies::value_type &b) {
        if (playrho::d2::Awaken(GetRef(b)))
        {
            ++awoken;
        }
    });
    return awoken;
}

void SetAccelerations(World& world, Acceleration acceleration) noexcept
{
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&](World::Bodies::value_type &b) {
        SetAcceleration(GetRef(b), acceleration);
    });
}

void SetAccelerations(World& world, LinearAcceleration2 acceleration) noexcept
{
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&](World::Bodies::value_type &b) {
        SetLinearAcceleration(GetRef(b), acceleration);
    });
}

Body* FindClosestBody(const World& world, Length2 location) noexcept
{
    const auto bodies = world.GetBodies();
    auto found = static_cast<decltype(bodies)::iterator_type::value_type>(nullptr);
    auto minLengthSquared = std::numeric_limits<Area>::infinity();
    for (const auto& b: bodies)
    {
        auto& body = GetRef(b);
        const auto bodyLoc = body.GetLocation();
        const auto lengthSquared = GetMagnitudeSquared(bodyLoc - location);
        if (minLengthSquared > lengthSquared)
        {
            minLengthSquared = lengthSquared;
            found = &body;
        }
    }
    return found;
}

void SetLocation(World& world, Body& body, Length2 value) noexcept
{
    world.SetTransform(body, value, body.GetSweep().pos1.angular);
}

void SetAngle(World& world, Body& body, Angle value) noexcept
{
    world.SetTransform(body, GetLocation(body), value);
}

void RotateAboutWorldPoint(World& world, Body& body, Angle amount, Length2 worldPoint)
{
    const auto xfm = body.GetTransformation();
    const auto p = xfm.p - worldPoint;
    const auto c = cos(amount);
    const auto s = sin(amount);
    const auto x = GetX(p) * c - GetY(p) * s;
    const auto y = GetX(p) * s + GetY(p) * c;
    const auto pos = Length2{x, y} + worldPoint;
    const auto angle = GetAngle(xfm.q) + amount;
    world.SetTransform(body, pos, angle);
}

void RotateAboutLocalPoint(World& world, Body& body, Angle amount, Length2 localPoint)
{
    RotateAboutWorldPoint(world, body, amount, GetWorldPoint(body, localPoint));
}

Acceleration CalcGravitationalAcceleration(const World& world, const Body& body) noexcept
{
    const auto m1 = GetMass(body);
    if (m1 != 0_kg)
    {
        const auto loc1 = GetLocation(body);
        auto sumForce = Force2{};
        const auto bodies = world.GetBodies();
        for (auto jt = begin(bodies); jt != end(bodies); jt = std::next(jt))
        {
            const auto& b2 = *(*jt);
            if (&b2 == &body)
            {
                continue;
            }
            const auto m2 = GetMass(b2);
            const auto delta = GetLocation(b2) - loc1;
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

BodyCounter GetWorldIndex(const World& world, const Body* body) noexcept
{
    if (body)
    {
        const auto bodies = world.GetBodies();
        auto i = BodyCounter{0};
        const auto it = std::find_if(cbegin(bodies), cend(bodies), [&](const Body *b) {
            return b == body || ((void) ++i, false);
        });
        if (it != end(bodies))
        {
            return i;
        }
    }
    return BodyCounter(-1);
}

} // namespace d2

RegStepStats& Update(RegStepStats& lhs, const IslandStats& rhs) noexcept
{
    lhs.maxIncImpulse = std::max(lhs.maxIncImpulse, rhs.maxIncImpulse);
    lhs.minSeparation = std::min(lhs.minSeparation, rhs.minSeparation);
    lhs.islandsSolved += rhs.solved;
    lhs.sumPosIters += rhs.positionIterations;
    lhs.sumVelIters += rhs.velocityIterations;
    lhs.bodiesSlept += rhs.bodiesSlept;
    return lhs;
}

} // namespace playrho
