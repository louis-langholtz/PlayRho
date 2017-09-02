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
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/BodyAtty.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/FixtureAtty.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>
#include <PlayRho/Dynamics/Island.hpp>
#include <PlayRho/Dynamics/JointAtty.hpp>
#include <PlayRho/Dynamics/ContactAtty.hpp>
#include <PlayRho/Dynamics/MovementConf.hpp>

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Joints/MouseJoint.hpp>
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

#include <new>
#include <functional>
#include <type_traits>
#include <memory>
#include <set>
#include <vector>
#include <unordered_map>
#include <algorithm>
#ifdef DO_PAR_UNSEQ
#include <atomic>
#endif

//#define DO_THREADED
#if defined(DO_THREADED)
#include <future>
#endif

#define PLAYRHO_MAGIC(x) (x)

using namespace std;

namespace playrho
{

using BodyPtr = Body*;
using BodyConstraintsPair = pair<const Body* const, BodyConstraint*>;
using BodyConstraints = vector<BodyConstraint>;
using PositionConstraints = vector<PositionConstraint>;
using VelocityConstraints = vector<VelocityConstraint>;

template <typename T>
class FlagGuard
{
public:
    FlagGuard(T& flag, T value) : m_flag(flag), m_value(value)
    {
        static_assert(is_unsigned<T>::value, "Unsigned integer required");
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
    explicit RaiiWrapper(function<void(T&)> on_destruction): m_on_destruction(on_destruction) {}
    ~RaiiWrapper() { m_on_destruction(m_wrapped); }
    T m_wrapped;

private:
    function<void(T&)> m_on_destruction;
};

namespace {
    
    struct PositionAndVelocity
    {
        Position position;
        Velocity velocity;
    };

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
    
    struct VelocityPair
    {
        Velocity a;
        Velocity b;
    };
    
    inline VelocityPair CalcWarmStartVelocityDeltas(const VelocityConstraint& vc)
    {
        auto vp = VelocityPair{
            Velocity{LinearVelocity2D{}, AngularVelocity{0}},
            Velocity{LinearVelocity2D{}, AngularVelocity{0}}
        };
        
        const auto normal = vc.GetNormal();
        const auto tangent = vc.GetTangent();
        const auto pointCount = vc.GetPointCount();
        const auto bodyA = vc.GetBodyA();
        const auto bodyB = vc.GetBodyB();

        const auto invMassA = bodyA->GetInvMass();
        const auto invRotInertiaA = bodyA->GetInvRotInertia();
        
        const auto invMassB = bodyB->GetInvMass();
        const auto invRotInertiaB = bodyB->GetInvRotInertia();
        
        for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
        {
            // inverse moment of inertia : L^-2 M^-1 QP^2
            // P is M L T^-2
            // GetPointRelPosA() is Length2D
            // Cross(Length2D, P) is: M L^2 T^-2
            // L^-2 M^-1 QP^2 M L^2 T^-2 is: QP^2 T^-2
            const auto& vcp = vc.GetPointAt(j);
            const auto P = vcp.normalImpulse * normal + vcp.tangentImpulse * tangent;
            const auto LA = Cross(vcp.relA, P) / Radian;
            const auto LB = Cross(vcp.relB, P) / Radian;
            vp.a -= Velocity{invMassA * P, invRotInertiaA * LA};
            vp.b += Velocity{invMassB * P, invRotInertiaB * LB};
        }
        return vp;
    }
    
    inline void WarmStartVelocities(const VelocityConstraints& velConstraints)
    {
        for_each(cbegin(velConstraints), cend(velConstraints),
                      [&](const VelocityConstraint& vc) {
            const auto vp = CalcWarmStartVelocityDeltas(vc);
            const auto bodyA = vc.GetBodyA();
            const auto bodyB = vc.GetBodyB();
            bodyA->SetVelocity(bodyA->GetVelocity() + vp.a);
            bodyB->SetVelocity(bodyB->GetVelocity() + vp.b);
        });
    }

    BodyConstraintsMap GetBodyConstraintsMap(const Island::Bodies& bodies,
                                             BodyConstraints &bodyConstraints)
    {
        auto map = BodyConstraintsMap{};
        map.reserve(bodies.size());
        for_each(cbegin(bodies), cend(bodies), [&](const BodyPtr& body) {
            const auto i = static_cast<size_t>(&body - bodies.data());
            assert(i < bodies.size());
#ifdef USE_VECTOR_MAP
            map.push_back(BodyConstraintPair{body, &bodyConstraints[i]});
#else
            map[body] = &bodyConstraints[i];
#endif
        });
#ifdef USE_VECTOR_MAP
        sort(begin(map), end(map), [](BodyConstraintPair a, BodyConstraintPair b) {
            return a.first < b.first;
        });
#endif
        return map;
    }
    
    BodyConstraints GetBodyConstraints(const Island::Bodies& bodies, Time h, MovementConf conf)
    {
        auto constraints = BodyConstraints{};
        constraints.reserve(bodies.size());
        transform(cbegin(bodies), cend(bodies), back_inserter(constraints), [&](const BodyPtr &b) {
            return GetBodyConstraint(*b, h, conf);
        });
        return constraints;
    }

    PositionConstraints GetPositionConstraints(const Island::Contacts& contacts,
                                               BodyConstraintsMap& bodies)
    {
        auto constraints = PositionConstraints{};
        constraints.reserve(contacts.size());
        transform(cbegin(contacts), cend(contacts), back_inserter(constraints),
                  [&](const Contact *contact) {
            const auto& manifold = static_cast<const Contact*>(contact)->GetManifold();
            
            const auto& fixtureA = *(contact->GetFixtureA());
            const auto& fixtureB = *(contact->GetFixtureB());
            
            const auto bodyA = fixtureA.GetBody();
            const auto shapeA = fixtureA.GetShape();
            
            const auto bodyB = fixtureB.GetBody();
            const auto shapeB = fixtureB.GetShape();
            
            const auto bodyConstraintA = At(bodies, bodyA);
            const auto bodyConstraintB = At(bodies, bodyB);
            
            const auto radiusA = GetVertexRadius(*shapeA);
            const auto radiusB = GetVertexRadius(*shapeB);
            
            return PositionConstraint{
                manifold, *bodyConstraintA, radiusA, *bodyConstraintB, radiusB
            };
        });
        return constraints;
    }

    /// @brief Gets the velocity constraints for the given inputs.
    /// @details Inializes the velocity constraints with the position dependent portions of
    ///   the current position constraints.
    /// @post Velocity constraints will have their "normal" field set to the world manifold
    ///   normal for them.
    /// @post Velocity constraints will have their constraint points set.
    /// @sa SolveVelocityConstraints.
    VelocityConstraints GetVelocityConstraints(const Island::Contacts& contacts,
                                               BodyConstraintsMap& bodies,
                                               const VelocityConstraint::Conf conf)
    {
        auto velConstraints = VelocityConstraints{};
        velConstraints.reserve(contacts.size());
        transform(cbegin(contacts), cend(contacts), back_inserter(velConstraints),
                  [&](const ContactPtr& contact) {
            const auto& manifold = contact->GetManifold();
            const auto fixtureA = contact->GetFixtureA();
            const auto fixtureB = contact->GetFixtureB();
            const auto friction = contact->GetFriction();
            const auto restitution = contact->GetRestitution();
            const auto tangentSpeed = contact->GetTangentSpeed();
            
            const auto bodyA = fixtureA->GetBody();
            const auto shapeA = fixtureA->GetShape();
            
            const auto bodyB = fixtureB->GetBody();
            const auto shapeB = fixtureB->GetShape();
            
            const auto bodyConstraintA = At(bodies, bodyA);
            const auto bodyConstraintB = At(bodies, bodyB);
            
            const auto radiusA = shapeA->GetVertexRadius();
            const auto radiusB = shapeB->GetVertexRadius();
    
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
        auto maxIncImpulse = Momentum{0};
        for_each(begin(velConstraints), end(velConstraints), [&](VelocityConstraint& vc)
        {
            maxIncImpulse = max(maxIncImpulse, GaussSeidel::SolveVelocityConstraint(vc));
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
        auto minSeparation = numeric_limits<Real>::infinity() * Meter;
        
        for_each(begin(posConstraints), end(posConstraints), [&](PositionConstraint &pc) {
            assert(pc.GetBodyA() != pc.GetBodyB()); // Confirms ContactManager::Add() did its job.
            const auto res = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
            pc.GetBodyA()->SetPosition(res.pos_a);
            pc.GetBodyB()->SetPosition(res.pos_b);
            minSeparation = min(minSeparation, res.min_separation);
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
                                    const BodyConstraint* bodyConstraintA, const BodyConstraint* bodyConstraintB,
                                    ConstraintSolverConf conf)
    {
        auto minSeparation = numeric_limits<Real>::infinity() * Meter;
        
        for_each(begin(posConstraints), end(posConstraints), [&](PositionConstraint &pc) {
            const auto moveA = (pc.GetBodyA() == bodyConstraintA) || (pc.GetBodyA() == bodyConstraintB);
            const auto moveB = (pc.GetBodyB() == bodyConstraintA) || (pc.GetBodyB() == bodyConstraintB);
            const auto res = SolvePositionConstraint(pc, moveA, moveB, conf);
            pc.GetBodyA()->SetPosition(res.pos_a);
            pc.GetBodyB()->SetPosition(res.pos_b);
            minSeparation = min(minSeparation, res.min_separation);
        });
        
        return minSeparation;
    }
#endif
    
    inline Time GetUnderActiveTime(const Body& b, const StepConf& conf) noexcept
    {
        const auto underactive = IsUnderActive(b.GetVelocity(), conf.linearSleepTolerance,
                                               conf.angularSleepTolerance);
        const auto sleepable = b.IsSleepingAllowed();
        return (sleepable && underactive)?
            b.GetUnderActiveTime() + conf.GetTime(): Second * Real{0};
    }

    inline Time UpdateUnderActiveTimes(Island::Bodies& bodies, const StepConf& conf)
    {
        auto minUnderActiveTime = Second * numeric_limits<Real>::infinity();
        for_each(cbegin(bodies), cend(bodies), [&](Body *b)
        {
            if (b->IsSpeedable())
            {
                const auto underActiveTime = GetUnderActiveTime(*b, conf);
                b->SetUnderActiveTime(underActiveTime);
                minUnderActiveTime = min(minUnderActiveTime, underActiveTime);
            }
        });
        return minUnderActiveTime;
    }
    
    inline BodyCounter Sleepem(Island::Bodies& bodies)
    {
        auto unawoken = BodyCounter{0};
        for_each(cbegin(bodies), cend(bodies), [&](Body *b)
        {
            if (Unawaken(*b))
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
    
    void FlagContactsForFiltering(const Body& bodyA, const Body& bodyB)
    {
        for (auto& ci: bodyB.GetContacts())
        {
            const auto contact = GetContactPtr(ci);
            const auto fA = contact->GetFixtureA();
            const auto fB = contact->GetFixtureB();
            const auto bA = fA->GetBody();
            const auto bB = fB->GetBody();
            const auto other = (bA != &bodyB)? bA: bB;
            if (other == &bodyA)
            {
                // Flag the contact for filtering at the next time step (where either
                // body is awake).
                contact->FlagForFiltering();
            }
        }
    }
    
    inline bool TestOverlap(const DynamicTree& tree,
                            const Fixture* fixtureA, ChildCounter indexA,
                            const Fixture* fixtureB, ChildCounter indexB)
    {
        const auto proxyIdA = fixtureA->GetProxy(indexA)->proxyId;
        const auto proxyIdB = fixtureB->GetProxy(indexB)->proxyId;
        return TestOverlap(tree, proxyIdA, proxyIdB);
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
    if (def.minVertexRadius > def.maxVertexRadius)
    {
        throw InvalidArgument("max vertex radius must be >= min vertex radius");
    }
    m_proxyKeys.reserve(1024);
    m_proxies.reserve(1024);
}

World::World(const World& other):
    m_gravity{other.m_gravity},
    m_destructionListener{other.m_destructionListener},
    m_contactListener{other.m_contactListener},
    m_contactFilter{other.m_contactFilter},
    m_flags{other.m_flags},
    m_inv_dt0{other.m_inv_dt0},
    m_minVertexRadius{other.m_minVertexRadius},
    m_maxVertexRadius{other.m_maxVertexRadius},
    m_tree{other.m_tree}
{
    auto bodyMap = map<const Body*, Body*>();
    auto fixtureMap = map<const Fixture*, Fixture*>();
    CopyBodies(bodyMap, fixtureMap, other.GetBodies());
    CopyJoints(bodyMap, other.GetJoints());
    CopyContacts(bodyMap, fixtureMap, other.GetContacts());
}

World& World::operator= (const World& other)
{
    Clear();
    
    m_gravity = other.m_gravity;
    m_destructionListener = other.m_destructionListener;
    m_contactListener = other.m_contactListener;
    m_contactFilter = other.m_contactFilter;
    m_flags = other.m_flags;
    m_inv_dt0 = other.m_inv_dt0;
    m_minVertexRadius = other.m_minVertexRadius;
    m_maxVertexRadius = other.m_maxVertexRadius;
    m_tree = other.m_tree;

    auto bodyMap = map<const Body*, Body*>();
    auto fixtureMap = map<const Fixture*, Fixture*>();
    CopyBodies(bodyMap, fixtureMap, other.GetBodies());
    CopyJoints(bodyMap, other.GetJoints());
    CopyContacts(bodyMap, fixtureMap, other.GetContacts());

    return *this;
}
    
World::~World()
{
    Clear();
}

void World::Clear() noexcept
{
    for_each(cbegin(m_joints), cend(m_joints), [&](const Joint *j) {
        JointAtty::Destroy(j);
    });
    for_each(begin(m_bodies), end(m_bodies), [&](Bodies::value_type& body) {
        auto& b = GetRef(body);
        BodyAtty::ClearContacts(b);
        BodyAtty::ClearJoints(b);
        BodyAtty::ClearFixtures(b, [&](Fixture& fixture) {
            if (m_destructionListener)
            {
                m_destructionListener->SayGoodbye(fixture);
            }
            DestroyProxies(fixture);
        });
    });

    for_each(cbegin(m_bodies), cend(m_bodies), [&](const Bodies::value_type& b) {
        delete GetPtr(b);
    });
    for_each(cbegin(m_contacts), cend(m_contacts), [&](const Contacts::value_type& c){
        delete GetPtr(c);
    });

    m_bodies.clear();
    m_joints.clear();
    m_contacts.clear();
}

void World::CopyBodies(map<const Body*, Body*>& bodyMap,
                       map<const Fixture*, Fixture*>& fixtureMap,
                       SizedRange<World::Bodies::const_iterator> range)
{
    for (const auto& otherBody: range)
    {
        const auto newBody = CreateBody(GetBodyDef(GetRef(otherBody)));
        for (const auto& of: GetRef(otherBody).GetFixtures())
        {
            const auto& otherFixture = GetRef(of);
            const auto shape = otherFixture.GetShape();
            const auto fixtureDef = GetFixtureDef(otherFixture);
            const auto newFixture = BodyAtty::CreateFixture(*newBody, shape, fixtureDef);
            fixtureMap[&otherFixture] = newFixture;
            const auto childCount = otherFixture.GetProxyCount();
            const auto proxies = static_cast<FixtureProxy*>(Alloc(sizeof(FixtureProxy) * childCount));
            for (auto childIndex = decltype(childCount){0}; childIndex < childCount; ++childIndex)
            {
                const auto proxyPtr = proxies + childIndex;
                const auto fp = otherFixture.GetProxy(childIndex);
                new (proxyPtr) FixtureProxy{fp->aabb, fp->proxyId, newFixture, childIndex};
                m_tree.SetUserData(fp->proxyId, proxyPtr);
            }
            FixtureAtty::SetProxies(*newFixture, Span<FixtureProxy>(proxies, childCount));
        }
        newBody->SetMassData(GetMassData(GetRef(otherBody)));
        bodyMap[GetPtr(otherBody)] = newBody;
    }
}

void World::CopyContacts(const map<const Body*, Body*>& bodyMap,
                         const map<const Fixture*, Fixture*>& fixtureMap,
                         SizedRange<World::Contacts::const_iterator> range)
{
    for (const auto& contact: range)
    {
        auto& otherContact = GetRef(contact);
        const auto otherFixtureA = otherContact.GetFixtureA();
        const auto otherFixtureB = otherContact.GetFixtureB();
        const auto childIndexA = otherContact.GetChildIndexA();
        const auto childIndexB = otherContact.GetChildIndexB();
        const auto newFixtureA = fixtureMap.at(otherFixtureA);
        const auto newFixtureB = fixtureMap.at(otherFixtureB);
        const auto newBodyA = bodyMap.at(otherFixtureA->GetBody());
        const auto newBodyB = bodyMap.at(otherFixtureB->GetBody());
        
        const auto newContact = new Contact{newFixtureA, childIndexA, newFixtureB, childIndexB};
        assert(newContact);
        if (newContact)
        {
            m_contacts.push_back(newContact);

            BodyAtty::Insert(*newBodyA, newContact);
            BodyAtty::Insert(*newBodyB, newContact);
            // No need to wake up the bodies - this should already be done due to above copy
            
            newContact->SetFriction(otherContact.GetFriction());
            newContact->SetRestitution(otherContact.GetRestitution());
            newContact->SetTangentSpeed(otherContact.GetTangentSpeed());
            auto& manifold = ContactAtty::GetMutableManifold(*newContact);
            manifold = otherContact.GetManifold();
            ContactAtty::CopyFlags(*newContact, otherContact);
            ContactAtty::SetToi(*newContact, otherContact.GetToi());
            ContactAtty::SetToiCount(*newContact, otherContact.GetToiCount());
        }
    }
}

void World::CopyJoints(const map<const Body*, Body*>& bodyMap,
                       SizedRange<World::Joints::const_iterator> range)
{
    auto jointMap = map<const Joint*, Joint*>();

    for (const auto& otherJoint: range)
    {
        const auto type = otherJoint->GetType();
        switch (type)
        {
            case JointType::Unknown:
            {
                break;
            }
            case JointType::Revolute:
            {
                const auto oJoint = static_cast<const RevoluteJoint*>(otherJoint);
                auto def = GetRevoluteJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Prismatic:
            {
                const auto oJoint = static_cast<const PrismaticJoint*>(otherJoint);
                auto def = GetPrismaticJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Distance:
            {
                const auto oJoint = static_cast<const DistanceJoint*>(otherJoint);
                auto def = GetDistanceJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Pulley:
            {
                const auto oJoint = static_cast<const PulleyJoint*>(otherJoint);
                auto def = GetPulleyJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Mouse:
            {
                const auto oJoint = static_cast<const MouseJoint*>(otherJoint);
                auto def = GetMouseJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Gear:
            {
                const auto oJoint = static_cast<const GearJoint*>(otherJoint);
                auto def = GetGearJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                def.joint1 = jointMap.at(def.joint1);
                def.joint2 = jointMap.at(def.joint2);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Wheel:
            {
                const auto oJoint = static_cast<const WheelJoint*>(otherJoint);
                auto def = GetWheelJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Weld:
            {
                const auto oJoint = static_cast<const WeldJoint*>(otherJoint);
                auto def = GetWeldJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Friction:
            {
                const auto oJoint = static_cast<const FrictionJoint*>(otherJoint);
                auto def = GetFrictionJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Rope:
            {
                const auto oJoint = static_cast<const RopeJoint*>(otherJoint);
                auto def = GetRopeJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
            case JointType::Motor:
            {
                const auto oJoint = static_cast<const MotorJoint*>(otherJoint);
                auto def = GetMotorJointDef(*oJoint);
                def.bodyA = bodyMap.at(def.bodyA);
                def.bodyB = bodyMap.at(def.bodyB);
                const auto j = JointAtty::Create(def);
                if (j)
                {
                    Add(j, def.bodyA, def.bodyB);
                    jointMap[oJoint] = j;
                }
                break;
            }
        }
    }
}

void World::SetGravity(const LinearAcceleration2D gravity) noexcept
{
    if (m_gravity != gravity)
    {
        const auto diff = gravity - m_gravity;
        for (auto& b: m_bodies)
        {
            auto& body = GetRef(b);
            ApplyLinearAcceleration(body, diff);
        }
        m_gravity = gravity;
    }
}

Body* World::CreateBody(const BodyDef& def)
{
    if (IsLocked())
    {
        throw LockedError("World::CreateBody: world is locked");
    }

    if (m_bodies.size() >= MaxBodies)
    {
        throw LengthError("World::CreateBody: operation would exceed MaxBodies");
    }
    
    auto& b = *(new Body(def, this));

    // Add to world bodies collection.
    //
    // Note: the order in which bodies are added matters! At least in-so-far as
    //   causing different results to occur when adding to the back vs. adding to
    //   the front. The World TilesComeToRest unit test for example runs faster
    //   with bodies getting added to the back (than when bodies are added to the
    //   front).
    //
    m_bodies.push_back(&b);

    b.SetAcceleration(m_gravity, AngularAcceleration{0});
    return &b;
}

bool World::Remove(const Body& b)
{
    const auto it = find_if(cbegin(m_bodies), cend(m_bodies), [&](const Bodies::value_type& body) {
        return GetPtr(body) == &b;
    });
    if (it != m_bodies.end())
    {
        delete GetPtr(*it);
        m_bodies.erase(it);
        return true;
    }
    return false;
}

void World::Destroy(Body* body)
{
    assert(body);
    assert(body->GetWorld() == this);
    
    if (IsLocked())
    {
        throw LockedError("World::Destroy: world is locked");
    }
    
    // Delete the attached joints.
    BodyAtty::ClearJoints(*body, [&](Joint& joint) {
        if (m_destructionListener)
        {
            m_destructionListener->SayGoodbye(joint);
        }
        InternalDestroy(&joint);
    });
    
    // Destroy the attached contacts.
    BodyAtty::EraseContacts(*body, [&](Contact& contact) {
        Destroy(&contact, body);
        return true;
    });
    
    // Delete the attached fixtures. This destroys broad-phase proxies.
    BodyAtty::ClearFixtures(*body, [&](Fixture& fixture) {
        if (m_destructionListener)
        {
            m_destructionListener->SayGoodbye(fixture);
        }
        DestroyProxies(fixture);
    });
    
    Remove(*body);
}

Joint* World::CreateJoint(const JointDef& def)
{
    if (IsLocked())
    {
        throw LockedError("World::CreateJoint: world is locked");
    }
    
    if (m_joints.size() >= MaxJoints)
    {
        throw LengthError("World::CreateJoint: operation would exceed MaxJoints");
    }
    
    // Note: creating a joint doesn't wake the bodies.
    const auto j = JointAtty::Create(def);
    if (j)
    {
        const auto bodyA = j->GetBodyA();
        const auto bodyB = j->GetBodyB();

        Add(j, bodyA, bodyB);
        
        // If the joint prevents collisions, then flag any contacts for filtering.
        if ((!def.collideConnected) && bodyA && bodyB)
        {
            FlagContactsForFiltering(*bodyA, *bodyB);
        }
    }
    return j;
}

bool World::Add(Joint* j, Body* bodyA, Body* bodyB)
{
    m_joints.push_back(j);
    BodyAtty::Insert(bodyA, j);
    BodyAtty::Insert(bodyB, j);
    return true;
}

bool World::Remove(Joint& j)
{
    const auto endIter = cend(m_joints);
    const auto iter = find(cbegin(m_joints), endIter, &j);
    if (iter != endIter)
    {
        m_joints.erase(iter);
        return true;
    }
    return false;
}

void World::Destroy(Joint* joint)
{
    if (!joint)
    {
        return;
    }
    if (IsLocked())
    {
        throw LockedError("World::Destroy: world is locked");
    }
    InternalDestroy(joint);
}
    
void World::InternalDestroy(Joint* joint)
{
    if (!Remove(*joint))
    {
        return;
    }
    
    // Disconnect from island graph.
    const auto bodyA = joint->GetBodyA();
    const auto bodyB = joint->GetBodyB();

    // Wake up connected bodies.
    if (bodyA)
    {
        bodyA->SetAwake();
        BodyAtty::Erase(*bodyA, joint);
    }
    if (bodyB)
    {
        bodyB->SetAwake();
        BodyAtty::Erase(*bodyB, joint);
    }

    const auto collideConnected = joint->GetCollideConnected();

    JointAtty::Destroy(joint);

    // If the joint prevented collisions, then flag any contacts for filtering.
    if ((!collideConnected) && bodyA && bodyB)
    {
        FlagContactsForFiltering(*bodyA, *bodyB);
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
    auto stack = vector<Body*>();
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
            assert(other->IsEnabled() || !other->IsAwake());
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

World::Bodies::size_type World::RemoveUnspeedablesFromIslanded(const vector<Body*>& bodies)
{
    auto numRemoved = Bodies::size_type{0};
    for_each(begin(bodies), end(bodies), [&](Body* body) {
        if (!body->IsSpeedable())
        {
            // Allow static bodies to participate in other islands.
            UnsetIslanded(body);
            ++numRemoved;
        }
    });
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
    for_each(begin(m_bodies), end(m_bodies), [](Bodies::value_type& b) {
        BodyAtty::UnsetIslanded(GetRef(b));
    });
    for_each(begin(m_contacts), end(m_contacts), [](Contacts::value_type& c) {
        ContactAtty::UnsetIslanded(GetRef(c));
    });
    for_each(begin(m_joints), end(m_joints), [](Joints::value_type& j) {
        JointAtty::UnsetIslanded(GetRef(j));
    });

#if defined(DO_THREADED)
    vector<future<World::IslandSolverResults>> futures;
    futures.reserve(remNumBodies);
#endif
    // Build and simulate all awake islands.
    for (auto&& b: m_bodies)
    {
        auto& body = GetRef(b);
        assert(!body.IsAwake() || body.IsSpeedable());
        assert(!body.IsAwake() || body.IsEnabled());
        if (!IsIslanded(&body) && body.IsAwake() && body.IsEnabled())
        {
            ++stats.islandsFound;

            // Size the island for the remaining un-evaluated bodies, contacts, and joints.
            Island island(remNumBodies, remNumContacts, remNumJoints);

            AddToIsland(island, body, remNumBodies, remNumContacts, remNumJoints);
            remNumBodies += RemoveUnspeedablesFromIslanded(island.m_bodies);

#if defined(DO_THREADED)
            // Updates bodies' sweep.pos0 to current sweep.pos1 and bodies' sweep.pos1 to new positions
            futures.push_back(async(launch::async, &World::SolveRegIslandViaGS,
                                         this, conf, island));
#else
            const auto solverResults = SolveRegIslandViaGS(conf, island);
            stats.maxIncImpulse = max(stats.maxIncImpulse, solverResults.maxIncImpulse);
            stats.minSeparation = min(stats.minSeparation, solverResults.minSeparation);
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
        stats.maxIncImpulse = max(stats.maxIncImpulse, solverResults.maxIncImpulse);
        stats.minSeparation = min(stats.minSeparation, solverResults.minSeparation);
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
        auto& body = GetRef(b);
        // A non-static body that was in an island may have moved.
        if (IsIslanded(&body) && body.IsSpeedable())
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

World::IslandSolverResults World::SolveRegIslandViaGS(const StepConf& conf, Island island)
{
    assert(island.m_bodies.size() > 0 || island.m_contacts.size() > 0 ||
           island.m_joints.size() > 0);
    
    auto results = IslandSolverResults{};
    results.positionIterations = conf.regPositionIterations;
    const auto h = conf.GetTime(); ///< Time step.

    // Update bodies' pos0 values.
    for_each(cbegin(island.m_bodies), cend(island.m_bodies), [&](Body* body) {
        BodyAtty::SetPosition0(*body, GetPosition1(*body)); // like Advance0(1) on the sweep.
    });
    
    // Copy bodies' pos1 and velocity data into local arrays.
    auto bodyConstraints = GetBodyConstraints(island.m_bodies, h, GetMovementConf(conf));
    auto bodyConstraintsMap = GetBodyConstraintsMap(island.m_bodies, bodyConstraints);
    auto posConstraints = GetPositionConstraints(island.m_contacts, bodyConstraintsMap);
    auto velConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraintsMap,
                                                      GetRegVelocityConstraintConf(conf));
    
    if (conf.doWarmStart)
    {
        WarmStartVelocities(velConstraints);
    }

    const auto psConf = GetRegConstraintSolverConf(conf);

    for_each(cbegin(island.m_joints), cend(island.m_joints), [&](Joint* joint) {
        JointAtty::InitVelocityConstraints(*joint, bodyConstraintsMap, conf, psConf);
    });
    
    results.velocityIterations = conf.regVelocityIterations;
    for (auto i = decltype(conf.regVelocityIterations){0}; i < conf.regVelocityIterations; ++i)
    {
        auto jointsOkay = true;
        for_each(cbegin(island.m_joints), cend(island.m_joints), [&](Joint* j) {
            jointsOkay &= JointAtty::SolveVelocityConstraints(*j, bodyConstraintsMap, conf);
        });

        // Note that the new incremental impulse can potentially be orders of magnitude
        // greater than the last incremental impulse used in this loop.
        const auto newIncImpulse = SolveVelocityConstraintsViaGS(velConstraints);
        results.maxIncImpulse = max(results.maxIncImpulse, newIncImpulse);

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
        results.minSeparation = min(results.minSeparation, minSeparation);
        const auto contactsOkay = (minSeparation >= conf.regMinSeparation);

        auto jointsOkay = true;
        for_each(cbegin(island.m_joints), cend(island.m_joints), [&](Joint* j) {
            jointsOkay &= JointAtty::SolvePositionConstraints(*j, bodyConstraintsMap, psConf);
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
        const auto i = static_cast<VelocityConstraints::size_type>(&vc - velConstraints.data());
        auto& manifold = ContactAtty::GetMutableManifold(*island.m_contacts[i]);
        AssignImpulses(manifold, vc);
    });
    
    for_each(cbegin(bodyConstraints), cend(bodyConstraints), [&](const BodyConstraint& bc) {
        const auto i = static_cast<size_t>(&bc - bodyConstraints.data());
        assert(i < bodyConstraints.size());
        UpdateBody(*island.m_bodies[i], bc.GetPosition(), bc.GetVelocity());
    });
    
    // XXX: Should contacts needing updating be updated now??

    if (m_contactListener)
    {
        Report(*m_contactListener, island.m_contacts, velConstraints,
               results.solved? results.positionIterations - 1: StepConf::InvalidIteration);
    }
    
    results.bodiesSlept = BodyCounter{0};
    const auto minUnderActiveTime = UpdateUnderActiveTimes(island.m_bodies, conf);
    if ((minUnderActiveTime >= conf.minStillTimeToSleep) && results.solved)
    {
        results.bodiesSlept = static_cast<decltype(results.bodiesSlept)>(Sleepem(island.m_bodies));
    }

    return results;
}

void World::ResetBodiesForSolveTOI()
{
    for_each(begin(m_bodies), end(m_bodies), [&](Bodies::value_type& body) {
        auto& b = GetRef(body);
        BodyAtty::UnsetIslanded(b);
        BodyAtty::ResetAlpha0(b);
    });
}

void World::ResetContactsForSolveTOI()
{
    for_each(begin(m_contacts), end(m_contacts), [&](Contacts::value_type &c) {
        auto& contact = GetRef(c);
        ContactAtty::UnsetIslanded(contact);
        ContactAtty::UnsetToi(contact);
        ContactAtty::ResetToiCount(contact);
    });
}

World::UpdateContactsData World::UpdateContactTOIs(const StepConf& conf)
{
    auto results = UpdateContactsData{};

    const auto toiConf = GetToiConf(conf);
    
    for (auto&& contact: m_contacts)
    {
        auto& c = GetRef(contact);
        if (c.HasValidToi())
        {
            ++results.numValidTOI;
            continue;
        }
        if (!c.IsEnabled() || HasSensor(c) || !IsActive(c) || !IsImpenetrable(c))
        {
            continue;
        }
        if (c.GetToiCount() >= conf.maxSubSteps)
        {
            // What are the pros/cons of this?
            // Larger m_maxSubSteps slows down the simulation.
            // m_maxSubSteps of 44 and higher seems to decrease the occurrance of tunneling of multiple
            // bullet body collisions with static objects.
            ++results.numAtMaxSubSteps;
            continue;
        }
        
        const auto fA = c.GetFixtureA();
        const auto fB = c.GetFixtureB();
        const auto bA = fA->GetBody();
        const auto bB = fB->GetBody();
                
        /*
         * Put the sweeps onto the same time interval.
         * Presumably no unresolved collisions happen before the maximum of the bodies' alpha-0 times.
         * So long as the least TOI of the contacts is always the first collision that gets dealt with,
         * this presumption is safe.
         */
        const auto alpha0 = max(bA->GetSweep().GetAlpha0(), bB->GetSweep().GetAlpha0());
        assert(alpha0 >= 0 && alpha0 < 1);
        BodyAtty::Advance0(*bA, alpha0);
        BodyAtty::Advance0(*bB, alpha0);
        
        // Compute the TOI for this contact (one or both bodies are active and impenetrable).
        // Computes the time of impact in interval [0, 1]
        const auto output = CalcToi(c, toiConf);
        
        // Use Min function to handle floating point imprecision which possibly otherwise
        // could provide a TOI that's greater than 1.
        const auto toi = IsValidForTime(output.get_state())?
            min(alpha0 + (1 - alpha0) * output.get_t(), Real{1}): Real{1};
        assert(toi >= alpha0 && toi <= 1);
        ContactAtty::SetToi(c, toi);
        
        results.maxDistIters = max(results.maxDistIters, output.get_max_dist_iters());
        results.maxToiIters = max(results.maxToiIters, output.get_toi_iters());
        results.maxRootIters = max(results.maxRootIters, output.get_max_root_iters());
        ++results.numUpdatedTOI;
    }

    return results;
}
    
World::ContactToiData World::GetSoonestContacts(const size_t reserveSize)
{
    auto minToi = nextafter(Real{1}, Real{0});
    auto minContacts = vector<Contact*>();
    minContacts.reserve(reserveSize);
    for (auto&& contact: m_contacts)
    {
        const auto c = GetPtr(contact);
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

ToiStepStats World::SolveToi(const StepConf& conf)
{
    auto stats = ToiStepStats{};

    if (IsStepComplete())
    {
        ResetBodiesForSolveTOI();
        ResetContactsForSolveTOI();
    }

    const auto subStepping = GetSubStepping();

    // Find TOI events and solve them.
    for (;;)
    {
        const auto updateData = UpdateContactTOIs(conf);
        stats.contactsAtMaxSubSteps += updateData.numAtMaxSubSteps;
        stats.contactsUpdatedToi += updateData.numUpdatedTOI;
        stats.maxDistIters = max(stats.maxDistIters, updateData.maxDistIters);
        stats.maxRootIters = max(stats.maxRootIters, updateData.maxRootIters);
        stats.maxToiIters = max(stats.maxToiIters, updateData.maxToiIters);
        
        const auto next = GetSoonestContacts(updateData.numValidTOI + updateData.numUpdatedTOI);
        const auto ncount = next.contacts.size();
        if (ncount == 0)
        {
            // No more TOI events to handle within the current time step. Done!
            SetStepComplete(true);
            break;
        }

        stats.maxSimulContacts = max(stats.maxSimulContacts,
                                          static_cast<decltype(stats.maxSimulContacts)>(ncount));
        stats.contactsFound += static_cast<ContactCounter>(ncount);
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

                const auto solverResults = SolveToi(conf, *contact);
                stats.minSeparation = min(stats.minSeparation, solverResults.minSeparation);
                stats.maxIncImpulse = max(stats.maxIncImpulse, solverResults.maxIncImpulse);
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
            auto& body = GetRef(b);
            if (IsIslanded(&body))
            {
                UnsetIslanded(&body);
                if (body.IsAccelerable())
                {
                    const auto xfm0 = GetTransform0(body.GetSweep());
                    const auto xfm1 = body.GetTransformation();
                    stats.proxiesMoved += Synchronize(body, xfm0, xfm1,
                                                      conf.displaceMultiplier, conf.aabbExtension);
                    ResetContactsForSolveTOI(body);
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

World::IslandSolverResults World::SolveToi(const StepConf& conf, Contact& contact)
{
    // Note:
    //   This method is what used to be b2World::SolveToi(const b2TimeStep& step).
    //   It also differs internally from Erin's implementation.
    //
    //   Here's some specific behavioral differences:
    //   1. Bodies don't get their under-active times reset (like they do in Erin's code).

    auto contactsUpdated = ContactCounter{0};
    auto contactsSkipped = ContactCounter{0};

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
            ContactAtty::Update(contact, Contact::GetUpdateConf(conf), m_contactListener);
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
    //     SolveToi(StepConf{conf}.SetTime((1 - toi) * conf.GetTime()), island);
    //
    auto subConf = StepConf{conf};
    auto results = SolveToiViaGS(subConf.SetTime((1 - toi) * conf.GetTime()), island);
    results.contactsUpdated += contactsUpdated;
    results.contactsSkipped += contactsSkipped;
    return results;
}

void World::UpdateBody(Body& body, const Position& pos, const Velocity& vel)
{
    assert(IsValid(pos));
    assert(IsValid(vel));
    BodyAtty::SetVelocity(body, vel);
    BodyAtty::SetPosition1(body, pos);
    BodyAtty::SetTransformation(body, GetTransformation(GetPosition1(body), body.GetLocalCenter()));
}

World::IslandSolverResults World::SolveToiViaGS(const StepConf& conf, Island& island)
{
    auto results = IslandSolverResults{};
    
    /*
     * Presumably the regular phase resolution has already taken care of updating the
     * body's velocity w.r.t. acceleration and damping such that this call here to get
     * the body constraint doesn't need to pass an elapsed time (and doesn't need to
     * update the velocity from what it already is).
     */
    auto bodyConstraints = GetBodyConstraints(island.m_bodies, Time(0), GetMovementConf(conf));
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
    
    auto posConstraints = GetPositionConstraints(island.m_contacts, bodyConstraintsMap);
    
    // Solve TOI-based position constraints.
    assert(results.minSeparation == numeric_limits<Real>::infinity() * Meter);
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
            results.minSeparation = min(results.minSeparation, minSeparation);
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
        
        BodyAtty::SetPosition0(*bodyA, bodyConstraintsMap.at(bodyA).GetPosition());
        BodyAtty::SetPosition0(*bodyB, bodyConstraintsMap.at(bodyB).GetPosition());
    }
#else
    for_each(cbegin(bodyConstraints), cend(bodyConstraints), [&](const BodyConstraint& bc) {
        const auto i = static_cast<size_t>(&bc - bodyConstraints.data());
        assert(i < bodyConstraints.size());
        BodyAtty::SetPosition0(*island.m_bodies[i], bc.GetPosition());
    });
#endif
    
    auto velConstraints = GetVelocityConstraints(island.m_contacts, bodyConstraintsMap,
                                                 GetToiVelocityConstraintConf(conf));

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.

    // Solve velocity constraints.
    assert(results.maxIncImpulse == Momentum{0});
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
        results.maxIncImpulse = max(results.maxIncImpulse, newIncImpulse);
    }
    
    // Don't store TOI contact forces for warm starting because they can be quite large.
    
    IntegratePositions(bodyConstraints, conf.GetTime());
    
    for_each(cbegin(bodyConstraints), cend(bodyConstraints), [&](const BodyConstraint& bc) {
        const auto i = static_cast<size_t>(&bc - bodyConstraints.data());
        assert(i < bodyConstraints.size());
        UpdateBody(*island.m_bodies[i], bc.GetPosition(), bc.GetVelocity());
    });

    if (m_contactListener)
    {
        Report(*m_contactListener, island.m_contacts, velConstraints, results.positionIterations);
    }
    
    return results;
}
    
void World::ResetContactsForSolveTOI(Body& body)
{
    // Invalidate all contact TOIs on this displaced body.
    const auto contacts = body.GetContacts();
    for_each(cbegin(contacts), cend(contacts), [&](KeyedContactPtr ci) {
        const auto contact = GetContactPtr(ci);
        UnsetIslanded(contact);
        ContactAtty::UnsetToi(*contact);
    });
}

World::ProcessContactsOutput
World::ProcessContactsForTOI(Island& island, Body& body, Real toi,
                             const StepConf& conf)
{
    assert(IsIslanded(&body));
    assert(body.IsAccelerable());
    assert(toi >= 0 && toi <= 1);

    auto results = ProcessContactsOutput{};
    assert(results.contactsUpdated == 0);
    assert(results.contactsSkipped == 0);
    
    const auto updateConf = Contact::GetUpdateConf(conf);

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
                ContactAtty::Update(*contact, updateConf, m_contactListener);
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
    assert((Length{m_maxVertexRadius} * Real{2}) +
           (Length{conf.linearSlop} / Real{4}) > (Length{m_maxVertexRadius} * Real{2}));
    
    if (IsLocked())
    {
        throw LockedError("World::Step: world is locked");
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

        if (conf.GetTime() != Second * Real{0})
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
                stepStats.toi = SolveToi(conf);
            }
        }
    }
    return stepStats;
}

void World::QueryAABB(AABB aabb, QueryFixtureCallback callback)
{
    m_tree.Query(aabb, [&](DynamicTree::size_type proxyId) {
        const auto proxy = static_cast<FixtureProxy*>(m_tree.GetUserData(proxyId));
        return callback(proxy->fixture, proxy->childIndex);
    });
}

void World::RayCast(Length2D point1, Length2D point2, RayCastCallback callback)
{
    m_tree.RayCast(RayCastInput{point1, point2, Real{1}},
                   [&](const RayCastInput& input, DynamicTree::size_type proxyId)
    {
        const auto userData = m_tree.GetUserData(proxyId);
        const auto proxy = static_cast<FixtureProxy*>(userData);
        const auto fixture = proxy->fixture;
        const auto index = proxy->childIndex;
        const auto shape = fixture->GetShape();
        const auto body = fixture->GetBody();
        const auto child = shape->GetChild(index);
        const auto transformation = body->GetTransformation();
        const auto output = playrho::RayCast(child, input, transformation);
        if (output.has_value())
        {
            const auto fraction = output->fraction;
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
            const auto opcode = callback(fixture, index, point, output->normal);
            switch (opcode)
            {
                case RayCastOpcode::Terminate: return Real(0);
                case RayCastOpcode::IgnoreFixture: return Real(-1);
                case RayCastOpcode::ClipRay: return Real{fraction};
                case RayCastOpcode::ResetRay: return Real{input.maxFraction};
            }
        }
        return Real{input.maxFraction};
    });
}

void World::ShiftOrigin(const Length2D newOrigin)
{
    if (IsLocked())
    {
        throw LockedError("World::ShiftOrigin: world is locked");
    }

    for (auto&& body: GetBodies())
    {
        auto& b = GetRef(body);

        auto transformation = b.GetTransformation();
        transformation.p -= newOrigin;
        BodyAtty::SetTransformation(b, transformation);
        
        auto sweep = b.GetSweep();
        sweep.pos0.linear -= newOrigin;
        sweep.pos1.linear -= newOrigin;
        BodyAtty::SetSweep(b, sweep);
    }

    for_each(begin(m_joints), end(m_joints), [&](Joints::value_type& j) {
        GetRef(j).ShiftOrigin(newOrigin);
    });

    m_tree.ShiftOrigin(newOrigin);
}

void World::InternalDestroy(Contact* contact, Body* from)
{
    if (m_contactListener && contact->IsTouching())
    {
        // EndContact hadn't been called in DestroyOrUpdateContacts() since is-touching, so call it now
        m_contactListener->EndContact(*contact);
    }
    
    const auto fixtureA = contact->GetFixtureA();
    const auto fixtureB = contact->GetFixtureB();
    const auto bodyA = fixtureA->GetBody();
    const auto bodyB = fixtureB->GetBody();
    
    if (bodyA != from)
    {
        BodyAtty::Erase(*bodyA, contact);
    }
    if (bodyB != from)
    {
        BodyAtty::Erase(*bodyB, contact);
    }
    
    if ((contact->GetManifold().GetPointCount() > 0) &&
        !fixtureA->IsSensor() && !fixtureB->IsSensor())
    {
        // Contact may have been keeping accelerable bodies of fixture A or B from moving.
        // Need to awaken those bodies now in case they are again movable.
        bodyA->SetAwake();
        bodyB->SetAwake();
    }
    
    delete contact;
}

void World::Destroy(Contact* contact, Body* from)
{
    assert(contact);

    InternalDestroy(contact, from);
    
    const auto it = find_if(cbegin(m_contacts), cend(m_contacts),
                            [&](const Contacts::value_type& c) {
        return GetPtr(c) == contact;
    });
    if (it != cend(m_contacts))
    {
        m_contacts.erase(it);
    }
}

World::DestroyContactsStats World::DestroyContacts(Contacts& contacts)
{
    auto stats = DestroyContactsStats{};
    
    contacts.erase(std::remove_if(begin(contacts), end(contacts), [&](Contacts::value_type& c)
    {
        auto& contact = GetRef(c);
        const auto indexA = contact.GetChildIndexA();
        const auto indexB = contact.GetChildIndexB();
        const auto fixtureA = contact.GetFixtureA();
        const auto fixtureB = contact.GetFixtureB();
        
        if (!TestOverlap(m_tree, fixtureA, indexA, fixtureB, indexB))
        {
            // Destroy contacts that cease to overlap in the broad-phase.
            InternalDestroy(&contact);
            ++stats.notOverlapping;
            return true;
        }
        
        // Is this contact flagged for filtering?
        if (contact.NeedsFiltering())
        {
            const auto bodyA = fixtureA->GetBody();
            const auto bodyB = fixtureB->GetBody();

            if (!::playrho::ShouldCollide(*bodyB, *bodyA) || !ShouldCollide(fixtureA, fixtureB))
            {
                InternalDestroy(&contact);
                ++stats.filteredOut;
                return true;
            }
            ContactAtty::UnflagForFiltering(contact);
        }

        ++stats.ignored;
        return false;
    }), end(contacts));
    
    return stats;
}

World::UpdateContactsStats World::UpdateContacts(Contacts& contacts, const StepConf& conf)
{
#ifdef DO_PAR_UNSEQ
    atomic<uint32_t> ignored;
    atomic<uint32_t> updated;
    atomic<uint32_t> skipped;
#else
    auto ignored = uint32_t(0);
    auto updated = uint32_t(0);
    auto skipped = uint32_t(0);
#endif

    const auto updateConf = Contact::GetUpdateConf(conf);
    
#if defined(DO_THREADED)
    vector<Contact*> contactsNeedingUpdate;
    contactsNeedingUpdate.reserve(contacts.size());
    vector<future<void>> futures;
    futures.reserve(contacts.size());
#endif

    // Update awake contacts.
    for_each(/*execution::par_unseq,*/ begin(contacts), end(contacts),
             [&](Contacts::value_type& c) {
        auto& contact = GetRef(c);
#if 0
        ContactAtty::Update(contact, updateConf, m_contactListener);
        ++updated;
#else
        const auto fixtureA = contact.GetFixtureA();
        const auto fixtureB = contact.GetFixtureB();
        const auto bodyA = fixtureA->GetBody();
        const auto bodyB = fixtureB->GetBody();
        
        // Awake && speedable (dynamic or kinematic) means collidable.
        // At least one body must be collidable
        assert(!bodyA->IsAwake() || bodyA->IsSpeedable());
        assert(!bodyB->IsAwake() || bodyB->IsSpeedable());
        if (!bodyA->IsAwake() && !bodyB->IsAwake())
        {
            assert(!contact.HasValidToi());
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
            //futures.push_back(async(&ContactAtty::Update, *contact, conf, m_contactListener)));
            //futures.push_back(async(launch::async, [=]{ ContactAtty::Update(*contact, conf, m_contactListener); }));
#else
            ContactAtty::Update(contact, updateConf, m_contactListener);
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
    auto numJobs = contactsNeedingUpdate.size();
    const auto jobsPerCore = numJobs / 4;
    for (auto i = decltype(numJobs){0}; numJobs > 0 && i < 3; ++i)
    {
        futures.push_back(async(launch::async, [=]{
            const auto offset = jobsPerCore * i;
            for (auto j = decltype(jobsPerCore){0}; j < jobsPerCore; ++j)
            {
	            ContactAtty::Update(*contactsNeedingUpdate[offset + j], updateConf, m_contactListener);
            }
        }));
        numJobs -= jobsPerCore;
    }
    if (numJobs > 0)
    {
        futures.push_back(async(launch::async, [=]{
            const auto offset = jobsPerCore * 3;
            for (auto j = decltype(numJobs){0}; j < numJobs; ++j)
            {
                ContactAtty::Update(*contactsNeedingUpdate[offset + j], updateConf, m_contactListener);
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

void World::RegisterForProcessing(ProxyId pid) noexcept
{
    assert(pid != DynamicTree::InvalidIndex);
    m_proxies.push_back(pid);
}

void World::UnregisterForProcessing(ProxyId pid) noexcept
{
    const auto itEnd = end(m_proxies);
    auto it = find(/*execution::par_unseq,*/ begin(m_proxies), itEnd, pid);
    if (it != itEnd)
    {
        *it = DynamicTree::InvalidIndex;
    }
}

ContactCounter World::FindNewContacts()
{
    m_proxyKeys.clear();

    for_each(cbegin(m_proxies), cend(m_proxies), [&](ProxyId pid) {
        const auto aabb = m_tree.GetAABB(pid);
        m_tree.ForEach(aabb, [&](DynamicTree::size_type nodeId) {
            // A proxy cannot form a pair with itself.
            if (nodeId != pid)
            {
                m_proxyKeys.push_back(ContactKey{nodeId, pid});
            }
        });
    });
    m_proxies.clear();

    sort(begin(m_proxyKeys), end(m_proxyKeys));

    auto count = ContactCounter{0};
    auto lastKey = ContactKey{};
    for_each(cbegin(m_proxyKeys), cend(m_proxyKeys), [&](ContactKey key)
    {
        if (key != lastKey)
        {
            const auto& proxyA = *static_cast<FixtureProxy*>(m_tree.GetUserData(key.GetMin()));
            const auto& proxyB = *static_cast<FixtureProxy*>(m_tree.GetUserData(key.GetMax()));
            
            if (Add(proxyA, proxyB))
            {
                ++count;
            }
            lastKey = key;
        }
    });
    return count;
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
    if (!::playrho::ShouldCollide(*bodyB, *bodyA) || !ShouldCollide(fixtureA, fixtureB))
    {
        return false;
    }

    const auto pidA = proxyA.proxyId;
    const auto pidB = proxyB.proxyId;
#ifndef NDEBUG
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
    const auto key = ContactKey{pidA, pidB};
    const auto searchBody = (bodyA->GetContacts().size() < bodyB->GetContacts().size())?
        bodyA: bodyB;
    
    const auto contacts = searchBody->GetContacts();
    const auto it = find_if(cbegin(contacts), cend(contacts), [&](KeyedContactPtr ci) {
        return ci.first == key;
    });
    if (it != cend(contacts))
    {
        return false;
    }
    
    assert(m_contacts.size() < MaxContacts);
    if (m_contacts.size() >= MaxContacts)
    {
        // New contact was needed, but denied due to MaxContacts count being reached.
        return false;
    }

    const auto contact = new Contact{fixtureA, proxyA.childIndex, fixtureB, proxyB.childIndex};
    
    // Insert into the contacts container.
    //
    // Should the new contact be added at front or back?
    //
    // Original strategy added to the front. Since processing done front to back, front
    // adding means container more a LIFO container, while back adding means more a FIFO.
    //
    m_contacts.push_back(contact);

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
    for_each(begin(m_fixturesForProxies), end(m_fixturesForProxies), [&](Fixture *f) {
        CreateAndDestroyProxies(*f, conf);
    });
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
    for_each(begin(m_bodiesForProxies), end(m_bodiesForProxies), [&](Body *b) {
        const auto xfm = b->GetTransformation();
        proxiesMoved += Synchronize(*b, xfm, xfm, conf.displaceMultiplier, conf.aabbExtension);
    });
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
        throw LockedError("World::SetType: world is locked");
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
        body.SetAcceleration(body.IsAccelerable()? GetGravity(): LinearAcceleration2D{},
                             AngularAcceleration{0});
        const auto fixtures = body.GetFixtures();
        for_each(begin(fixtures), end(fixtures), [&](Body::Fixtures::value_type& f) {
            InternalTouchProxies(GetRef(f));
        });
    }
}

Fixture* World::CreateFixture(Body& body, shared_ptr<const Shape> shape,
                              const FixtureDef& def, bool resetMassData)
{
    if (body.GetWorld() != this)
    {
        throw InvalidArgument("World::CreateFixture: invalid body");
    }

    if (!shape)
    {
        throw InvalidArgument("World::CreateFixture: null shape");
    }
    const auto vr = GetVertexRadius(*shape);
    if (!(vr >= GetMinVertexRadius()))
    {
        throw InvalidArgument("World::CreateFixture: vertex radius < min");
    }
    if (!(vr <= GetMaxVertexRadius()))
    {
        throw InvalidArgument("World::CreateFixture: vertex radius > max");
    }
    
    if (IsLocked())
    {
        throw LockedError("World::CreateFixture: world is locked");
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
        throw LockedError("World::DestroyFixture: world is locked");
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
    const auto proxies = static_cast<FixtureProxy*>(Alloc(sizeof(FixtureProxy) * childCount));
    for (auto childIndex = decltype(childCount){0}; childIndex < childCount; ++childIndex)
    {
        const auto dp = shape->GetChild(childIndex);
        const auto aabb = ComputeAABB(dp, xfm);
        const auto proxyPtr = proxies + childIndex;

        // Note: proxyId from CreateProxy can be higher than the number of fixture proxies.
        const auto fattenedAABB = GetFattenedAABB(aabb, aabbExtension);
        const auto proxyId = m_tree.CreateProxy(fattenedAABB, proxyPtr);
        RegisterForProcessing(proxyId);
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
        UnregisterForProcessing(proxies[i].proxyId);
        m_tree.DestroyProxy(proxies[i].proxyId);
        proxies[i].~FixtureProxy();
    }
    Free(proxies.begin());

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
        RegisterForProcessing(fixture.GetProxy(i)->proxyId);
    }
}

ChildCounter World::Synchronize(Fixture& fixture,
                                 const Transformation xfm1, const Transformation xfm2,
                                 const Real multiplier, const Length extension)
{
    assert(::playrho::IsValid(xfm1));
    assert(::playrho::IsValid(xfm2));
    
    auto updatedCount = ChildCounter{0};
    const auto shape = fixture.GetShape();
    const auto expandedDisplacement = multiplier * (xfm2.p - xfm1.p);
    const auto proxies = FixtureAtty::GetProxies(fixture);
    for (auto& proxy: proxies)
    {
        const auto dp = shape->GetChild(proxy.childIndex);

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        const auto aabb1 = ComputeAABB(dp, xfm1);
        const auto aabb2 = ComputeAABB(dp, xfm2);
        proxy.aabb = GetEnclosingAABB(aabb1, aabb2);
        
        if (!m_tree.GetAABB(proxy.proxyId).Contains(proxy.aabb))
        {
            const auto newAabb = GetDisplacedAABB(GetFattenedAABB(proxy.aabb, extension),
                                                  expandedDisplacement);
            m_tree.UpdateProxy(proxy.proxyId, newAabb);
            RegisterForProcessing(proxy.proxyId);
            ++updatedCount;
        }
    }
    return updatedCount;
}

ContactCounter World::Synchronize(Body& body,
                                   const Transformation& xfm1, const Transformation& xfm2,
                                   const Real multiplier, const Length aabbExtension)
{
    auto updatedCount = ContactCounter{0};
    for_each(begin(body.GetFixtures()), end(body.GetFixtures()), [&](Body::Fixtures::value_type& f) {
        updatedCount += Synchronize(GetRef(f), xfm1, xfm2, multiplier, aabbExtension);
    });
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

ContactCounter GetTouchingCount(const World& world) noexcept
{
    const auto contacts = world.GetContacts();
    return static_cast<ContactCounter>(count_if(cbegin(contacts), cend(contacts),
                                                [&](const World::Contacts::value_type &c) {
        return GetRef(c).IsTouching();
    }));
}

size_t GetFixtureCount(const World& world) noexcept
{
    auto sum = size_t{0};
    for_each(cbegin(world.GetBodies()), cend(world.GetBodies()),
             [&](const World::Bodies::value_type &body) {
        sum += GetFixtureCount(GetRef(body));
    });
    return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
    auto shapes = set<const Shape*>();
    for_each(cbegin(world.GetBodies()), cend(world.GetBodies()), [&](const World::Bodies::value_type &b) {
        const auto fixtures = GetRef(b).GetFixtures();
        for_each(cbegin(fixtures), cend(fixtures), [&](const Body::Fixtures::value_type& f) {
            shapes.insert(GetRef(f).GetShape().get());
        });
    });
    return shapes.size();
}

BodyCounter GetAwakeCount(const World& world) noexcept
{
    return static_cast<BodyCounter>(count_if(cbegin(world.GetBodies()), cend(world.GetBodies()),
                                             [&](const World::Bodies::value_type &b) {
                                                 return GetRef(b).IsAwake(); }));
}
    
BodyCounter Awaken(World& world) noexcept
{
    // Can't use count_if since body gets modified.
    auto awoken = BodyCounter{0};
    for_each(begin(world.GetBodies()), end(world.GetBodies()), [&](World::Bodies::value_type &b) {
        if (playrho::Awaken(GetRef(b)))
        {
            ++awoken;
        }
    });
    return awoken;
}

void ClearForces(World& world) noexcept
{
    const auto g = world.GetGravity();
    for_each(begin(world.GetBodies()), end(world.GetBodies()), [&](World::Bodies::value_type &b) {
        GetRef(b).SetAcceleration(g, AngularAcceleration{0});
    });
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

} // namespace playrho
