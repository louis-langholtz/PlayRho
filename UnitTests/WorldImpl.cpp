/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"

#include <PlayRho/Dynamics/WorldImpl.hpp>

#include <PlayRho/Dynamics/WorldImplBody.hpp>
#include <PlayRho/Dynamics/WorldImplMisc.hpp>
#include <PlayRho/Dynamics/WorldImplJoint.hpp>
#include <PlayRho/Dynamics/WorldImplContact.hpp>
#include <PlayRho/Dynamics/WorldImplFixture.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Dynamics/ContactImpulsesList.hpp>

#include <PlayRho/Collision/Shapes/ChainShapeConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Collision.hpp>
#include <PlayRho/Collision/DynamicTree.hpp> // for GetTree
#include <PlayRho/Collision/RayCastInput.hpp>
#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Collision/Manifold.hpp>

#include <PlayRho/Common/LengthError.hpp>
#include <PlayRho/Common/WrongState.hpp>

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/TargetJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RopeJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJointConf.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJointConf.hpp>
#include <PlayRho/Dynamics/Joints/WeldJointConf.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJointConf.hpp>
#include <PlayRho/Dynamics/Joints/MotorJointConf.hpp>
#include <PlayRho/Dynamics/Joints/WheelJointConf.hpp>
#include <PlayRho/Dynamics/Joints/GearJointConf.hpp>

#include <chrono>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

namespace {

template <typename T>
struct PushBackListener
{
    std::vector<T> ids;
    void operator()(T id)
    {
        ids.push_back(id);
    }
};

void SetEnabled(WorldImpl& world, BodyID id, bool value)
{
    auto copy = world.GetBody(id);
    SetEnabled(copy, value);
    world.SetBody(id, copy);
}

void SetType(WorldImpl& world, BodyID id, BodyType value)
{
    auto body = world.GetBody(id);
    SetType(body, value);
    world.SetBody(id, body);
}

} // namespace

TEST(WorldImpl, ByteSize)
{
    EXPECT_NE(sizeof(WorldImpl), sizeof(void*));
    // It's 944 bytes on at least one 64-but platform.
}

TEST(WorldImpl, DefaultInit)
{
    WorldImpl world;

    EXPECT_EQ(world.GetBodies().size(), BodyCounter(0));
    EXPECT_EQ(world.GetTree().GetLeafCount(), ContactCounter(0));
    EXPECT_EQ(world.GetJoints().size(), JointCounter(0));
    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));
    EXPECT_EQ(GetHeight(world.GetTree()), ContactCounter(0));
    EXPECT_EQ(ComputePerimeterRatio(world.GetTree()), Real(0));

    {
        const auto& bodies = world.GetBodies();
        EXPECT_TRUE(bodies.empty());
        EXPECT_EQ(bodies.size(), BodyCounter(0));
        EXPECT_EQ(bodies.begin(), bodies.end());
        EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());
    }
    {
        const auto& w = static_cast<const WorldImpl&>(world);
        const auto& bodies = w.GetBodies();
        EXPECT_TRUE(bodies.empty());
        EXPECT_EQ(bodies.size(), BodyCounter(0));
        EXPECT_EQ(bodies.begin(), bodies.end());
        EXPECT_EQ(w.GetBodies().begin(), w.GetBodies().end());
    }

    EXPECT_TRUE(world.GetContacts().empty());
    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));
    EXPECT_EQ(world.GetContacts().begin(), world.GetContacts().end());
    
    EXPECT_TRUE(world.GetJoints().empty());
    EXPECT_EQ(world.GetJoints().size(), JointCounter(0));
    EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
    
    EXPECT_FALSE(world.GetSubStepping());
    EXPECT_FALSE(world.IsLocked());
}

TEST(WorldImpl, Init)
{
    WorldImpl world{};
    EXPECT_FALSE(world.IsLocked());
    
    {
        auto calls = 0;
        Query(world.GetTree(), AABB{}, [&](FixtureID, ChildCounter) {
            ++calls;
            return true;
        });
        EXPECT_EQ(calls, 0);
    }
}

TEST(WorldImpl, InvalidArgumentInit)
{
    const auto min = Positive<Length>(4_m);
    const auto max = Positive<Length>(8_m);
    ASSERT_GT(max, min);
    const auto def = WorldConf{}.UseMinVertexRadius(max).UseMaxVertexRadius(min);
    EXPECT_THROW(WorldImpl{def}, InvalidArgument);
}

TEST(WorldImpl, Clear)
{
    auto jointListener = PushBackListener<JointID>{};
    auto fixtureListener = PushBackListener<FixtureID>{};

    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), std::size_t(0));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(0));

    world.SetJointDestructionListener(std::ref(jointListener));
    world.SetFixtureDestructionListener(std::ref(fixtureListener));

    const auto b0 = world.CreateBody();
    ASSERT_NE(b0, InvalidBodyID);
    const auto fixtureConf0 = FixtureConf{}.UseBody(b0).UseShape(DiskShapeConf{});
    const auto f0 = world.CreateFixture(fixtureConf0);
    ASSERT_NE(f0, InvalidFixtureID);
    ASSERT_EQ(GetFixtures(world, b0).size(), std::size_t(1));;

    const auto b1 = world.CreateBody();
    ASSERT_NE(b1, InvalidBodyID);
    const auto fixtureConf1 = FixtureConf{}.UseBody(b1).UseShape(DiskShapeConf{});
    const auto f1 = CreateFixture(world, fixtureConf1);
    ASSERT_NE(f1, InvalidFixtureID);
    ASSERT_EQ(GetFixtures(world, b1).size(), std::size_t(1));;

    const auto j0 = world.CreateJoint(Joint{DistanceJointConf{b0, b1}});
    ASSERT_NE(j0, InvalidJointID);

    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(1));

    EXPECT_NO_THROW(world.Clear());

    EXPECT_EQ(world.GetBodies().size(), std::size_t(0));
    EXPECT_EQ(world.GetJoints().size(), std::size_t(0));

    ASSERT_EQ(fixtureListener.ids.size(), std::size_t(2));
    EXPECT_EQ(fixtureListener.ids.at(0), f0);
    EXPECT_EQ(fixtureListener.ids.at(1), f1);

    ASSERT_EQ(jointListener.ids.size(), std::size_t(1));
    EXPECT_EQ(jointListener.ids.at(0), j0);

    const auto b2 = world.CreateBody();
    EXPECT_LE(b2, b1);
    const auto fixtureConf2 = FixtureConf{}.UseBody(b2).UseShape(DiskShapeConf{});
    const auto f2 = CreateFixture(world, fixtureConf2);
    EXPECT_LE(f2, f1);
}

TEST(WorldImpl, CreateDestroyEmptyStaticBody)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto bodyID = world.CreateBody(BodyConf{}.UseType(BodyType::Static));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = world.GetBody(bodyID);
    EXPECT_EQ(GetType(body), BodyType::Static);
    EXPECT_FALSE(IsSpeedable(body));
    EXPECT_FALSE(IsAccelerable(body));
    EXPECT_TRUE(IsImpenetrable(body));
    EXPECT_EQ(GetFixtures(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(world.GetBodies().size(), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);

    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});

    world.Destroy(bodyID);
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));

    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
}

TEST(WorldImpl, CreateDestroyEmptyDynamicBody)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto bodyID = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = world.GetBody(bodyID);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(body));
    EXPECT_TRUE(IsAccelerable(body));
    EXPECT_FALSE(IsImpenetrable(body));
    EXPECT_EQ(GetFixtures(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(world.GetBodies().size(), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
    
    world.Destroy(bodyID);
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
}

TEST(WorldImpl, CreateDestroyDynamicBodyAndFixture)
{
    // Created this test after receiving issue #306:
    //   Rapid create/destroy between step() causes SEGFAULT
    
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto bodyID = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = world.GetBody(bodyID);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(body));
    EXPECT_TRUE(IsAccelerable(body));
    EXPECT_FALSE(IsImpenetrable(body));
    EXPECT_EQ(GetFixtures(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(world.GetBodies().size(), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});

    const auto fixtureConf = FixtureConf{}.UseBody(bodyID).UseShape(DiskShapeConf{1_m});
    const auto fixture = CreateFixture(world, fixtureConf);
    ASSERT_NE(fixture, InvalidFixtureID);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(GetFixtures(world, bodyID).size(), std::size_t{1});
    ASSERT_EQ(world.GetFixturesForProxies().size(), std::size_t{1});
    EXPECT_EQ(*world.GetFixturesForProxies().begin(), fixture);

    world.Destroy(bodyID); // should clear fixtures for proxies!
    
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
}

TEST(WorldImpl, CreateDestroyContactingBodies)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), BodyCounter(0));
    ASSERT_EQ(world.GetJoints().size(), JointCounter(0));
    ASSERT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    ASSERT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(0));
    ASSERT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    auto contacts = world.GetContacts();
    ASSERT_TRUE(contacts.empty());
    ASSERT_EQ(contacts.size(), ContactCounter(0));

    const auto l1 = Length2{};
    const auto l2 = Length2{};

    const auto body1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    const auto body2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l2));
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(2));
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    EXPECT_NE(world.CreateFixture(FixtureConf{}.UseBody(body1).UseShape(DiskShapeConf{1_m}.UseDensity(1_kgpm2))),
              InvalidFixtureID);
    EXPECT_NE(world.CreateFixture(FixtureConf{}.UseBody(body2).UseShape(DiskShapeConf{1_m}.UseDensity(1_kgpm2))),
              InvalidFixtureID);
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(2));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    const auto stepConf = StepConf{};

    const auto stats0 = world.Step(stepConf);

    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(3));

    EXPECT_EQ(stats0.pre.proxiesMoved, static_cast<decltype(stats0.pre.proxiesMoved)>(0));
    EXPECT_EQ(stats0.pre.destroyed, static_cast<decltype(stats0.pre.destroyed)>(0));
    EXPECT_EQ(stats0.pre.added, static_cast<decltype(stats0.pre.added)>(1));
    EXPECT_EQ(stats0.pre.ignored, static_cast<decltype(stats0.pre.ignored)>(0));
    EXPECT_EQ(stats0.pre.updated, static_cast<decltype(stats0.pre.updated)>(1));
    EXPECT_EQ(stats0.pre.skipped, static_cast<decltype(stats0.pre.skipped)>(0));

    EXPECT_EQ(stats0.reg.minSeparation, -2.0_m);
    EXPECT_EQ(stats0.reg.maxIncImpulse, 0.0_Ns);
    EXPECT_EQ(stats0.reg.islandsFound, static_cast<decltype(stats0.reg.islandsFound)>(1));
    EXPECT_EQ(stats0.reg.islandsSolved, static_cast<decltype(stats0.reg.islandsSolved)>(0));
    EXPECT_EQ(stats0.reg.contactsAdded, static_cast<decltype(stats0.reg.contactsAdded)>(0));
    EXPECT_EQ(stats0.reg.bodiesSlept, static_cast<decltype(stats0.reg.bodiesSlept)>(0));
    EXPECT_EQ(stats0.reg.proxiesMoved, static_cast<decltype(stats0.reg.proxiesMoved)>(0));
    EXPECT_EQ(stats0.reg.sumPosIters, static_cast<decltype(stats0.reg.sumPosIters)>(3));
    EXPECT_EQ(stats0.reg.sumVelIters, static_cast<decltype(stats0.reg.sumVelIters)>(1));

    EXPECT_EQ(stats0.toi.minSeparation, std::numeric_limits<Length>::infinity());
    EXPECT_EQ(stats0.toi.maxIncImpulse, 0.0_Ns);
    EXPECT_EQ(stats0.toi.islandsFound, static_cast<decltype(stats0.toi.islandsFound)>(0));
    EXPECT_EQ(stats0.toi.islandsSolved, static_cast<decltype(stats0.toi.islandsSolved)>(0));
    EXPECT_EQ(stats0.toi.contactsFound, static_cast<decltype(stats0.toi.contactsFound)>(0));
    EXPECT_EQ(stats0.toi.contactsAtMaxSubSteps, static_cast<decltype(stats0.toi.contactsAtMaxSubSteps)>(0));
    EXPECT_EQ(stats0.toi.contactsUpdatedToi, static_cast<decltype(stats0.toi.contactsUpdatedToi)>(0));
    EXPECT_EQ(stats0.toi.contactsUpdatedTouching, static_cast<decltype(stats0.toi.contactsUpdatedTouching)>(0));
    EXPECT_EQ(stats0.toi.contactsSkippedTouching, static_cast<decltype(stats0.toi.contactsSkippedTouching)>(0));
    EXPECT_EQ(stats0.toi.contactsAdded, static_cast<decltype(stats0.toi.contactsAdded)>(0));
    EXPECT_EQ(stats0.toi.proxiesMoved, static_cast<decltype(stats0.toi.proxiesMoved)>(0));
    EXPECT_EQ(stats0.toi.sumPosIters, static_cast<decltype(stats0.toi.sumPosIters)>(0));
    EXPECT_EQ(stats0.toi.sumVelIters, static_cast<decltype(stats0.toi.sumVelIters)>(0));
    EXPECT_EQ(stats0.toi.maxSimulContacts, static_cast<decltype(stats0.toi.maxSimulContacts)>(0));
    EXPECT_EQ(stats0.toi.maxDistIters, static_cast<decltype(stats0.toi.maxDistIters)>(0));
    EXPECT_EQ(stats0.toi.maxToiIters, static_cast<decltype(stats0.toi.maxToiIters)>(0));
    EXPECT_EQ(stats0.toi.maxRootIters, static_cast<decltype(stats0.toi.maxRootIters)>(0));

    contacts = world.GetContacts();
    EXPECT_FALSE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(1));
    if (contacts.size() == 1u) {
        EXPECT_EQ(contacts.begin()->first.GetMin(),
                  static_cast<decltype(contacts.begin()->first.GetMin())>(0));
        EXPECT_EQ(contacts.begin()->first.GetMax(),
                  static_cast<decltype(contacts.begin()->first.GetMax())>(1));
        EXPECT_EQ(GetFixtureA(world, contacts.begin()->second),
                  *GetFixtures(world, body1).begin());
        EXPECT_EQ(GetFixtureB(world, contacts.begin()->second),
                  *GetFixtures(world, body2).begin());
    }

    world.Destroy(body1);
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(1));
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(1));

    world.Step(stepConf);
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(1));
    contacts = world.GetContacts();
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));

    world.Destroy(body2);
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));
    contacts = world.GetContacts();
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));
}

TEST(WorldImpl, SetTypeOfBody)
{
    auto world = WorldImpl{};
    const auto bodyID = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto& body = world.GetBody(bodyID);
    ASSERT_EQ(GetType(body), BodyType::Dynamic);
    auto other = WorldImpl{};
    EXPECT_THROW(SetBody(other, bodyID, body), std::out_of_range);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    auto body2 = body;
    SetType(body2, BodyType::Static);
    world.SetBody(bodyID, body2);
    EXPECT_EQ(GetType(world.GetBody(bodyID)), BodyType::Static);
}

TEST(WorldImpl, Proxies)
{
    const auto density = 2_kgpm2;
    const auto friction = Real(0.5);
    const auto restitution = Real(0.4);
    const auto isSensor = true;

    {
        const auto shape = Shape{
            DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density)
        };

        auto world = WorldImpl{};
        const auto body = CreateBody(world);
        const auto def = FixtureConf{}.UseBody(body).UseShape(shape).UseIsSensor(isSensor);
        const auto fixtureID = CreateFixture(world, def);
        const auto& fixture = world.GetFixture(fixtureID);

        ASSERT_EQ(GetBody(fixture), body);
        ASSERT_EQ(GetShape(fixture), shape);
        ASSERT_EQ(GetDensity(fixture), density);
        ASSERT_EQ(GetFriction(fixture), friction);
        ASSERT_EQ(GetRestitution(fixture), restitution);
        ASSERT_EQ(IsSensor(fixture), isSensor);
        ASSERT_EQ(world.GetProxies(fixtureID).size(), ChildCounter{0});

        const auto stepConf = StepConf{};
        Step(world, stepConf);
        EXPECT_EQ(world.GetProxies(fixtureID).size(), ChildCounter{1});
        EXPECT_EQ(GetProxy(world, fixtureID, 0), ContactCounter{0});
    }

    {
        const auto shape = Shape{
            ChainShapeConf{}.Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
        };

        auto world = WorldImpl{};
        const auto body = CreateBody(world);
        const auto def = FixtureConf{}.UseBody(body).UseShape(shape).UseIsSensor(isSensor);
        const auto fixtureID = CreateFixture(world, def);
        const auto& fixture = world.GetFixture(fixtureID);

        ASSERT_EQ(GetBody(fixture), body);
        ASSERT_EQ(GetShape(fixture), shape);
        ASSERT_EQ(IsSensor(fixture), isSensor);
        ASSERT_EQ(GetProxyCount(world, fixtureID), ChildCounter{0});

        const auto stepConf = StepConf{};
        Step(world, stepConf);
        EXPECT_EQ(GetProxyCount(world, fixtureID), ChildCounter{2});
        EXPECT_EQ(GetProxy(world, fixtureID, 0), ContactCounter{0});
        EXPECT_EQ(GetProxy(world, fixtureID, 1), ContactCounter{1});
    }

    {
        const auto shape = Shape{
            ChainShapeConf{}.Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
            .Add(Length2{0_m, +2_m}).Add(Length2{2_m, 2_m})
        };

        auto world = WorldImpl{};
        const auto body = CreateBody(world);
        const auto def = FixtureConf{}.UseBody(body).UseShape(shape).UseIsSensor(isSensor);
        const auto fixtureID = CreateFixture(world, def);
        const auto& fixture = world.GetFixture(fixtureID);

        ASSERT_EQ(GetBody(fixture), body);
        ASSERT_EQ(GetShape(fixture), shape);
        ASSERT_EQ(IsSensor(fixture), isSensor);
        ASSERT_EQ(GetProxyCount(world, fixtureID), ChildCounter{0});

        const auto stepConf = StepConf{};
        Step(world, stepConf);
        EXPECT_EQ(GetProxyCount(world, fixtureID), ChildCounter{4});
        EXPECT_EQ(GetProxy(world, fixtureID, 0), ContactCounter{0});
        EXPECT_EQ(GetProxy(world, fixtureID, 1), ContactCounter{1});
        EXPECT_EQ(GetProxy(world, fixtureID, 2), ContactCounter{3});
        EXPECT_EQ(GetProxy(world, fixtureID, 3), ContactCounter{5});
    }
}

TEST(WorldImpl, SetEnabledBody)
{
    auto stepConf = StepConf{};

    auto world = WorldImpl{};
    ASSERT_EQ(world.GetFixturesForProxies().size(), 0u);
    ASSERT_EQ(world.GetBodiesForProxies().size(), 0u);

    const auto body0 = CreateBody(world);
    const auto body1 = CreateBody(world);
    const auto valid_shape = Shape{DiskShapeConf(1_m)};

    const auto fixture0 = CreateFixture(world, FixtureConf{}.UseBody(body0).UseShape(valid_shape));
    const auto fixture1 = CreateFixture(world, FixtureConf{}.UseBody(body1).UseShape(valid_shape));
    ASSERT_NE(fixture0, InvalidFixtureID);
    ASSERT_NE(fixture1, InvalidFixtureID);

    ASSERT_TRUE(IsEnabled(world.GetBody(body0)));
    ASSERT_EQ(GetProxyCount(world, fixture0), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 2u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    // Test that set enabled to flag already set is not a toggle
    {
        auto copyBody0 = world.GetBody(body0);
        EXPECT_NO_THROW(SetEnabled(copyBody0, true));
        EXPECT_NO_THROW(world.SetBody(body0, copyBody0));
        EXPECT_TRUE(IsEnabled(world.GetBody(body0)));
    }
    {
        auto copyBody1 = world.GetBody(body1);
        EXPECT_NO_THROW(SetEnabled(copyBody1, false));
        EXPECT_NO_THROW(world.SetBody(body1, copyBody1));
        EXPECT_FALSE(IsEnabled(world.GetBody(body1)));
    }
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, false));
    EXPECT_FALSE(IsEnabled(world.GetBody(body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body1)));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 3u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, false));
    EXPECT_FALSE(IsEnabled(world.GetBody(body1)));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 5u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, false));
    EXPECT_FALSE(IsEnabled(world.GetBody(body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body1)));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 7u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(GetProxyCount(world, fixture0), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body0)));
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
}

TEST(WorldImpl, CreateAndDestroyFixture)
{
    auto world = WorldImpl{};

    auto body = CreateBody(world);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_TRUE(GetFixtures(world, body).empty());
    EXPECT_FALSE(IsMassDataDirty(world.GetBody(body)));

    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1_kgpm2;
    const auto shape = Shape(conf);

    {
        auto fixture = CreateFixture(world, FixtureConf{}.UseBody(body).UseShape(shape));
        const auto fshape = GetShape(world.GetFixture(fixture));
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(TypeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
        EXPECT_FALSE(GetFixtures(world, body).empty());
        {
            auto i = 0;
            for (const auto& f: GetFixtures(world, body))
            {
                EXPECT_EQ(f, fixture);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(IsMassDataDirty(world.GetBody(body)));

        ASSERT_EQ(GetFixturesForProxies(world).size(), std::size_t{1});
        EXPECT_EQ(*GetFixturesForProxies(world).begin(), fixture);

        EXPECT_TRUE(world.Destroy(fixture));
        EXPECT_TRUE(GetFixtures(world, body).empty());
        EXPECT_TRUE(IsMassDataDirty(world.GetBody(body)));

        EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t(0));
    }
    {
        auto fixture = CreateFixture(world, FixtureConf{}.UseBody(body).UseShape(shape));
        const auto fshape = GetShape(world.GetFixture(fixture));
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(TypeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
        EXPECT_FALSE(GetFixtures(world, body).empty());
        {
            auto i = 0;
            for (const auto& f: GetFixtures(world, body))
            {
                EXPECT_EQ(f, fixture);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(IsMassDataDirty(world.GetBody(body)));
        EXPECT_FALSE(GetFixtures(world, body).empty());
    }
}

TEST(WorldImpl, SetTypeBody)
{
    auto world = WorldImpl{};

    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);
    ASSERT_EQ(GetType(world.GetBody(body)), BodyType::Dynamic);

    SetType(world, body, BodyType::Static);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    EXPECT_EQ(GetType(world.GetBody(body)), BodyType::Static);

    SetType(world, body, BodyType::Kinematic);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    EXPECT_EQ(GetType(world.GetBody(body)), BodyType::Kinematic);

    SetType(world, body, BodyType::Dynamic);
    EXPECT_EQ(GetType(world.GetBody(body)), BodyType::Dynamic);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
}
