/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Body.hpp> // for GetBody
#include <PlayRho/Contact.hpp>
#include <PlayRho/Dynamics/ContactImpulsesList.hpp>

#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Collision/Shapes/ChainShapeConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/DynamicTree.hpp> // for GetTree
#include <PlayRho/Collision/RayCastInput.hpp>
#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Collision/Manifold.hpp>

#include <PlayRho/LengthError.hpp>
#include <PlayRho/WrongState.hpp>

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
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
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
    }
    {
        const auto& w = static_cast<const WorldImpl&>(world);
        const auto& bodies = w.GetBodies();
        EXPECT_TRUE(bodies.empty());
        EXPECT_EQ(bodies.size(), BodyCounter(0));
        EXPECT_EQ(bodies.begin(), bodies.end());
    }

    EXPECT_TRUE(world.GetContacts().empty());
    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));

    EXPECT_TRUE(world.GetJoints().empty());
    EXPECT_EQ(world.GetJoints().size(), JointCounter(0));

    EXPECT_FALSE(world.GetSubStepping());
    EXPECT_FALSE(world.IsLocked());
}

TEST(WorldImpl, Init)
{
    WorldImpl world{};
    EXPECT_FALSE(world.IsLocked());
    {
        auto calls = 0;
        Query(world.GetTree(), AABB{}, [&](BodyID, ShapeID, ChildCounter) {
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
    auto shapeListener = PushBackListener<ShapeID>{};
    auto associationListener = PushBackListener<std::pair<BodyID, ShapeID>>{};

    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), std::size_t(0));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(0));

    world.SetJointDestructionListener(std::ref(jointListener));
    world.SetShapeDestructionListener(std::ref(shapeListener));
    world.SetDetachListener(std::ref(associationListener));

    const auto shapeId0 = world.CreateShape(Shape(DiskShapeConf{}));
    const auto b0 = CreateBody(world);
    ASSERT_NE(b0, InvalidBodyID);
    ASSERT_NO_THROW(Attach(world, b0, shapeId0));
    ASSERT_EQ(GetShapes(world, b0).size(), std::size_t(1));;

    const auto b1 = CreateBody(world);
    ASSERT_NE(b1, InvalidBodyID);
    ASSERT_NO_THROW(Attach(world, b1, shapeId0));
    ASSERT_EQ(GetShapes(world, b1).size(), std::size_t(1));;

    const auto j0 = world.CreateJoint(Joint{DistanceJointConf{b0, b1}});
    ASSERT_NE(j0, InvalidJointID);
    ASSERT_EQ(j0, JointID{0u});
    ASSERT_FALSE(world.IsDestroyed(JointID{0u}));

    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(1));
    ASSERT_EQ(world.GetJointRange(), 1u);

    EXPECT_NO_THROW(world.Clear());

    EXPECT_EQ(world.GetBodies().size(), std::size_t(0));
    EXPECT_EQ(world.GetJoints().size(), std::size_t(0));
    EXPECT_EQ(world.GetJointRange(), 0u);
    EXPECT_FALSE(world.IsDestroyed(JointID{0u})); // out-of-range so not destroyed

    EXPECT_EQ(shapeListener.ids.size(), std::size_t(1));

    ASSERT_EQ(associationListener.ids.size(), std::size_t(0));

    ASSERT_EQ(jointListener.ids.size(), std::size_t(1));
    EXPECT_EQ(jointListener.ids.at(0), j0);

    const auto shapeId1 = world.CreateShape(Shape(DiskShapeConf{}));
    const auto b2 = CreateBody(world);
    EXPECT_LE(b2, b1);
    ASSERT_NO_THROW(Attach(world, b2, shapeId1));
}

TEST(WorldImpl, CreateDestroyEmptyStaticBody)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), BodyCounter(0));
    const auto bodyID = CreateBody(world, BodyConf{}.UseType(BodyType::Static));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = world.GetBody(bodyID);
    EXPECT_EQ(GetType(body), BodyType::Static);
    EXPECT_FALSE(IsSpeedable(body));
    EXPECT_FALSE(IsAccelerable(body));
    EXPECT_TRUE(IsImpenetrable(body));
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{0});

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
    const auto bodyID = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = world.GetBody(bodyID);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(body));
    EXPECT_TRUE(IsAccelerable(body));
    EXPECT_FALSE(IsImpenetrable(body));
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{0});

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
    const auto bodyID = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = world.GetBody(bodyID);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(body));
    EXPECT_TRUE(IsAccelerable(body));
    EXPECT_FALSE(IsImpenetrable(body));
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(world.GetBodies().size(), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});

    const auto shapeId = world.CreateShape(Shape(DiskShapeConf{1_m}));
    ASSERT_NO_THROW(Attach(world, bodyID, shapeId));
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{1});
    ASSERT_EQ(world.GetFixturesForProxies().size(), std::size_t{1});
    EXPECT_EQ(*world.GetFixturesForProxies().begin(), std::make_pair(bodyID, shapeId));

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

    const auto body1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    const auto body2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l2));
    EXPECT_EQ(world.GetBodies().size(), BodyCounter(2));
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    const auto shapeId = world.CreateShape(Shape(DiskShapeConf{1_m}.UseDensity(1_kgpm2)));
    ASSERT_NO_THROW(Attach(world, body1, shapeId));
    ASSERT_NO_THROW(Attach(world, body2, shapeId));
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), static_cast<decltype(world.GetFixturesForProxies().size())>(2));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));
    ASSERT_EQ(GetShapes(world, body1).size(), 1u);
    ASSERT_EQ(GetShapes(world, body2).size(), 1u);

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
        EXPECT_EQ(to_underlying(contacts.begin()->second), 0u);
        EXPECT_EQ(GetShapeA(world.GetContact(contacts.begin()->second)),
                  *GetShapes(world, body1).begin());
        EXPECT_EQ(GetShapeB(world.GetContact(contacts.begin()->second)),
                  *GetShapes(world, body2).begin());
        EXPECT_EQ(world.GetContactRange(), 1u);
        EXPECT_FALSE(world.IsDestroyed(ContactID{0u}));
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
    EXPECT_TRUE(world.IsDestroyed(ContactID{0u}));

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
    const auto bodyID = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto& body = world.GetBody(bodyID);
    ASSERT_EQ(GetType(body), BodyType::Dynamic);
    auto other = WorldImpl{};
    EXPECT_THROW(SetBody(other, bodyID, body), std::out_of_range);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    auto body2 = body;
    SetType(body2, BodyType::Static);
    EXPECT_NO_THROW(world.SetBody(bodyID, body2));
    EXPECT_EQ(GetType(world.GetBody(bodyID)), BodyType::Static);
}

TEST(WorldImpl, Proxies)
{
    constexpr auto density = 2_kgpm2;
    constexpr auto friction = Real(0.5);
    constexpr auto restitution = Real(0.4);
    constexpr auto isSensor = true;

    {
        auto world = WorldImpl{};
        const auto shapeId = world.CreateShape(
            Shape(DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density).UseIsSensor(isSensor))
        );
        const auto body = CreateBody(world);
        ASSERT_NO_THROW(Attach(world, body, shapeId));
        ASSERT_EQ(std::size(GetShapes(world, body)), std::size_t(1));
        ASSERT_EQ(GetShapes(world, body).at(0), shapeId);

        const auto& shape = world.GetShape(shapeId);
        ASSERT_EQ(GetDensity(shape), density);
        ASSERT_EQ(GetFriction(shape), friction);
        ASSERT_EQ(GetRestitution(shape), restitution);
        ASSERT_EQ(IsSensor(shape), isSensor);

        auto psize = std::size_t(0u);
        ASSERT_NO_THROW(psize = world.GetProxies(BodyID(0)).size());
        EXPECT_EQ(psize, 0u);
        ASSERT_EQ(world.GetFixturesForProxies().size(), 1u);
        EXPECT_EQ(*world.GetFixturesForProxies().begin(), std::make_pair(body, shapeId));

        const auto stepConf = StepConf{};
        ASSERT_NO_THROW(Step(world, stepConf));
        ASSERT_EQ(world.GetProxies(body).size(), 1u);
        EXPECT_EQ(world.GetProxies(body)[0], 0u);
    }

    {
        const auto shape = Shape{
            ChainShapeConf{}.UseIsSensor(isSensor).Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
        };

        auto world = WorldImpl{};
        const auto shapeId = world.CreateShape(shape);
        const auto body = CreateBody(world);
        ASSERT_NO_THROW(Attach(world, body, shapeId));

        ASSERT_EQ(std::size(GetShapes(world, body)), 1u);
        ASSERT_EQ(GetShapes(world, body).at(0), shapeId);
        ASSERT_EQ(IsSensor(shape), isSensor);
        ASSERT_EQ(world.GetProxies(body).size(), 0u);

        const auto stepConf = StepConf{};
        ASSERT_NO_THROW(Step(world, stepConf));
        ASSERT_EQ(world.GetProxies(body).size(), 2u);
        EXPECT_EQ(world.GetProxies(body)[0], 0u);
        EXPECT_EQ(world.GetProxies(body)[1], 1u);
    }

    {
        const auto shape = Shape{
            ChainShapeConf{}.UseIsSensor(isSensor).Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
            .Add(Length2{0_m, +2_m}).Add(Length2{2_m, 2_m})
        };

        auto world = WorldImpl{};
        const auto shapeId = world.CreateShape(shape);
        const auto body = CreateBody(world);
        ASSERT_NO_THROW(Attach(world, body, shapeId));

        ASSERT_EQ(IsSensor(shape), isSensor);
        ASSERT_EQ(world.GetProxies(body).size(), 0u);

        const auto stepConf = StepConf{};
        ASSERT_NO_THROW(Step(world, stepConf));
        ASSERT_EQ(world.GetProxies(body).size(), 4u);
        EXPECT_EQ(world.GetProxies(body)[0], 0u);
        EXPECT_EQ(world.GetProxies(body)[1], 1u);
        EXPECT_EQ(world.GetProxies(body)[2], 3u);
        EXPECT_EQ(world.GetProxies(body)[3], 5u);
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
    const auto shapeId = world.CreateShape(valid_shape);

    ASSERT_NO_THROW(Attach(world, body0, shapeId));
    ASSERT_NO_THROW(Attach(world, body1, shapeId));

    ASSERT_TRUE(IsEnabled(world.GetBody(body0)));
    ASSERT_EQ(world.GetProxies(body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 2u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(world.GetProxies(body0).size(), 1u);
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
    EXPECT_EQ(world.GetProxies(body0).size(), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, false));
    EXPECT_FALSE(IsEnabled(world.GetBody(body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body1)));
    EXPECT_EQ(world.GetProxies(body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, false));
    EXPECT_FALSE(IsEnabled(world.GetBody(body1)));
    EXPECT_EQ(world.GetProxies(body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, false));
    EXPECT_FALSE(IsEnabled(world.GetBody(body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body1)));
    EXPECT_EQ(world.GetProxies(body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(world.GetProxies(body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, true));
    EXPECT_TRUE(IsEnabled(world.GetBody(body0)));
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(world.GetProxies(body0).size(), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
}

TEST(WorldImpl, AttachAndDetachShape)
{
    auto world = WorldImpl{};

    auto body = CreateBody(world);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_TRUE(GetShapes(world, body).empty());
    EXPECT_FALSE(IsMassDataDirty(world.GetBody(body)));

    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1_kgpm2;
    const auto shape = Shape(conf);
    const auto shapeId = world.CreateShape(shape);

    {
        Attach(world, body, shapeId);
        const auto& fshape = world.GetShape(shapeId);
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(TypeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
        EXPECT_FALSE(GetShapes(world, body).empty());
        {
            auto i = 0;
            for (const auto& f: GetShapes(world, body))
            {
                EXPECT_EQ(f, shapeId);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(IsMassDataDirty(world.GetBody(body)));

        ASSERT_EQ(GetFixturesForProxies(world).size(), std::size_t{1});
        EXPECT_EQ(*GetFixturesForProxies(world).begin(), std::make_pair(body, shapeId));

        EXPECT_TRUE(Detach(world, body, shapeId));
        EXPECT_FALSE(Detach(world, body, shapeId));
        EXPECT_TRUE(GetShapes(world, body).empty());
        EXPECT_TRUE(IsMassDataDirty(world.GetBody(body)));

        EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t(0));
    }
    {
        Attach(world, body, shapeId);
        const auto& fshape = world.GetShape(shapeId);
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(TypeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
        EXPECT_FALSE(GetShapes(world, body).empty());
        {
            auto i = 0;
            for (const auto& f: GetShapes(world, body))
            {
                EXPECT_EQ(f, shapeId);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(IsMassDataDirty(world.GetBody(body)));
        EXPECT_FALSE(GetShapes(world, body).empty());
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

TEST(WorldImpl, ThrowsLengthErrorOnMaxShapes)
{
    auto world = WorldImpl{};
    const auto shape = Shape{DiskShapeConf{}};
    for (auto i = ShapeCounter{0u}; i < MaxShapes; ++i) {
        EXPECT_NO_THROW(world.CreateShape(shape));
    }
    EXPECT_THROW(world.CreateShape(shape), LengthError);
}

TEST(WorldImpl, GetBodyRange)
{
    auto world = WorldImpl{};
    EXPECT_EQ(world.GetBodyRange(), BodyCounter{0u});
    EXPECT_EQ(world.GetBodies().size(), 0u);
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    EXPECT_EQ(world.GetBodyRange(), BodyCounter{1u});
    EXPECT_EQ(world.GetBodies().size(), 1u);
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    EXPECT_EQ(world.GetBodyRange(), BodyCounter{2u});
    EXPECT_EQ(world.GetBodies().size(), 2u);
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    EXPECT_EQ(world.GetBodyRange(), BodyCounter{3u});
    EXPECT_EQ(world.GetBodies().size(), 3u);
    EXPECT_NO_THROW(Destroy(world, BodyID{0u}));
    EXPECT_EQ(world.GetBodyRange(), BodyCounter{3u});
    EXPECT_EQ(world.GetBodies().size(), 2u);
    EXPECT_NO_THROW(Destroy(world, BodyID{1u}));
    EXPECT_EQ(world.GetBodyRange(), BodyCounter{3u});
    EXPECT_EQ(world.GetBodies().size(), 1u);
}

TEST(WorldImpl, GetShapeRange)
{
    const auto shape = Shape{DiskShapeConf{}};
    auto world = WorldImpl{};
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{0u});
    const auto shapeId = world.CreateShape(shape);
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{1u});
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 0u);
    EXPECT_NO_THROW(Attach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 1u);
    EXPECT_NO_THROW(Attach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 2u);
    EXPECT_NO_THROW(Detach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 1u);
    EXPECT_NO_THROW(Detach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 0u);
    EXPECT_NO_THROW(world.Destroy(shapeId));
    EXPECT_EQ(world.GetShapeRange(), ShapeCounter{1u});
}

TEST(WorldImpl, GetJointRange)
{
    auto world = WorldImpl{};
    EXPECT_EQ(world.GetJointRange(), JointCounter{0u});
}

TEST(WorldImpl, GetContactRange)
{
    auto world = WorldImpl{};
    EXPECT_EQ(world.GetContactRange(), ContactCounter{0u});
}

TEST(WorldImpl, IsDestroyedBody)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodies().size(), 0u);
    EXPECT_FALSE(world.IsDestroyed(BodyID{0u}));

    auto id = InvalidBodyID;
    ASSERT_NO_THROW(id = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    ASSERT_EQ(to_underlying(id), 0u);
    ASSERT_EQ(world.GetBodies().size(), 1u);
    EXPECT_FALSE(world.IsDestroyed(id));

    ASSERT_NO_THROW(id = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    ASSERT_EQ(to_underlying(id), 1u);
    ASSERT_EQ(world.GetBodies().size(), 2u);
    EXPECT_FALSE(world.IsDestroyed(id));

    ASSERT_NO_THROW(Destroy(world, BodyID{0u}));
    EXPECT_TRUE(world.IsDestroyed(BodyID{0u}));
    EXPECT_FALSE(world.IsDestroyed(BodyID{1u}));
    ASSERT_NO_THROW(Destroy(world, BodyID{1u}));
    EXPECT_TRUE(world.IsDestroyed(BodyID{0u}));
    EXPECT_TRUE(world.IsDestroyed(BodyID{1u}));
}

TEST(WorldImpl, AttachDetach)
{
    const auto shape = Shape{DiskShapeConf{}};
    auto world = WorldImpl{};
    const auto shapeId = world.CreateShape(shape);
    ASSERT_NO_THROW(CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic)));
    ASSERT_NO_THROW(Attach(world, BodyID{0u}, shapeId));
    ASSERT_EQ(GetShapes(world, BodyID{0}).size(), 1u);
    ASSERT_EQ(GetShapes(world, BodyID{0})[0], shapeId);

    ASSERT_NO_THROW(Attach(world, BodyID{0u}, shapeId));
    ASSERT_EQ(GetShapes(world, BodyID{0}).size(), 2u);
    ASSERT_EQ(GetShapes(world, BodyID{0})[0], shapeId);
    ASSERT_EQ(GetShapes(world, BodyID{0})[1], shapeId);

    ASSERT_NO_THROW(Detach(world, BodyID{0u}, shapeId));
    ASSERT_EQ(GetShapes(world, BodyID{0}).size(), 1u);
    ASSERT_EQ(GetShapes(world, BodyID{0})[0], shapeId);

    ASSERT_NO_THROW(Detach(world, BodyID{0u}, shapeId));
    ASSERT_EQ(GetShapes(world, BodyID{0}).size(), 0u);
}

TEST(WorldImpl, SetShapeThrowsWithOutOfRangeID)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetShapeRange(), 0u);
    EXPECT_THROW(world.SetShape(ShapeID(0), Shape{}), std::out_of_range);
}

TEST(WorldImpl, CreateBodyThrowsWithOutOfRangeShapeID)
{
    auto world = WorldImpl{};
    auto body = Body{};
    ASSERT_NO_THROW(body.Attach(ShapeID(0)));
    EXPECT_THROW(world.CreateBody(body), std::out_of_range);
}

TEST(WorldImpl, CreateBodyWithInRangeShapeIDs)
{
    auto world = WorldImpl{};

    ASSERT_EQ(world.GetShapeRange(), 0u);
    auto shapeId0 = InvalidShapeID;
    auto shapeId1 = InvalidShapeID;
    ASSERT_NO_THROW(shapeId0 = world.CreateShape(Shape(DiskShapeConf{})));
    ASSERT_NO_THROW(shapeId1 = world.CreateShape(Shape(DiskShapeConf{})));
    ASSERT_EQ(world.GetShapeRange(), 2u);

    auto body = Body{};
    ASSERT_EQ(size(body.GetShapes()), 0u);
    ASSERT_NO_THROW(body.Attach(shapeId0));
    ASSERT_EQ(size(body.GetShapes()), 1u);
    ASSERT_NO_THROW(body.Attach(shapeId1));
    ASSERT_EQ(size(body.GetShapes()), 2u);

    ASSERT_EQ(world.GetBodyRange(), 0u);
    auto bodyId = InvalidBodyID;
    EXPECT_NO_THROW(bodyId = world.CreateBody(body));
    EXPECT_EQ(world.GetBodyRange(), 1u);
    ASSERT_EQ(size(world.GetBody(bodyId).GetShapes()), 2u);
    ASSERT_EQ(world.GetBody(bodyId).GetShapes()[0], shapeId0);
    ASSERT_EQ(world.GetBody(bodyId).GetShapes()[1], shapeId1);
    ASSERT_EQ(size(world.GetFixturesForProxies()), 2u);
    EXPECT_EQ(*begin(world.GetFixturesForProxies()), std::make_pair(bodyId, shapeId0));
    EXPECT_EQ(*(begin(world.GetFixturesForProxies()) + 1), std::make_pair(bodyId, shapeId1));
    EXPECT_EQ(size(world.GetProxies(bodyId)), 0u);

    EXPECT_NO_THROW(world.Step(StepConf{}));
    EXPECT_EQ(size(world.GetFixturesForProxies()), 0u);
    EXPECT_EQ(size(world.GetProxies(bodyId)), 2u);
}

TEST(WorldImpl, SetBodyThrowsWithOutOfRangeID)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodyRange(), 0u);
    EXPECT_THROW(world.SetBody(BodyID(0), Body{}), std::out_of_range);
}

TEST(WorldImpl, SetBodyThrowsWithOutOfRangeShapeID)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetBodyRange(), 0u);
    ASSERT_NO_THROW(world.CreateBody(Body()));
    ASSERT_EQ(world.GetBodyRange(), 1u);
    auto body = Body{};
    ASSERT_NO_THROW(world.SetBody(BodyID(0), body));
    ASSERT_NO_THROW(body.Attach(ShapeID(0)));
    EXPECT_THROW(world.SetBody(BodyID(0), body), std::out_of_range);
}

TEST(WorldImpl, SetShapeWithGeometryChange)
{
    const auto stepConf = StepConf{};
    auto bodyId = InvalidBodyID;
    auto shapeId = InvalidShapeID;
    auto shapeIdOther = InvalidShapeID;
    auto world = WorldImpl{};
    auto diskShapeConf = DiskShapeConf{};
    ASSERT_EQ(GetChildCount(diskShapeConf), 1u);
    ASSERT_NO_THROW(shapeId = world.CreateShape(Shape{diskShapeConf}));
    ASSERT_NO_THROW(shapeIdOther = world.CreateShape(Shape{diskShapeConf}));
    auto body = Body{BodyConf{}.Use(BodyType::Dynamic).Use(shapeId)};
    ASSERT_NO_THROW(body.Attach(shapeIdOther)); // to also cover the false match path
    ASSERT_NE(shapeId, shapeIdOther);
    ASSERT_NO_THROW(bodyId = world.CreateBody(body));
    ASSERT_TRUE(IsEnabled(world.GetBody(bodyId)));
    EXPECT_EQ(size(world.GetFixturesForProxies()), 2u);
    ASSERT_NO_THROW(world.Step(stepConf));
    EXPECT_EQ(size(world.GetProxies(bodyId)), 2u);
    auto chainShapeConf = ChainShapeConf{};
    chainShapeConf.Add(Length2{0_m, 0_m});
    chainShapeConf.Add(Length2{2_m, 0_m});
    chainShapeConf.Add(Length2{2_m, 1_m});
    ASSERT_EQ(GetChildCount(chainShapeConf), 2u); // 2 kids here means 2 proxies get made!
    EXPECT_NO_THROW(world.SetShape(shapeId, Shape{chainShapeConf})); // replaces 1 proxy w/ 2
    EXPECT_EQ(size(world.GetFixturesForProxies()), 1u);
    if (!empty(world.GetFixturesForProxies())) {
        EXPECT_EQ(world.GetFixturesForProxies()[0].first, bodyId);
        EXPECT_EQ(world.GetFixturesForProxies()[0].second, shapeId);
    }
    EXPECT_EQ(size(world.GetProxies(bodyId)), 1u);
    EXPECT_NO_THROW(world.Step(stepConf)); // makes 1 proxy for shapeIdOther + 2 for shapeId
    EXPECT_EQ(size(world.GetFixturesForProxies()), 0u);
    EXPECT_EQ(size(world.GetProxies(bodyId)), 3u);
}

TEST(WorldImpl, SetFreedShapeThrows)
{
    auto world = WorldImpl{};
    auto id = InvalidShapeID;
    ASSERT_NO_THROW(id = world.CreateShape(Shape()));
    ASSERT_NO_THROW(world.Destroy(id));
    EXPECT_THROW(world.SetShape(id, Shape()), InvalidArgument);
}

TEST(WorldImpl, SetFreedBodyThrows)
{
    auto world = WorldImpl{};
    auto id = InvalidBodyID;
    ASSERT_NO_THROW(id = world.CreateBody(Body()));
    ASSERT_NO_THROW(world.Destroy(id));
    EXPECT_THROW(world.SetBody(id, Body()), InvalidArgument);
}

TEST(WorldImpl, SetFreedJointThrows)
{
    auto world = WorldImpl{};
    auto id = InvalidJointID;
    ASSERT_NO_THROW(id = world.CreateJoint(Joint()));
    ASSERT_NO_THROW(world.Destroy(id));
    EXPECT_THROW(world.SetJoint(id, Joint()), InvalidArgument);
}

TEST(WorldImpl, SetBodyWithShapeID)
{
    auto world = WorldImpl{};

    ASSERT_EQ(world.GetShapeRange(), 0u);
    auto shapeId = InvalidShapeID;
    ASSERT_NO_THROW(shapeId = world.CreateShape(Shape(DiskShapeConf{})));
    ASSERT_EQ(world.GetShapeRange(), 1u);

    ASSERT_EQ(world.GetBodyRange(), 0u);
    auto bodyId = InvalidBodyID;
    ASSERT_NO_THROW(bodyId = world.CreateBody(Body()));
    ASSERT_EQ(world.GetBodyRange(), 1u);
    ASSERT_EQ(size(world.GetBody(bodyId).GetShapes()), 0u);

    auto body = Body{};
    ASSERT_EQ(size(body.GetShapes()), 0u);

    ASSERT_NO_THROW(body.Attach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 1u);
    EXPECT_NO_THROW(world.SetBody(bodyId, body));
    EXPECT_EQ(size(world.GetBody(bodyId).GetShapes()), 1u);
    EXPECT_EQ(size(world.GetFixturesForProxies()), 1u);

    ASSERT_NO_THROW(body.Detach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 0u);
    EXPECT_NO_THROW(world.SetBody(bodyId, body));
    EXPECT_EQ(size(world.GetBody(bodyId).GetShapes()), 0u);
    EXPECT_EQ(size(world.GetFixturesForProxies()), 0u);

    ASSERT_NO_THROW(body.Attach(shapeId));
    ASSERT_NO_THROW(body.Attach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 2u);
    EXPECT_NO_THROW(world.SetBody(bodyId, body));
    EXPECT_EQ(size(world.GetBody(bodyId).GetShapes()), 2u);
    EXPECT_EQ(size(world.GetFixturesForProxies()), 2u);

    ASSERT_NO_THROW(body.Detach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 1u);
    EXPECT_NO_THROW(world.SetBody(bodyId, body));
    EXPECT_EQ(size(world.GetBody(bodyId).GetShapes()), 1u);
    // Detaching the shape currently gets rid of all attachments to the body of that shape...
    EXPECT_EQ(size(world.GetFixturesForProxies()), 0u);
}

TEST(WorldImpl, CreateJointThrowsWithOutOfRangeBodyID)
{
    auto world = WorldImpl{};
    auto joint = Joint(FrictionJointConf{}.UseBodyA(BodyID(0)));
    EXPECT_THROW(world.CreateJoint(joint), std::out_of_range);
}

TEST(WorldImpl, SetJointThrowsWithOutOfRangeID)
{
    auto world = WorldImpl{};
    ASSERT_EQ(world.GetJointRange(), 0u);
    auto joint = Joint(FrictionJointConf{}.UseBodyA(BodyID(0)));
    EXPECT_THROW(world.SetJoint(JointID(0), joint), std::out_of_range);
}

TEST(WorldImpl, SetJointThrowsWithOutOfRangeBodyID)
{
    const auto b0 = BodyID(0);
    const auto b1 = BodyID(1);
    const auto j0 = JointID(0);
    auto world = WorldImpl{};
    ASSERT_NO_THROW(world.CreateBody(Body()));
    ASSERT_EQ(world.GetBodyRange(), 1u);
    ASSERT_EQ(world.GetJointRange(), 0u);
    ASSERT_NO_THROW(world.CreateJoint(Joint(FrictionJointConf{}.UseBodyA(b0).UseBodyB(b0))));
    ASSERT_EQ(world.GetJointRange(), 1u);
    ASSERT_NO_THROW(world.SetJoint(j0, Joint(FrictionJointConf{}.UseBodyA(b0).UseBodyB(b0))));
    EXPECT_THROW(world.SetJoint(j0, Joint(FrictionJointConf{}.UseBodyA(b1).UseBodyB(b0))),
                 std::out_of_range);
    EXPECT_THROW(world.SetJoint(j0, Joint(FrictionJointConf{}.UseBodyA(b0).UseBodyB(b1))),
                 std::out_of_range);
}
