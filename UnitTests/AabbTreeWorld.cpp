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

#include <type_traits>

#include <playrho/Contact.hpp>
#include <playrho/LengthError.hpp>
#include <playrho/OutOfRange.hpp>
#include <playrho/StepConf.hpp>
#include <playrho/to_underlying.hpp>
#include <playrho/WrongState.hpp>

#include <playrho/d2/Body.hpp> // for GetBody
#include <playrho/d2/BodyConf.hpp>
#include <playrho/d2/ContactImpulsesList.hpp>
#include <playrho/d2/AabbTreeWorld.hpp>
#include <playrho/d2/Shape.hpp>
#include <playrho/d2/ChainShapeConf.hpp>
#include <playrho/d2/DiskShapeConf.hpp>
#include <playrho/d2/PolygonShapeConf.hpp>
#include <playrho/d2/EdgeShapeConf.hpp>
#include <playrho/d2/DynamicTree.hpp> // for GetTree
#include <playrho/d2/RayCastInput.hpp>
#include <playrho/d2/RayCastOutput.hpp>
#include <playrho/d2/Manifold.hpp>
#include <playrho/d2/Joint.hpp>
#include <playrho/d2/TargetJointConf.hpp>
#include <playrho/d2/RopeJointConf.hpp>
#include <playrho/d2/RevoluteJointConf.hpp>
#include <playrho/d2/PrismaticJointConf.hpp>
#include <playrho/d2/DistanceJointConf.hpp>
#include <playrho/d2/PulleyJointConf.hpp>
#include <playrho/d2/WeldJointConf.hpp>
#include <playrho/d2/FrictionJointConf.hpp>
#include <playrho/d2/MotorJointConf.hpp>
#include <playrho/d2/WheelJointConf.hpp>
#include <playrho/d2/GearJointConf.hpp>
#include <playrho/d2/World.hpp>

#include "gtest/gtest.h"

using namespace playrho;
using namespace playrho::d2;

namespace {

template <typename T, class Exception = void>
struct PushBackListener
{
    std::vector<T> ids;
    void operator()(T id)
    {
        ids.push_back(id);
        if constexpr (!std::is_same_v<void, Exception>) {
            throw Exception{"PushBackListener"};
        }
    }
};

void SetEnabled(AabbTreeWorld& world, BodyID id, bool value)
{
    auto copy = GetBody(world, id);
    SetEnabled(copy, value);
    SetBody(world, id, copy);
}

void SetType(AabbTreeWorld& world, BodyID id, BodyType value)
{
    auto body = GetBody(world, id);
    SetType(body, value);
    SetBody(world, id, body);
}

} // namespace

TEST(AabbTreeWorld, DefaultInit)
{
    const AabbTreeWorld world;

    EXPECT_EQ(GetBodies(world).size(), BodyCounter(0));
    EXPECT_EQ(GetTree(world).GetLeafCount(), ContactCounter(0));
    EXPECT_EQ(GetJoints(world).size(), JointCounter(0));
    EXPECT_EQ(GetContacts(world).size(), ContactCounter(0));
    EXPECT_EQ(GetHeight(GetTree(world)), ContactCounter(0));
    EXPECT_EQ(ComputePerimeterRatio(GetTree(world)), Real(0));

    {
        const auto& bodies = GetBodies(world);
        EXPECT_TRUE(bodies.empty());
        EXPECT_EQ(bodies.size(), BodyCounter(0));
        EXPECT_EQ(bodies.begin(), bodies.end());
    }
    {
        const auto& w = static_cast<const AabbTreeWorld&>(world);
        const auto& bodies = GetBodies(w);
        EXPECT_TRUE(bodies.empty());
        EXPECT_EQ(bodies.size(), BodyCounter(0));
        EXPECT_EQ(bodies.begin(), bodies.end());
    }

    EXPECT_TRUE(GetContacts(world).empty());
    EXPECT_EQ(GetContacts(world).size(), ContactCounter(0));

    EXPECT_TRUE(GetJoints(world).empty());
    EXPECT_EQ(GetJoints(world).size(), JointCounter(0));

    EXPECT_FALSE(GetSubStepping(world));
    EXPECT_FALSE(IsLocked(world));

    const auto stats = GetResourceStats(world);
    EXPECT_FALSE(stats.has_value());

    EXPECT_TRUE(world == world);
    EXPECT_FALSE(world != world);
}

TEST(AabbTreeWorld, Equality)
{
    EXPECT_TRUE(AabbTreeWorld() == AabbTreeWorld());
    {
        AabbTreeWorld world{};
        EXPECT_TRUE(AabbTreeWorld() == world);
        const auto shapeId = CreateShape(world, Shape(DiskShapeConf{}));
        EXPECT_FALSE(AabbTreeWorld() == world);
        EXPECT_NO_THROW(Destroy(world, shapeId));
        EXPECT_TRUE(AabbTreeWorld() == world);
    }
}

TEST(AabbTreeWorld, Inequality)
{
    EXPECT_FALSE(AabbTreeWorld() != AabbTreeWorld());
    {
        AabbTreeWorld world{};
        EXPECT_FALSE(AabbTreeWorld() != world);
        const auto shapeId = CreateShape(world, Shape(DiskShapeConf{}));
        EXPECT_TRUE(AabbTreeWorld() != world);
        EXPECT_NO_THROW(Destroy(world, shapeId));
        EXPECT_FALSE(AabbTreeWorld() != world);
    }
}

TEST(AabbTreeWorld, Init)
{
    AabbTreeWorld world{};
    EXPECT_FALSE(IsLocked(world));
    {
        auto calls = 0;
        Query(GetTree(world), AABB{}, [&](BodyID, ShapeID, ChildCounter) {
            ++calls;
            return true;
        });
        EXPECT_EQ(calls, 0);
    }
}

TEST(AabbTreeWorld, Clear)
{
    auto jointListener = PushBackListener<JointID, InvalidArgument>{};
    auto shapeListener = PushBackListener<ShapeID, InvalidArgument>{};
    auto associationListener = PushBackListener<std::pair<BodyID, ShapeID>>{};

    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodies(world).size(), std::size_t(0));
    ASSERT_EQ(GetJoints(world).size(), std::size_t(0));

    SetJointDestructionListener(world, std::ref(jointListener));
    SetShapeDestructionListener(world, std::ref(shapeListener));
    SetDetachListener(world, std::ref(associationListener));

    const auto shapeId0 = CreateShape(world, Shape(DiskShapeConf{}));
    const auto b0 = CreateBody(world);
    ASSERT_NE(b0, InvalidBodyID);
    ASSERT_NO_THROW(Attach(world, b0, shapeId0));
    ASSERT_EQ(GetShapes(world, b0).size(), std::size_t(1));;

    const auto b1 = CreateBody(world);
    ASSERT_NE(b1, InvalidBodyID);
    ASSERT_NO_THROW(Attach(world, b1, shapeId0));
    ASSERT_EQ(GetShapes(world, b1).size(), std::size_t(1));;

    const auto j0 = CreateJoint(world, Joint{DistanceJointConf{b0, b1}});
    ASSERT_NE(j0, InvalidJointID);
    ASSERT_EQ(j0, JointID{0u});
    ASSERT_FALSE(IsDestroyed(world, JointID{0u}));

    ASSERT_EQ(GetBodies(world).size(), std::size_t(2));
    ASSERT_EQ(GetJoints(world).size(), std::size_t(1));
    ASSERT_EQ(GetJointRange(world), 1u);

    EXPECT_NO_THROW(Clear(world));

    EXPECT_EQ(GetBodies(world).size(), std::size_t(0));
    EXPECT_EQ(GetJoints(world).size(), std::size_t(0));
    EXPECT_EQ(GetJointRange(world), 0u);
    EXPECT_THROW(IsDestroyed(world, JointID{0u}), OutOfRange<JointID>);

    EXPECT_EQ(shapeListener.ids.size(), std::size_t(1));

    ASSERT_EQ(associationListener.ids.size(), std::size_t(0));

    ASSERT_EQ(jointListener.ids.size(), std::size_t(1));
    EXPECT_EQ(jointListener.ids.at(0), j0);

    const auto shapeId1 = CreateShape(world, Shape(DiskShapeConf{}));
    const auto b2 = CreateBody(world);
    EXPECT_LE(b2, b1);
    ASSERT_NO_THROW(Attach(world, b2, shapeId1));
}

TEST(AabbTreeWorld, GetType)
{
    EXPECT_EQ(GetType(World{AabbTreeWorld{}}), GetTypeID<AabbTreeWorld>());
}

TEST(AabbTreeWorld, TypeCast)
{
    {
        auto world = World{AabbTreeWorld{}};
        EXPECT_EQ(TypeCast<int>(&world), nullptr);
        EXPECT_THROW(TypeCast<int>(world), std::bad_cast);
        EXPECT_NE(TypeCast<AabbTreeWorld>(&world), nullptr);
        EXPECT_NO_THROW(TypeCast<AabbTreeWorld>(world));
    }
    {
        const auto world = World{AabbTreeWorld{}};
        EXPECT_EQ(TypeCast<const int>(&world), nullptr);
        EXPECT_THROW(TypeCast<int>(world), std::bad_cast);
        EXPECT_NE(TypeCast<const AabbTreeWorld>(&world), nullptr);
        EXPECT_NO_THROW(TypeCast<AabbTreeWorld>(world));
    }
}

TEST(AabbTreeWorld, GetResourceStatsWhenOff)
{
    auto conf = WorldConf();
    conf.doStats = false;
    conf.reserveBuffers = 0;
    conf.reserveBodyStack = 0u;
    conf.reserveBodyConstraints = 0u;
    conf.reserveDistanceConstraints = 0u;
    conf.reserveContactKeys = 0u;
    auto world = AabbTreeWorld{conf};
    EXPECT_FALSE(GetResourceStats(world).has_value());
    const auto stepConf = StepConf{};
    Step(world, stepConf);
    EXPECT_FALSE(GetResourceStats(world).has_value());
}

TEST(AabbTreeWorld, GetResourceStatsWhenOn)
{
    auto conf = WorldConf();
    conf.doStats = true;
    conf.reserveBuffers = 0;
    conf.reserveBodyStack = 0u;
    conf.reserveBodyConstraints = 0u;
    conf.reserveDistanceConstraints = 0u;
    conf.reserveContactKeys = 0u;
    auto world = AabbTreeWorld{conf};
    auto stats = std::optional<pmr::StatsResource::Stats>{};
    stats = GetResourceStats(world);
    ASSERT_TRUE(stats.has_value());
    const auto oldstats = stats;
    EXPECT_EQ(stats->blocksAllocated, 0u);
    EXPECT_EQ(stats->bytesAllocated, 0u);
    EXPECT_EQ(stats->maxBlocksAllocated, 0u);
    EXPECT_EQ(stats->maxBytesAllocated, 0u);
    EXPECT_EQ(stats->maxBytes, 0u);
    EXPECT_EQ(stats->maxAlignment, 0u);
    const auto stepConf = StepConf{};
    Step(world, stepConf);
    stats = GetResourceStats(world);
    ASSERT_TRUE(stats.has_value());
#if defined(_WIN64) && !defined(NDEBUG)
    EXPECT_EQ(stats->blocksAllocated, 1u);
    EXPECT_EQ(stats->bytesAllocated, 16u);
    EXPECT_EQ(stats->maxBlocksAllocated, 1u);
    EXPECT_EQ(stats->maxBytesAllocated, 16u);
    EXPECT_EQ(stats->maxBytes, 16u);
    EXPECT_EQ(stats->maxAlignment, 8u);
#elif defined(_WIN32) && !defined(NDEBUG)
    EXPECT_EQ(stats->blocksAllocated, 1u);
    EXPECT_EQ(stats->bytesAllocated, 8u);
    EXPECT_EQ(stats->maxBlocksAllocated, 1u);
    EXPECT_EQ(stats->maxBytesAllocated, 8u);
    EXPECT_EQ(stats->maxBytes, 8u);
    EXPECT_EQ(stats->maxAlignment, 4u);
#else
    EXPECT_EQ(stats->blocksAllocated, oldstats->blocksAllocated);
    EXPECT_EQ(stats->bytesAllocated, oldstats->bytesAllocated);
    EXPECT_EQ(stats->maxBlocksAllocated, oldstats->maxBlocksAllocated);
    EXPECT_EQ(stats->maxBytesAllocated, oldstats->maxBytesAllocated);
    EXPECT_EQ(stats->maxBytes, oldstats->maxBytes);
    EXPECT_EQ(stats->maxAlignment, oldstats->maxAlignment);
#endif
}

TEST(AabbTreeWorld, CreateDestroyEmptyStaticBody)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodies(world).size(), BodyCounter(0));
    const auto bodyID = CreateBody(world, BodyConf{}.Use(BodyType::Static));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = GetBody(world, bodyID);
    EXPECT_EQ(GetType(body), BodyType::Static);
    EXPECT_FALSE(IsSpeedable(body));
    EXPECT_FALSE(IsAccelerable(body));
    EXPECT_TRUE(IsImpenetrable(body));
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(GetBodies(world).size(), BodyCounter(1));
    const auto bodies1 = GetBodies(world);
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);

    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t{0});

    Destroy(world, bodyID);
    EXPECT_EQ(GetBodies(world).size(), BodyCounter(0));
    const auto& bodies2 = GetBodies(world);
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));

    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t{0});
}

TEST(AabbTreeWorld, CreateDestroyEmptyDynamicBody)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodies(world).size(), BodyCounter(0));
    const auto bodyID = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = GetBody(world, bodyID);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(body));
    EXPECT_TRUE(IsAccelerable(body));
    EXPECT_FALSE(IsImpenetrable(body));
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(GetBodies(world).size(), BodyCounter(1));
    const auto bodies1 = GetBodies(world);
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);

    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t{0});

    Destroy(world, bodyID);
    EXPECT_EQ(GetBodies(world).size(), BodyCounter(0));
    const auto& bodies2 = GetBodies(world);
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));

    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t{0});
}

TEST(AabbTreeWorld, CreateDestroyDynamicBodyAndFixture)
{
    // Created this test after receiving issue #306:
    //   Rapid create/destroy between step() causes SEGFAULT
    
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodies(world).size(), BodyCounter(0));
    const auto bodyID = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic));
    ASSERT_NE(bodyID, InvalidBodyID);

    const auto& body = GetBody(world, bodyID);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(body));
    EXPECT_TRUE(IsAccelerable(body));
    EXPECT_FALSE(IsImpenetrable(body));
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{0});

    EXPECT_EQ(GetBodies(world).size(), BodyCounter(1));
    const auto bodies1 = GetBodies(world);
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyID, *first);
    
    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t{0});

    const auto shapeId = CreateShape(world, Shape(DiskShapeConf{1_m}));
    EXPECT_FALSE(IsDestroyed(world, shapeId));
    ASSERT_NO_THROW(Attach(world, bodyID, shapeId));
    
    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetShapes(world, bodyID).size(), std::size_t{1});
    ASSERT_EQ(GetFixturesForProxies(world).size(), std::size_t{1});
    EXPECT_EQ(*GetFixturesForProxies(world).begin(), std::make_pair(bodyID, shapeId));

    Destroy(world, bodyID); // should clear fixtures for proxies!
    EXPECT_TRUE(IsDestroyed(world, bodyID));
    EXPECT_FALSE(IsDestroyed(world, shapeId));

    EXPECT_EQ(GetBodies(world).size(), BodyCounter(0));
    const auto& bodies2 = GetBodies(world);
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
    
    EXPECT_EQ(GetBodiesForProxies(world).size(), std::size_t{0});
    EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t{0});
}

TEST(AabbTreeWorld, CreateDestroyContactingBodies)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodies(world).size(), BodyCounter(0));
    ASSERT_EQ(GetJoints(world).size(), JointCounter(0));
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);
    ASSERT_EQ(GetFixturesForProxies(world).size(), 0u);
    ASSERT_EQ(GetTree(world).GetNodeCount(), 0u);

    auto contacts = GetContacts(world);
    ASSERT_TRUE(contacts.empty());
    ASSERT_EQ(contacts.size(), ContactCounter(0));

    const auto l1 = Length2{};
    const auto l2 = Length2{};

    const auto body1 = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic).UseLocation(l1));
    const auto body2 = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic).UseLocation(l2));
    EXPECT_EQ(GetBodies(world).size(), BodyCounter(2));
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetTree(world).GetNodeCount(), 0u);

    const auto shapeId = CreateShape(world, Shape(DiskShapeConf{1_m}.UseDensity(1_kgpm2)));
    ASSERT_NO_THROW(Attach(world, body1, shapeId));
    ASSERT_NO_THROW(Attach(world, body2, shapeId));
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 2u);
    EXPECT_EQ(GetTree(world).GetNodeCount(), 0u);
    ASSERT_EQ(GetShapes(world, body1).size(), 1u);
    ASSERT_EQ(GetShapes(world, body2).size(), 1u);

    const auto stepConf = StepConf{};

    const auto stats0 = Step(world, stepConf);

    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetTree(world).GetNodeCount(), 3u);

    EXPECT_EQ(stats0.pre.proxiesMoved, 0u);
    EXPECT_EQ(stats0.pre.contactsDestroyed, 0u);
    EXPECT_EQ(stats0.pre.contactsAdded, 1u);
    EXPECT_EQ(stats0.pre.contactsUpdated, 0u);
    EXPECT_EQ(stats0.pre.contactsSkipped, 0u);

    EXPECT_EQ(stats0.reg.minSeparation, -2.0_m);
    EXPECT_EQ(stats0.reg.maxIncImpulse, 0.0_Ns);
    EXPECT_EQ(stats0.reg.islandsFound, 1u);
    EXPECT_EQ(stats0.reg.islandsSolved, 0u);
    EXPECT_EQ(stats0.reg.contactsAdded, 0u);
    EXPECT_EQ(stats0.reg.bodiesSlept, 0u);
    EXPECT_EQ(stats0.reg.proxiesMoved, 0u);
    EXPECT_EQ(stats0.reg.sumPosIters, 3u);
    EXPECT_EQ(stats0.reg.sumVelIters, 1u);

    EXPECT_EQ(stats0.toi.minSeparation, std::numeric_limits<Length>::infinity());
    EXPECT_EQ(stats0.toi.maxIncImpulse, 0.0_Ns);
    EXPECT_EQ(stats0.toi.islandsFound, 0u);
    EXPECT_EQ(stats0.toi.islandsSolved, 0u);
    EXPECT_EQ(stats0.toi.contactsFound, 0u);
    EXPECT_EQ(stats0.toi.contactsAtMaxSubSteps, 0u);
    EXPECT_EQ(stats0.toi.contactsUpdatedToi, 0u);
    EXPECT_EQ(stats0.toi.contactsUpdatedTouching, 0u);
    EXPECT_EQ(stats0.toi.contactsSkippedTouching, 1u);
    EXPECT_EQ(stats0.toi.contactsAdded, 0u);
    EXPECT_EQ(stats0.toi.proxiesMoved, 0u);
    EXPECT_EQ(stats0.toi.sumPosIters, 0u);
    EXPECT_EQ(stats0.toi.sumVelIters, 0u);
    EXPECT_EQ(stats0.toi.maxDistIters, 0u);
    EXPECT_EQ(stats0.toi.maxToiIters, 0u);
    EXPECT_EQ(stats0.toi.maxRootIters, 0u);

    contacts = GetContacts(world);
    EXPECT_FALSE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(1));
    if (contacts.size() == 1u) {
        EXPECT_EQ(contacts.begin()->first.GetMin(),
                  static_cast<decltype(contacts.begin()->first.GetMin())>(0));
        EXPECT_EQ(contacts.begin()->first.GetMax(),
                  static_cast<decltype(contacts.begin()->first.GetMax())>(1));
        EXPECT_EQ(to_underlying(contacts.begin()->second), 0u);
        EXPECT_EQ(GetShapeA(GetContact(world, contacts.begin()->second)),
                  *GetShapes(world, body1).begin());
        EXPECT_EQ(GetShapeB(GetContact(world, contacts.begin()->second)),
                  *GetShapes(world, body2).begin());
        EXPECT_EQ(GetContactRange(world), 1u);
        EXPECT_FALSE(IsDestroyed(world, ContactID{0u}));
    }

    Destroy(world, body1);
    EXPECT_EQ(GetBodies(world).size(), BodyCounter(1));
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetTree(world).GetNodeCount(), 1u);

    Step(world, stepConf);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetTree(world).GetNodeCount(), 1u);
    contacts = GetContacts(world);
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));
    EXPECT_TRUE(IsDestroyed(world, ContactID{0u}));
    EXPECT_TRUE(IsDestroyed(GetContact(world, ContactID{0u})));
    EXPECT_THROW(SetContact(world, ContactID{0u}, Contact{}), OutOfRange<BodyID>);

    Destroy(world, body2);
    EXPECT_EQ(GetBodies(world).size(), BodyCounter(0));
    EXPECT_EQ(GetTree(world).GetNodeCount(), static_cast<decltype(GetTree(world).GetNodeCount())>(0));
    contacts = GetContacts(world);
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));
}

TEST(AabbTreeWorld, SetTypeOfBody)
{
    auto world = AabbTreeWorld{};
    const auto bodyID = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic));
    const auto& body = GetBody(world, bodyID);
    ASSERT_EQ(GetType(body), BodyType::Dynamic);
    auto other = AabbTreeWorld{};
    EXPECT_THROW(SetBody(other, bodyID, body), OutOfRange<BodyID>);
    EXPECT_EQ(GetType(body), BodyType::Dynamic);
    auto body2 = body;
    SetType(body2, BodyType::Static);
    EXPECT_NO_THROW(SetBody(world, bodyID, body2));
    EXPECT_EQ(GetType(GetBody(world, bodyID)), BodyType::Static);
}

TEST(AabbTreeWorld, SetContact)
{
    auto world = AabbTreeWorld{};
    EXPECT_THROW(SetContact(world, ContactID(0), Contact()), OutOfRange<BodyID>);
    EXPECT_THROW(SetContact(world, ContactID(0), Contact(Contactable(), Contactable())), OutOfRange<BodyID>);
    const auto bodyId0 = CreateBody(world);
    auto cA = Contactable{bodyId0, ShapeID(0), 0u};
    EXPECT_THROW(SetContact(world, ContactID(0), Contact(cA, cA)), OutOfRange<ShapeID>);
    const auto l0 = Length2{0_m, 0_m};
    const auto l1 = Length2{1_m, 0_m};
    const auto s0 = CreateShape(world, Shape(EdgeShapeConf{l0, l1}));
    ASSERT_EQ(s0, ShapeID(0));
    cA.childId = 1u;
    EXPECT_THROW(SetContact(world, ContactID(0), Contact(cA, cA)), InvalidArgument);
    cA.childId = 0u;
    EXPECT_THROW(SetContact(world, ContactID(0), Contact(cA, cA)), OutOfRange<ContactID>);
    auto body0 = GetBody(world, bodyId0);
    body0.Attach(s0);
    SetBody(world, bodyId0, body0);
    auto body1 = Body{BodyConf{}.Use(BodyType::Dynamic).Use(s0)};
    const auto bodyId1 = CreateBody(world, body1);
    auto step = StepConf{};
    step.deltaTime = {};
    Step(world, step);
    ASSERT_EQ(GetContactRange(world), 1u);
    const auto original = GetContact(world, ContactID(0));
    auto cB = Contactable{bodyId1, ShapeID(0), 0u};
    auto contact0 = original;
    contact0.UnsetImpenetrable();
    EXPECT_THROW(SetContact(world, ContactID(0), contact0), InvalidArgument);
    contact0.SetImpenetrable();
    EXPECT_NO_THROW(SetContact(world, ContactID(0), contact0));
    contact0.SetSensor();
    EXPECT_THROW(SetContact(world, ContactID(0), contact0), InvalidArgument);
    contact0.UnsetSensor();
    EXPECT_NO_THROW(SetContact(world, ContactID(0), contact0));
    contact0.SetDestroyed();
    EXPECT_THROW(SetContact(world, ContactID(0), contact0), InvalidArgument);
    contact0.UnsetDestroyed();
    EXPECT_NO_THROW(SetContact(world, ContactID(0), contact0));
    EXPECT_THROW(SetContact(world, ContactID(0), Contact{cA, Contactable{bodyId0, ShapeID(0), 0u}}), InvalidArgument);
    EXPECT_THROW(SetContact(world, ContactID(0), Contact{Contactable{bodyId1, ShapeID(0), 0u}, cB}), InvalidArgument);
    SetLocation(body1, Length2{10_m, 10_m});
    SetBody(world, bodyId1, body1);
    Step(world, step);
    ASSERT_TRUE(IsDestroyed(world, ContactID(0)));
    EXPECT_TRUE(IsDestroyed(GetContact(world, ContactID(0))));
    EXPECT_THROW(SetContact(world, ContactID(0), contact0), WasDestroyed<ContactID>);
}

TEST(AabbTreeWorld, SetManifold)
{
    auto world = AabbTreeWorld{};
    EXPECT_THROW(SetManifold(world, ContactID(0), Manifold()), OutOfRange<ContactID>);
    const auto bodyId0 = CreateBody(world);
    const auto l0 = Length2{0_m, 0_m};
    const auto l1 = Length2{1_m, 0_m};
    const auto s0 = CreateShape(world, Shape(EdgeShapeConf{l0, l1}));
    ASSERT_EQ(s0, ShapeID(0));
    auto body0 = GetBody(world, bodyId0);
    body0.Attach(s0);
    SetBody(world, bodyId0, body0);
    auto body1 = Body{BodyConf{}.Use(BodyType::Dynamic).Use(s0)};
    const auto bodyId1 = CreateBody(world, body1);
    auto step = StepConf{};
    step.deltaTime = {};
    Step(world, step);
    ASSERT_EQ(GetContactRange(world), 1u);
    const auto original = GetManifold(world, ContactID(0));
    ASSERT_EQ(unsigned(original.GetType()), unsigned(Manifold::e_faceA));
    ASSERT_EQ(unsigned(original.GetPointCount()), 2u);
    const auto imp0 = original.GetImpulses(0u);
    ASSERT_EQ(imp0[0], 0_Ns);
    ASSERT_EQ(imp0[1], 0_Ns);
    const auto imp1 = original.GetImpulses(1u);
    ASSERT_EQ(imp1[0], 0_Ns);
    ASSERT_EQ(imp1[1], 0_Ns);
    auto newValue = Manifold();
    EXPECT_THROW(SetManifold(world, ContactID(0), newValue), InvalidArgument); // can't change type
    newValue = Manifold::GetForFaceA(UnitVec::GetLeft(), Length2{});
    ASSERT_EQ(unsigned(newValue.GetType()), unsigned(original.GetType()));
    ASSERT_NE(unsigned(newValue.GetPointCount()), unsigned(original.GetPointCount()));
    EXPECT_THROW(SetManifold(world, ContactID(0), newValue), InvalidArgument); // can't change point count
    newValue = original;
    EXPECT_NO_THROW(SetManifold(world, ContactID(0), newValue));
    newValue.SetImpulses(0, Momentum2{1_Ns, 2_Ns});
    EXPECT_NO_THROW(SetManifold(world, ContactID(0), newValue));

    ASSERT_NO_THROW(SetLocation(body1, Length2{10_m, 10_m}));
    ASSERT_NO_THROW(SetBody(world, bodyId1, body1));
    ASSERT_NO_THROW(Step(world, StepConf{}));
    ASSERT_EQ(GetContactRange(world), 1u);
    ASSERT_TRUE(GetContacts(world).empty());
    EXPECT_THROW(SetManifold(world, ContactID(0), newValue), WasDestroyed<ContactID>);
}

TEST(AabbTreeWorld, Proxies)
{
    constexpr auto density = 2_kgpm2;
    constexpr auto friction = Real(0.5);
    constexpr auto restitution = Real(0.4);
    constexpr auto isSensor = true;

    {
        auto world = AabbTreeWorld{};
        const auto shapeId = CreateShape(world, 
            Shape(DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density).UseIsSensor(isSensor))
        );
        const auto body = CreateBody(world);
        ASSERT_NO_THROW(Attach(world, body, shapeId));
        ASSERT_EQ(std::size(GetShapes(world, body)), std::size_t(1));
        ASSERT_EQ(GetShapes(world, body).at(0), shapeId);

        const auto& shape = GetShape(world, shapeId);
        ASSERT_EQ(GetDensity(shape), density);
        ASSERT_EQ(GetFriction(shape), friction);
        ASSERT_EQ(GetRestitution(shape), restitution);
        ASSERT_EQ(IsSensor(shape), isSensor);

        auto psize = std::size_t(0u);
        ASSERT_NO_THROW(psize = GetProxies(world, BodyID(0)).size());
        EXPECT_EQ(psize, 0u);
        ASSERT_EQ(GetFixturesForProxies(world).size(), 1u);
        EXPECT_EQ(*GetFixturesForProxies(world).begin(), std::make_pair(body, shapeId));

        const auto stepConf = StepConf{};
        ASSERT_NO_THROW(Step(world, stepConf));
        ASSERT_EQ(GetProxies(world, body).size(), 1u);
        EXPECT_EQ(GetProxies(world, body)[0], 0u);
    }

    {
        const auto shape = Shape{
            ChainShapeConf{}.UseIsSensor(isSensor).Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
        };

        auto world = AabbTreeWorld{};
        const auto shapeId = CreateShape(world, shape);
        const auto body = CreateBody(world);
        ASSERT_NO_THROW(Attach(world, body, shapeId));

        ASSERT_EQ(std::size(GetShapes(world, body)), 1u);
        ASSERT_EQ(GetShapes(world, body).at(0), shapeId);
        ASSERT_EQ(IsSensor(shape), isSensor);
        ASSERT_EQ(GetProxies(world, body).size(), 0u);

        const auto stepConf = StepConf{};
        ASSERT_NO_THROW(Step(world, stepConf));
        ASSERT_EQ(GetProxies(world, body).size(), 2u);
        EXPECT_EQ(GetProxies(world, body)[0], 0u);
        EXPECT_EQ(GetProxies(world, body)[1], 1u);
    }

    {
        const auto shape = Shape{
            ChainShapeConf{}.UseIsSensor(isSensor).Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
            .Add(Length2{0_m, +2_m}).Add(Length2{2_m, 2_m})
        };

        auto world = AabbTreeWorld{};
        const auto shapeId = CreateShape(world, shape);
        const auto body = CreateBody(world);
        ASSERT_NO_THROW(Attach(world, body, shapeId));

        ASSERT_EQ(IsSensor(shape), isSensor);
        ASSERT_EQ(GetProxies(world, body).size(), 0u);

        const auto stepConf = StepConf{};
        ASSERT_NO_THROW(Step(world, stepConf));
        ASSERT_EQ(GetProxies(world, body).size(), 4u);
        EXPECT_EQ(GetProxies(world, body)[0], 0u);
        EXPECT_EQ(GetProxies(world, body)[1], 1u);
        EXPECT_EQ(GetProxies(world, body)[2], 3u);
        EXPECT_EQ(GetProxies(world, body)[3], 5u);
    }
}

TEST(AabbTreeWorld, SetEnabledBody)
{
    auto stepConf = StepConf{};

    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetFixturesForProxies(world).size(), 0u);
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);

    const auto body0 = CreateBody(world);
    const auto body1 = CreateBody(world);
    const auto valid_shape = Shape{DiskShapeConf(1_m)};
    const auto shapeId = CreateShape(world, valid_shape);

    ASSERT_NO_THROW(Attach(world, body0, shapeId));
    ASSERT_NO_THROW(Attach(world, body1, shapeId));

    ASSERT_TRUE(IsEnabled(GetBody(world, body0)));
    ASSERT_EQ(GetProxies(world, body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 2u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(GetProxies(world, body0).size(), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    // Test that set enabled to flag already set is not a toggle
    {
        auto copyBody0 = GetBody(world, body0);
        EXPECT_NO_THROW(SetEnabled(copyBody0, true));
        EXPECT_NO_THROW(SetBody(world, body0, copyBody0));
        EXPECT_TRUE(IsEnabled(GetBody(world, body0)));
    }
    {
        auto copyBody1 = GetBody(world, body1);
        EXPECT_NO_THROW(SetEnabled(copyBody1, false));
        EXPECT_NO_THROW(SetBody(world, body1, copyBody1));
        EXPECT_FALSE(IsEnabled(GetBody(world, body1)));
    }
    EXPECT_EQ(GetProxies(world, body0).size(), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, false));
    EXPECT_FALSE(IsEnabled(GetBody(world, body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, true));
    EXPECT_TRUE(IsEnabled(GetBody(world, body1)));
    EXPECT_EQ(GetProxies(world, body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, true));
    EXPECT_TRUE(IsEnabled(GetBody(world, body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, false));
    EXPECT_FALSE(IsEnabled(GetBody(world, body1)));
    EXPECT_EQ(GetProxies(world, body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, false));
    EXPECT_FALSE(IsEnabled(GetBody(world, body0)));
    EXPECT_NO_THROW(SetEnabled(world, body1, true));
    EXPECT_TRUE(IsEnabled(GetBody(world, body1)));
    EXPECT_EQ(GetProxies(world, body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(GetProxies(world, body0).size(), 0u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(SetEnabled(world, body0, true));
    EXPECT_TRUE(IsEnabled(GetBody(world, body0)));
    EXPECT_EQ(GetFixturesForProxies(world).size(), 1u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);

    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(GetProxies(world, body0).size(), 1u);
    EXPECT_EQ(GetFixturesForProxies(world).size(), 0u);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
}

TEST(AabbTreeWorld, AttachAndDetachShape)
{
    auto world = AabbTreeWorld{};

    auto body = CreateBody(world);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_TRUE(GetShapes(world, body).empty());
    EXPECT_FALSE(IsMassDataDirty(GetBody(world, body)));

    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1_kgpm2;
    const auto shape = Shape(conf);
    const auto shapeId = CreateShape(world, shape);

    {
        Attach(world, body, shapeId);
        const auto& fshape = GetShape(world, shapeId);
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
        EXPECT_TRUE(IsMassDataDirty(GetBody(world, body)));

        ASSERT_EQ(GetFixturesForProxies(world).size(), std::size_t{1});
        EXPECT_EQ(*GetFixturesForProxies(world).begin(), std::make_pair(body, shapeId));

        EXPECT_TRUE(Detach(world, body, shapeId));
        EXPECT_FALSE(Detach(world, body, shapeId));
        EXPECT_TRUE(GetShapes(world, body).empty());
        EXPECT_TRUE(IsMassDataDirty(GetBody(world, body)));

        EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t(0));
    }
    {
        Attach(world, body, shapeId);
        const auto& fshape = GetShape(world, shapeId);
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
        EXPECT_TRUE(IsMassDataDirty(GetBody(world, body)));
        EXPECT_FALSE(GetShapes(world, body).empty());
    }
}

TEST(AabbTreeWorld, SetTypeBody)
{
    auto world = AabbTreeWorld{};

    const auto body = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic));
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);
    ASSERT_EQ(GetType(GetBody(world, body)), BodyType::Dynamic);

    SetType(world, body, BodyType::Static);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    EXPECT_EQ(GetType(GetBody(world, body)), BodyType::Static);

    SetType(world, body, BodyType::Kinematic);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    EXPECT_EQ(GetType(GetBody(world, body)), BodyType::Kinematic);

    SetType(world, body, BodyType::Dynamic);
    EXPECT_EQ(GetType(GetBody(world, body)), BodyType::Dynamic);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
}

TEST(AabbTreeWorld, ThrowsLengthErrorOnMaxShapes)
{
    auto world = AabbTreeWorld{};
    const auto shape = Shape{DiskShapeConf{}};
    for (auto i = ShapeCounter{0u}; i < MaxShapes; ++i) {
        EXPECT_NO_THROW(CreateShape(world, shape));
    }
    EXPECT_THROW(CreateShape(world, shape), LengthError);
}

TEST(AabbTreeWorld, GetBodyRange)
{
    auto world = AabbTreeWorld{};
    EXPECT_EQ(GetBodyRange(world), BodyCounter{0u});
    EXPECT_EQ(GetBodies(world).size(), 0u);
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
    EXPECT_EQ(GetBodyRange(world), BodyCounter{1u});
    EXPECT_EQ(GetBodies(world).size(), 1u);
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
    EXPECT_EQ(GetBodyRange(world), BodyCounter{2u});
    EXPECT_EQ(GetBodies(world).size(), 2u);
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
    EXPECT_EQ(GetBodyRange(world), BodyCounter{3u});
    EXPECT_EQ(GetBodies(world).size(), 3u);
    EXPECT_NO_THROW(Destroy(world, BodyID{0u}));
    EXPECT_EQ(GetBodyRange(world), BodyCounter{3u});
    EXPECT_EQ(GetBodies(world).size(), 2u);
    EXPECT_NO_THROW(Destroy(world, BodyID{1u}));
    EXPECT_EQ(GetBodyRange(world), BodyCounter{3u});
    EXPECT_EQ(GetBodies(world).size(), 1u);
}

TEST(AabbTreeWorld, GetShapeRange)
{
    const auto shape = Shape{DiskShapeConf{}};
    auto world = AabbTreeWorld{};
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{0u});
    const auto shapeId = CreateShape(world, shape);
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{1u});
    EXPECT_NO_THROW(CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 0u);
    EXPECT_NO_THROW(Attach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 1u);
    EXPECT_NO_THROW(Attach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 2u);
    EXPECT_NO_THROW(Detach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 1u);
    EXPECT_NO_THROW(Detach(world, BodyID{0u}, shapeId));
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{1u});
    EXPECT_EQ(GetShapes(world, BodyID{0u}).size(), 0u);
    EXPECT_NO_THROW(Destroy(world, shapeId));
    EXPECT_TRUE(IsDestroyed(world, shapeId));
    EXPECT_EQ(GetShapeRange(world), ShapeCounter{1u});
}

TEST(AabbTreeWorld, GetJointRange)
{
    auto world = AabbTreeWorld{};
    EXPECT_EQ(GetJointRange(world), JointCounter{0u});
}

TEST(AabbTreeWorld, GetContactRange)
{
    auto world = AabbTreeWorld{};
    EXPECT_EQ(GetContactRange(world), ContactCounter{0u});
}

TEST(AabbTreeWorld, IsDestroyedBody)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodies(world).size(), 0u);
    EXPECT_THROW(IsDestroyed(world, BodyID{0u}), OutOfRange<BodyID>);

    auto id = InvalidBodyID;
    ASSERT_NO_THROW(id = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
    ASSERT_EQ(to_underlying(id), 0u);
    ASSERT_EQ(GetBodies(world).size(), 1u);
    auto is_destroyed = true;
    EXPECT_NO_THROW(is_destroyed = IsDestroyed(world, id));
    EXPECT_FALSE(is_destroyed);

    ASSERT_NO_THROW(id = CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
    ASSERT_EQ(to_underlying(id), 1u);
    ASSERT_EQ(GetBodies(world).size(), 2u);
    EXPECT_FALSE(IsDestroyed(world, id));

    ASSERT_NO_THROW(Destroy(world, BodyID{0u}));
    EXPECT_TRUE(IsDestroyed(world, BodyID{0u}));
    EXPECT_TRUE(IsDestroyed(GetBody(world, BodyID{0u})));
    EXPECT_FALSE(IsDestroyed(world, BodyID{1u}));
    ASSERT_NO_THROW(Destroy(world, BodyID{1u}));
    EXPECT_TRUE(IsDestroyed(world, BodyID{0u}));
    EXPECT_TRUE(IsDestroyed(world, BodyID{1u}));
}

TEST(AabbTreeWorld, AttachDetach)
{
    const auto shape = Shape{DiskShapeConf{}};
    auto world = AabbTreeWorld{};
    const auto shapeId = CreateShape(world, shape);
    ASSERT_NO_THROW(CreateBody(world, BodyConf{}.Use(BodyType::Dynamic)));
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

TEST(AabbTreeWorld, SetShapeWithEmpty)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetShapeRange(world), 0u);
    auto id = ShapeID{};
    ASSERT_NO_THROW(id = CreateShape(world, Shape{EdgeShapeConf{}}));
    ASSERT_NE(id, InvalidShapeID);
    EXPECT_THROW(SetShape(world, id, Shape{}), WasDestroyed<Shape>);
}

TEST(AabbTreeWorld, SetShapeOfBodyAwakensBody)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetShapeRange(world), 0u);
    auto id = ShapeID{};
    ASSERT_NO_THROW(id = CreateShape(world, Shape{EdgeShapeConf{}}));
    ASSERT_NE(id, InvalidShapeID);
    auto bodyId = BodyID{};
    const auto bodyConf = BodyConf{}.Use(BodyType::Dynamic).Use(id).UseAwake(false);
    ASSERT_NO_THROW(bodyId = CreateBody(world, Body{bodyConf}));
    ASSERT_FALSE(IsAwake(GetBody(world, bodyId)));

    EXPECT_THROW(SetShape(world, id, Shape{}), WasDestroyed<Shape>);
    EXPECT_FALSE(IsAwake(GetBody(world, bodyId)));
    EXPECT_NO_THROW(SetShape(world, id, Shape{DiskShapeConf{}}));
    EXPECT_TRUE(IsAwake(GetBody(world, bodyId)));
}

TEST(AabbTreeWorld, SetShapeThrowsWithOutOfRangeID)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetShapeRange(world), 0u);
    EXPECT_THROW(SetShape(world, ShapeID(0), Shape{}),
                 OutOfRange<ShapeID>);
    EXPECT_THROW(SetShape(world, ShapeID(0), Shape{EdgeShapeConf{}}),
                 OutOfRange<ShapeID>);
}

TEST(AabbTreeWorld, CreateBodyThrowsWithOutOfRangeShapeID)
{
    auto world = AabbTreeWorld{};
    auto body = Body{};
    ASSERT_NO_THROW(body.Attach(ShapeID(0)));
    EXPECT_THROW(CreateBody(world, body), OutOfRange<ShapeID>);
}

TEST(AabbTreeWorld, CreateBodyWithInRangeShapeIDs)
{
    auto world = AabbTreeWorld{};

    ASSERT_EQ(GetShapeRange(world), 0u);
    auto shapeId0 = InvalidShapeID;
    auto shapeId1 = InvalidShapeID;
    ASSERT_NO_THROW(shapeId0 = CreateShape(world, Shape(DiskShapeConf{})));
    ASSERT_NO_THROW(shapeId1 = CreateShape(world, Shape(DiskShapeConf{})));
    ASSERT_EQ(GetShapeRange(world), 2u);

    auto body = Body{};
    ASSERT_EQ(size(body.GetShapes()), 0u);
    ASSERT_NO_THROW(body.Attach(shapeId0));
    ASSERT_EQ(size(body.GetShapes()), 1u);
    ASSERT_NO_THROW(body.Attach(shapeId1));
    ASSERT_EQ(size(body.GetShapes()), 2u);

    ASSERT_EQ(GetBodyRange(world), 0u);
    auto bodyId = InvalidBodyID;
    EXPECT_NO_THROW(bodyId = CreateBody(world, body));
    EXPECT_EQ(GetBodyRange(world), 1u);
    ASSERT_EQ(size(GetBody(world, bodyId).GetShapes()), 2u);
    ASSERT_EQ(GetBody(world, bodyId).GetShapes()[0], shapeId0);
    ASSERT_EQ(GetBody(world, bodyId).GetShapes()[1], shapeId1);
    ASSERT_EQ(size(GetFixturesForProxies(world)), 2u);
    EXPECT_EQ(*begin(GetFixturesForProxies(world)), std::make_pair(bodyId, shapeId0));
    EXPECT_EQ(*(begin(GetFixturesForProxies(world)) + 1), std::make_pair(bodyId, shapeId1));
    EXPECT_EQ(size(GetProxies(world, bodyId)), 0u);

    EXPECT_NO_THROW(Step(world, StepConf{}));
    EXPECT_EQ(size(GetFixturesForProxies(world)), 0u);
    EXPECT_EQ(size(GetProxies(world, bodyId)), 2u);
}

TEST(AabbTreeWorld, SetBodyThrowsWithOutOfRangeID)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodyRange(world), 0u);
    EXPECT_THROW(SetBody(world, BodyID(0), Body{}), OutOfRange<BodyID>);
}

TEST(AabbTreeWorld, SetBodyThrowsWithDestroyed)
{
    auto world = AabbTreeWorld{};
    auto id = BodyID{};
    ASSERT_NO_THROW(id = CreateBody(world));
    ASSERT_EQ(GetBodyRange(world), 1u);
    ASSERT_NO_THROW(Destroy(world, id));
    ASSERT_EQ(GetBodyRange(world), 1u);
    EXPECT_THROW(SetBody(world, BodyID(0), Body{}), WasDestroyed<BodyID>);
}

TEST(AabbTreeWorld, SetBodyThrowsWithDestroyedChanged)
{
    auto world = AabbTreeWorld{};
    auto body = Body{};
    auto id = BodyID{};
    ASSERT_NO_THROW(id = CreateBody(world, body));
    ASSERT_EQ(GetBodyRange(world), 1u);
    body.SetDestroyed();
    EXPECT_THROW(SetBody(world, BodyID(0), body), InvalidArgument);
}

TEST(AabbTreeWorld, SetBodyThrowsWithOutOfRangeShapeID)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetBodyRange(world), 0u);
    ASSERT_NO_THROW(CreateBody(world, Body()));
    ASSERT_EQ(GetBodyRange(world), 1u);
    auto body = Body{};
    ASSERT_NO_THROW(SetBody(world, BodyID(0), body));
    ASSERT_NO_THROW(body.Attach(ShapeID(0)));
    EXPECT_THROW(SetBody(world, BodyID(0), body), OutOfRange<ShapeID>);
}

TEST(AabbTreeWorld, SetShapeWithGeometryChange)
{
    const auto stepConf = StepConf{};
    auto bodyId = InvalidBodyID;
    auto shapeId = InvalidShapeID;
    auto shapeIdOther = InvalidShapeID;
    auto world = AabbTreeWorld{};
    auto diskShapeConf = DiskShapeConf{};
    ASSERT_EQ(GetChildCount(diskShapeConf), 1u);
    ASSERT_NO_THROW(shapeId = CreateShape(world, Shape{diskShapeConf}));
    ASSERT_NO_THROW(shapeIdOther = CreateShape(world, Shape{diskShapeConf}));
    auto body = Body{BodyConf{}.Use(BodyType::Dynamic).Use(shapeId)};
    ASSERT_NO_THROW(body.Attach(shapeIdOther)); // to also cover the false match path
    ASSERT_NE(shapeId, shapeIdOther);
    ASSERT_NO_THROW(bodyId = CreateBody(world, body));
    ASSERT_TRUE(IsEnabled(GetBody(world, bodyId)));
    EXPECT_EQ(size(GetFixturesForProxies(world)), 2u);
    ASSERT_NO_THROW(Step(world, stepConf));
    EXPECT_EQ(size(GetProxies(world, bodyId)), 2u);
    auto chainShapeConf = ChainShapeConf{};
    chainShapeConf.Add(Length2{0_m, 0_m});
    chainShapeConf.Add(Length2{2_m, 0_m});
    chainShapeConf.Add(Length2{2_m, 1_m});
    ASSERT_EQ(GetChildCount(chainShapeConf), 2u); // 2 kids here means 2 proxies get made!
    EXPECT_NO_THROW(SetShape(world, shapeId, Shape{chainShapeConf})); // replaces 1 proxy w/ 2
    EXPECT_EQ(size(GetFixturesForProxies(world)), 1u);
    if (!empty(GetFixturesForProxies(world))) {
        EXPECT_EQ(GetFixturesForProxies(world)[0].first, bodyId);
        EXPECT_EQ(GetFixturesForProxies(world)[0].second, shapeId);
    }
    EXPECT_EQ(size(GetProxies(world, bodyId)), 1u);
    EXPECT_NO_THROW(Step(world, stepConf)); // makes 1 proxy for shapeIdOther + 2 for shapeId
    EXPECT_EQ(size(GetFixturesForProxies(world)), 0u);
    EXPECT_EQ(size(GetProxies(world, bodyId)), 3u);
}

TEST(AabbTreeWorld, CreateEmptyShapeThrows)
{
    auto world = AabbTreeWorld{};
    EXPECT_THROW(CreateShape(world, Shape()), WasDestroyed<Shape>);
}

TEST(AabbTreeWorld, SetDestroyedShapeThrows)
{
    auto world = AabbTreeWorld{};
    auto id = InvalidShapeID;
    ASSERT_NO_THROW(id = CreateShape(world, Shape(EdgeShapeConf{})));
    ASSERT_NO_THROW(Destroy(world, id));
    ASSERT_TRUE(IsDestroyed(world, id));
    EXPECT_THROW(SetShape(world, id, Shape()), WasDestroyed<ShapeID>);
}

TEST(AabbTreeWorld, SetFreedBodyThrows)
{
    auto world = AabbTreeWorld{};
    auto id = InvalidBodyID;
    ASSERT_NO_THROW(id = CreateBody(world, Body()));
    ASSERT_NO_THROW(Destroy(world, id));
    EXPECT_THROW(SetBody(world, id, Body()), WasDestroyed<BodyID>);
}

TEST(AabbTreeWorld, CreateEmptyJointThrows)
{
    auto world = AabbTreeWorld{};
    EXPECT_THROW(CreateJoint(world, Joint()), WasDestroyed<Joint>);
}

TEST(AabbTreeWorld, SetDestroyedJointThrows)
{
    auto world = AabbTreeWorld{};
    auto id = InvalidJointID;
    ASSERT_NO_THROW(id = CreateJoint(world, Joint(DistanceJointConf{})));
    ASSERT_NO_THROW(Destroy(world, id));
    EXPECT_THROW(SetJoint(world, id, Joint()), WasDestroyed<JointID>);
}

TEST(AabbTreeWorld, SetEmptyJointThrows)
{
    auto world = AabbTreeWorld{};
    auto id = InvalidJointID;
    ASSERT_NO_THROW(id = CreateJoint(world, Joint(DistanceJointConf{})));
    EXPECT_THROW(SetJoint(world, id, Joint()), WasDestroyed<Joint>);
}

TEST(AabbTreeWorld, SetBodyWithShapeID)
{
    auto world = AabbTreeWorld{};

    ASSERT_EQ(GetShapeRange(world), 0u);
    auto shapeId = InvalidShapeID;
    ASSERT_NO_THROW(shapeId = CreateShape(world, Shape(DiskShapeConf{})));
    ASSERT_EQ(GetShapeRange(world), 1u);

    ASSERT_EQ(GetBodyRange(world), 0u);
    auto bodyId = InvalidBodyID;
    ASSERT_NO_THROW(bodyId = CreateBody(world, Body()));
    ASSERT_EQ(GetBodyRange(world), 1u);
    ASSERT_EQ(size(GetBody(world, bodyId).GetShapes()), 0u);

    auto body = Body{};
    ASSERT_EQ(size(body.GetShapes()), 0u);

    ASSERT_NO_THROW(body.Attach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 1u);
    EXPECT_NO_THROW(SetBody(world, bodyId, body));
    EXPECT_EQ(size(GetBody(world, bodyId).GetShapes()), 1u);
    EXPECT_EQ(size(GetFixturesForProxies(world)), 1u);

    ASSERT_NO_THROW(body.Detach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 0u);
    EXPECT_NO_THROW(SetBody(world, bodyId, body));
    EXPECT_EQ(size(GetBody(world, bodyId).GetShapes()), 0u);
    EXPECT_EQ(size(GetFixturesForProxies(world)), 0u);

    ASSERT_NO_THROW(body.Attach(shapeId));
    ASSERT_NO_THROW(body.Attach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 2u);
    EXPECT_NO_THROW(SetBody(world, bodyId, body));
    EXPECT_EQ(size(GetBody(world, bodyId).GetShapes()), 2u);
    EXPECT_EQ(size(GetFixturesForProxies(world)), 2u);

    ASSERT_NO_THROW(body.Detach(shapeId));
    ASSERT_EQ(size(body.GetShapes()), 1u);
    EXPECT_NO_THROW(SetBody(world, bodyId, body));
    EXPECT_EQ(size(GetBody(world, bodyId).GetShapes()), 1u);
    // Detaching the shape currently gets rid of all attachments to the body of that shape...
    EXPECT_EQ(size(GetFixturesForProxies(world)), 0u);
}

TEST(AabbTreeWorld, CreateJointThrowsWithOutOfRangeBodyID)
{
    auto world = AabbTreeWorld{};
    auto joint = Joint(FrictionJointConf{}.UseBodyA(BodyID(0)));
    EXPECT_THROW(CreateJoint(world, joint), OutOfRange<BodyID>);
}

TEST(AabbTreeWorld, SetJointThrowsWithOutOfRangeID)
{
    auto world = AabbTreeWorld{};
    ASSERT_EQ(GetJointRange(world), 0u);
    auto joint = Joint(FrictionJointConf{}.UseBodyA(BodyID(0)));
    EXPECT_THROW(SetJoint(world, JointID(0), joint), OutOfRange<JointID>);
}

TEST(AabbTreeWorld, SetJointThrowsWithOutOfRangeBodyID)
{
    const auto b0 = BodyID(0);
    const auto b1 = BodyID(1);
    const auto j0 = JointID(0);
    auto world = AabbTreeWorld{};
    ASSERT_NO_THROW(CreateBody(world, Body()));
    ASSERT_EQ(GetBodyRange(world), 1u);
    ASSERT_EQ(GetJointRange(world), 0u);
    ASSERT_NO_THROW(CreateJoint(world, Joint(FrictionJointConf{}.UseBodyA(b0).UseBodyB(b0))));
    ASSERT_EQ(GetJointRange(world), 1u);
    ASSERT_NO_THROW(SetJoint(world, j0, Joint(FrictionJointConf{}.UseBodyA(b0).UseBodyB(b0))));
    EXPECT_THROW(SetJoint(world, j0, Joint(FrictionJointConf{}.UseBodyA(b1).UseBodyB(b0))),
                 OutOfRange<BodyID>);
    EXPECT_THROW(SetJoint(world, j0, Joint(FrictionJointConf{}.UseBodyA(b0).UseBodyB(b1))),
                 OutOfRange<BodyID>);
}

// Added herein since only AabbTreeWorld uses EraseFirst and saves making new file.
TEST(Templates, EraseFirst)
{
    auto container = std::vector<int>{0, 1, 2};
    EXPECT_FALSE(EraseFirst(container, 99));
    EXPECT_EQ(size(container), 3u);
    EXPECT_TRUE(EraseFirst(container, 1));
    EXPECT_EQ(size(container), 2u);
    EXPECT_EQ(container, (std::vector<int>{0, 2}));
}

TEST(AabbTreeWorld, GetSoonestContact)
{
    auto ids = std::vector<KeyedContactID>{};
    auto contacts = std::vector<Contact>{};
    EXPECT_EQ(GetSoonestContact(ids, contacts), InvalidContactID);
    auto c = Contact{};
    contacts.push_back(c);
    EXPECT_EQ(GetSoonestContact(ids, contacts), InvalidContactID);
    ids.emplace_back(ContactKey(), ContactID(0));
    EXPECT_EQ(GetSoonestContact(ids, contacts), InvalidContactID);
    c.SetToi(Real(0.5));
    contacts.push_back(c);
    ids.emplace_back(ContactKey(), ContactID(1));
    EXPECT_EQ(GetSoonestContact(ids, contacts), ContactID(1));
    c.SetToi(Real(0.2));
    contacts.push_back(c);
    ids.emplace_back(ContactKey(), ContactID(2));
    EXPECT_EQ(GetSoonestContact(ids, contacts), ContactID(2));
    c.SetToi(Real(0.6));
    contacts.push_back(c);
    ids.emplace_back(ContactKey(), ContactID(3));
    EXPECT_EQ(GetSoonestContact(ids, contacts), ContactID(2));
}
