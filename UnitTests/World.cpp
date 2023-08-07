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

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldContact.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Dynamics/ContactImpulsesList.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/PointStates.hpp>
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

template <typename T>
struct PushBackListener
{
    std::vector<T> ids;
    void operator()(T id)
    {
        ids.push_back(id);
    }
};

TEST(World, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    EXPECT_EQ(sizeof(World), sizeof(void*));
}

TEST(World, WorldLockedError)
{
    const auto value = WrongState{"world is locked"};
    EXPECT_STREQ(value.what(), "world is locked");
}

TEST(World, DefaultInit)
{
    World world;

    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    EXPECT_EQ(world.GetTree().GetLeafCount(), ContactCounter(0));
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_EQ(GetContactCount(world), ContactCounter(0));
    EXPECT_EQ(GetHeight(world.GetTree()), ContactCounter(0));
    EXPECT_EQ(ComputePerimeterRatio(world.GetTree()), Real(0));

    {
        const auto& bodies = world.GetBodies();
        EXPECT_TRUE(bodies.empty());
        EXPECT_EQ(bodies.size(), BodyCounter(0));
        EXPECT_EQ(bodies.begin(), bodies.end());
    }
    {
        const auto& w = static_cast<const World&>(world);
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

TEST(World, Init)
{
    World world{};
    EXPECT_FALSE(world.IsLocked());
    
    {
        auto calls = 0;
        Query(world.GetTree(), AABB{}, [&](BodyID, ShapeID, ChildCounter) {
            ++calls;
            return true;
        });
        EXPECT_EQ(calls, 0);
    }
    {
        const auto p1 = Length2{0_m, 0_m};
        const auto p2 = Length2{100_m, 0_m};
        auto calls = 0;
        RayCast(world, RayCastInput{p1, p2, UnitInterval<Real>{1}},
                [&](BodyID, ShapeID, ChildCounter, Length2, UnitVec) {
            ++calls;
            return RayCastOpcode::ResetRay;
        });
        EXPECT_EQ(calls, 0);
    }
}

TEST(World, InvalidArgumentInit)
{
    const auto min = Positive<Length>(4_m);
    const auto max = Positive<Length>(8_m);
    ASSERT_GT(max, min);
    const auto def = WorldConf{}.UseMinVertexRadius(max).UseMaxVertexRadius(min);
    EXPECT_THROW(World{def}, InvalidArgument);
}

TEST(World, Clear)
{
    auto jointListener = PushBackListener<JointID>{};
    auto shapeListener = PushBackListener<ShapeID>{};
    auto associationListener = PushBackListener<std::pair<BodyID, ShapeID>>{};

    auto world = World{};
    ASSERT_EQ(world.GetBodies().size(), std::size_t(0));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(0));
    ASSERT_EQ(GetJointRange(world), 0u);

    SetJointDestructionListener(world, std::ref(jointListener));
    SetShapeDestructionListener(world, std::ref(shapeListener));
    SetDetachListener(world, std::ref(associationListener));

    const auto b0 = CreateBody(world);
    ASSERT_NE(b0, InvalidBodyID);
    const auto f0 = CreateShape(world, DiskShapeConf{});
    ASSERT_NE(f0, InvalidShapeID);
    Attach(world, b0, f0);
    ASSERT_EQ(world.GetShapes(b0).size(), std::size_t(1));;

    const auto b1 = CreateBody(world);
    ASSERT_NE(b1, InvalidBodyID);
    const auto f1 = CreateShape(world, Shape{DiskShapeConf{}});
    ASSERT_NE(f1, InvalidShapeID);
    Attach(world, b1, f1);
    ASSERT_EQ(world.GetShapes(b1).size(), std::size_t(1));;

    const auto j0 = world.CreateJoint(Joint{DistanceJointConf{b0, b1}});
    ASSERT_NE(j0, InvalidJointID);

    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(1));
    ASSERT_EQ(GetJointRange(world), 1u);

    EXPECT_NO_THROW(world.Clear());
    EXPECT_EQ(world.GetShapeRange(), 0u);
    EXPECT_EQ(world.GetBodyRange(), 0u);
    EXPECT_EQ(world.GetJointRange(), 0u);
    EXPECT_EQ(world.GetContactRange(), 0u);

    EXPECT_EQ(world.GetBodies().size(), std::size_t(0));
    EXPECT_EQ(world.GetJoints().size(), std::size_t(0));

    ASSERT_EQ(shapeListener.ids.size(), std::size_t(2));
    EXPECT_EQ(shapeListener.ids.at(0), f0);
    EXPECT_EQ(shapeListener.ids.at(1), f1);

    ASSERT_EQ(jointListener.ids.size(), std::size_t(1));
    EXPECT_EQ(jointListener.ids.at(0), j0);

    const auto b2 = CreateBody(world);
    EXPECT_LE(b2, b1);
    const auto f2 = CreateShape(world, Shape{DiskShapeConf{}});
    EXPECT_EQ(f2, ShapeID(0));
}

TEST(World, SetSubSteppingFreeFunction)
{
    World world;
    ASSERT_FALSE(GetSubStepping(world));
    EXPECT_NO_THROW(SetSubStepping(world, true));
    EXPECT_TRUE(GetSubStepping(world));
    EXPECT_NO_THROW(SetSubStepping(world, false));
    EXPECT_FALSE(GetSubStepping(world));
    EXPECT_NO_THROW(SetSubStepping(world, true));
    EXPECT_TRUE(GetSubStepping(world));
    auto stepConf = StepConf{};
    stepConf.deltaTime = Real(1) / 100_Hz;
    EXPECT_NO_THROW(Step(world, stepConf));
    EXPECT_TRUE(GetSubStepping(world));
}

TEST(World, IsStepComplete)
{
    auto world = World{};
    ASSERT_FALSE(world.GetSubStepping());
    EXPECT_TRUE(world.IsStepComplete());

    world.SetSubStepping(true);
    ASSERT_TRUE(world.GetSubStepping());
    EXPECT_TRUE(world.IsStepComplete());

    auto stepConf = StepConf{};
    stepConf.deltaTime = Real(1) / 100_Hz;
    world.Step(stepConf);
    ASSERT_TRUE(world.GetSubStepping());
    EXPECT_TRUE(world.IsStepComplete());

    const auto b0 = CreateBody(world, BodyConf{}
                                     .UseType(BodyType::Dynamic)
                                     .UseLocation(Length2{-2_m, 2_m})
                                     .UseLinearAcceleration(EarthlyGravity));
    ASSERT_NE(b0, InvalidBodyID);

    const auto shapeId0 = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
    Attach(world, b0, shapeId0);

    const auto b1 = CreateBody(world, BodyConf{}
                                     .UseType(BodyType::Dynamic)
                                     .UseLocation(Length2{+2_m, 2_m})
                                     .UseLinearAcceleration(EarthlyGravity));
    ASSERT_NE(b1, InvalidBodyID);
    const auto shapeId1 = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
    Attach(world, b1, shapeId1);

    const auto stabody = CreateBody(world, BodyConf{}.UseType(BodyType::Static));
    const auto shapeId2 = CreateShape(world, EdgeShapeConf{Length2{-10_m, 0_m}, Length2{+10_m, 0_m}});
    Attach(world, stabody, shapeId2);

    auto i = 0ull;
    while (world.IsStepComplete() && i < 100000) {
        world.Step(stepConf);
        ++i;
    }
    EXPECT_LT(i, 1000000ull);
    EXPECT_FALSE(world.IsStepComplete());
    world.Step(stepConf);
    EXPECT_FALSE(world.IsStepComplete());
    world.Step(stepConf);
    EXPECT_TRUE(world.IsStepComplete());
}

TEST(World, CopyConstruction)
{
    auto world = World{};
    {
        const auto copy = World{world};
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(GetHeight(world.GetTree()), GetHeight(copy.GetTree()));
        EXPECT_EQ(world.GetTree().GetLeafCount(), copy.GetTree().GetLeafCount());
        EXPECT_EQ(GetMaxImbalance(world.GetTree()), GetMaxImbalance(copy.GetTree()));
    }
    
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b1, shapeId);
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b2, shapeId);

    // Add another body on top of previous and that's not part of any joints to ensure at 1 contact
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b3, shapeId);

    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b4, shapeId);
    const auto b5 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b5, shapeId);

    const auto rj1 = world.CreateJoint(Joint{RevoluteJointConf{b1, b2}});
    const auto rj2 = world.CreateJoint(Joint{RevoluteJointConf{b3, b4}});
    world.CreateJoint(Joint{PrismaticJointConf{b1, b2}});
    world.CreateJoint(Joint{GetPulleyJointConf(world, b1, b2, Length2{}, Length2{},
                                               Length2{}, Length2{}).UseRatio(Real(1))});
    world.CreateJoint(Joint{DistanceJointConf{b4, b5}});
    world.CreateJoint(Joint{GetWeldJointConf(world, b4, b5)});
    world.CreateJoint(Joint{FrictionJointConf{b4, b5}});
    world.CreateJoint(Joint{RopeJointConf{b4, b5}});
    world.CreateJoint(Joint{GetMotorJointConf(world, b4, b5)});
    world.CreateJoint(Joint{WheelJointConf{b4, b5}});
    world.CreateJoint(Joint{TargetJointConf{b4}});
    world.CreateJoint(Joint{GetGearJointConf(world, rj1, rj2)});

    auto stepConf = StepConf{};
    world.Step(stepConf);
    ASSERT_FALSE(world.GetContacts().empty());

    {
        const auto copy = World{world};
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        const auto minJoints = std::min(world.GetJoints().size(), copy.GetJoints().size());

        const auto worldJoints = world.GetJoints();
        const auto copyJoints = copy.GetJoints();
        auto worldJointIter = worldJoints.begin();
        auto copyJointIter = copyJoints.begin();
        for (auto i = decltype(minJoints){0}; i < minJoints; ++i)
        {
            EXPECT_EQ(GetType(world, *worldJointIter), GetType(copy, *copyJointIter));
            ++worldJointIter;
            ++copyJointIter;
        }
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(GetHeight(world.GetTree()), GetHeight(copy.GetTree()));
        EXPECT_EQ(world.GetTree().GetLeafCount(), copy.GetTree().GetLeafCount());
        EXPECT_EQ(GetMaxImbalance(world.GetTree()), GetMaxImbalance(copy.GetTree()));
    }
}

TEST(World, CopyAssignment)
{
    auto world = World{};
    {
        auto copy = World{};
        copy = world;
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(GetHeight(world.GetTree()), GetHeight(copy.GetTree()));
        EXPECT_EQ(world.GetTree().GetLeafCount(), copy.GetTree().GetLeafCount());
        EXPECT_EQ(GetMaxImbalance(world.GetTree()), GetMaxImbalance(copy.GetTree()));
    }

    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b1, shapeId);
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b2, shapeId);

    world.CreateJoint(Joint{RevoluteJointConf{b1, b2, Length2{}}});
    world.CreateJoint(Joint{GetPrismaticJointConf(world, b1, b2, Length2{}, UnitVec::GetRight())});
    world.CreateJoint(Joint{GetPulleyJointConf(world, b1, b2, Length2{}, Length2{},
                                               Length2{}, Length2{}).UseRatio(Real(1))});

    auto stepConf = StepConf{};
    world.Step(stepConf);
    {
        auto copy = World{};
        copy = world;
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        const auto minJoints = std::min(world.GetJoints().size(), copy.GetJoints().size());
        const auto worldJoints = world.GetJoints();
        const auto copyJoints = copy.GetJoints();
        auto worldJointIter = worldJoints.begin();
        auto copyJointIter = copyJoints.begin();
        for (auto i = decltype(minJoints){0}; i < minJoints; ++i)
        {
            EXPECT_EQ(GetType(world, *worldJointIter), GetType(copy, *copyJointIter));
            ++worldJointIter;
            ++copyJointIter;
        }
        EXPECT_EQ(world.GetShapeRange(), copy.GetShapeRange());
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(GetHeight(world.GetTree()), GetHeight(copy.GetTree()));
        EXPECT_EQ(world.GetTree().GetLeafCount(), copy.GetTree().GetLeafCount());
        EXPECT_EQ(GetMaxImbalance(world.GetTree()), GetMaxImbalance(copy.GetTree()));
    }
}

TEST(World, MoveConstruction)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b1, shapeId);
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b2, shapeId);
    world.CreateJoint(Joint{RevoluteJointConf{b1, b2, Length2{}}});
    world.CreateJoint(Joint{GetPrismaticJointConf(world, b1, b2, Length2{}, UnitVec::GetRight())});
    world.CreateJoint(Joint{GetPulleyJointConf(world, b1, b2, Length2{}, Length2{},
                                               Length2{}, Length2{}).UseRatio(Real(1))});
    auto stepConf = StepConf{};
    world.Step(stepConf);
    {
        auto other = World{std::move(world)};
        EXPECT_EQ(other.GetBodies().size(), 2u);
        EXPECT_EQ(other.GetJoints().size(), 3u);
    }
}

TEST(World, MoveAssignment)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b1, shapeId);
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, b2, shapeId);
    world.CreateJoint(Joint{RevoluteJointConf{b1, b2, Length2{}}});
    world.CreateJoint(Joint{GetPrismaticJointConf(world, b1, b2, Length2{}, UnitVec::GetRight())});
    world.CreateJoint(Joint{GetPulleyJointConf(world, b1, b2, Length2{}, Length2{},
                                               Length2{}, Length2{}).UseRatio(Real(1))});
    auto stepConf = StepConf{};
    world.Step(stepConf);
    {
        auto other = World{};
        other = std::move(world);
        EXPECT_EQ(other.GetBodies().size(), 2u);
        EXPECT_EQ(other.GetJoints().size(), 3u);
    }
}

TEST(World, CreateDestroyEmptyDynamicBody)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);
    
    EXPECT_EQ(GetType(world, body), BodyType::Dynamic);
    EXPECT_EQ(GetAngle(world, body), 0_deg);
    EXPECT_TRUE(IsSpeedable(world, body));
    EXPECT_TRUE(IsAccelerable(world, body));
    EXPECT_FALSE(IsImpenetrable(world, body));
    EXPECT_EQ(GetShapes(world, body).size(), std::size_t{0});

    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(body, *first);

    world.Destroy(body);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
}

TEST(World, CreateDestroyDynamicBodyAndFixture)
{
    // Created this test after receiving issue #306:
    //   Rapid create/destroy between step() causes SEGFAULT
    
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto bodyId = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(bodyId, InvalidBodyID);
    
    EXPECT_EQ(GetType(world, bodyId), BodyType::Dynamic);
    EXPECT_TRUE(IsAwake(world, bodyId));
    EXPECT_TRUE(IsSpeedable(world, bodyId));
    EXPECT_TRUE(IsAccelerable(world, bodyId));
    EXPECT_FALSE(IsImpenetrable(world, bodyId));
    EXPECT_EQ(GetShapes(world, bodyId).size(), std::size_t{0});

    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto first = bodies1.begin();
    EXPECT_EQ(bodyId, *first);

    Attach(world, bodyId, CreateShape(world, DiskShapeConf{1_m}));
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(GetShapes(world, bodyId).size(), std::size_t{1});

    world.Destroy(bodyId); // should clear fixtures for proxies!
    EXPECT_EQ(GetType(world, bodyId), BodyType::Static);
    EXPECT_FALSE(IsAwake(world, bodyId));
    EXPECT_FALSE(IsSpeedable(world, bodyId));

    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
}

TEST(World, CreateDestroyJoinedBodies)
{
    auto jointListener = PushBackListener<JointID>{};
    auto shapeListener = PushBackListener<ShapeID>{};
    auto associationListener = PushBackListener<std::pair<BodyID, ShapeID>>{};

    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetJointCount(world), JointCounter(0));

    SetJointDestructionListener(world, std::ref(jointListener));
    SetShapeDestructionListener(world, std::ref(shapeListener));
    SetDetachListener(world, std::ref(associationListener));

    const auto body1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto& bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    ASSERT_NE(body1, InvalidBodyID);
    EXPECT_EQ(body1, *bodies1.begin());

    const auto body2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));

    const auto shapeId1 = CreateShape(world, DiskShapeConf{1_m});
    Attach(world, body1, shapeId1);
    const auto shapeId2 = CreateShape(world, DiskShapeConf{1_m});
    Attach(world, body2, shapeId2);

    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    ASSERT_EQ(world.GetContacts().size(), ContactCounter(1));
    const auto contact0 = std::get<ContactID>(*world.GetContacts().begin());
    const auto contactBodyA = GetBodyA(world, contact0);
    const auto contactBodyB = GetBodyB(world, contact0);
    EXPECT_EQ(contactBodyA, body1);
    EXPECT_EQ(contactBodyB, body2);
    const auto worldContacts = world.GetContacts();
    const auto c0 = worldContacts.begin();
    EXPECT_FALSE(NeedsFiltering(world, c0->second));

    const auto joint = world.CreateJoint(Joint{DistanceJointConf{body1, body2}});
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetJointCount(world), JointCounter(1));
    EXPECT_TRUE(NeedsFiltering(world, c0->second));

    world.Destroy(body1);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));

    const auto& bodies0 = world.GetBodies();
    EXPECT_FALSE(bodies0.empty());
    EXPECT_EQ(bodies0.size(), BodyCounter(1));
    EXPECT_NE(bodies0.begin(), bodies0.end());
    world.Destroy(body2);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    
    ASSERT_EQ(associationListener.ids.size(), std::size_t(2));
    EXPECT_EQ(associationListener.ids.at(0), std::make_pair(body1, shapeId1));
    EXPECT_EQ(associationListener.ids.at(1), std::make_pair(body2, shapeId2));

    ASSERT_EQ(jointListener.ids.size(), std::size_t(1));
    EXPECT_EQ(jointListener.ids.at(0), joint);
}

TEST(World, CreateDestroyContactingBodies)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetJointCount(world), JointCounter(0));
    ASSERT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    ASSERT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    auto contacts = world.GetContacts();
    ASSERT_TRUE(contacts.empty());
    ASSERT_EQ(contacts.size(), ContactCounter(0));

    const auto l1 = Length2{};
    const auto l2 = Length2{};

    const auto body1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    const auto body2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l2));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    Attach(world, body1, CreateShape(world, DiskShapeConf{1_m}.UseDensity(1_kgpm2)));
    Attach(world, body2, CreateShape(world, DiskShapeConf{1_m}.UseDensity(1_kgpm2)));
    ASSERT_EQ(size(GetShapes(world, body1)), 1u);
    ASSERT_EQ(size(GetShapes(world, body2)), 1u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(2));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));

    const auto stepConf = StepConf{};

    const auto stats0 = world.Step(stepConf);

    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
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
        EXPECT_EQ(GetShapeA(world, contacts.begin()->second),
                  *GetShapes(world, body1).begin());
        EXPECT_EQ(GetShapeB(world, contacts.begin()->second),
                  *GetShapes(world, body2).begin());
    }

    world.Destroy(body1);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(1));

    world.Step(stepConf);
    EXPECT_EQ(world.GetBodiesForProxies().size(), static_cast<decltype(world.GetBodiesForProxies().size())>(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(1));
    contacts = world.GetContacts();
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));

    world.Destroy(body2);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    EXPECT_EQ(world.GetTree().GetNodeCount(), static_cast<decltype(world.GetTree().GetNodeCount())>(0));
    contacts = world.GetContacts();
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(0));
}

TEST(World, SetUnsetSetImpenetrable)
{
    auto world = World{};
    EXPECT_THROW(SetImpenetrable(world, InvalidBodyID), std::out_of_range);
    EXPECT_THROW(UnsetImpenetrable(world, InvalidBodyID), std::out_of_range);
    const auto body = CreateBody(world);
    EXPECT_NO_THROW(SetImpenetrable(world, body));
    EXPECT_TRUE(IsImpenetrable(world, body));
    EXPECT_NO_THROW(UnsetImpenetrable(world, body));
    EXPECT_FALSE(IsImpenetrable(world, body));
    EXPECT_NO_THROW(SetImpenetrable(world, body));
    EXPECT_TRUE(IsImpenetrable(world, body));
}

TEST(World, SetSleepingAllowed)
{
    auto world = World{};
    EXPECT_THROW(SetSleepingAllowed(world, InvalidBodyID, true), std::out_of_range);
    const auto body = CreateBody(world);

    SetType(world, body, BodyType::Static);
    ASSERT_FALSE(IsSpeedable(GetType(world, body)));
    EXPECT_NO_THROW(SetSleepingAllowed(world, body, true));
    EXPECT_TRUE(IsSleepingAllowed(world, body));
    EXPECT_NO_THROW(SetSleepingAllowed(world, body, false));
    EXPECT_TRUE(IsSleepingAllowed(world, body));

    SetType(world, body, BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(GetType(world, body)));
    EXPECT_NO_THROW(SetSleepingAllowed(world, body, true));
    EXPECT_TRUE(IsSleepingAllowed(world, body));
    EXPECT_NO_THROW(SetSleepingAllowed(world, body, false));
    EXPECT_FALSE(IsSleepingAllowed(world, body));
}

TEST(World, GetSetAngle)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Kinematic));
    ASSERT_EQ(GetAngle(world, body), 0_deg);
    ASSERT_EQ(GetAngularVelocity(world, body), 0_rpm);
    EXPECT_NO_THROW(SetAngle(world, body, Pi * 0.5_rad));
    EXPECT_NEAR(double(StripUnit(GetAngle(world, body))), Pi * 0.5, 0.00001);
    EXPECT_NO_THROW(SetAngle(world, body, Pi * 2.1_rad));
    EXPECT_NEAR(double(StripUnit(GetAngle(world, body))), Pi * 2.1, 0.00001);
    EXPECT_NO_THROW(SetAngle(world, body, Pi * 0_rad));
    EXPECT_NO_THROW(SetVelocity(world, body, 60_rpm));
    for (auto i = 0; i < 105; ++i) {
        EXPECT_NO_THROW(Step(world));
    }
    EXPECT_NEAR(double(StripUnit(GetAngle(GetTransformation(world, body).q))), //
                double(StripUnit(GetNormalized(Angle{60_rpm * 105 * (1_s / 60)}))), //
                0.001);
}

TEST(World, SetLinearDamping)
{
    auto world = World{};
    auto value = 1_Hz;
    EXPECT_THROW(SetLinearDamping(world, InvalidBodyID, value), std::out_of_range);
    const auto body = CreateBody(world);
    value = 2_Hz;
    EXPECT_NO_THROW(SetLinearDamping(world, body, value));
    EXPECT_EQ(GetLinearDamping(world, body), value);
    value = 23_Hz;
    EXPECT_NO_THROW(SetLinearDamping(world, body, value));
    EXPECT_EQ(GetLinearDamping(world, body), value);
}

TEST(World, SetAngularDamping)
{
    auto world = World{};
    auto value = 1_Hz;
    EXPECT_THROW(SetAngularDamping(world, InvalidBodyID, value), std::out_of_range);
    const auto body = CreateBody(world);
    value = 2_Hz;
    EXPECT_NO_THROW(SetAngularDamping(world, body, value));
    EXPECT_EQ(GetAngularDamping(world, body), value);
    value = 23_Hz;
    EXPECT_NO_THROW(SetAngularDamping(world, body, value));
    EXPECT_EQ(GetAngularDamping(world, body), value);
}

TEST(World, SynchronizeProxies)
{
    auto world = World{};
    const auto stepConf = StepConf{};
    
    EXPECT_EQ(world.Step(stepConf).pre.proxiesMoved, PreStepStats::counter_type(0));
    const auto bodyA = CreateBody(world);
    Attach(world, bodyA, CreateShape(world, DiskShapeConf(1_m)));
    EXPECT_EQ(world.Step(stepConf).pre.proxiesMoved, PreStepStats::counter_type(0));
    SetLocation(world, bodyA, Length2{10_m, -4_m});
    EXPECT_EQ(world.Step(stepConf).pre.proxiesMoved, PreStepStats::counter_type(1));
}

TEST(World, SetTypeOfBody)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);
    auto other = World{};
    EXPECT_THROW(SetType(other, body, BodyType::Static), std::out_of_range);
    EXPECT_EQ(GetType(world, body), BodyType::Dynamic);
    SetType(world, body, BodyType::Static);
    EXPECT_EQ(GetType(world, body), BodyType::Static);
}

TEST(World, Query)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(world, body));
    ASSERT_TRUE(IsAccelerable(world, body));
    ASSERT_FALSE(IsImpenetrable(world, body));
    ASSERT_EQ(GetX(GetLocation(world, body)), 0_m);
    ASSERT_EQ(GetY(GetLocation(world, body)), 0_m);
    ASSERT_EQ(GetX(GetLinearAcceleration(world, body)), 0_mps2);
    ASSERT_EQ(GetY(GetLinearAcceleration(world, body)), 0_mps2);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    ASSERT_EQ(GetChildCount(conf), ChildCounter(1));
    const auto shapeId = CreateShape(world, Shape{conf});
    Attach(world, body, shapeId);
    
    auto stepConf = StepConf{};
    stepConf.deltaTime = 0_s;
    world.Step(stepConf);

    {
        auto foundOurs = 0;
        auto foundOthers = 0;
        Query(world.GetTree(), AABB{v1, v2}, [&](BodyID, ShapeID f, ChildCounter i) {
            if (f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return true;
        });
        EXPECT_EQ(foundOurs, 1);
        EXPECT_EQ(foundOthers, 0);
    }
}

TEST(World, RayCast)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));

    const auto p0 = Length2{-10_m, +3_m};
    const auto b0 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p0));
    Attach(world, b0, CreateShape(world, DiskShapeConf{1_m}));

    const auto p1 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    Attach(world, b1, CreateShape(world, DiskShapeConf{0.1_m}));

    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Static).UseLocation(Length2{-100_m, -100_m}));
    Attach(world, b2, CreateShape(world, EdgeShapeConf{Length2{}, Length2{-20_m, -20_m}}));

    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(world, body));
    ASSERT_TRUE(IsAccelerable(world, body));
    ASSERT_FALSE(IsImpenetrable(world, body));
    ASSERT_EQ(GetX(GetLocation(world, body)), 0_m);
    ASSERT_EQ(GetY(GetLocation(world, body)), 0_m);
    ASSERT_EQ(GetX(GetLinearAcceleration(world, body)), 0_mps2);
    ASSERT_EQ(GetY(GetLinearAcceleration(world, body)), 0_mps2);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    const auto shape = Shape{conf};
    ASSERT_EQ(GetChildCount(shape), ChildCounter(1));
    const auto shapeId = CreateShape(world, shape);
    Attach(world, body, shapeId);
    
    auto stepConf = StepConf{};
    stepConf.deltaTime = 0_s;
    world.Step(stepConf);
    
    {
        const auto p2 = Length2{-2_m, 0_m};
        const auto p3 = Length2{+2_m, 0_m};

        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world, RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](BodyID b, ShapeID f, ChildCounter i, Length2, UnitVec) {
            if (b == body && f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return RayCastOpcode::ResetRay;
        });
        EXPECT_FALSE(retval);
        EXPECT_EQ(foundOurs, 1);
        EXPECT_EQ(foundOthers, 1);
    }
    
    {
        const auto p2 = Length2{-2_m, 0_m};
        const auto p3 = Length2{+2_m, 0_m};
        
        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world, RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](BodyID b, ShapeID f, ChildCounter i, Length2, UnitVec) {
            if (b == body && f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return RayCastOpcode::Terminate;
        });
        EXPECT_TRUE(retval);
        EXPECT_EQ(foundOurs, 1);
        EXPECT_EQ(foundOthers, 0);
    }
    
    {
        const auto p2 = Length2{-2_m, 0_m};
        const auto p3 = Length2{+2_m, 0_m};
        
        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world, RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](BodyID b, ShapeID f, ChildCounter i, Length2, UnitVec) {
            if (b == body && f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return RayCastOpcode::IgnoreFixture;
        });
        EXPECT_FALSE(retval);
        EXPECT_EQ(foundOurs, 1);
        EXPECT_EQ(foundOthers, 1);
    }
    
    {
        const auto p2 = Length2{ +5_m, 0_m};
        const auto p3 = Length2{+10_m, 0_m};
        
        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world, RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](BodyID b, ShapeID f, ChildCounter i, Length2, UnitVec) {
            if (b == body && f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return RayCastOpcode::IgnoreFixture;
        });
        EXPECT_FALSE(retval);
        EXPECT_EQ(foundOurs, 0);
        EXPECT_EQ(foundOthers, 0);
    }
    
    {
        const auto p2 = Length2{-2_m, 0_m};
        const auto p3 = Length2{+2_m, 0_m};
        
        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world, RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](BodyID b, ShapeID f, ChildCounter i, Length2, UnitVec) {
            if (b == body && f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return RayCastOpcode::ClipRay;
        });
        EXPECT_TRUE(retval);
        EXPECT_EQ(foundOurs, 1);
        EXPECT_EQ(foundOthers, 0);
    }
    
    {
        const auto p2 = Length2{-3_m,  0_m};
        const auto p3 = Length2{+2_m, 10_m};
        
        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world, RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](BodyID b, ShapeID f, ChildCounter i, Length2, UnitVec) {
            if (b == body && f == shapeId && i == 0)
            {
                ++foundOurs;
            }
            else
            {
                ++foundOthers;
            }
            return RayCastOpcode::ResetRay;
        });
        EXPECT_FALSE(retval);
        EXPECT_EQ(foundOurs, 0);
        EXPECT_EQ(foundOthers, 0);
    }
    {
        auto found = 0;
        const auto rci = RayCastInput{Length2{-100_m, -101_m}, Length2{-120_m, -121_m}, Real{0.9f}};
        const auto retval = RayCast(world, rci,
                                    [&](BodyID, ShapeID, ChildCounter, Length2, UnitVec) {
            ++found;
            return RayCastOpcode::Terminate;
        });
        EXPECT_FALSE(retval);
        EXPECT_EQ(found, 0);
    }
}

TEST(World, ClearForcesFreeFunction)
{
    World world;
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    ASSERT_NE(body, InvalidBodyID);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(world, body));
    ASSERT_TRUE(IsAccelerable(world, body));
    ASSERT_FALSE(IsImpenetrable(world, body));
    ASSERT_EQ(GetX(GetLinearAcceleration(world, body)), GetX(EarthlyGravity));
    ASSERT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    Attach(world, body, CreateShape(world, conf));

    ApplyForceToCenter(world, body, Force2(2_N, 4_N));
    ASSERT_NE(GetX(GetLinearAcceleration(world, body)), GetX(EarthlyGravity));
    ASSERT_NE(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
    
    ClearForces(world);
    EXPECT_EQ(GetX(GetLinearAcceleration(world, body)), 0_mps2);
    EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), 0_mps2);
}

TEST(World, SetAccelerationsFunctionalFF)
{
    World world;
    const auto a1 = Acceleration{
        LinearAcceleration2{1_mps2, 2_mps2}, 2.1f * RadianPerSquareSecond
    };
    const auto a2 = a1 * 2;
    ASSERT_EQ(a1.linear * 2, a2.linear);
    ASSERT_EQ(a1.angular * 2, a2.angular);

    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b1, InvalidBodyID);
    ASSERT_TRUE(IsAccelerable(world, b1));
    SetAcceleration(world, b1, a1);
    ASSERT_EQ(GetAcceleration(world, b1), a1);

    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b2, InvalidBodyID);
    ASSERT_TRUE(IsAccelerable(world, b2));
    SetAcceleration(world, b2, a2);
    ASSERT_EQ(GetAcceleration(world, b2), a2);

    SetAccelerations(world, [](const World& world, BodyID b) {
        return GetAcceleration(world, b) * 2;
    });
    EXPECT_EQ(GetAcceleration(world, b1), a1 * 2);
    EXPECT_EQ(GetAcceleration(world, b2), a2 * 2);
}

TEST(World, SetLinearAccelerationsFF)
{
    World world;
    const auto a1 = Acceleration{
        LinearAcceleration2{1_mps2, 2_mps2}, 2.1f * RadianPerSquareSecond
    };
    const auto a2 = a1 * 2;
    ASSERT_EQ(a1.linear * 2, a2.linear);
    ASSERT_EQ(a1.angular * 2, a2.angular);
    
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b1, InvalidBodyID);
    ASSERT_TRUE(IsAccelerable(world, b1));
    SetAcceleration(world, b1, a1);
    ASSERT_EQ(GetAcceleration(world, b1), a1);
    
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b2, InvalidBodyID);
    ASSERT_TRUE(IsAccelerable(world, b2));
    SetAcceleration(world, b2, a2);
    ASSERT_EQ(GetAcceleration(world, b2), a2);

    ASSERT_EQ(GetAcceleration(world, b1), a1);
    ASSERT_EQ(GetAcceleration(world, b2), a2);

    SetAccelerations(world, a1.linear * 2);
    EXPECT_EQ(GetAcceleration(world, b1), (Acceleration{a1.linear * 2, a1.angular}));
    EXPECT_EQ(GetAcceleration(world, b2), (Acceleration{a1.linear * 2, a2.angular}));
}

TEST(World, FindClosestBodyFF)
{
    World world;
    ASSERT_EQ(FindClosestBody(world, Length2{}), InvalidBodyID);
    const auto b1 = CreateBody(world, BodyConf{}.UseLocation(Length2{10_m, 10_m}));
    EXPECT_EQ(FindClosestBody(world, Length2{0_m, 0_m}), b1);
    const auto b2 = CreateBody(world, BodyConf{}.UseLocation(Length2{1_m, -2_m}));
    EXPECT_EQ(FindClosestBody(world, Length2{0_m, 0_m}), b2);
    const auto b3 = CreateBody(world, BodyConf{}.UseLocation(Length2{-5_m, 4_m}));
    EXPECT_NE(FindClosestBody(world, Length2{0_m, 0_m}), b3);
    EXPECT_EQ(FindClosestBody(world, Length2{0_m, 0_m}), b2);
}

TEST(World, GetAssociationCountFreeFunction)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetAssociationCount(world), std::size_t(0));
    
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shapeConf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);

    const auto shapeId1 = CreateShape(world, shapeConf);
    ASSERT_NE(shapeId1, InvalidShapeID);

    ASSERT_NO_THROW(Attach(world, body, shapeId1));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(1));

    ASSERT_NO_THROW(Attach(world, body, shapeId1));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(2));
    
    const auto shapeId2 = CreateShape(world, shapeConf);
    ASSERT_NE(shapeId2, InvalidShapeID);

    ASSERT_NO_THROW(Attach(world, body, shapeId2));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(3));
}

TEST(World, GetUsedShapesCountFreeFunction)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetUsedShapesCount(world), std::size_t(0));

    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);

    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shapeConf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);

    const auto shapeId1 = CreateShape(world, shapeConf);
    ASSERT_NE(shapeId1, InvalidShapeID);
    ASSERT_NO_THROW(Attach(world, body, shapeId1));
    EXPECT_EQ(GetUsedShapesCount(world), std::size_t(1));
    ASSERT_NO_THROW(Attach(world, body, shapeId1));
    EXPECT_EQ(GetUsedShapesCount(world), std::size_t(1));

    const auto shapeId2 = CreateShape(world, shapeConf);
    ASSERT_NE(shapeId2, InvalidShapeID);
    ASSERT_NO_THROW(Attach(world, body, shapeId2));
    EXPECT_EQ(GetUsedShapesCount(world), std::size_t(2));
}

TEST(World, GetAssociationCount)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetAssociationCount(world), std::size_t(0));

    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);

    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shapeId = CreateShape(world, EdgeShapeConf{}
        .UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2));

    EXPECT_NO_THROW(Attach(world, body, shapeId));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(1));

    EXPECT_NO_THROW(Attach(world, body, shapeId));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(2));

    EXPECT_NO_THROW(Attach(world, body, shapeId));
    EXPECT_EQ(GetAssociationCount(world), std::size_t(3));
}

TEST(World, AwakenFreeFunction)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, InvalidBodyID);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(world, body));
    ASSERT_TRUE(IsAccelerable(world, body));
    ASSERT_FALSE(IsImpenetrable(world, body));
    ASSERT_EQ(GetX(GetLinearAcceleration(world, body)), Real(0) * MeterPerSquareSecond);
    ASSERT_EQ(GetY(GetLinearAcceleration(world, body)), Real(0) * MeterPerSquareSecond);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shapeId = CreateShape(world, EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2));
    ASSERT_NO_THROW(Attach(world, body, shapeId));
    
    ASSERT_TRUE(IsAwake(world, body));
    auto stepConf = StepConf{};
    while (IsAwake(world, body))
        world.Step(stepConf);
    ASSERT_FALSE(IsAwake(world, body));
    
    Awaken(world);
    EXPECT_TRUE(IsAwake(world, body));
}

TEST(World, GetTouchingCountFreeFunction)
{
    World world;
    EXPECT_EQ(GetTouchingCount(world), ContactCounter(0));
    auto stepConf = StepConf{};
    world.Step(stepConf);
    EXPECT_EQ(GetTouchingCount(world), ContactCounter(0));
    stepConf.deltaTime = Real(1) / 100_Hz;
    world.Step(stepConf);
    EXPECT_EQ(GetTouchingCount(world), ContactCounter(0));

    const auto groundConf = EdgeShapeConf{}
        .Set(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter);
    const auto ground = CreateBody(world);
    Attach(world, ground, CreateShape(world, groundConf));

    const auto lowerBodyConf = BodyConf{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0.0f, 0.5f) * Meter);
    const auto diskConf = DiskShapeConf{}.UseDensity(10_kgpm2);
    const auto smallerDiskConf = DiskShapeConf(diskConf).UseRadius(0.5_m);
    const auto lowerBody = CreateBody(world, lowerBodyConf);
    Attach(world, lowerBody, CreateShape(world, smallerDiskConf));
    
    ASSERT_EQ(GetAwakeCount(world), 1);
    while (GetAwakeCount(world) > 0)
    {
        world.Step(stepConf);
        EXPECT_EQ(GetTouchingCount(world), ContactCounter(1));
    }
}

TEST(World, ShiftOriginFreeFunction)
{
    const auto origin = Length2{0_m, 0_m};
    const auto location = Length2{1_m, 1_m};
    
    ASSERT_NE(origin, location);

    World world;
    EXPECT_NO_THROW(ShiftOrigin(world, origin));
    
    auto bodyConf = BodyConf{};
    bodyConf.UseLocation(location);
    const auto body = CreateBody(world, bodyConf);
    EXPECT_EQ(GetLocation(world, body), location);

    EXPECT_NO_THROW(ShiftOrigin(world, location));
    EXPECT_EQ(GetLocation(world, body), origin);
}

TEST(World, DynamicEdgeBodyHasCorrectMass)
{
    World world;
    
    auto bodyConf = BodyConf{};
    bodyConf.type = BodyType::Dynamic;
    const auto body = CreateBody(world, bodyConf);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    const auto shape = Shape{conf};
    ASSERT_EQ(GetVertexRadius(shape, 0), 1_m);

    const auto shapeId = CreateShape(world, shape);
    ASSERT_NE(shapeId, InvalidShapeID);
    Attach(world, body, shapeId);
    ASSERT_EQ(GetDensity(world, shapeId), 1_kgpm2);

    const auto circleMass = Mass{GetDensity(world, shapeId) * (Pi * Square(GetVertexRadius(shape, 0)))};
    const auto rectMass = Mass{GetDensity(world, shapeId) * (GetVertexRadius(shape, 0) * 2 * GetMagnitude(v2 - v1))};
    const auto totalMass = Mass{circleMass + rectMass};
    
    EXPECT_EQ(GetType(world, body), BodyType::Dynamic);
    EXPECT_NEAR(static_cast<double>(Real{GetInvMass(world, body) * 1_kg}),
                static_cast<double>(Real{1_kg / totalMass}),
                0.000001);
}

TEST(World, CreateAndDestroyJoint)
{
    World world;

    const auto body1 = CreateBody(world);
    const auto body2 = CreateBody(world);
    EXPECT_NE(body1, InvalidBodyID);
    EXPECT_NE(body2, InvalidBodyID);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_TRUE(world.GetJoints().empty());
    
    const auto anchorA = Length2{+0.4_m, -1.2_m};
    const auto anchorB = Length2{-2.3_m, +0.7_m};
    const auto joint = world.CreateJoint(Joint{GetDistanceJointConf(world, body1, body2,
                                                                    anchorA, anchorB)});
    EXPECT_EQ(GetJointCount(world), JointCounter(1));
    EXPECT_FALSE(world.GetJoints().empty());
    const auto first = *world.GetJoints().begin();
    EXPECT_EQ(joint, first);
    EXPECT_EQ(GetType(world, joint), GetTypeID<DistanceJointConf>());
    EXPECT_EQ(GetBodyA(world, joint), body1);
    EXPECT_EQ(GetBodyB(world, joint), body2);
    EXPECT_EQ(GetLocalAnchorA(world, joint), anchorA);
    EXPECT_EQ(GetLocalAnchorB(world, joint), anchorB);
    EXPECT_FALSE(GetCollideConnected(world, joint));

    world.Destroy(joint);
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_TRUE(world.GetJoints().empty());
}

TEST(World, MaxBodies)
{
    World world;
    for (auto i = decltype(MaxBodies){0}; i < MaxBodies; ++i)
    {
        const auto body = CreateBody(world);
        ASSERT_NE(body, InvalidBodyID);
    }
    {
        EXPECT_THROW(CreateBody(world), LengthError);
    }
}

TEST(World, MaxJoints)
{
    World world;
    
    const auto body1 = CreateBody(world);
    ASSERT_NE(body1, InvalidBodyID);
    const auto body2 = CreateBody(world);
    ASSERT_NE(body2, InvalidBodyID);
    
    for (auto i = decltype(MaxJoints){0}; i < MaxJoints; ++i)
    {
        const auto joint = world.CreateJoint(Joint{RopeJointConf{body1, body2}});
        ASSERT_NE(joint, InvalidJointID);
    }
    {
        EXPECT_THROW(world.CreateJoint(Joint{RopeJointConf{body1, body2}}), LengthError);
    }
}

TEST(World, StepZeroTimeDoesNothing)
{
    World world{};
    
    BodyConf def;
    def.location = Length2{31.9_m, -19.24_m};
    def.type = BodyType::Dynamic;
    def.linearAcceleration = EarthlyGravity;
    
    const auto body = CreateBody(world, def);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_EQ(GetLocation(world, body), def.location);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetX(GetLinearAcceleration(world, body)), Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
    
    const auto time_inc = 0_s;
    
    auto pos = GetLocation(world, body);
    auto vel = GetLinearVelocity(world, body);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, time_inc);
        
        EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
        
        EXPECT_EQ(GetX(GetLocation(world, body)), GetX(def.location));
        EXPECT_EQ(GetY(GetLocation(world, body)), GetY(pos));
        pos = GetLocation(world, body);
        
        EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLinearVelocity(world, body)) / 1_mps}, Real{GetY(vel) / 1_mps}));
        vel = GetLinearVelocity(world, body);
    }
}

TEST(World, GravitationalBodyMovement)
{
    const auto a = Real(-10);

    auto p0 = Length2{0_m, 1_m};

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.location = p0;
    body_def.linearAcceleration = LinearAcceleration2{0, a * MeterPerSquareSecond};

    const auto t = .01_s;
    auto world = World{};

    const auto body = CreateBody(world, body_def);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_FALSE(IsImpenetrable(world, body));
    EXPECT_EQ(GetType(world, body), BodyType::Dynamic);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetLocation(world, body), p0);

    Step(world, t);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body)), a * (t * Real{1}) * MeterPerSquareSecond);
    EXPECT_EQ(GetX(GetLocation(world, body)), GetX(p0));
    EXPECT_EQ(GetY(GetLocation(world, body)), GetY(p0) + GetY(GetLinearVelocity(world, body)) * t);

    p0 = GetLocation(world, body);
    Step(world, t);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body)), a * (t * Real{2}) * MeterPerSquareSecond);
    EXPECT_EQ(GetX(GetLocation(world, body)), GetX(p0));
    EXPECT_EQ(GetY(GetLocation(world, body)), GetY(p0) + GetY(GetLinearVelocity(world, body)) * t);
    
    p0 = GetLocation(world, body);
    Step(world, t);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_NEAR(double(Real{GetY(GetLinearVelocity(world, body)) / 1_mps}),
                double(Real{a * (t * Real{3}) / 1_s}), 0.00001);
    EXPECT_EQ(GetX(GetLocation(world, body)), GetX(p0));
    EXPECT_EQ(GetY(GetLocation(world, body)), GetY(p0) + GetY(GetLinearVelocity(world, body)) * t);
}

TEST(World, ComputeMassData)
{
    auto world = World{};
    auto massData = MassData{};

    EXPECT_THROW(massData = ComputeMassData(world, InvalidBodyID), std::out_of_range);

    const auto body = CreateBody(world);
    EXPECT_NO_THROW(massData = ComputeMassData(world, body));
    EXPECT_EQ(massData.center, Length2{});
    EXPECT_EQ(massData.mass, 0_kg);
    EXPECT_EQ(massData.I, RotInertia(0));

    // Creates a 4x2 rectangular shape with 8_m2 area of 8_kg
    Attach(world, body, CreateShape(world, PolygonShapeConf{2_m, 1_m}.UseDensity(1_kgpm2)));
    EXPECT_NO_THROW(massData = ComputeMassData(world, body));
    EXPECT_EQ(massData.center, Length2{});
    EXPECT_EQ(massData.mass, 8_kg);
    EXPECT_NEAR(static_cast<double>(StripUnit(massData.I)), 13.3333, 0.0001);
}

#if defined(BODY_DOESNT_GROW_UNBOUNDED)
TEST(World, BodyAngleDoesntGrowUnbounded)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}
                                       .UseType(BodyType::Dynamic)
                                       .UseAngularVelocity(10_rad / Second));
    ASSERT_EQ(GetAngle(world, body), 0_rad);
    auto stepConf = StepConf{};
    auto lastAngle = 0_rad;
    auto maxAngle = 0_rad;
    for (auto i = 0; i < 1000000; ++i)
    {
        world.Step(stepConf);
        const auto angle = GetAngle(world, body);
        EXPECT_NE(angle, lastAngle);
        ASSERT_LE(angle, 360_deg);
        maxAngle = std::max(maxAngle, angle);
        lastAngle = angle;
    }
}
#endif

TEST(World, BodyAccelPerSpecWithNoVelOrPosIterations)
{
    auto world = World{};
    
    BodyConf def;
    def.location = Length2{31.9_m, -19.24_m};
    def.type = BodyType::Dynamic;
    def.linearAcceleration = EarthlyGravity;
    
    const auto body = CreateBody(world, def);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_EQ(GetLocation(world, body), def.location);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetX(GetLinearAcceleration(world, body)), Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
    
    const auto time_inc = 0.01_s;
    
    auto pos = GetLocation(world, body);
    auto vel = GetLinearVelocity(world, body);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, time_inc, 0, 0);
        
        EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
        
        EXPECT_EQ(GetX(GetLocation(world, body)), GetX(def.location));
        EXPECT_LT(GetY(GetLocation(world, body)), GetY(pos));
        EXPECT_EQ(GetY(GetLocation(world, body)), GetY(pos) + ((GetY(vel) + GetY(EarthlyGravity) * time_inc) * time_inc));
        pos = GetLocation(world, body);
        
        EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
        EXPECT_LT(GetY(GetLinearVelocity(world, body)), GetY(vel));
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLinearVelocity(world, body)) / 1_mps},
                                Real{(GetY(vel) + GetY(EarthlyGravity) * time_inc) / 1_mps}));
        vel = GetLinearVelocity(world, body);
    }
}

TEST(World, BodyAccelRevPerSpecWithNegativeTimeAndNoVelOrPosIterations)
{
    World world{};
    
    BodyConf def;
    def.location = Length2{31.9_m, -19.24_m};
    def.linearVelocity = LinearVelocity2{0, -9.8_mps};
    def.type = BodyType::Dynamic;
    def.linearAcceleration = EarthlyGravity;
    
    const auto body = CreateBody(world, def);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_EQ(GetLocation(world, body), def.location);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body)), -9.8_mps);
    EXPECT_EQ(GetX(GetLinearAcceleration(world, body)), Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
    
    const auto time_inc = -0.01_s;
    auto stepConf = StepConf{};
    stepConf.deltaTime = time_inc;
    stepConf.dtRatio = -1;
    stepConf.regPositionIters = 0;
    stepConf.regVelocityIters = 0;
    stepConf.toiPositionIters = 0;
    stepConf.toiVelocityIters = 0;
    
    auto pos = GetLocation(world, body);
    auto vel = GetLinearVelocity(world, body);
    for (auto i = 0; i < 99; ++i)
    {
        world.Step(stepConf);
        
        EXPECT_EQ(GetY(GetLinearAcceleration(world, body)), GetY(EarthlyGravity));
        
        EXPECT_EQ(GetX(GetLocation(world, body)), GetX(def.location));
        EXPECT_GT(GetY(GetLocation(world, body)), GetY(pos));
        EXPECT_FLOAT_EQ(Real(GetY(GetLocation(world, body)) / 1_m),
                        Real((GetY(pos) + ((GetY(vel) + GetY(EarthlyGravity) * time_inc) * time_inc)) / 1_m));
        pos = GetLocation(world, body);
        
        EXPECT_EQ(GetX(GetLinearVelocity(world, body)), 0_mps);
        EXPECT_GT(GetY(GetLinearVelocity(world, body)), GetY(vel));
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLinearVelocity(world, body)) / 1_mps},
                                Real{(GetY(vel) + GetY(EarthlyGravity) * time_inc) / 1_mps}));
        vel = GetLinearVelocity(world, body);
    }
}

struct MyContactListener
{
    using PreSolver = std::function<void(ContactID, const Manifold&)>;
    using PostSolver = std::function<void(ContactID, const ContactImpulsesList&, unsigned)>;
    using Ender = std::function<void(ContactID)>;

    MyContactListener(World& w, PreSolver&& pre, PostSolver&& post, Ender&& end):
        world(w), presolver(pre), postsolver(post), ender(end) {}

    void BeginContact(ContactID contact)
    {
        ++begin_contacts;
        contacting = true;
        touching = IsTouching(world, contact);
        const auto bA = GetBodyA(world, contact);
        const auto sA = GetShapeA(world, contact);
        const auto bB = GetBodyB(world, contact);
        const auto sB = GetShapeB(world, contact);
        body_a[0] = GetLocation(world, bA);
        body_b[0] = GetLocation(world, bB);
        EXPECT_THROW(CreateBody(world), WrongState);
        EXPECT_THROW(world.SetJoint(InvalidJointID, Joint{}), WrongState);
        EXPECT_THROW(world.SetShape(sA, Shape()), WrongState);
        const auto typeA = GetType(world, bA);
        if (typeA != BodyType::Kinematic)
        {
            EXPECT_NO_THROW(SetType(world, bA, typeA));
            EXPECT_THROW(SetType(world, bA, BodyType::Kinematic), WrongState);
        }
        EXPECT_THROW(world.Destroy(bA), WrongState);
        EXPECT_THROW(world.Destroy(sA), WrongState);
        EXPECT_THROW(world.CreateJoint(Joint{DistanceJointConf{bA, bB}}), WrongState);
        EXPECT_THROW(world.Step(stepConf), WrongState);
        EXPECT_THROW(world.ShiftOrigin(Length2{}), WrongState);
        EXPECT_THROW(world.CreateShape(Shape{DiskShapeConf{}}), WrongState);
        EXPECT_THROW(Attach(world, bA, sA), WrongState);
        EXPECT_THROW(Detach(world, bA, sA), WrongState);
        EXPECT_THROW(Detach(world, bB, sB), WrongState);
    }

    void EndContact(ContactID contact)
    {
        ++end_contacts;
        contacting = false;
        touching = IsTouching(world, contact);
        body_a[1] = GetLocation(world, GetBodyA(world, contact));
        body_b[1] = GetLocation(world, GetBodyB(world, contact));
        if (ender)
        {
            ender(contact);
        }
    }
    
    void PreSolve(ContactID id, const Manifold& oldManifold)
    {
        ++pre_solves;
        presolver(id, oldManifold);
    }
    
    void PostSolve(ContactID id, const ContactImpulsesList& impulses, unsigned solved)
    {
        ++post_solves;
        postsolver(id, impulses, solved);
    }

    World& world;
    unsigned begin_contacts = 0;
    unsigned end_contacts = 0;
    unsigned pre_solves = 0;
    unsigned post_solves = 0;
    bool contacting = false;
    bool touching = false;
    Length2 body_a[2] = {Length2{}, Length2{}};
    Length2 body_b[2] = {Length2{}, Length2{}};
    PreSolver presolver;
    PostSolver postsolver;
    Ender ender;
    const StepConf stepConf{};
};

TEST(World, NoCorrectionsWithNoVelOrPosIterations)
{
    const auto x = Real(10); // other test parameters tuned to this value being 10

    auto presolved = unsigned{0};
    auto postsolved = unsigned{0};
    World world{};
    MyContactListener listener{
        world,
        [&](ContactID, const Manifold&) { ++presolved; },
        [&](ContactID, const ContactImpulsesList&, unsigned) { ++postsolved; },
        [&](ContactID) {},
    };

    world.SetBeginContactListener([&listener](ContactID id) {
        listener.BeginContact(id);
    });
    world.SetEndContactListener([&listener](ContactID id) {
        listener.EndContact(id);
    });
    world.SetPreSolveContactListener([&listener](ContactID id, const Manifold& manifold) {
        listener.PreSolve(id, manifold);
    });
    world.SetPostSolveContactListener([&listener](ContactID id,
                                                  const ContactImpulsesList& impulses,
                                                  unsigned count){
        listener.PostSolve(id, impulses, count);
    });
    
    ASSERT_EQ(listener.begin_contacts, unsigned(0));
    ASSERT_EQ(listener.end_contacts, unsigned(0));
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = true;

    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(1_m).UseRestitution(Real(1)).UseDensity(1_kgpm2));
    
    body_def.location = Length2{-x * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{+x * 1_mps, 0_mps};
    const auto body_a = CreateBody(world, body_def);
    ASSERT_NE(body_a, InvalidBodyID);
    EXPECT_EQ(GetType(world, body_a), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(world, body_a));
    EXPECT_TRUE(IsAccelerable(world, body_a));
    Attach(world, body_a, shapeId);
    
    body_def.location = Length2{+x * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{-x * 1_mps, 0_mps};
    const auto body_b = CreateBody(world, body_def);
    ASSERT_NE(body_b, InvalidBodyID);
    Attach(world, body_b, shapeId);
    EXPECT_EQ(GetType(world, body_b), BodyType::Dynamic);
    EXPECT_TRUE(IsSpeedable(world, body_b));
    EXPECT_TRUE(IsAccelerable(world, body_b));

    EXPECT_EQ(GetX(GetLinearVelocity(world, body_a)), +x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body_a)), 0_mps);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body_b)), -x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body_b)), 0_mps);

    const auto time_inc = .01_s;

    auto pos_a = GetLocation(world, body_a);
    auto pos_b = GetLocation(world, body_b);
    ASSERT_LT(GetX(pos_a), GetX(pos_b));

    auto conf = StepConf{};
    conf.deltaTime = time_inc;
    conf.regPositionIters = 0;
    conf.regVelocityIters = 0;
    conf.toiPositionIters = 0;
    conf.toiVelocityIters = 0;
    conf.tolerance = nextafter(StripUnit(conf.targetDepth), Real{0}) * Meter;

    auto steps = unsigned{0};
    while (GetX(pos_a) < (x * Meter) && GetX(pos_b) > (-x * Meter))
    {
        world.Step(conf);
        ++steps;
        
        EXPECT_TRUE(AlmostEqual(Real{GetX(GetLocation(world, body_a)) / Meter},
                                Real{(GetX(pos_a) + x * time_inc * 1_mps) / Meter}));
        EXPECT_EQ(GetY(GetLocation(world, body_a)), 0_m);
        EXPECT_TRUE(AlmostEqual(Real{GetX(GetLocation(world, body_b)) / Meter},
                                Real{(GetX(pos_b) - x * time_inc * 1_mps) / Meter}));
        EXPECT_EQ(GetY(GetLocation(world, body_b)), 0_m);

        EXPECT_EQ(GetX(GetLinearVelocity(world, body_a)), +x * 1_mps);
        EXPECT_EQ(GetY(GetLinearVelocity(world, body_a)), 0_mps);
        EXPECT_EQ(GetX(GetLinearVelocity(world, body_b)), -x * 1_mps);
        EXPECT_EQ(GetY(GetLinearVelocity(world, body_b)), 0_mps);

        pos_a = GetLocation(world, body_a);
        pos_b = GetLocation(world, body_b);
    }
    
    // d = v * t
    // d = 20, v = 10:
    // 20 = 10 * t, t = d/v = 20 / 10 = 2
    // steps = t / time_inc = 200
    EXPECT_GE(steps, 199u);
    EXPECT_LE(steps, 201u);
    //EXPECT_EQ(int64_t(steps), static_cast<int64_t>(round(((x * 2) / x) / time_inc)));
}

TEST(World, HeavyOnLight)
{
    constexpr auto AngularSlop = (Pi * Real{2} * 1_rad) / Real{180};
    constexpr auto LargerLinearSlop = playrho::Meter / playrho::Real(200);
    constexpr auto SmallerLinearSlop = playrho::Meter / playrho::Real(1000);

    const auto bd = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity);
    const auto upperBodyConf = BodyConf(bd).UseLocation(Vec2(0.0f, 6.0f) * Meter);
    const auto lowerBodyConf = BodyConf(bd).UseLocation(Vec2(0.0f, 0.5f) * Meter);

    const auto groundConf = EdgeShapeConf{}
        .Set(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter);
    
    const auto diskConf = DiskShapeConf{}.UseDensity(10_kgpm2);
    const auto smallerDiskConf = DiskShapeConf(diskConf).UseRadius(0.5_m);
    const auto biggerDiskConf = DiskShapeConf(diskConf).UseRadius(5.0_m);
    
    const auto baseStepConf = []() {
        auto step = StepConf{};
        step.deltaTime = Real(1) / 60_Hz;
        return step;
    }();
    const auto largerStepConf = [=](StepConf step) {
        step.linearSlop = LargerLinearSlop;
        step.regMinSeparation = -LargerLinearSlop * Real(3);
        step.toiMinSeparation = -LargerLinearSlop * Real(1.5f);
        step.targetDepth = LargerLinearSlop * Real(3);
        step.tolerance = LargerLinearSlop / Real(4);
        step.maxLinearCorrection = LargerLinearSlop * Real(40);
        step.maxAngularCorrection = AngularSlop * Real{4};
        step.aabbExtension = LargerLinearSlop * Real(20);
        step.maxTranslation = Length{Meter * Real(4)};
        step.velocityThreshold = (Real{8} / Real{10}) * 1_mps;
        step.maxSubSteps = std::uint8_t{48};
        return step;
    }(baseStepConf);
    const auto smallerStepConf = [=](StepConf step) {
        step.linearSlop = SmallerLinearSlop;
        step.regMinSeparation = -SmallerLinearSlop * Real(3);
        step.toiMinSeparation = -SmallerLinearSlop * Real(1.5f);
        step.targetDepth = SmallerLinearSlop * Real(3);
        step.tolerance = SmallerLinearSlop / Real(4);
        step.maxLinearCorrection = SmallerLinearSlop * Real(40);
        step.maxAngularCorrection = AngularSlop * Real{4};
        step.aabbExtension = SmallerLinearSlop * Real(20);
        step.maxTranslation = Length{Meter * Real(4)};
        step.velocityThreshold = (Real{8} / Real{10}) * 1_mps;
        step.maxSubSteps = std::uint8_t{48};
        return step;
    }(baseStepConf);
    
    // Create lower body, then upper body using the larger step conf
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = CreateBody(world);
        Attach(world, ground, CreateShape(world, groundConf));

        const auto lowerBody = CreateBody(world, lowerBodyConf);
        const auto upperBody = CreateBody(world, upperBodyConf);
        ASSERT_LT(GetY(GetLocation(world, lowerBody)), GetY(GetLocation(world, upperBody)));

        Attach(world, lowerBody, CreateShape(world, smallerDiskConf));
        Attach(world, upperBody, CreateShape(world, biggerDiskConf));
        ASSERT_LT(GetMass(world, lowerBody), GetMass(world, upperBody));

        auto upperBodysLowestPoint = GetY(GetLocation(world, upperBody));
        auto numSteps = 0ul;
        while (GetAwakeCount(world) > 0)
        {
            world.Step(largerStepConf);
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(GetLocation(world, upperBody)));
            ++numSteps;
        }
        
        // The least num steps is 145
        switch (sizeof(Real))
        {
            case 4: EXPECT_EQ(numSteps, 145ul); break; // Effected by Contact IsActive cache errs
            case 8: EXPECT_EQ(numSteps, 145ul); break;
            case 16: EXPECT_EQ(numSteps, 145ul); break;
        }
        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9475154876708984, 0.001);
    }

    // Create upper body, then lower body using the larger step conf
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = CreateBody(world);
        Attach(world, ground, CreateShape(world, groundConf));

        const auto upperBody = CreateBody(world, upperBodyConf);
        const auto lowerBody = CreateBody(world, lowerBodyConf);
        ASSERT_LT(GetY(GetLocation(world, lowerBody)), GetY(GetLocation(world, upperBody)));

        Attach(world, lowerBody, CreateShape(world, smallerDiskConf));
        Attach(world, upperBody, CreateShape(world, biggerDiskConf));
        ASSERT_LT(GetMass(world, lowerBody), GetMass(world, upperBody));

        auto upperBodysLowestPoint = GetY(GetLocation(world, upperBody));
        auto numSteps = 0ul;
        while (GetAwakeCount(world) > 0)
        {
            world.Step(largerStepConf);
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(GetLocation(world, upperBody)));
            ++numSteps;
        }

        // Here we see that creating the upper body after the lower body, results in
        // a different step count, and a higher count at that.
        EXPECT_EQ(numSteps, 152ul);
        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9470911026000977, 0.001);
    }
    
    // Create lower body, then upper body using the smaller step conf
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = CreateBody(world);
        Attach(world, ground, CreateShape(world, groundConf));

        const auto lowerBody = CreateBody(world, lowerBodyConf);
        const auto upperBody = CreateBody(world, upperBodyConf);
        ASSERT_LT(GetY(GetLocation(world, lowerBody)), GetY(GetLocation(world, upperBody)));

        Attach(world, lowerBody, CreateShape(world, smallerDiskConf));
        Attach(world, upperBody, CreateShape(world, biggerDiskConf));
        ASSERT_LT(GetMass(world, lowerBody), GetMass(world, upperBody));

        auto upperBodysLowestPoint = GetY(GetLocation(world, upperBody));
        auto numSteps = 0ul;
        while (GetAwakeCount(world) > 0)
        {
            world.Step(smallerStepConf);
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(GetLocation(world, upperBody)));
            ++numSteps;
        }
        
        // This here is the highest step count.
        // XXX Is this a bug or did the algorithm just work least well here?
        switch (sizeof(Real))
        {
            case 4: EXPECT_EQ(numSteps, 736ul); break; // Effected by Contact IsActive cache errs
            case 8: EXPECT_EQ(numSteps, 736ul); break;
            case 16: EXPECT_EQ(numSteps, 736ul); break;
        }

        // Here we see that the upper body at some point sunk into most of the lower body.
        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9473052024841309, 0.001);
    }
    
    // Create upper body, then lower body using the smaller step conf
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = CreateBody(world);
        Attach(world, ground, CreateShape(world, groundConf));

        const auto upperBody = CreateBody(world, upperBodyConf);
        const auto lowerBody = CreateBody(world, lowerBodyConf);
        ASSERT_LT(GetY(GetLocation(world, lowerBody)), GetY(GetLocation(world, upperBody)));

        Attach(world, lowerBody, CreateShape(world, smallerDiskConf));
        Attach(world, upperBody, CreateShape(world, biggerDiskConf));
        ASSERT_LT(GetMass(world, lowerBody), GetMass(world, upperBody));

        auto upperBodysLowestPoint = GetY(GetLocation(world, upperBody));
        auto numSteps = 0ul;
        EXPECT_EQ(GetAwakeCount(world), 2);
        while (GetAwakeCount(world) > 0)
        {
            world.Step(smallerStepConf);
            EXPECT_EQ(GetTouchingCount(world), ContactCounter(2));
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(GetLocation(world, upperBody)));
            ++numSteps;
        }

        // Given that this section of code is one of the two sections that
        // uses the smaller linear slop, I expect this block of code's step
        // count to be higher than either block using the larger linear slop.
        // I guess a step count of some 3.5 times higher is reasonable for
        // the step conf that's five times smaller.
        switch (sizeof(Real))
        {
            case 4: EXPECT_EQ(numSteps, 724ul); break;
            case 8: EXPECT_EQ(numSteps, 724ul); break;
            case 16: EXPECT_EQ(numSteps, 724ul); break;
        }

        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9476470947265625, 0.001);
    }
    
    // Create upper body, then lower body using the smaller step conf, and using sensors
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = CreateBody(world);
        Attach(world, ground, CreateShape(world, groundConf));

        const auto upperBody = CreateBody(world, upperBodyConf);
        const auto lowerBody = CreateBody(world, lowerBodyConf);
        ASSERT_LT(GetY(GetLocation(world, lowerBody)), GetY(GetLocation(world, upperBody)));
        
        Attach(world, lowerBody, CreateShape(world, DiskShapeConf{smallerDiskConf}.UseIsSensor(true)));
        Attach(world, upperBody, CreateShape(world, DiskShapeConf{biggerDiskConf}.UseIsSensor(true)));
        ASSERT_LT(GetMass(world, lowerBody), GetMass(world, upperBody));

        EXPECT_EQ(GetAwakeCount(world), BodyCounter(2));
        world.Step(smallerStepConf);
        EXPECT_EQ(GetTouchingCount(world), ContactCounter(2));
    }
}

TEST(World, PerfectlyOverlappedSameCirclesStayPut)
{
    auto world = World{};

    const auto radius = 1_m;
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(radius).UseDensity(1_kgpm2).UseRestitution(Real(1)));

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.location = Length2{0_m, 0_m};
    const auto body1 = CreateBody(world, body_def);
    Attach(world, body1, shapeId);
    ASSERT_EQ(GetLocation(world, body1), body_def.location);
    
    const auto body2 = CreateBody(world, body_def);
    Attach(world, body2, shapeId);
    ASSERT_EQ(GetLocation(world, body2), body_def.location);
    
    const auto time_inc = Real(.01);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, 1_s * time_inc);
        EXPECT_EQ(GetLocation(world, body1), body_def.location);
        EXPECT_EQ(GetLocation(world, body2), body_def.location);
    }
}

TEST(World, PerfectlyOverlappedConcentricCirclesStayPut)
{
    World world{};

    const auto radius1 = 1_m;
    const auto radius2 = 0.6_m;
    const auto shapeId1 = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius1));
    const auto shapeId2 = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius2));

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.location = Length2{};
    
    const auto body1 = CreateBody(world, body_def);
    Attach(world, body1, shapeId1);
    ASSERT_EQ(GetLocation(world, body1), body_def.location);
    
    const auto body2 = CreateBody(world, body_def);
    Attach(world, body2, shapeId2);
    ASSERT_EQ(GetLocation(world, body2), body_def.location);

    const auto time_inc = Real(.01);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, 1_s * time_inc);
        EXPECT_EQ(GetLocation(world, body1), body_def.location);
        EXPECT_EQ(GetLocation(world, body2), body_def.location);
    }
}

TEST(World, ListenerCalledForCircleBodyWithinCircleBody)
{
    World world{};
    MyContactListener listener{
        world,
        [&](ContactID, const Manifold&) {},
        [&](ContactID, const ContactImpulsesList&, unsigned) {},
        [&](ContactID) {},
    };
    world.SetBeginContactListener([&listener](ContactID id) {
        listener.BeginContact(id);
    });
    world.SetEndContactListener([&listener](ContactID id) {
        listener.EndContact(id);
    });
    world.SetPreSolveContactListener([&listener](ContactID id, const Manifold& manifold) {
        listener.PreSolve(id, manifold);
    });
    world.SetPostSolveContactListener([&listener](ContactID id,
                                                  const ContactImpulsesList& impulses,
                                                  unsigned count){
        listener.PostSolve(id, impulses, count);
    });

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(1_m));
    for (auto i = 0; i < 2; ++i)
    {
        const auto body = CreateBody(world, body_def);
        ASSERT_NE(body, InvalidBodyID);
        Attach(world, body, shapeId);
    }

    ASSERT_EQ(listener.begin_contacts, 0u);
    ASSERT_EQ(listener.end_contacts, 0u);
    ASSERT_EQ(listener.pre_solves, 0u);
    ASSERT_EQ(listener.post_solves, 0u);

    Step(world, 1_s);

    EXPECT_NE(listener.begin_contacts, 0u);
    EXPECT_EQ(listener.end_contacts, 0u);
    EXPECT_NE(listener.pre_solves, 0u);
    EXPECT_NE(listener.post_solves, 0u);
}

TEST(World, ListenerCalledForSquareBodyWithinSquareBody)
{
    World world{};
    MyContactListener listener{
        world,
        [&](ContactID, const Manifold&) {},
        [&](ContactID, const ContactImpulsesList&, unsigned) {},
        [&](ContactID) {},
    };
    world.SetBeginContactListener([&listener](ContactID id) {
        listener.BeginContact(id);
    });
    world.SetEndContactListener([&listener](ContactID id) {
        listener.EndContact(id);
    });
    world.SetPreSolveContactListener([&listener](ContactID id, const Manifold& manifold) {
        listener.PreSolve(id, manifold);
    });
    world.SetPostSolveContactListener([&listener](ContactID id,
                                                  const ContactImpulsesList& impulses,
                                                  unsigned count){
        listener.PostSolve(id, impulses, count);
    });

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{};
    auto conf = PolygonShapeConf{};
    conf.UseVertexRadius(1_m);
    conf.SetAsBox(2_m, 2_m);
    conf.UseDensity(1_kgpm2);
    conf.UseRestitution(Real(1));
    const auto shapeId = CreateShape(world, conf);
    for (auto i = 0; i < 2; ++i)
    {
        const auto body = CreateBody(world, body_def);
        ASSERT_NE(body, InvalidBodyID);
        Attach(world, body, shapeId);
    }
    
    ASSERT_EQ(listener.begin_contacts, 0u);
    ASSERT_EQ(listener.end_contacts, 0u);
    ASSERT_EQ(listener.pre_solves, 0u);
    ASSERT_EQ(listener.post_solves, 0u);
    
    Step(world, 1_s);
    
    EXPECT_NE(listener.begin_contacts, 0u);
    EXPECT_EQ(listener.end_contacts, 0u);
    EXPECT_NE(listener.pre_solves, 0u);
    EXPECT_NE(listener.post_solves, 0u);
}

TEST(World, DropDisks)
{
    const auto diskRadius = 0.5_m;
    const auto numDisks = static_cast<std::uint16_t>(10000u);
    auto world = World{};
    auto shapeId = InvalidShapeID;
    ASSERT_NO_THROW(shapeId = world.CreateShape(Shape{DiskShapeConf{}.UseRadius(diskRadius)}));
    for (auto i = decltype(numDisks){0}; i < numDisks; ++i) {
        const auto x = i * diskRadius * 4;
        const auto location = Length2{x, 0_m};
        const auto body = CreateBody(world, playrho::d2::BodyConf{}
                                           .UseType(playrho::BodyType::Dynamic)
                                           .UseLocation(location)
                                           .UseLinearAcceleration(playrho::d2::EarthlyGravity));
        ASSERT_NO_THROW(Attach(world, body, shapeId));
    }
    ASSERT_EQ(size(world.GetBodies()), numDisks);
    ASSERT_EQ(world.GetTree().GetNodeCount(), 0u);
    ASSERT_EQ(world.GetTree().GetNodeCapacity(), 4096u);

    const auto stepConf = playrho::StepConf{};
    auto stats = StepStats{};
    ASSERT_NO_THROW(stats = world.Step(stepConf));

    EXPECT_EQ(world.GetTree().GetNodeCount(), 19999u);
    EXPECT_EQ(world.GetTree().GetNodeCapacity(), 32768u);

    EXPECT_EQ(stats.reg.islandsFound, numDisks);
    EXPECT_EQ(stats.reg.islandsSolved, numDisks);
    EXPECT_EQ(stats.reg.maxIslandBodies, 1u);
    EXPECT_EQ(stats.reg.proxiesMoved, 0u);
    EXPECT_EQ(stats.reg.contactsAdded, 0u);
    EXPECT_EQ(stats.reg.maxIncImpulse, Momentum(0));
    EXPECT_EQ(stats.reg.minSeparation, std::numeric_limits<Length>::infinity());
    EXPECT_EQ(stats.reg.bodiesSlept, 0u);
    EXPECT_EQ(stats.reg.sumPosIters, numDisks);
    EXPECT_EQ(stats.reg.sumVelIters, numDisks);

    EXPECT_EQ(stats.toi.minSeparation, std::numeric_limits<Length>::infinity());
    EXPECT_EQ(stats.toi.maxIncImpulse, Momentum(0));
    EXPECT_EQ(stats.toi.islandsFound, 0u);
    EXPECT_EQ(stats.toi.islandsSolved, 0u);
    EXPECT_EQ(stats.toi.contactsFound, 0u);
    EXPECT_EQ(stats.toi.contactsAdded, 0u);
    EXPECT_EQ(stats.toi.proxiesMoved, 0u);
    EXPECT_EQ(stats.toi.sumPosIters, 0u);
    EXPECT_EQ(stats.toi.sumVelIters, 0u);

    for (auto i = decltype(numDisks){0}; i < numDisks; ++i) {
        const auto& body = world.GetBody(BodyID(i));
        EXPECT_EQ(GetX(body.GetVelocity().linear), 0_mps);
        EXPECT_LT(GetY(body.GetVelocity().linear), 0_mps);
    }

    for (auto i = 1; i < 8; ++i) {
        EXPECT_NO_THROW(stats = world.Step(stepConf));
        SCOPED_TRACE(std::string("#-steps is ") + std::to_string(i));
        EXPECT_EQ(stats.reg.proxiesMoved, 0u);
    }

    EXPECT_NO_THROW(stats = world.Step(stepConf));
    EXPECT_EQ(stats.reg.proxiesMoved, numDisks);
    EXPECT_EQ(stats.toi.islandsFound, 0u);
    EXPECT_EQ(stats.toi.islandsSolved, 0u);
    EXPECT_EQ(stats.toi.contactsFound, 0u);
    EXPECT_EQ(stats.toi.contactsAdded, 0u);
    EXPECT_EQ(stats.toi.proxiesMoved, 0u);
    EXPECT_EQ(stats.toi.sumPosIters, 0u);
    EXPECT_EQ(stats.toi.sumVelIters, 0u);

    EXPECT_EQ(world.GetTree().GetNodeCount(), 19999u);
    EXPECT_EQ(world.GetTree().GetNodeCapacity(), 32768u);
}

TEST(World, PartiallyOverlappedSameCirclesSeparate)
{
    const auto radius = Real(1);
    
    World world{};
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false; // separation is faster if true.
    const auto shape = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius * Meter));
    const auto body1pos = Length2{(-radius/4) * Meter, 0_m};
    body_def.location = body1pos;
    const auto body1 = CreateBody(world, body_def);
    Attach(world, body1, shape);
    ASSERT_EQ(GetLocation(world, body1), body_def.location);
    
    const auto body2pos = Length2{(+radius/4) * Meter, 0_m};
    body_def.location = body2pos;
    const auto body2 = CreateBody(world, body_def);
    Attach(world, body2, shape);
    ASSERT_EQ(GetLocation(world, body2), body_def.location);
    
    auto position_diff = body2pos - body1pos;
    auto distance = GetMagnitude(position_diff);

    const auto angle = GetAngle(position_diff);
    ASSERT_EQ(angle, 0_deg);

    auto lastpos1 = GetLocation(world, body1);
    auto lastpos2 = GetLocation(world, body2);

    const auto time_inc = .01_s;
    StepConf step;
    step.deltaTime = time_inc;

    // Solver won't separate more than -step.linearSlop.
    const auto full_separation = radius * 2_m - Length{step.linearSlop};
    for (auto i = 0; i < 100; ++i)
    {
        world.Step(step);

        const auto new_pos_diff = GetLocation(world, body2) - GetLocation(world, body1);
        const auto new_distance = GetMagnitude(new_pos_diff);
        
        if (AlmostEqual(Real{new_distance / Meter}, Real{full_separation / Meter}) || new_distance > full_separation)
        {
            break;
        }
        
        ASSERT_GE(new_distance, distance);

        if (new_distance == distance)
        {
            // position resolution has come to tolerance
            ASSERT_GE(new_distance, radius * 2_m - Length{step.linearSlop} * Real{4});
            break;
        }
        else // new_distance > distance
        {
            if (cos(angle) != 0)
            {
                EXPECT_LT(GetX(GetLocation(world, body1)), GetX(lastpos1));
                EXPECT_GT(GetX(GetLocation(world, body2)), GetX(lastpos2));
            }
            if (sin(angle) != 0)
            {
                EXPECT_LT(GetY(GetLocation(world, body1)), GetY(lastpos1));
                EXPECT_GT(GetY(GetLocation(world, body2)), GetY(lastpos2));
            }
        }

        ASSERT_NE(GetLocation(world, body1), lastpos1);
        ASSERT_NE(GetLocation(world, body2), lastpos2);
        
        lastpos1 = GetLocation(world, body1);
        lastpos2 = GetLocation(world, body2);

        ASSERT_NE(new_pos_diff, position_diff);
        position_diff = new_pos_diff;

        ASSERT_NE(new_distance, distance);
        distance = new_distance;

        // angle of the delta of their positions should stay the same as they move away
        const auto new_angle = GetAngle(new_pos_diff);
        EXPECT_EQ(angle, new_angle);
    }
}

TEST(World, PerfectlyOverlappedSameSquaresSeparateHorizontally)
{
    World world{};
    const auto shape = CreateShape(world, PolygonShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).SetAsBox(1_m, 1_m));

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.location = Length2{};
    
    const auto body1 = CreateBody(world, body_def);
    Attach(world, body1, shape);
    ASSERT_EQ(GetLocation(world, body1), body_def.location);
    
    const auto body2 = CreateBody(world, body_def);
    Attach(world, body2, shape);
    ASSERT_EQ(GetLocation(world, body2), body_def.location);
    
    auto lastpos1 = GetLocation(world, body1);
    auto lastpos2 = GetLocation(world, body2);

    auto stepConf = StepConf{};
    const auto time_inc = .01_s;
    stepConf.deltaTime = time_inc;
    stepConf.maxLinearCorrection = Real{0.0001f * 40} * Meter;
    for (auto i = 0; i < 100; ++i)
    {
        world.Step(stepConf);
        
        // body1 moves left only
        EXPECT_LT(GetX(GetLocation(world, body1)), GetX(lastpos1));
        EXPECT_EQ(GetY(GetLocation(world, body1)), GetY(lastpos1));

        // body2 moves right only
        EXPECT_GT(GetX(GetLocation(world, body2)), GetX(lastpos2));
        EXPECT_EQ(GetY(GetLocation(world, body2)), GetY(lastpos2));
        
        // body1 and body2 move away from each other equally.
        EXPECT_EQ(GetX(GetLocation(world, body1)), -GetX(GetLocation(world, body2)));
        EXPECT_EQ(GetY(GetLocation(world, body1)), -GetY(GetLocation(world, body2)));
        
        lastpos1 = GetLocation(world, body1);
        lastpos2 = GetLocation(world, body2);
    }
}

TEST(World, PartiallyOverlappedSquaresSeparateProperly)
{
    /*
     * Sets up 2 equally sized squares - body A and body B - where body A is to the right of body B
     * but they partially overlap. Position solver code should move body A to the right more and
     * move body B to the left more till they're almost separated.
     *
     * This tests at a high level what the position solver code does with overlapping shapes.
     */

    World world{};
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false; // separation is faster if true.
    
    const auto half_dim = Real(64); // 1 causes additional y-axis separation
    const auto shape = CreateShape(world, PolygonShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).SetAsBox(half_dim * Meter, half_dim * Meter));
    
    const auto body1pos = Length2{Real(half_dim/2) * Meter, 0_m}; // 0 causes additional y-axis separation
    body_def.location = body1pos;
    const auto body1 = CreateBody(world, body_def);
    Attach(world, body1, shape);
    ASSERT_EQ(GetLocation(world, body1), body1pos);
    
    const auto body2pos = Length2{-Real(half_dim/2) * Meter, 0_m}; // 0 causes additional y-axis separation
    body_def.location = body2pos;
    const auto body2 = CreateBody(world, body_def);
    Attach(world, body2, shape);
    ASSERT_EQ(GetLocation(world, body2), body2pos);

    ASSERT_EQ(GetAngle(world, body1), 0_deg);
    ASSERT_EQ(GetAngle(world, body2), 0_deg);
    auto last_angle_1 = GetAngle(world, body1);
    auto last_angle_2 = GetAngle(world, body2);

    ASSERT_EQ(world.GetBodies().size(), World::Bodies::size_type(2));
    ASSERT_EQ(world.GetContacts().size(), World::Contacts::size_type(0));

    auto position_diff = body1pos - body2pos;
    auto distance = GetMagnitude(position_diff);
    
    auto angle = GetAngle(position_diff);
    EXPECT_TRUE(AlmostEqual(Real{angle / 1_rad}, Real{0}));
    
    auto lastpos1 = GetLocation(world, body1);
    auto lastpos2 = GetLocation(world, body2);
    
    const auto velocity_iters = 10u;
    const auto position_iters = 10u;
    
    const auto time_inc = Real(.01);
    StepConf step;
    step.deltaTime = 1_s * time_inc;
    // Solver won't separate more than -step.linearSlop.
    const auto full_separation = half_dim * 2_m - Length{step.linearSlop};
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, 1_s * time_inc, velocity_iters, position_iters);
        
        ASSERT_EQ(world.GetContacts().size(), decltype(world.GetContacts().size())(1));

        auto count = decltype(world.GetContacts().size())(0);
        const auto& contacts = world.GetContacts();
        for (auto&& contact: contacts)
        {
            ++count;
            const auto c = contact.second;
            const auto body_a = GetBodyA(world, c);
            const auto body_b = GetBodyB(world, c);
            EXPECT_EQ(body_a, body1);
            EXPECT_EQ(body_b, body2);
            const auto& manifold = GetManifold(world, c);
            EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
            EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
        }
        ASSERT_EQ(count, decltype(world.GetContacts().size())(1));

        const auto v1 = GetVelocity(world, body1);
        EXPECT_EQ(v1.angular, 0_deg / 1_s);
        EXPECT_EQ(GetX(v1.linear), 0_mps);
        EXPECT_EQ(GetY(v1.linear), 0_mps);

        const auto v2 = GetVelocity(world, body2);
        EXPECT_EQ(v2.angular, 0_deg / 1_s);
        EXPECT_EQ(GetX(v2.linear), 0_mps);
        EXPECT_EQ(GetY(v2.linear), 0_mps);

        EXPECT_TRUE(AlmostEqual(Real{GetAngle(world, body1) / 1_rad}, Real{last_angle_1 / 1_rad}));
        EXPECT_TRUE(AlmostEqual(Real{GetAngle(world, body2) / 1_rad}, Real{last_angle_2 / 1_rad}));
        last_angle_1 = GetAngle(world, body1);
        last_angle_2 = GetAngle(world, body2);

        const auto new_pos_diff = GetLocation(world, body1) - GetLocation(world, body2);
        const auto new_distance = GetMagnitude(new_pos_diff);
        
        if (AlmostEqual(Real{new_distance / Meter}, Real{full_separation / Meter}) || new_distance > full_separation)
        {
            break;
        }
        
        if (new_distance == distance)
        {
            if (cos(angle) != 0)
            {
                EXPECT_NE(GetX(GetLocation(world, body1)), GetX(lastpos1));
                EXPECT_NE(GetX(GetLocation(world, body2)), GetX(lastpos2));
            }
            if (sin(angle) != 0)
            {
                EXPECT_NE(GetY(GetLocation(world, body1)), GetY(lastpos1));
                EXPECT_NE(GetY(GetLocation(world, body2)), GetY(lastpos2));
            }
            ASSERT_GE(new_distance, 2_m);
            break;
        }
        
        ASSERT_NE(GetLocation(world, body1), lastpos1);
        ASSERT_NE(GetLocation(world, body2), lastpos2);
        
        // The body 1 moves right only.
        EXPECT_GT(GetX(GetLocation(world, body1)), GetX(lastpos1));
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLocation(world, body1)) / Meter}, Real{GetY(lastpos1) / Meter}));

        // The body 2 moves left only.
        EXPECT_LT(GetX(GetLocation(world, body2)), GetX(lastpos2));
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLocation(world, body2)) / Meter}, Real{GetY(lastpos2) / Meter}));

        lastpos1 = GetLocation(world, body1);
        lastpos2 = GetLocation(world, body2);
        
        ASSERT_NE(new_pos_diff, position_diff);
        position_diff = new_pos_diff;
        
        ASSERT_NE(new_distance, distance);
        distance = new_distance;
        
        const auto new_angle = GetAngle(new_pos_diff);
        EXPECT_TRUE(AlmostEqual(Real{angle / 1_rad}, Real{new_angle / 1_rad}));
        
        angle = new_angle;
    }
}

TEST(World, CollidingDynamicBodies)
{
    const auto radius = 1_m;
    const auto x = Real(10); // other test parameters tuned to this value being 10
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;

    World world{};
    MyContactListener listener{
        world,
        [&](ContactID, const Manifold&) {},
        [&](ContactID, const ContactImpulsesList&, unsigned) {},
        [&](ContactID) {},
    };
    world.SetBeginContactListener([&listener](ContactID id) {
        listener.BeginContact(id);
    });
    world.SetEndContactListener([&listener](ContactID id) {
        listener.EndContact(id);
    });
    world.SetPreSolveContactListener([&listener](ContactID id, const Manifold& manifold) {
        listener.PreSolve(id, manifold);
    });
    world.SetPostSolveContactListener([&listener](ContactID id,
                                                  const ContactImpulsesList& impulses,
                                                  unsigned count){
        listener.PostSolve(id, impulses, count);
    });
    
    const auto shape = CreateShape(world, DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius));

    body_def.location = Length2{-(x + 1) * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{+x * 1_mps, 0_mps};
    const auto body_a = CreateBody(world, body_def);
    ASSERT_NE(body_a, InvalidBodyID);
    ASSERT_EQ(GetType(world, body_a), BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(world, body_a));
    ASSERT_TRUE(IsAccelerable(world, body_a));
    Attach(world, body_a, shape);

    body_def.location = Length2{+(x + 1) * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{-x * 1_mps, 0_mps};
    const auto body_b = CreateBody(world, body_def);
    ASSERT_NE(body_b, InvalidBodyID);
    ASSERT_EQ(GetType(world, body_b), BodyType::Dynamic);
    ASSERT_TRUE(IsSpeedable(world, body_b));
    ASSERT_TRUE(IsAccelerable(world, body_b));
    Attach(world, body_b, shape);

    EXPECT_EQ(GetX(GetLinearVelocity(world, body_a)), +x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body_a)), 0_mps);
    EXPECT_EQ(GetX(GetLinearVelocity(world, body_b)), -x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body_b)), 0_mps);
    
    const auto time_collision = 1.0099994_s; // only valid for x >= around 4.214
    const auto time_inc = 0.01_s;
    
    auto elapsed_time = 0_s;
    for (;;) {
        Step(world, time_inc);
        elapsed_time += time_inc;
        if (listener.contacting || elapsed_time >= 600_s) {
            break;
        }
    }
    EXPECT_LT(elapsed_time, 600_s);
    EXPECT_TRUE(listener.contacting);
    
    // Call SetSensor to add some unit test coverage of these methods.
    EXPECT_FALSE(GetContacts(world, body_a).empty());
    for (const auto& ci: GetContacts(world, body_a))
    {
        EXPECT_FALSE(NeedsFiltering(world, ci.second));
        EXPECT_TRUE(NeedsUpdating(world, ci.second));
    }
    auto filter = GetFilterData(world, shape);
    filter.categoryBits = ~filter.categoryBits;
    EXPECT_NO_THROW(SetFilterData(world, shape, filter));
    EXPECT_FALSE(IsSensor(world, shape));
    SetSensor(world, shape, true);
    EXPECT_TRUE(IsSensor(world, shape));
    SetSensor(world, shape, false);
    EXPECT_FALSE(IsSensor(world, shape));
    EXPECT_FALSE(GetContacts(world, body_a).empty());
    for (auto&& ci: GetContacts(world, body_a))
    {
        EXPECT_TRUE(NeedsFiltering(world, ci.second));
        EXPECT_TRUE(NeedsUpdating(world, ci.second));
    }

    const auto time_contacting = elapsed_time;

    EXPECT_TRUE(listener.touching);
    EXPECT_NEAR(double(time_contacting / 1_s), double(time_collision / 1_s), 0.02);
    EXPECT_EQ(GetY(GetLocation(world, body_a)), 0_m);
    EXPECT_EQ(GetY(GetLocation(world, body_b)), 0_m);

    const auto tolerance = x / 100;
    
    // x position for body1 depends on restitution but it should be around -1
    EXPECT_GE(GetX(GetLocation(world, body_a)) / Meter, Real(-1) - tolerance);
    EXPECT_LT(GetX(GetLocation(world, body_a)) / Meter, Real(-1) + tolerance);

    // x position for body2 depends on restitution but it should be around +1
    EXPECT_LE(GetX(GetLocation(world, body_b)) / Meter, Real(+1) + tolerance);
    EXPECT_GT(GetX(GetLocation(world, body_b)) / Meter, Real(+1) - tolerance);
    
    // and their deltas from -1 and +1 should be about equal.
    EXPECT_TRUE(AlmostEqual(Real{(GetX(GetLocation(world, body_a)) + 1_m) / Meter},
                            Real{(1_m - GetX(GetLocation(world, body_b))) / Meter}));

    EXPECT_GE(GetX(listener.body_a[0]), -1_m);
    EXPECT_LE(GetX(listener.body_b[0]), +1_m);

    for (;;)
    {
        Step(world, time_inc);
        elapsed_time += time_inc;
        if (!listener.contacting && !listener.touching)
        {
            break;
        }
    }
    EXPECT_FALSE(listener.touching);

    EXPECT_TRUE(AlmostEqual(double(elapsed_time / 1_s), double((time_contacting + time_inc) / 1_s)));

    // collision should be fully resolved now...
    EXPECT_LT(GetX(GetLocation(world, body_a)), -1_m);
    EXPECT_GT(GetX(GetLocation(world, body_b)), +1_m);

    // and their deltas from -1 and +1 should be about equal.
    EXPECT_TRUE(AlmostEqual(Real{(GetX(GetLocation(world, body_a)) + 1_m) / Meter},
                            Real{(1_m - GetX(GetLocation(world, body_b))) / Meter}));

    EXPECT_LT(GetX(listener.body_a[1]), -1_m);
    EXPECT_GT(GetX(listener.body_b[1]), +1_m);

    // confirm conservation of momentum:
    // velocities should now be same magnitude but in opposite directions
    EXPECT_NEAR(double(Real{GetX(GetLinearVelocity(world, body_a)) / 1_mps}),
                double(-x), 0.0001);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body_a)), 0_mps);
    EXPECT_NEAR(double(Real{GetX(GetLinearVelocity(world, body_b)) / 1_mps}),
                double(+x), 0.0001);
    EXPECT_EQ(GetY(GetLinearVelocity(world, body_b)), 0_mps);
}

TEST(World_Longer, TilesComesToRest)
{
    constexpr auto LinearSlop = Meter / 1000;
    constexpr auto AngularSlop = (Pi * 2 * 1_rad) / 180;
    constexpr auto VertexRadius = LinearSlop * 2;
    auto conf = PolygonShapeConf{}.UseVertexRadius(VertexRadius);
    const auto world = std::make_unique<World>(WorldConf{}.UseMinVertexRadius(VertexRadius));
    
    constexpr auto e_count = 36;
    auto createdBodyCount = 0ul;
    
    {
        const auto a = Real{0.5f};
        //const auto ground = CreateBody(*world, BodyConf{}.UseLocation(Length2{0, -a * Meter}));
        auto ground = Body(BodyConf{}.UseLocation(Length2{0, -a * Meter}));
        const auto N = 200;
        const auto M = 10;
        Length2 position;
        GetY(position) = 0.0_m;
        for (auto j = 0; j < M; ++j) {
            GetX(position) = -N * a * Meter;
            for (auto i = 0; i < N; ++i) {
                conf.SetAsBox(a * Meter, a * Meter, position, 0_deg);
                ground.Attach(CreateShape(*world, conf));
                GetX(position) += 2.0f * a * Meter;
            }
            GetY(position) -= 2.0f * a * Meter;
        }
        ASSERT_EQ(size(ground.GetShapes()), 2000u);
        ASSERT_NO_THROW(CreateBody(*world, ground));
        ++createdBodyCount;
    }
    
    {
        const auto a = Real{0.5f};
        conf.UseDensity(5_kgpm2);
        conf.SetAsBox(a * Meter, a * Meter);
        const auto shapeId = world->CreateShape(Shape(conf));

        Length2 x(-7.0_m, 0.75_m);
        Length2 y;
        const auto deltaX = Length2(0.5625_m, 1.25_m);
        const auto deltaY = Length2(1.125_m, 0.0_m);

        for (auto i = 0; i < e_count; ++i) {
            y = x;
            for (auto j = i; j < e_count; ++j) {
                auto body = Body{BodyConf{}.UseType(BodyType::Dynamic).UseLocation(y).UseLinearAcceleration(EarthlyGravity)};
                ASSERT_NO_THROW(body.Attach(shapeId));
                ASSERT_NO_THROW(CreateBody(*world, body));
                ++createdBodyCount;
                y += deltaY;
            }
            x += deltaX;
        }
    }

    EXPECT_EQ(createdBodyCount, 667u);
    EXPECT_EQ(GetBodyCount(*world), 667u);

    StepConf step;
    step.deltaTime = 1_s / 60;
    step.linearSlop = LinearSlop;
    step.regMinSeparation = -LinearSlop * Real(3);
    step.toiMinSeparation = -LinearSlop * Real(1.5f);
    step.targetDepth = LinearSlop * Real(3);
    step.tolerance = LinearSlop / Real(4);
    step.maxLinearCorrection = LinearSlop * Real(40);
    step.maxAngularCorrection = AngularSlop * Real{4};
    step.aabbExtension = LinearSlop * Real(20);
    step.maxTranslation = Length{Meter * Real(4)};
    step.velocityThreshold = (Real{8} / Real{10}) * 1_mps;
    step.maxSubSteps = std::uint8_t{48};

    auto numSteps = 0ul;
    auto sumRegPosIters = 0ul;
    auto sumRegVelIters = 0ul;
    auto sumToiPosIters = 0ul;
    auto sumToiVelIters = 0ul;
    //const auto start_time = std::chrono::high_resolution_clock::now();
    auto lastStats = StepStats{};
    auto firstStepWithZeroMoved = std::optional<unsigned long>{};
    auto totalBodiesSlept = 0ul;
    auto awakeCount = 0ul;
    constexpr auto maxSteps = 3000ul;
    while ((awakeCount = GetAwakeCount(*world)) > 0 && numSteps < maxSteps) {
        const auto stats = world->Step(step);
        sumRegPosIters += stats.reg.sumPosIters;
        sumRegVelIters += stats.reg.sumVelIters;
        sumToiPosIters += stats.toi.sumPosIters;
        sumToiVelIters += stats.toi.sumVelIters;
        lastStats = stats;
        if (stats.reg.proxiesMoved == 0u && stats.toi.proxiesMoved == 0u) {
            firstStepWithZeroMoved = numSteps;
        }
        EXPECT_GT(stats.reg.islandsFound, 0u);
        totalBodiesSlept += stats.reg.bodiesSlept;
        ++numSteps;
    }
    ASSERT_LT(numSteps, maxSteps);
    switch (sizeof(Real)) {
    case 4u:
#if defined(__core2__)
        EXPECT_EQ(world->GetContactRange(), 1447u);
        EXPECT_EQ(totalBodiesSlept, createdBodyCount + 1u);
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
            EXPECT_EQ(*firstStepWithZeroMoved, 1798u);
        }
#elif defined(_WIN64)
        EXPECT_EQ(world->GetContactRange(), 1448u);
        EXPECT_EQ(totalBodiesSlept, 667u);
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
            EXPECT_EQ(*firstStepWithZeroMoved, 1812u);
        }
#elif defined(_WIN32)
        EXPECT_EQ(world->GetContactRange(), 1448u);
        EXPECT_EQ(totalBodiesSlept, 671u);
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
            EXPECT_EQ(*firstStepWithZeroMoved, 1801u);
        }
#elif defined(__amd64__) // includes __k8__
        EXPECT_EQ(world->GetContactRange(), 1448u);
        EXPECT_EQ(totalBodiesSlept, 667u);
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
            EXPECT_EQ(*firstStepWithZeroMoved, 1800u);
        }
#elif defined(__arm64__) // At least for Apple Silicon
        EXPECT_GE(world->GetContactRange(), 1447u);
        EXPECT_LE(world->GetContactRange(), 1451u);
        EXPECT_GE(totalBodiesSlept, 667u);
        EXPECT_LE(totalBodiesSlept, 670u);
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
#if defined(PLAYRHO_USE_BOOST_UNITS)
            EXPECT_GE(*firstStepWithZeroMoved, 1797u);
#else
            EXPECT_GE(*firstStepWithZeroMoved, 1798u);
#endif
            EXPECT_LE(*firstStepWithZeroMoved, 1812u);
        }
#else // unrecognized arch; just check results are within range of others
        EXPECT_GE(world->GetContactRange(), 1447u);
        EXPECT_LE(world->GetContactRange(), 1450u);
        EXPECT_GE(totalBodiesSlept, 667u);
        EXPECT_LE(totalBodiesSlept, 668u);
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
            EXPECT_GE(*firstStepWithZeroMoved, 1798u);
            EXPECT_LE(*firstStepWithZeroMoved, 1812u);
        }
#endif
        break;
    case 8u:
        EXPECT_EQ(world->GetContactRange(), 1447u);
#if defined(__GNUC__) && defined(__clang__) && !defined(__core2__) && !defined(__arm64__)
        EXPECT_EQ(totalBodiesSlept, createdBodyCount);
#else
        EXPECT_EQ(totalBodiesSlept, createdBodyCount + 3u);
#endif
        EXPECT_TRUE(firstStepWithZeroMoved);
        if (firstStepWithZeroMoved) {
#if defined(__core2__) || defined(__arm64__) || (defined(__GNUC__) && !defined(__clang__))
            EXPECT_EQ(*firstStepWithZeroMoved, 1827u);
#else
            EXPECT_EQ(*firstStepWithZeroMoved, 1792u);
#endif
        }
        break;
    }
    EXPECT_EQ(world->GetTree().GetNodeCount(), 5331u);
    //const auto end_time = std::chrono::high_resolution_clock::now();
    
    //const std::chrono::duration<double> elapsed_time = end_time - start_time;
    
    // seeing e_count=20 times around:
    //   0.447077s with Real=float and NDEBUG defined.
    //   6.45222s with Real=float and NDEBUG not defined.
    //   0.456306s with Real=double and NDEBUG defined.
    //   6.74324s with Real=double and NDEBUG not defined.
    // seeing e_count=24 times around:
    //   0.956078s with Real=float and NDEBUG defined.
    //   0.989387s with Real=double and NDEBUG defined.
    // seeing e_count=30 times around:
    //   2.35464s with Real=float and NDEBUG defined.
    //   2.51661s with Real=double and NDEBUG defined.
    // seeing e_count=36 times around:
    //   4.163s with Real=float and NDEBUG defined.
    //   5.374s with Real=double and NDEBUG defined.

    EXPECT_EQ(awakeCount, 0u);
    if (awakeCount == 0u) {
#if defined(_WIN32) || defined(PLAYRHO_USE_BOOST_UNITS) // odd
        EXPECT_EQ(lastStats.reg.proxiesMoved, 1u);
#else
        EXPECT_EQ(lastStats.reg.proxiesMoved, 0u);
#endif
        EXPECT_EQ(lastStats.toi.proxiesMoved, 0u);
    }

    // The final stats seem dependent on the host the test is run on.
    // Presume that this is most closely associated with the actual CPU/FPU.

    // Note about commit 6b16f3722d5daac80ebaefd1dfda424939498dd4:
    //   Changed the order in which bodies get added to the world body list
    //   from being added to the front of the list to being added to the back
    //   of the list. Adding bodies to the front of World::m_bodies list
    //   resulted in the world index of bodies changing as new bodies got added.
    //   This wasn't the desired behavior. Time trials of this test with bodies
    //   being added to the back of the m_bodies list also got faster than
    //   when bodies were getting added to the front of the list.
    
    // Note about commit 04f9188c47961cafe76c55eb6b766a608593ee08:
    //   Changed the way velocity constraint resolution was done. Added a check
    //   to see if any changes to velocity were introduced. If not, new code
    //   does an early exit from its velocityIters looping.
    
    // Note about commit d361c51d6aca13079e9d44b701715e62cec18a63:
    //   Changes were introduced that modified the way manifold calculations are done.
    //   While many of the following counts appear to have increased, this new
    //   mechanism for manifold calculations has benefits like no longer needing
    //   "ghost-vertices" to avoid sticking of things like boxes being dragged across
    //   a floor made up of chained edges nor rectangles. As to why the new manifold
    //   calculating method makes the counts change, that's not clear to me since
    //   it doesn't seem that the changes to the manifold calculation would be seen in
    //   this test. That some of these counts actually became lower (in the Core-2 case)
    //   suggests that the change to these counts has more to do with differences in
    //   floating point hardware than in the modifications actually adversely impacting
    //   the algorithmic efficiency of the code.
    //
#if defined(__core2__)
    switch (sizeof(Real))
    {
        case  4:
        {
#ifndef __FAST_MATH__
            EXPECT_EQ(numSteps, 1799ul);
            EXPECT_EQ(sumRegPosIters, 36515ul);
            EXPECT_EQ(sumRegVelIters, 46956ul);
            EXPECT_EQ(sumToiPosIters, 44131ul);
            EXPECT_EQ(sumToiVelIters, 114088ul);
#else
            EXPECT_EQ(numSteps, 1003ul);
            EXPECT_EQ(sumRegPosIters, 52909ul);
            EXPECT_EQ(sumRegVelIters, 103896ul);
            EXPECT_EQ(sumToiPosIters, 20616ul);
            EXPECT_EQ(sumToiVelIters, 30175ul);
#endif
            break;
        }
        case  8:
        {
            EXPECT_EQ(numSteps,         1828ul);
            EXPECT_EQ(sumRegPosIters,  36540ul);
            EXPECT_EQ(sumRegVelIters,  47173ul);
            EXPECT_EQ(sumToiPosIters,  44005ul);
            EXPECT_EQ(sumToiVelIters, 114357ul);
            break;
        }
        case 16:
        {
            break;
        }
        default:
        {
            FAIL(); break;
        }
    }
#elif defined(__k8__)
    switch (sizeof(Real))
    {
        case  4:
        {
            // From commits after 507a7c15c
            EXPECT_EQ(numSteps,         1801ul);
            EXPECT_EQ(sumRegPosIters,  36518ul);
            EXPECT_EQ(sumRegVelIters,  46948ul);
            EXPECT_EQ(sumToiPosIters,  44170ul);
            EXPECT_EQ(sumToiVelIters, 114108ul);
            break;
        }
        case  8:
        {
            EXPECT_EQ(numSteps,         1828ul);
            EXPECT_EQ(sumRegPosIters,  36540ul);
            EXPECT_EQ(sumRegVelIters,  47173ul);
            EXPECT_EQ(sumToiPosIters,  44005ul);
#ifdef NDEBUG // odd, that this changes results - is there compiler "as-if" rule violation?
            EXPECT_EQ(sumToiVelIters, 114399ul);
#else
            EXPECT_EQ(sumToiVelIters, 114406ul);
#endif
            break;
        }
    }
#elif defined(_WIN64) // This is likely wrong as the results are more likely arch dependent
    EXPECT_EQ(numSteps, 1814ul);
    EXPECT_EQ(sumRegPosIters, 36530ul);
    EXPECT_EQ(sumRegVelIters, 47061ul);
    EXPECT_EQ(sumToiPosIters, 43786ul);
    EXPECT_EQ(sumToiVelIters, 112952ul);
#elif defined(_WIN32)
    EXPECT_EQ(numSteps, 1803ul);
    EXPECT_EQ(sumRegPosIters, 36528ul);
    EXPECT_EQ(sumRegVelIters, 46981ul);
    EXPECT_EQ(sumToiPosIters, 43684ul);
    EXPECT_EQ(sumToiVelIters, 112778ul);
#elif defined(__arm64__)
    // At least for Apple Silicon...
    switch (sizeof(Real))
    {
        case 4u:
#if defined(__clang_major__) && (__clang_major__ >= 14)
#if defined(PLAYRHO_USE_BOOST_UNITS)
            EXPECT_EQ(numSteps, 1800ul);
            EXPECT_EQ(sumRegPosIters, 36516ul);
            EXPECT_EQ(sumRegVelIters, 46948ul);
            EXPECT_EQ(sumToiPosIters, 44022ul);
            EXPECT_EQ(sumToiVelIters, 113284ul);
#else
            EXPECT_EQ(numSteps, 1804ul);
            EXPECT_EQ(sumRegPosIters, 36527ul);
            EXPECT_EQ(sumRegVelIters, 46979ul);
            EXPECT_EQ(sumToiPosIters, 43806ul);
            EXPECT_EQ(sumToiVelIters, 112175ul);
#endif
#else
            EXPECT_EQ(numSteps, 1799ul);
            EXPECT_EQ(sumRegPosIters, 36512ul);
            EXPECT_EQ(sumRegVelIters, 46940ul);
            EXPECT_EQ(sumToiPosIters, 44021ul);
            EXPECT_EQ(sumToiVelIters, 113137ul);
#endif
            break;
        case 8u:
            EXPECT_EQ(numSteps, 1828ul);
            EXPECT_EQ(sumRegPosIters, 36540ul);
            EXPECT_EQ(sumRegVelIters, 47173ul);
            EXPECT_EQ(sumToiPosIters, 44005ul);
#if defined(__clang_major__) && (__clang_major__ >= 14) && !defined(PLAYRHO_USE_BOOST_UNITS)
            EXPECT_EQ(sumToiVelIters, 114308ul);
#else
            EXPECT_EQ(sumToiVelIters, 114196ul);
#endif
            break;
    }
#else
    // These will likely fail and need to be tweaked for the particular hardware...
    EXPECT_EQ(numSteps, 1814ul);
    EXPECT_EQ(sumRegPosIters, 36600ul);
    EXPECT_EQ(sumRegVelIters, 264096ul);
    EXPECT_EQ(sumToiPosIters, 45022ul);
    EXPECT_EQ(sumToiVelIters, 148560ul);
#endif
    
    //std::cout << "Time: " << elapsed_time.count() << "s" << std::endl;
    // EXPECT_LT(elapsed_time.count(), 7.0);
}

TEST(World, SpeedingBulletBallWontTunnel)
{
    constexpr auto LinearSlop = playrho::Meter / playrho::Real(1000);
    constexpr auto AngularSlop = (Pi * Real{2} * 1_rad) / Real{180};
    constexpr auto VertexRadius = playrho::Length{LinearSlop * playrho::Real(2)};
    
    World world{WorldConf{}.UseMinVertexRadius(VertexRadius)};
    MyContactListener listener{
        world,
        [&](ContactID, const Manifold&) {},
        [&](ContactID, const ContactImpulsesList&, unsigned) {},
        [&](ContactID) {},
    };
    world.SetBeginContactListener([&listener](ContactID id) {
        listener.BeginContact(id);
    });
    world.SetEndContactListener([&listener](ContactID id) {
        listener.EndContact(id);
    });
    world.SetPreSolveContactListener([&listener](ContactID id, const Manifold& manifold) {
        listener.PreSolve(id, manifold);
    });
    world.SetPostSolveContactListener([&listener](ContactID id,
                                                  const ContactImpulsesList& impulses,
                                                  unsigned count){
        listener.PostSolve(id, impulses, count);
    });

    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    const auto left_edge_x = -0.1_m;
    const auto right_edge_x = +0.1_m;

    const auto edgeConf = EdgeShapeConf{}
        .UseVertexRadius(VertexRadius)
        .UseRestitution(Real(1))
        .Set(Length2{0_m, +10_m}, Length2{0_m, -10_m});
    const auto edge_shape = CreateShape(world, edgeConf);

    BodyConf body_def;
    body_def.type = BodyType::Static;

    body_def.location = Length2{left_edge_x, 0_m};
    const auto left_wall_body = CreateBody(world, body_def);
    ASSERT_NE(left_wall_body, InvalidBodyID);
    Attach(world, left_wall_body, edge_shape);

    body_def.location = Length2{right_edge_x, 0_m};
    const auto right_wall_body = CreateBody(world, body_def);
    ASSERT_NE(right_wall_body, InvalidBodyID);
    Attach(world, right_wall_body, edge_shape);
    
    const auto begin_x = Real(0);

    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{begin_x * Meter, 0_m};
    body_def.bullet = false;
    const auto ball_body = CreateBody(world, body_def);
    ASSERT_NE(ball_body, InvalidBodyID);
    
    const auto ball_radius = 0.01_m;
    const auto circle_shape = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(ball_radius));
    Attach(world, ball_body, CreateShape(world, circle_shape));

    const auto velocity = LinearVelocity2{+1_mps, 0_mps};
    SetVelocity(world, ball_body, Velocity{velocity, 0_deg / 1_s});

    const auto time_inc = .01_s;
    auto step = StepConf{};
    step.deltaTime = time_inc;
    step.linearSlop = LinearSlop;
    step.regMinSeparation = -LinearSlop * Real(3);
    step.toiMinSeparation = -LinearSlop * Real(1.5f);
    step.targetDepth = LinearSlop * Real(3);
    step.tolerance = LinearSlop / Real(4);
    step.maxLinearCorrection = LinearSlop * Real(40);
    step.maxAngularCorrection = AngularSlop * Real{4};
    step.aabbExtension = LinearSlop * Real(20);
    step.velocityThreshold = (Real{8} / Real{10}) * 1_mps;
    step.maxSubSteps = std::uint8_t{48};
    world.Step(step);

    const auto max_velocity = step.maxTranslation / time_inc;

    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    EXPECT_GT(GetX(GetLocation(world, ball_body)) / Meter, begin_x);

    EXPECT_EQ(GetLinearVelocity(world, ball_body), velocity);
    
    const auto max_travel = unsigned{10000};

    auto increments = int{1};
    for (auto laps = int{1}; laps < 100; ++laps)
    {
        // traveling to the right
        listener.begin_contacts = 0;
        for (auto travel_r = unsigned{0}; ; ++travel_r)
        {
            if (travel_r == max_travel)
            {
                std::cout << "begin_contacts=" << listener.begin_contacts << std::endl;
                ASSERT_LT(travel_r, max_travel);
            }

            const auto last_contact_count = listener.begin_contacts;
            world.Step(step);

            EXPECT_LT(GetX(GetLocation(world, ball_body)), right_edge_x - (ball_radius/Real{2}));
            EXPECT_GT(GetX(GetLocation(world, ball_body)), left_edge_x + (ball_radius/Real{2}));

            if (GetX(GetVelocity(world, ball_body).linear) >= max_velocity)
            {
                return;
            }

            if (listener.begin_contacts % 2 != 0) // direction switched
            {
                EXPECT_LT(GetX(GetVelocity(world, ball_body).linear), 0_mps);
                break; // going left now
            }
            else if (listener.begin_contacts > last_contact_count)
            {
                ++increments;
                SetVelocity(world, ball_body, Velocity{
                    LinearVelocity2{
                        static_cast<Real>(increments) * GetX(velocity),
                        GetY(GetVelocity(world, ball_body).linear)
                    }, GetVelocity(world, ball_body).angular});
            }
            else
            {
                EXPECT_TRUE(AlmostEqual(Real{GetX(GetVelocity(world, ball_body).linear) / 1_mps},
                                        Real{static_cast<Real>(increments) * GetX(velocity) / 1_mps}));
            }
        }
        
        // traveling to the left
        listener.begin_contacts = 0;
        for (auto travel_l = unsigned{0}; ; ++travel_l)
        {
            if (travel_l == max_travel)
            {
                std::cout << "begin_contacts=" << listener.begin_contacts << std::endl;
                ASSERT_LT(travel_l, max_travel);
            }
            
            const auto last_contact_count = listener.begin_contacts;
            world.Step(step);
            
            EXPECT_LT(GetX(GetLocation(world, ball_body)), right_edge_x - (ball_radius/Real{2}));
            EXPECT_GT(GetX(GetLocation(world, ball_body)), left_edge_x + (ball_radius/Real{2}));

            if (GetX(GetVelocity(world, ball_body).linear) <= -max_velocity)
            {
                return;
            }

            if (listener.begin_contacts % 2 != 0) // direction switched
            {
                EXPECT_GT(GetX(GetVelocity(world, ball_body).linear), 0_mps);
                break; // going right now
            }
            else if (listener.begin_contacts > last_contact_count)
            {
                ++increments;
                SetVelocity(world, ball_body, Velocity{
                    LinearVelocity2{
                        -static_cast<Real>(increments) * GetX(velocity),
                        GetY(GetVelocity(world, ball_body).linear)
                    }, GetVelocity(world, ball_body).angular});
            }
            else
            {
                EXPECT_TRUE(AlmostEqual(Real{GetX(GetVelocity(world, ball_body).linear) / 1_mps},
                                        Real{-static_cast<Real>(increments) * GetX(velocity) / 1_mps}));
            }
        }
        
        ++increments;
        SetVelocity(world, ball_body, Velocity{
            LinearVelocity2{
                static_cast<Real>(increments) * GetX(velocity),
                GetY(GetVelocity(world, ball_body).linear)
            }, GetVelocity(world, ball_body).angular});
    }
}

TEST(World_Longer, TargetJointWontCauseTunnelling)
{
    World world{};
    
    const auto half_box_width = Real(0.2);
    const auto left_edge_x = -half_box_width;
    const auto right_edge_x = +half_box_width;

    const auto half_box_height = Real(0.2);
    const auto btm_edge_y = -half_box_height;
    const auto top_edge_y = +half_box_height;

    AABB container_aabb;

    BodyConf body_def;
    body_def.type = BodyType::Static;

    auto edgeConf = EdgeShapeConf{};
    edgeConf.UseFriction(Real(0.4f));
    edgeConf.UseRestitution(Real(0.94f)); // changes where bodies will be after collision
    
    // Setup vertical bounderies
    edgeConf.Set(Length2{0, +half_box_height * 2_m}, Length2{0, -half_box_height * 2_m});

    body_def.location = Length2{left_edge_x * Meter, 0_m};
    {
        const auto left_wall_body = CreateBody(world, body_def);
        ASSERT_NE(left_wall_body, InvalidBodyID);
        Attach(world, left_wall_body, CreateShape(world, edgeConf));
        Include(container_aabb, ComputeAABB(world, left_wall_body));
    }
    
    body_def.location = Length2{right_edge_x * Meter, 0_m};
    {
        const auto right_wall_body = CreateBody(world, body_def);
        ASSERT_NE(right_wall_body, InvalidBodyID);
        Attach(world, right_wall_body, CreateShape(world, edgeConf));
        Include(container_aabb, ComputeAABB(world, right_wall_body));
    }

    // Setup horizontal bounderies
    edgeConf.Set(Length2{-half_box_width * 2_m, 0_m}, Length2{+half_box_width * 2_m, 0_m});
    
    body_def.location = Length2{0, btm_edge_y * Meter};
    {
        const auto btm_wall_body = CreateBody(world, body_def);
        ASSERT_NE(btm_wall_body, InvalidBodyID);
        Attach(world, btm_wall_body, CreateShape(world, edgeConf));
        Include(container_aabb, ComputeAABB(world, btm_wall_body));
    }
    
    body_def.location = Length2{0, top_edge_y * Meter};
    {
        const auto top_wall_body = CreateBody(world, body_def);
        ASSERT_NE(top_wall_body, InvalidBodyID);
        Attach(world, top_wall_body, CreateShape(world, edgeConf));
        Include(container_aabb, ComputeAABB(world, top_wall_body));
    }

    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{};
    body_def.bullet = true;
    
    const auto ball_body = CreateBody(world, body_def);
    ASSERT_NE(ball_body, InvalidBodyID);
    ASSERT_EQ(GetX(GetLocation(world, ball_body)), 0_m);
    ASSERT_EQ(GetY(GetLocation(world, ball_body)), 0_m);
    
    const auto ball_radius = Real(half_box_width / 4) * Meter;
    const auto object_shape = CreateShape(world, PolygonShapeConf{}.UseDensity(10_kgpm2).SetAsBox(ball_radius, ball_radius));
    Attach(world, ball_body, object_shape);

    constexpr auto numBodies = 1u;
    Length2 last_opos[numBodies];
    BodyID bodies[numBodies];
    for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
    {
        const auto angle = i * 2 * Pi / numBodies;
        const auto x = ball_radius * Real(2.1) * cos(angle);
        const auto y = ball_radius * Real(2.1) * sin(angle);
        body_def.location = Length2{x, y};
        bodies[i] = CreateBody(world, body_def);
        ASSERT_NE(bodies[i], InvalidBodyID);
        ASSERT_EQ(GetX(GetLocation(world, bodies[i])), x);
        ASSERT_EQ(GetY(GetLocation(world, bodies[i])), y);
        last_opos[i] = GetLocation(world, bodies[i]);
        Attach(world, bodies[i], object_shape);
    }

    const auto spare_body = [&](){
        BodyConf bodyConf;
        bodyConf.UseType(BodyType::Static);
        bodyConf.UseEnabled(false);
        bodyConf.UseLocation(Length2{-ball_radius / Real{2}, +ball_radius / Real{2}});
        return CreateBody(world, bodyConf);
    }();

    const auto target_joint = [&]() {
        TargetJointConf mjd;
        mjd.bodyA = spare_body;
        mjd.bodyB = ball_body;
        const auto ball_body_pos = GetLocation(world, ball_body);
        mjd.target = Length2{
            GetX(ball_body_pos) - ball_radius / Real{2},
            GetY(ball_body_pos) + ball_radius / Real{2}
        };
        mjd.maxForce = Real(1000) * GetMass(world, ball_body) * MeterPerSquareSecond;
        return world.CreateJoint(Joint{mjd});
    }();
    ASSERT_NE(target_joint, InvalidJointID);

    SetAwake(world, ball_body);

    auto max_x = Real(0);
    auto min_x = Real(0);
    auto max_y = Real(0);
    auto min_y = Real(0);

    auto max_velocity = Real(0);

    //const auto time_inc = Real(.0043268126901); // numBodies = 6, somewhat dependent on fixture density (10 or less?).
    //const auto time_inc = Real(.0039224); // numBodies = 4, maybe dependent on fixture density
    //const auto time_inc = Real(.003746); // numBodies = 2, maybe dependent on fixture density
    //const auto time_inc = Real(.0036728129); // numBodies = 1, maybe dependent on fixture density
    const auto time_inc = Real(.00367281295); // numBodies = 1, maybe dependent on fixture density

    auto angle = Real(0);
    auto anglular_speed = Real(0.01); // radians / timestep
    const auto anglular_accel = Real(1.002);
    auto distance = half_box_width / 2;
    auto distance_speed = Real(0.003); // meters / timestep
    const auto distance_accel = Real(1.001);

    MyContactListener listener{world,
        [&](ContactID contact, const Manifold& old_manifold)
        {
            // PreSolve...
            const auto new_manifold = GetManifold(world, contact);
            const auto pointStates = GetPointStates(old_manifold, new_manifold);
            const auto oldPointCount = old_manifold.GetPointCount();
            switch (oldPointCount)
            {
                case 0:
                    ASSERT_EQ(pointStates.state1[0], PointState::Null);
                    ASSERT_EQ(pointStates.state1[1], PointState::Null);
                    break;
                case 1:
                    ASSERT_NE(pointStates.state1[0], PointState::Null);
                    ASSERT_EQ(pointStates.state1[1], PointState::Null);
                    break;
                case 2:
                    ASSERT_NE(pointStates.state1[0], PointState::Null);
                    ASSERT_NE(pointStates.state1[1], PointState::Null);
                    break;
                default:
                    ASSERT_LE(oldPointCount, 2);
                    break;
            }
            const auto newPointCount = new_manifold.GetPointCount();
            switch (newPointCount)
            {
                case 0:
                    ASSERT_EQ(pointStates.state2[0], PointState::Null);
                    ASSERT_EQ(pointStates.state2[1], PointState::Null);
                    break;
                case 1:
                    ASSERT_NE(pointStates.state2[0], PointState::Null);
                    ASSERT_EQ(pointStates.state2[1], PointState::Null);
                    break;
                case 2:
                    ASSERT_NE(pointStates.state2[0], PointState::Null);
                    ASSERT_NE(pointStates.state2[1], PointState::Null);
                    break;
                default:
                    ASSERT_LE(newPointCount, 2);
                    break;
            }
            ASSERT_THROW(Destroy(world, target_joint), WrongState);
            if (HasValidToi(world, contact)) {
                auto toi = Real(0);
                EXPECT_NO_THROW(toi = GetToi(world, contact));
                EXPECT_GE(toi, Real(0));
                EXPECT_LE(toi, Real(1));
            }
        },
        [&](ContactID contact, const ContactImpulsesList& impulse, unsigned solved)
        {
            const auto body_a = GetBodyA(world, contact);
            const auto body_b = GetBodyB(world, contact);

            ASSERT_NE(body_a, InvalidBodyID);
            ASSERT_NE(body_b, InvalidBodyID);

            auto fail_count = unsigned{0};
            for (auto&& body: {body_a, body_b})
            {
                if (!IsSpeedable(world, body))
                {
                    continue;
                }
                const auto bpos = GetLocation(world, body);
                const auto lt = Length2{right_edge_x * Meter, top_edge_y * Meter} - bpos;
                const auto gt = bpos - Length2{left_edge_x * Meter, btm_edge_y * Meter};
                
                if (GetX(lt) <= 0_m || GetY(lt) <= 0_m || GetX(gt) <= 0_m || GetY(gt) <= 0_m)
                {
                    if (!TestOverlap(container_aabb, ComputeAABB(world, body)))
                    {
                        // Check if body gets out of bounds and no longer even overlapping
                        // container!
                        EXPECT_LT(GetX(GetLocation(world, body)), right_edge_x * Meter);
                        EXPECT_LT(GetY(GetLocation(world, body)), top_edge_y * Meter);
                        EXPECT_GT(GetX(GetLocation(world, body)), left_edge_x * Meter);
                        EXPECT_GT(GetY(GetLocation(world, body)), btm_edge_y * Meter);
                        ++fail_count;
                    }
                }
            }
            if (fail_count > 0)
            {
                std::cout << " angl=" << angle;
                std::cout << " ctoi=" << 0 + GetToiCount(world, contact);
                std::cout << " solv=" << 0 + solved;
                std::cout << " targ=(" << distance * cos(angle) << "," << distance * sin(angle) << ")";
                std::cout << " maxv=" << max_velocity;
                std::cout << " rang=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
                std::cout << " bpos=(" << GetX(GetLocation(world, ball_body)) << "," << GetY(GetLocation(world, ball_body)) << ")";
                std::cout << std::endl;
                for (auto i = decltype(impulse.GetCount()){0}; i < impulse.GetCount(); ++i)
                {
                    std::cout << " i#" << (0 + i) << "={n" << impulse.GetEntryNormal(i) << ",t" << impulse.GetEntryTanget(i) << "}";
                }
                std::cout << std::endl;

                std::cout << " bodyA=(" << GetX(GetLocation(world, body_a)) << "," << GetY(GetLocation(world, body_a)) << ")";
                if (body_a == ball_body) std::cout << " ball";
                if (!IsSpeedable(world, body_a)) std::cout << " wall";
                std::cout << " " << to_underlying(body_a);
                std::cout << std::endl;
                std::cout << " bodyB=(" << GetX(GetLocation(world, body_b)) << "," << GetY(GetLocation(world, body_b)) << ")";
                if (body_b == ball_body) std::cout << " ball";
                if (!IsSpeedable(world, body_b)) std::cout << " wall";
                std::cout << " " << to_underlying(body_b);
                std::cout << std::endl;

                //GTEST_FATAL_FAILURE_("");                
            }
            if (HasValidToi(world, contact)) {
                auto toi = Real(0);
                EXPECT_NO_THROW(toi = GetToi(world, contact));
                EXPECT_GE(toi, Real(0));
                EXPECT_LE(toi, Real(1));
            }
        },
        [&](ContactID contact) {
            const auto body_a = GetBodyA(world, contact);
            const auto body_b = GetBodyB(world, contact);

            auto escaped = false;
            for (auto&& body: {body_a, body_b})
            {
                if (!IsSpeedable(world, body))
                {
                    continue;
                }

                if (GetX(GetLocation(world, body)) >= right_edge_x * Meter)
                {
                    escaped = true;
                }
                if (GetY(GetLocation(world, body)) >= top_edge_y * Meter)
                {
                    escaped = true;
                }
                if (GetX(GetLocation(world, body)) <= left_edge_x * Meter)
                {
                    escaped = true;
                }
                if (GetY(GetLocation(world, body)) <= btm_edge_y * Meter)
                {
                    escaped = true;
                }
            }
            if (escaped && !IsTouching(world, contact))
            {
                std::cout << "Escaped at EndContact[" << &contact << "]:";
                std::cout << " toiSteps=" << static_cast<unsigned>(GetToiCount(world, contact));
                std::cout << " toiValid=" << HasValidToi(world, contact);
                std::cout << " a[" << to_underlying(body_a) << "]@(" << GetX(GetLocation(world, body_a)) << "," << GetY(GetLocation(world, body_a)) << ")";
                std::cout << " b[" << to_underlying(body_b) << "]@(" << GetX(GetLocation(world, body_b)) << "," << GetY(GetLocation(world, body_b)) << ")";
                std::cout << std::endl;
                //exit(1);
            }
        },
    };
    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    SetBeginContactListener(world, [&listener](ContactID id) {
        listener.BeginContact(id);
    });
    SetEndContactListener(world, [&listener](ContactID id) {
        listener.EndContact(id);
    });
    SetPreSolveContactListener(world, [&listener](ContactID id, const Manifold& manifold) {
        listener.PreSolve(id, manifold);
    });
    SetPostSolveContactListener(world, [&listener](ContactID id,
                                                  const ContactImpulsesList& impulses,
                                                  unsigned count){
        listener.PostSolve(id, impulses, count);
    });

    for (auto outer = unsigned{0}; outer < 2000; ++outer)
    {
        auto last_pos = GetLocation(world, ball_body);
        for (auto loops = unsigned{0};; ++loops)
        {
            SetTarget(world, target_joint, Length2{distance * cos(angle) * Meter, distance * sin(angle) * Meter});
            angle += anglular_speed;
            distance += distance_speed;

            Step(world, 1_s * time_inc, 8, 3);
            
            ASSERT_LT(GetX(GetLocation(world, ball_body)), right_edge_x * Meter);
            ASSERT_LT(GetY(GetLocation(world, ball_body)), top_edge_y * Meter);
            ASSERT_GT(GetX(GetLocation(world, ball_body)), left_edge_x * Meter);
            ASSERT_GT(GetY(GetLocation(world, ball_body)), btm_edge_y * Meter);
            for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
            {
                ASSERT_LT(GetX(GetLocation(world, bodies[i])), right_edge_x * Meter);
                ASSERT_LT(GetY(GetLocation(world, bodies[i])), top_edge_y * Meter);
                ASSERT_GT(GetX(GetLocation(world, bodies[i])), left_edge_x * Meter);
                ASSERT_GT(GetY(GetLocation(world, bodies[i])), btm_edge_y * Meter);
            }

            max_x = std::max(Real{GetX(GetLocation(world, ball_body)) / Meter}, max_x);
            min_x = std::min(Real{GetX(GetLocation(world, ball_body)) / Meter}, min_x);

            max_y = std::max(Real{GetY(GetLocation(world, ball_body)) / Meter}, max_y);
            min_y = std::min(Real{GetY(GetLocation(world, ball_body)) / Meter}, min_y);

            const auto linVel = GetVelocity(world, ball_body).linear;
            max_velocity = std::max(GetMagnitude(GetVec2(linVel)), max_velocity);

            if (loops > 50)
            {
                if (GetX(GetTarget(world, target_joint)) < 0_m)
                {
                    if (GetX(GetLocation(world, ball_body)) >= GetX(last_pos))
                        break;                    
                }
                else
                {
                    if (GetX(GetLocation(world, ball_body)) <= GetX(last_pos))
                        break;
                }
                if (GetY(GetTarget(world, target_joint)) < 0_m)
                {
                    if (GetY(GetLocation(world, ball_body)) >= GetY(last_pos))
                        break;
                }
                else
                {
                    if (GetY(GetLocation(world, ball_body)) <= GetY(last_pos))
                        break;
                }
            }
            last_pos = GetLocation(world, ball_body);
        }
        anglular_speed *= anglular_accel;
        distance_speed *= distance_accel;

        ASSERT_NE(GetLocation(world, ball_body), (Length2{}));
#if 0
        if (outer > 100)
        {
            for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
            {
                // a sanity check to ensure the other bodies are getting moved
                EXPECT_NE(last_opos[i], GetLocation(world, bodies[i]));
                last_opos[i] = GetLocation(world, bodies[i]);
            }
        }
#endif
    }
#if 0
    std::cout << "angle=" << angle;
    std::cout << " target=(" << distance * cos(angle) << "," << distance * sin(angle) << ")";
    std::cout << " maxvel=" << max_velocity;
    std::cout << " range=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
    std::cout << std::endl;
#endif

    const auto target0 = GetTarget(world, target_joint);
    const auto shift = Length2{2_m, 2_m};
    world.ShiftOrigin(shift);
    const auto target1 = GetTarget(world, target_joint);
    EXPECT_EQ(target0 - shift, target1);
}

#if 0
class VerticalStackTest: public ::testing::TestWithParam<Real>
{
public:
    virtual void SetUp()
    {
        const auto hw_ground = 40.0_m;
        const auto ground = CreateBody(world);
        Attach(world, ground, CreateShape(world, EdgeShapeConf{}.Set(Length2{-hw_ground, 0_m}, Length2{hw_ground, 0_m})));
        const auto numboxes = boxes.size();
        original_x = GetParam();
        
        const auto boxShape = CreateShape(world, PolygonShapeConf{}.UseDensity(1_kgpm2).UseFriction(Real(0.3f)).SetAsBox(hdim, hdim));
        for (auto i = decltype(numboxes){0}; i < numboxes; ++i)
        {
            // (hdim + 0.05f) + (hdim * 2 + 0.1f) * i
            const auto location = Length2{original_x * Meter, (i + Real{1}) * hdim * Real{4}};
            const auto box = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location));
            Attach(world, box, boxShape);
            boxes[i] = box;
        }
        
        SetAccelerations(world, Acceleration{LinearAcceleration2{
            Real(0) * MeterPerSquareSecond, -Real(10) * MeterPerSquareSecond
        }, 0 * RadianPerSquareSecond});
        auto stepConf = StepConf{};
        stepConf.deltaTime = 1_s / 60;
        while (loopsTillSleeping < maxLoops)
        {
            world.Step(stepConf);
            if (GetAwakeCount(world) == 0)
            {
                break;
            }
            ++loopsTillSleeping;
        }
    }

protected:
    World world{};
    std::size_t loopsTillSleeping = 0;
    const std::size_t maxLoops = 10000;
    std::vector<BodyID> boxes{10};
    Real original_x = 0;
    const Length hdim = 0.1_m;
};

TEST_P(VerticalStackTest, EndsBeforeMaxLoops)
{
    EXPECT_LT(loopsTillSleeping, maxLoops);
}

TEST_P(VerticalStackTest, BoxesAtOriginalX)
{
    for (auto&& box: boxes)
    {
        EXPECT_EQ(GetX(GetLocation(world, box)), original_x * Meter);
    }
}

TEST_P(VerticalStackTest, EachBoxAboveLast)
{
    auto lasty = 0_m;
    for (auto&& box: boxes)
    {
        EXPECT_GT(GetY(GetLocation(world, box)), lasty + hdim);
        lasty = GetY(GetLocation(world, box));
    }
}

TEST_P(VerticalStackTest, EachBodyLevel)
{
    for (auto&& box: boxes)
    {
        EXPECT_EQ(GetAngle(world, box), 0_deg);
    }
}

static std::string test_suffix_generator(::testing::TestParamInfo<Real> param_info)
{
    std::stringstream strbuf;
    strbuf << param_info.index;
    return strbuf.str();
}

static ::testing::internal::ParamGenerator<VerticalStackTest::ParamType> gtest_WorldVerticalStackTest_EvalGenerator_();
static ::std::string gtest_WorldVerticalStackTest_EvalGenerateName_(const ::testing::TestParamInfo<VerticalStackTest::ParamType>& info);

INSTANTIATE_TEST_CASE_P(World, VerticalStackTest, ::testing::Values(Real(0), Real(5)), test_suffix_generator);
#endif
