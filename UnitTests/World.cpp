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
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Dynamics/ContactImpulsesList.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Collision.hpp>
#include <PlayRho/Collision/RayCastInput.hpp>
#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Dynamics/Joints/TargetJoint.hpp>
#include <PlayRho/Dynamics/Joints/RopeJoint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Joints/MotorJoint.hpp>
#include <PlayRho/Dynamics/Joints/WheelJoint.hpp>
#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Common/LengthError.hpp>
#include <PlayRho/Common/WrongState.hpp>
#include <chrono>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

class UnitTestDestructionListener: public playrho::d2::DestructionListener
{
    void SayGoodbye(const Joint& joint) noexcept override
    {
        joints.push_back(&joint);
    }
    
    void SayGoodbye(const Fixture& fixture) noexcept override
    {
        fixtures.push_back(&fixture);
    }
    
public:
    std::vector<const Joint*> joints;
    std::vector<const Fixture*> fixtures;
};

TEST(World, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
        {
            // Size is OS dependent.
            // Seems linux containers are bigger in size...
#ifdef __APPLE__
            EXPECT_EQ(sizeof(World), std::size_t(232));
#endif
#ifdef __linux__
            EXPECT_EQ(sizeof(World), std::size_t(232));
#endif
            break;
        }
        case  8:
        {
#ifdef __APPLE__
            EXPECT_EQ(sizeof(World), std::size_t(248));
#endif
#ifdef __linux__
            EXPECT_EQ(sizeof(World), std::size_t(248));
#endif
            break;
        }
        case 16:
            EXPECT_EQ(sizeof(World), std::size_t(272));
            break;
        default: FAIL(); break;
    }
}

TEST(World, Conf)
{
    const auto worldConf = WorldConf{};
    const auto defaultConf = GetDefaultWorldConf();
    
    EXPECT_EQ(defaultConf.maxVertexRadius, worldConf.maxVertexRadius);
    EXPECT_EQ(defaultConf.minVertexRadius, worldConf.minVertexRadius);
    const auto stepConf = StepConf{};

    const auto v = Real(1);
    const auto n = nextafter(v, Real(0));
    const auto time_inc = (v - n) * 1_s;
    ASSERT_GT(time_inc, 0_s);
    ASSERT_LT(time_inc, 1_s);
    const auto max_inc = time_inc * stepConf.maxTranslation;
    EXPECT_GT(max_inc, 0_m * 1_s);
}

TEST(World, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<World>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<World>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<World>::value);
    
    EXPECT_TRUE(std::is_constructible<World>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<World>::value);
    EXPECT_FALSE(std::is_trivially_constructible<World>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<World>::value);
    EXPECT_FALSE(std::is_nothrow_copy_constructible<World>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<World>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<World>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<World>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<World>::value);
    
    EXPECT_TRUE(std::is_destructible<World>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<World>::value);
    EXPECT_FALSE(std::is_trivially_destructible<World>::value);
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
        EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());
    }
    {
        const auto& w = static_cast<const World&>(world);
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

TEST(World, Init)
{
    World world{};
    EXPECT_FALSE(world.IsLocked());
    
    {
        auto calls = 0;
        Query(world.GetTree(), AABB{}, [&](Fixture*, ChildCounter) {
            ++calls;
            return true;
        });
        EXPECT_EQ(calls, 0);
    }
    {
        const auto p1 = Length2{0_m, 0_m};
        const auto p2 = Length2{100_m, 0_m};
        auto calls = 0;
        RayCast(world.GetTree(), RayCastInput{p1, p2, UnitInterval<Real>{1}}, [&](Fixture*, ChildCounter, Length2, UnitVec) {
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
    auto world = World{};
    ASSERT_EQ(world.GetBodies().size(), std::size_t(0));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(0));
    
    auto listener = UnitTestDestructionListener{};
    world.SetDestructionListener(&listener);
    
    const auto b0 = world.CreateBody();
    const auto f0 = b0->CreateFixture(Shape{DiskShapeConf{}});
    const auto b1 = world.CreateBody();
    const auto f1 = b1->CreateFixture(Shape{DiskShapeConf{}});
    const auto j0 = world.CreateJoint(DistanceJointConf{b0, b1});
    ASSERT_NE(j0, nullptr);
    
    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    ASSERT_EQ(world.GetJoints().size(), std::size_t(1));
    
    world.Clear();

    EXPECT_EQ(world.GetBodies().size(), std::size_t(0));
    EXPECT_EQ(world.GetJoints().size(), std::size_t(0));
    
    ASSERT_EQ(listener.fixtures.size(), std::size_t(2));
    EXPECT_EQ(listener.fixtures.at(0), f0);
    EXPECT_EQ(listener.fixtures.at(1), f1);
    
    ASSERT_EQ(listener.joints.size(), std::size_t(1));
    EXPECT_EQ(listener.joints.at(0), j0);
}

TEST(World, SetSubStepping)
{
    World world;
    
    ASSERT_FALSE(world.GetSubStepping());

    world.SetSubStepping(true);
    EXPECT_TRUE(world.GetSubStepping());

    world.SetSubStepping(false);
    EXPECT_FALSE(world.GetSubStepping());

    world.SetSubStepping(true);
    EXPECT_TRUE(world.GetSubStepping());

    auto stepConf = StepConf{};
    stepConf.SetInvTime(100_Hz);
    world.Step(stepConf);
    EXPECT_TRUE(world.GetSubStepping());
}

TEST(World, IsStepComplete)
{
    auto world{World{}};
    
    ASSERT_FALSE(world.GetSubStepping());
    EXPECT_TRUE(world.IsStepComplete());

    world.SetSubStepping(true);
    EXPECT_TRUE(world.GetSubStepping());
    EXPECT_TRUE(world.IsStepComplete());

    auto stepConf = StepConf{};
    stepConf.SetInvTime(100_Hz);
    world.Step(stepConf);
    EXPECT_TRUE(world.GetSubStepping());
    EXPECT_TRUE(world.IsStepComplete());
    
    const auto b0 = world.CreateBody(BodyConf{}
                                     .UseType(BodyType::Dynamic)
                                     .UseLocation(Length2{-2_m, 2_m})
                                     .UseLinearAcceleration(EarthlyGravity));
    b0->CreateFixture(Shape{DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m)});
    
    const auto b1 = world.CreateBody(BodyConf{}
                                     .UseType(BodyType::Dynamic)
                                     .UseLocation(Length2{+2_m, 2_m})
                                     .UseLinearAcceleration(EarthlyGravity));
    b1->CreateFixture(Shape{DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m)});

    const auto stabody = world.CreateBody(BodyConf{}.UseType(BodyType::Static));
    stabody->CreateFixture(Shape{EdgeShapeConf{Length2{-10_m, 0_m}, Length2{+10_m, 0_m}}});
    
    while (world.IsStepComplete())
    {
        world.Step(stepConf);
    }
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
    
    const auto shape = Shape{DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m)};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b1->CreateFixture(shape);
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b2->CreateFixture(shape);
    
    // Add another body on top of previous and that's not part of any joints to ensure at 1 contact
    const auto b3 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b3->CreateFixture(shape);

    const auto b4 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b4->CreateFixture(shape);
    const auto b5 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b5->CreateFixture(shape);

    const auto rj1 = world.CreateJoint(RevoluteJointConf{b1, b2, Length2{}});
    const auto rj2 = world.CreateJoint(RevoluteJointConf{b3, b4, Length2{}});
    world.CreateJoint(PrismaticJointConf{b1, b2, Length2{}, UnitVec::GetRight()});
    world.CreateJoint(PulleyJointConf{b1, b2, Length2{}, Length2{},
        Length2{}, Length2{}}.UseRatio(Real(1)));
    world.CreateJoint(DistanceJointConf{b4, b5});
    world.CreateJoint(WeldJointConf{b4, b5, Length2{}});
    world.CreateJoint(FrictionJointConf{b4, b5, Length2{}});
    world.CreateJoint(RopeJointConf{b4, b5});
    world.CreateJoint(MotorJointConf{b4, b5});
    world.CreateJoint(WheelJointConf{b4, b5, Length2{}, UnitVec::GetRight()});
    world.CreateJoint(TargetJointConf{b4});
    world.CreateJoint(GearJointConf{rj1, rj2});

    auto stepConf = StepConf{};
    world.Step(stepConf);
    ASSERT_FALSE(world.GetContacts().empty());

    {
        const auto copy = World{world};
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        const auto minJoints = std::min(world.GetJoints().size(), copy.GetJoints().size());
        
        auto worldJointIter = world.GetJoints().begin();
        auto copyJointIter = copy.GetJoints().begin();
        for (auto i = decltype(minJoints){0}; i < minJoints; ++i)
        {
            EXPECT_EQ(GetType(*(*worldJointIter)), GetType(*(*copyJointIter)));
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
    
    const auto shape = Shape{DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m)};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b1->CreateFixture(shape);
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    b2->CreateFixture(shape);
    
    world.CreateJoint(RevoluteJointConf{b1, b2, Length2{}});
    world.CreateJoint(PrismaticJointConf{b1, b2, Length2{}, UnitVec::GetRight()});
    world.CreateJoint(PulleyJointConf{b1, b2, Length2{}, Length2{},
        Length2{}, Length2{}}.UseRatio(Real(1)));
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    
    {
        auto copy = World{};
        copy = world;
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        const auto minJoints = std::min(world.GetJoints().size(), copy.GetJoints().size());
        
        auto worldJointIter = world.GetJoints().begin();
        auto copyJointIter = copy.GetJoints().begin();
        for (auto i = decltype(minJoints){0}; i < minJoints; ++i)
        {
            EXPECT_EQ(GetType(*(*worldJointIter)), GetType(*(*copyJointIter)));
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

TEST(World, CreateDestroyEmptyStaticBody)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Static));
    ASSERT_NE(body, nullptr);
    
    EXPECT_EQ(body->GetType(), BodyType::Static);
    EXPECT_FALSE(body->IsSpeedable());
    EXPECT_FALSE(body->IsAccelerable());
    EXPECT_TRUE(body->IsImpenetrable());
    EXPECT_EQ(body->GetFixtures().size(), std::size_t{0});

    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto& first = GetRef(*bodies1.begin());
    EXPECT_EQ(body, &first);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
    
    world.Destroy(body);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));

    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
}

TEST(World, CreateDestroyEmptyDynamicBody)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body->IsSpeedable());
    EXPECT_TRUE(body->IsAccelerable());
    EXPECT_FALSE(body->IsImpenetrable());
    EXPECT_EQ(body->GetFixtures().size(), std::size_t{0});

    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto& first = GetRef(*bodies1.begin());
    EXPECT_EQ(body, &first);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
    
    world.Destroy(body);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
}

TEST(World, CreateDestroyDynamicBodyAndFixture)
{
    // Created this test after receiving issue #306:
    //   Rapid create/destroy between step() causes SEGFAULT
    
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body->IsSpeedable());
    EXPECT_TRUE(body->IsAccelerable());
    EXPECT_FALSE(body->IsImpenetrable());
    EXPECT_EQ(body->GetFixtures().size(), std::size_t{0});

    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto& first = GetRef(*bodies1.begin());
    EXPECT_EQ(body, &first);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
    
    const auto fixture = body->CreateFixture(Shape{DiskShapeConf{1_m}});
    ASSERT_NE(fixture, nullptr);
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(body->GetFixtures().size(), std::size_t{1});
    ASSERT_EQ(world.GetFixturesForProxies().size(), std::size_t{1});
    EXPECT_EQ(*world.GetFixturesForProxies().begin(), fixture);

    world.Destroy(body); // should clear fixtures for proxies!
    
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto& bodies2 = world.GetBodies();
    EXPECT_TRUE(bodies2.empty());
    EXPECT_EQ(bodies2.size(), BodyCounter(0));
    
    EXPECT_EQ(world.GetBodiesForProxies().size(), std::size_t{0});
    EXPECT_EQ(world.GetFixturesForProxies().size(), std::size_t{0});
}

TEST(World, CreateDestroyJoinedBodies)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetJointCount(world), JointCounter(0));
    
    auto listener = UnitTestDestructionListener{};
    world.SetDestructionListener(&listener);

    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto& bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto& first = GetRef(*bodies1.begin());
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body, &first);

    const auto body2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));

    const auto f0 = body->CreateFixture(Shape{DiskShapeConf{1_m}});
    const auto f1 = body2->CreateFixture(Shape{DiskShapeConf{1_m}});

    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    ASSERT_EQ(world.GetContacts().size(), ContactCounter(1));
    const auto c0 = world.GetContacts().begin();
    EXPECT_FALSE(c0->second->NeedsFiltering());

    const auto joint = world.CreateJoint(DistanceJointConf{body, body2});
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(GetJointCount(world), JointCounter(1));
    EXPECT_TRUE(c0->second->NeedsFiltering());

    world.Destroy(body);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_EQ(world.GetContacts().size(), ContactCounter(0));

    const auto& bodies0 = world.GetBodies();
    EXPECT_FALSE(bodies0.empty());
    EXPECT_EQ(bodies0.size(), BodyCounter(1));
    EXPECT_NE(bodies0.begin(), bodies0.end());
    world.Destroy(body2);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    
    ASSERT_EQ(listener.fixtures.size(), std::size_t(2));
    EXPECT_EQ(listener.fixtures.at(0), f0);
    EXPECT_EQ(listener.fixtures.at(1), f1);

    ASSERT_EQ(listener.joints.size(), std::size_t(1));
    EXPECT_EQ(listener.joints.at(0), joint);
}

TEST(World, CreateDestroyContactingBodies)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetJointCount(world), JointCounter(0));
    auto contacts = world.GetContacts();
    ASSERT_TRUE(contacts.empty());
    ASSERT_EQ(contacts.size(), ContactCounter(0));

    const auto l1 = Length2{};
    const auto l2 = Length2{};

    const auto body1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto body2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l2));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));
    
    EXPECT_NE(body1->CreateFixture(Shape{DiskShapeConf{1_m}.UseDensity(1_kgpm2)}), nullptr);
    EXPECT_NE(body2->CreateFixture(Shape{DiskShapeConf{1_m}.UseDensity(1_kgpm2)}), nullptr);
    EXPECT_EQ(GetFixtureCount(world), std::size_t(2));
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    contacts = world.GetContacts();
    EXPECT_FALSE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(1));

    world.Destroy(body1);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    world.Destroy(body2);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    contacts = world.GetContacts();
    EXPECT_TRUE(contacts.empty());
    EXPECT_EQ(contacts.size(), ContactCounter(0));
    EXPECT_EQ(GetFixtureCount(world), std::size_t(0));
}

#if 0
TEST(World, CreateAndDestroyFixture)
{
    auto world = World{};
    auto other = World{};

    const auto bodyA = world.CreateBody();
    const auto bodyB = world.CreateBody();
    ASSERT_EQ(GetFixtureCount(*bodyA), std::size_t(0));
    ASSERT_EQ(GetFixtureCount(*bodyB), std::size_t(0));
    
    EXPECT_THROW(world.CreateFixture(*bodyA, Shape{DiskShapeConf(0_m)}), InvalidArgument);
    EXPECT_THROW(world.CreateFixture(*bodyA, Shape{DiskShapeConf(WorldConf{}.maxVertexRadius * 2)}), InvalidArgument);

    const auto fixtureA = world.CreateFixture(*bodyA, Shape{DiskShapeConf(1_m)});
    ASSERT_NE(fixtureA, nullptr);
    ASSERT_EQ(GetFixtureCount(*bodyA), std::size_t(1));
    EXPECT_FALSE(other.TouchProxies(*fixtureA));
    
    EXPECT_TRUE(world.Destroy(fixtureA));
    EXPECT_EQ(GetFixtureCount(*bodyA), std::size_t(0));
    
    EXPECT_FALSE(world.Destroy(static_cast<Fixture*>(nullptr)));
    
    const auto bodyC = other.CreateBody();
    ASSERT_NE(bodyC, nullptr);
    const auto fixtureC = other.CreateFixture(*bodyC, Shape{DiskShapeConf(1_m)});
    ASSERT_NE(fixtureC, nullptr);
    EXPECT_FALSE(world.Destroy(fixtureC));
    
    EXPECT_THROW(world.CreateFixture(*bodyC, Shape{DiskShapeConf(1_m)}), InvalidArgument);
}
#endif

TEST(World, SynchronizeProxies)
{
    auto world = World{};
    const auto stepConf = StepConf{};
    
    EXPECT_EQ(world.Step(stepConf).pre.proxiesMoved, PreStepStats::counter_type(0));
    const auto bodyA = world.CreateBody();
    bodyA->CreateFixture(Shape{DiskShapeConf(1_m)});
    EXPECT_EQ(world.Step(stepConf).pre.proxiesMoved, PreStepStats::counter_type(0));
    SetLocation(*bodyA, Length2{10_m, -4_m});
    EXPECT_EQ(world.Step(stepConf).pre.proxiesMoved, PreStepStats::counter_type(1));
}

#if 0
TEST(World, SetTypeOfBody)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    auto other = World{};
    other.SetType(*body, BodyType::Static);
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    world.SetType(*body, BodyType::Static);
    EXPECT_EQ(body->GetType(), BodyType::Static);
}

TEST(World, RegisterBodyForProxies)
{
    auto world = World{};
    EXPECT_FALSE(world.RegisterForProxies(static_cast<Body*>(nullptr)));
    const auto body = world.CreateBody();
    EXPECT_TRUE(world.RegisterForProxies(body));
}

TEST(World, RegisterFixtureForProxies)
{
    auto world = World{};
    EXPECT_FALSE(world.RegisterForProxies(static_cast<Fixture*>(nullptr)));
    const auto body = world.CreateBody();
    const auto fixture = body->CreateFixture(Shape{DiskShapeConf(1_m)});
    EXPECT_TRUE(world.RegisterForProxies(fixture));
}
#endif

TEST(World, Query)
{
    auto world = World{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body->IsSpeedable());
    ASSERT_TRUE(body->IsAccelerable());
    ASSERT_FALSE(body->IsImpenetrable());
    ASSERT_EQ(GetX(body->GetLocation()), 0_m);
    ASSERT_EQ(GetY(body->GetLocation()), 0_m);
    ASSERT_EQ(GetX(body->GetLinearAcceleration()), 0_mps2);
    ASSERT_EQ(GetY(body->GetLinearAcceleration()), 0_mps2);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    ASSERT_EQ(GetChildCount(conf), ChildCounter(1));
    const auto fixture = body->CreateFixture(Shape{conf});
    ASSERT_NE(fixture, nullptr);
    
    auto stepConf = StepConf{};
    stepConf.SetTime(0_s);
    world.Step(stepConf);

    {
        auto foundOurs = 0;
        auto foundOthers = 0;
        Query(world.GetTree(), AABB{v1, v2}, [&](Fixture* f, ChildCounter i) {
            if (f == fixture && i == 0)
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
    const auto b0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p0));
    ASSERT_NE(b0->CreateFixture(Shape{DiskShapeConf{1_m}}), nullptr);

    const auto p1 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    ASSERT_NE(b1->CreateFixture(Shape{DiskShapeConf{0.1_m}}), nullptr);

    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Static).UseLocation(Length2{-100_m, -100_m}));
    ASSERT_NE(b2->CreateFixture(Shape{EdgeShapeConf{Length2{}, Length2{-20_m, -20_m}}}), nullptr);

    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body->IsSpeedable());
    ASSERT_TRUE(body->IsAccelerable());
    ASSERT_FALSE(body->IsImpenetrable());
    ASSERT_EQ(GetX(body->GetLocation()), 0_m);
    ASSERT_EQ(GetY(body->GetLocation()), 0_m);
    ASSERT_EQ(GetX(body->GetLinearAcceleration()), 0_mps2);
    ASSERT_EQ(GetY(body->GetLinearAcceleration()), 0_mps2);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    const auto shape = Shape{conf};
    ASSERT_EQ(GetChildCount(shape), ChildCounter(1));
    const auto fixture = body->CreateFixture(shape);
    ASSERT_NE(fixture, nullptr);
    
    auto stepConf = StepConf{};
    stepConf.SetTime(0_s);
    world.Step(stepConf);
    
    {
        const auto p2 = Length2{-2_m, 0_m};
        const auto p3 = Length2{+2_m, 0_m};

        auto foundOurs = 0;
        auto foundOthers = 0;
        const auto retval = RayCast(world.GetTree(), RayCastInput{p2, p3, UnitInterval<Real>{1}},
                    [&](Fixture* f, ChildCounter i, Length2, UnitVec) {
            if (f == fixture && i == 0)
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
        const auto retval = RayCast(world.GetTree(), RayCastInput{p2, p3, UnitInterval<Real>{1}},
                    [&](Fixture* f, ChildCounter i, Length2, UnitVec) {
            if (f == fixture && i == 0)
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
        const auto retval = RayCast(world.GetTree(), RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](Fixture* f, ChildCounter i, Length2, UnitVec) {
            if (f == fixture && i == 0)
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
        const auto retval = RayCast(world.GetTree(), RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](Fixture* f, ChildCounter i, Length2, UnitVec) {
            if (f == fixture && i == 0)
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
        const auto retval = RayCast(world.GetTree(), RayCastInput{p2, p3, UnitInterval<Real>{1}},
                                    [&](Fixture* f, ChildCounter i, Length2, UnitVec) {
            if (f == fixture && i == 0)
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
        const auto retval = RayCast(world.GetTree(), RayCastInput{p2, p3, UnitInterval<Real>{1}},
          [&](Fixture* f, ChildCounter i, Length2, UnitVec) {
            if (f == fixture && i == 0)
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
        const auto retval = RayCast(world.GetTree(), rci, [&](Fixture*, ChildCounter, Length2, UnitVec) {
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
    
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body->IsSpeedable());
    ASSERT_TRUE(body->IsAccelerable());
    ASSERT_FALSE(body->IsImpenetrable());
    ASSERT_EQ(GetX(body->GetLinearAcceleration()), GetX(EarthlyGravity));
    ASSERT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    const auto shape = Shape{conf};
    const auto fixture = body->CreateFixture(shape);
    ASSERT_NE(fixture, nullptr);

    ApplyForceToCenter(*body, Force2(2_N, 4_N));
    ASSERT_NE(GetX(body->GetLinearAcceleration()), GetX(EarthlyGravity));
    ASSERT_NE(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
    
    ClearForces(world);
    EXPECT_EQ(GetX(body->GetLinearAcceleration()), 0_mps2);
    EXPECT_EQ(GetY(body->GetLinearAcceleration()), 0_mps2);
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

    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b1, nullptr);
    ASSERT_TRUE(b1->IsAccelerable());
    SetAcceleration(*b1, a1);
    ASSERT_EQ(GetAcceleration(*b1), a1);
  
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b2, nullptr);
    ASSERT_TRUE(b2->IsAccelerable());
    SetAcceleration(*b2, a2);
    ASSERT_EQ(GetAcceleration(*b2), a2);
   
    SetAccelerations(world, [](const Body& b){ return GetAcceleration(b) * 2; });
    EXPECT_EQ(GetAcceleration(*b1), a1 * 2);
    EXPECT_EQ(GetAcceleration(*b2), a2 * 2);
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
    
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b1, nullptr);
    ASSERT_TRUE(b1->IsAccelerable());
    SetAcceleration(*b1, a1);
    ASSERT_EQ(GetAcceleration(*b1), a1);
    
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(b2, nullptr);
    ASSERT_TRUE(b2->IsAccelerable());
    SetAcceleration(*b2, a2);
    ASSERT_EQ(GetAcceleration(*b2), a2);

    ASSERT_EQ(GetAcceleration(*b1), a1);
    ASSERT_EQ(GetAcceleration(*b2), a2);

    SetAccelerations(world, a1.linear * 2);
    EXPECT_EQ(GetAcceleration(*b1), (Acceleration{a1.linear * 2, a1.angular}));
    EXPECT_EQ(GetAcceleration(*b2), (Acceleration{a1.linear * 2, a2.angular}));
}

TEST(World, FindClosestBodyFF)
{
    World world;
    ASSERT_EQ(FindClosestBody(world, Length2{}), nullptr);
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(Length2{10_m, 10_m}));
    EXPECT_EQ(FindClosestBody(world, Length2{0_m, 0_m}), b1);
    const auto b2 = world.CreateBody(BodyConf{}.UseLocation(Length2{1_m, -2_m}));
    EXPECT_EQ(FindClosestBody(world, Length2{0_m, 0_m}), b2);
    const auto b3 = world.CreateBody(BodyConf{}.UseLocation(Length2{-5_m, 4_m}));
    EXPECT_NE(FindClosestBody(world, Length2{0_m, 0_m}), b3);
    EXPECT_EQ(FindClosestBody(world, Length2{0_m, 0_m}), b2);
}

TEST(World, GetShapeCountFreeFunction)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetShapeCount(world), std::size_t(0));
    
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shapeConf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);

    const auto shape1 = Shape{shapeConf};
    
    const auto fixture1 = body->CreateFixture(shape1);
    ASSERT_NE(fixture1, nullptr);
    EXPECT_EQ(GetShapeCount(world), std::size_t(1));

    const auto fixture2 = body->CreateFixture(shape1);
    ASSERT_NE(fixture2, nullptr);
    EXPECT_EQ(GetShapeCount(world), std::size_t(1));
    
    const auto shape2 = Shape{shapeConf};

    const auto fixture3 = body->CreateFixture(shape2);
    ASSERT_NE(fixture3, nullptr);
    EXPECT_EQ(GetShapeCount(world), std::size_t(2));
}

TEST(World, GetFixtureCountFreeFunction)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetFixtureCount(world), std::size_t(0));
    
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shapeConf = EdgeShapeConf{}
        .UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    
    const auto shape = Shape{shapeConf};
    
    const auto fixture1 = body->CreateFixture(shape);
    ASSERT_NE(fixture1, nullptr);
    EXPECT_EQ(GetFixtureCount(world), std::size_t(1));
    
    const auto fixture2 = body->CreateFixture(shape);
    ASSERT_NE(fixture2, nullptr);
    EXPECT_EQ(GetFixtureCount(world), std::size_t(2));
    
    const auto fixture3 = body->CreateFixture(shape);
    ASSERT_NE(fixture3, nullptr);
    EXPECT_EQ(GetFixtureCount(world), std::size_t(3));
}

TEST(World, AwakenFreeFunction)
{
    World world{};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body->IsSpeedable());
    ASSERT_TRUE(body->IsAccelerable());
    ASSERT_FALSE(body->IsImpenetrable());
    ASSERT_EQ(GetX(body->GetLinearAcceleration()), Real(0) * MeterPerSquareSecond);
    ASSERT_EQ(GetY(body->GetLinearAcceleration()), Real(0) * MeterPerSquareSecond);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto shape = Shape{EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2)};
    const auto fixture = body->CreateFixture(shape);
    ASSERT_NE(fixture, nullptr);
    
    ASSERT_TRUE(body->IsAwake());
    auto stepConf = StepConf{};
    while (body->IsAwake())
        world.Step(stepConf);
    ASSERT_FALSE(body->IsAwake());
    
    Awaken(world);
    EXPECT_TRUE(body->IsAwake());
}

TEST(World, GetTouchingCountFreeFunction)
{
    World world;
    EXPECT_EQ(GetTouchingCount(world), ContactCounter(0));
    auto stepConf = StepConf{};
    world.Step(stepConf);
    EXPECT_EQ(GetTouchingCount(world), ContactCounter(0));
    stepConf.SetInvTime(100_Hz);
    world.Step(stepConf);
    EXPECT_EQ(GetTouchingCount(world), ContactCounter(0));
    
    const auto groundConf = EdgeShapeConf{}
        .Set(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter);
    const auto ground = world.CreateBody();
    ground->CreateFixture(Shape(groundConf));

    const auto lowerBodyConf = BodyConf{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0.0f, 0.5f) * Meter);
    const auto diskConf = DiskShapeConf{}.UseDensity(10_kgpm2);
    const auto smallerDiskConf = DiskShapeConf(diskConf).UseRadius(0.5_m);
    const auto lowerBody = world.CreateBody(lowerBodyConf);
    lowerBody->CreateFixture(Shape(smallerDiskConf));
    
    ASSERT_EQ(GetAwakeCount(world), 1);
    while (GetAwakeCount(world) > 0)
    {
        world.Step(stepConf);
        EXPECT_EQ(GetTouchingCount(world), ContactCounter(1));
    }
}

TEST(World, ShiftOrigin)
{
    const auto origin = Length2{0_m, 0_m};
    const auto location = Length2{1_m, 1_m};
    
    ASSERT_NE(origin, location);

    World world;
    EXPECT_NO_THROW(world.ShiftOrigin(origin));
    
    auto bodyConf = BodyConf{};
    bodyConf.UseLocation(location);
    const auto body = world.CreateBody(bodyConf);
    EXPECT_EQ(body->GetLocation(), location);

    EXPECT_NO_THROW(world.ShiftOrigin(location));
    EXPECT_EQ(body->GetLocation(), origin);
}

TEST(World, DynamicEdgeBodyHasCorrectMass)
{
    World world;
    
    auto bodyConf = BodyConf{};
    bodyConf.type = BodyType::Dynamic;
    const auto body = world.CreateBody(bodyConf);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto conf = EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2).Set(v1, v2);
    const auto shape = Shape{conf};
    ASSERT_EQ(GetVertexRadius(shape, 0), 1_m);

    const auto fixture = body->CreateFixture(shape);
    ASSERT_NE(fixture, nullptr);
    ASSERT_EQ(fixture->GetDensity(), 1_kgpm2);

    const auto circleMass = Mass{fixture->GetDensity() * (Pi * Square(GetVertexRadius(shape, 0)))};
    const auto rectMass = Mass{fixture->GetDensity() * (GetVertexRadius(shape, 0) * 2 * GetMagnitude(v2 - v1))};
    const auto totalMass = Mass{circleMass + rectMass};
    
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    EXPECT_NEAR(static_cast<double>(Real{body->GetInvMass() * 1_kg}),
                static_cast<double>(Real{1_kg / totalMass}),
                0.000001);
}

TEST(World, CreateAndDestroyJoint)
{
    World world;

    const auto body1 = world.CreateBody();
    const auto body2 = world.CreateBody();
    EXPECT_NE(body1, nullptr);
    EXPECT_NE(body2, nullptr);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_TRUE(world.GetJoints().empty());
    EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
    
    const auto anchorA = Length2{+0.4_m, -1.2_m};
    const auto anchorB = Length2{-2.3_m, +0.7_m};
    const auto joint = world.CreateJoint(DistanceJointConf{body1, body2, anchorA, anchorB});
    EXPECT_EQ(GetJointCount(world), JointCounter(1));
    EXPECT_FALSE(world.GetJoints().empty());
    EXPECT_NE(world.GetJoints().begin(), world.GetJoints().end());
    const auto first = *world.GetJoints().begin();
    EXPECT_EQ(joint, first);
    EXPECT_EQ(GetType(*joint), JointType::Distance);
    EXPECT_EQ(joint->GetBodyA(), body1);
    EXPECT_EQ(joint->GetBodyB(), body2);
    EXPECT_EQ(joint->GetAnchorA(), anchorA);
    EXPECT_EQ(joint->GetAnchorB(), anchorB);
    EXPECT_FALSE(joint->GetCollideConnected());

    world.Destroy(joint);
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_TRUE(world.GetJoints().empty());
    EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
    
    EXPECT_THROW(world.CreateJoint(JointConf{JointType::Unknown}), InvalidArgument);
}

TEST(World, MaxBodies)
{
    World world;
    for (auto i = decltype(MaxBodies){0}; i < MaxBodies; ++i)
    {
        const auto body = world.CreateBody();
        ASSERT_NE(body, nullptr);
    }
    {
        EXPECT_THROW(world.CreateBody(), LengthError);
    }
}

TEST(World, MaxJoints)
{
    World world;
    
    const auto body1 = world.CreateBody();
    ASSERT_NE(body1, nullptr);
    const auto body2 = world.CreateBody();
    ASSERT_NE(body2, nullptr);
    
    for (auto i = decltype(MaxJoints){0}; i < MaxJoints; ++i)
    {
        const auto joint = world.CreateJoint(RopeJointConf{body1, body2});
        ASSERT_NE(joint, nullptr);
    }
    {
        EXPECT_THROW(world.CreateJoint(RopeJointConf{body1, body2}), LengthError);
    }
}

TEST(World, StepZeroTimeDoesNothing)
{
    World world{};
    
    BodyConf def;
    def.location = Length2{31.9_m, -19.24_m};
    def.type = BodyType::Dynamic;
    def.linearAcceleration = EarthlyGravity;
    
    const auto body = world.CreateBody(def);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->GetLocation(), def.location);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetX(body->GetLinearAcceleration()), Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
    
    const auto time_inc = 0_s;
    
    auto pos = body->GetLocation();
    auto vel = GetLinearVelocity(*body);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, time_inc);
        
        EXPECT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
        
        EXPECT_EQ(GetX(body->GetLocation()), GetX(def.location));
        EXPECT_EQ(GetY(body->GetLocation()), GetY(pos));
        pos = body->GetLocation();
        
        EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLinearVelocity(*body)) / 1_mps}, Real{GetY(vel) / 1_mps}));
        vel = GetLinearVelocity(*body);
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

    const auto body = world.CreateBody(body_def);
    ASSERT_NE(body, nullptr);
    EXPECT_FALSE(body->IsImpenetrable());
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(body->GetLocation(), p0);

    Step(world, t);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body)), a * (t * Real{1}) * MeterPerSquareSecond);
    EXPECT_EQ(GetX(body->GetLocation()), GetX(p0));
    EXPECT_EQ(GetY(body->GetLocation()), GetY(p0) + GetY(GetLinearVelocity(*body)) * t);

    p0 = body->GetLocation();
    Step(world, t);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body)), a * (t * Real{2}) * MeterPerSquareSecond);
    EXPECT_EQ(GetX(body->GetLocation()), GetX(p0));
    EXPECT_EQ(GetY(body->GetLocation()), GetY(p0) + GetY(GetLinearVelocity(*body)) * t);
    
    p0 = body->GetLocation();
    Step(world, t);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_NEAR(double(Real{GetY(GetLinearVelocity(*body)) / 1_mps}),
                double(Real{a * (t * Real{3}) / 1_s}), 0.00001);
    EXPECT_EQ(GetX(body->GetLocation()), GetX(p0));
    EXPECT_EQ(GetY(body->GetLocation()), GetY(p0) + GetY(GetLinearVelocity(*body)) * t);
}

#if 0
TEST(World, BodyAngleDoesntGrowUnbounded)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}
                                       .UseType(BodyType::Dynamic)
                                       .UseAngularVelocity(10_rad / Second));
    ASSERT_EQ(GetAngle(*body), 0_rad);
    auto stepConf = StepConf{};
    auto lastAngle = 0_rad;
    auto maxAngle = 0_rad;
    for (auto i = 0; i < 1000000; ++i)
    {
        world.Step(stepConf);
        const auto angle = GetAngle(*body);
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
    
    const auto body = world.CreateBody(def);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->GetLocation(), def.location);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetX(body->GetLinearAcceleration()), Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
    
    const auto time_inc = 0.01_s;
    
    auto pos = body->GetLocation();
    auto vel = GetLinearVelocity(*body);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, time_inc, 0, 0);
        
        EXPECT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
        
        EXPECT_EQ(GetX(body->GetLocation()), GetX(def.location));
        EXPECT_LT(GetY(body->GetLocation()), GetY(pos));
        EXPECT_EQ(GetY(body->GetLocation()), GetY(pos) + ((GetY(vel) + GetY(EarthlyGravity) * time_inc) * time_inc));
        pos = body->GetLocation();
        
        EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
        EXPECT_LT(GetY(GetLinearVelocity(*body)), GetY(vel));
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLinearVelocity(*body)) / 1_mps},
                                Real{(GetY(vel) + GetY(EarthlyGravity) * time_inc) / 1_mps}));
        vel = GetLinearVelocity(*body);
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
    
    const auto body = world.CreateBody(def);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->GetLocation(), def.location);
    EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body)), -9.8_mps);
    EXPECT_EQ(GetX(body->GetLinearAcceleration()), Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
    
    const auto time_inc = -0.01_s;
    auto stepConf = StepConf{};
    stepConf.SetTime(time_inc);
    stepConf.dtRatio = -1;
    stepConf.regPositionIterations = 0;
    stepConf.regVelocityIterations = 0;
    stepConf.toiPositionIterations = 0;
    stepConf.toiVelocityIterations = 0;
    
    auto pos = body->GetLocation();
    auto vel = GetLinearVelocity(*body);
    for (auto i = 0; i < 99; ++i)
    {
        world.Step(stepConf);
        
        EXPECT_EQ(GetY(body->GetLinearAcceleration()), GetY(EarthlyGravity));
        
        EXPECT_EQ(GetX(body->GetLocation()), GetX(def.location));
        EXPECT_GT(GetY(body->GetLocation()), GetY(pos));
        EXPECT_EQ(GetY(body->GetLocation()), GetY(pos) + ((GetY(vel) + GetY(EarthlyGravity) * time_inc) * time_inc));
        pos = body->GetLocation();
        
        EXPECT_EQ(GetX(GetLinearVelocity(*body)), 0_mps);
        EXPECT_GT(GetY(GetLinearVelocity(*body)), GetY(vel));
        EXPECT_TRUE(AlmostEqual(Real{GetY(GetLinearVelocity(*body)) / 1_mps},
                                Real{(GetY(vel) + GetY(EarthlyGravity) * time_inc) / 1_mps}));
        vel = GetLinearVelocity(*body);
    }
}

class MyContactListener: public ContactListener
{
public:
    using PreSolver = std::function<void(Contact&, const Manifold&)>;
    using PostSolver = std::function<void(Contact&, const ContactImpulsesList&, ContactListener::iteration_type)>;
    using Ender = std::function<void(Contact&)>;

    MyContactListener(PreSolver&& pre, PostSolver&& post, Ender&& end): presolver(pre), postsolver(post), ender(end) {}

    virtual ~MyContactListener() {}

    void BeginContact(Contact& contact) override
    {
        ++begin_contacts;
        contacting = true;
        touching = contact.IsTouching();
        
        const auto fA = contact.GetFixtureA();
        const auto fB = contact.GetFixtureB();
        const auto bA = fA->GetBody();
        const auto bB = fB->GetBody();
        const auto w = bA->GetWorld();
        body_a[0] = bA->GetLocation();
        body_b[0] = bB->GetLocation();
        
        EXPECT_THROW(w->CreateBody(), WrongState);
        const auto typeA = bA->GetType();
        if (typeA != BodyType::Kinematic)
        {
            EXPECT_NO_THROW(bA->SetType(typeA));
            EXPECT_THROW(bA->SetType(BodyType::Kinematic), WrongState);
        }
        EXPECT_THROW(w->Destroy(bA), WrongState);
        EXPECT_THROW(w->Clear(), WrongState);
        EXPECT_THROW(w->CreateJoint(DistanceJointConf{bA, bB}), WrongState);
        EXPECT_THROW(w->Step(stepConf), WrongState);
        EXPECT_THROW(w->ShiftOrigin(Length2{}), WrongState);
        EXPECT_THROW(bA->CreateFixture(Shape{DiskShapeConf{}}), WrongState);
        EXPECT_THROW(bA->Destroy(fA), WrongState);
    }
    
    void EndContact(Contact& contact) override
    {
        ++end_contacts;
        contacting = false;
        touching = contact.IsTouching();

        body_a[1] = contact.GetFixtureA()->GetBody()->GetLocation();
        body_b[1] = contact.GetFixtureB()->GetBody()->GetLocation();
        
        if (ender)
        {
            ender(contact);
        }
    }
    
    void PreSolve(Contact& contact, const Manifold& oldManifold) override
    {
        ++pre_solves;
        presolver(contact, oldManifold);
    }
    
    void PostSolve(Contact& contact, const ContactImpulsesList& impulse, ContactListener::iteration_type solved) override
    {
        ++post_solves;
        postsolver(contact, impulse, solved);
    }

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
    MyContactListener listener{
        [&](Contact&, const Manifold&) { ++presolved; },
        [&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) { ++postsolved; },
        [&](Contact&) {},
    };

    World world{};
    world.SetContactListener(&listener);
    
    ASSERT_EQ(listener.begin_contacts, unsigned(0));
    ASSERT_EQ(listener.end_contacts, unsigned(0));
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = true;
    
    const auto shape = Shape{DiskShapeConf{}.UseRadius(1_m).UseRestitution(Real(1)).UseDensity(1_kgpm2)};
    
    body_def.location = Length2{-x * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{+x * 1_mps, 0_mps};
    const auto body_a = world.CreateBody(body_def);
    ASSERT_NE(body_a, nullptr);
    EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body_a->IsSpeedable());
    EXPECT_TRUE(body_a->IsAccelerable());
    const auto fixture1 = body_a->CreateFixture(shape);
    ASSERT_NE(fixture1, nullptr);
    
    body_def.location = Length2{+x * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{-x * 1_mps, 0_mps};
    const auto body_b = world.CreateBody(body_def);
    ASSERT_NE(body_b, nullptr);
    const auto fixture2 = body_b->CreateFixture(shape);
    ASSERT_NE(fixture2, nullptr);
    EXPECT_EQ(body_b->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body_b->IsSpeedable());
    EXPECT_TRUE(body_b->IsAccelerable());

    EXPECT_EQ(GetX(GetLinearVelocity(*body_a)), +x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body_a)), 0_mps);
    EXPECT_EQ(GetX(GetLinearVelocity(*body_b)), -x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body_b)), 0_mps);

    const auto time_inc = .01_s;

    auto pos_a = body_a->GetLocation();
    auto pos_b = body_b->GetLocation();
    ASSERT_LT(GetX(pos_a), GetX(pos_b));

    auto conf = StepConf{};
    conf.SetTime(time_inc);
    conf.regPositionIterations = 0;
    conf.regVelocityIterations = 0;
    conf.toiPositionIterations = 0;
    conf.toiVelocityIterations = 0;
    conf.tolerance = nextafter(StripUnit(conf.targetDepth), Real{0}) * Meter;

    auto steps = unsigned{0};
    while (GetX(pos_a) < (x * Meter) && GetX(pos_b) > (-x * Meter))
    {
        world.Step(conf);
        ++steps;
        
        EXPECT_TRUE(AlmostEqual(Real{GetX(body_a->GetLocation()) / Meter},
                                Real{(GetX(pos_a) + x * time_inc * 1_mps) / Meter}));
        EXPECT_EQ(GetY(body_a->GetLocation()), 0_m);
        EXPECT_TRUE(AlmostEqual(Real{GetX(body_b->GetLocation()) / Meter},
                                Real{(GetX(pos_b) - x * time_inc * 1_mps) / Meter}));
        EXPECT_EQ(GetY(body_b->GetLocation()), 0_m);

        EXPECT_EQ(GetX(GetLinearVelocity(*body_a)), +x * 1_mps);
        EXPECT_EQ(GetY(GetLinearVelocity(*body_a)), 0_mps);
        EXPECT_EQ(GetX(GetLinearVelocity(*body_b)), -x * 1_mps);
        EXPECT_EQ(GetY(GetLinearVelocity(*body_b)), 0_mps);

        pos_a = body_a->GetLocation();
        pos_b = body_b->GetLocation();
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
    PLAYRHO_CONSTEXPR const auto AngularSlop = (Pi * Real{2} * 1_rad) / Real{180};
    PLAYRHO_CONSTEXPR const auto LargerLinearSlop = playrho::Meter / playrho::Real(200);
    PLAYRHO_CONSTEXPR const auto SmallerLinearSlop = playrho::Meter / playrho::Real(1000);

    const auto bd = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity);
    const auto upperBodyConf = BodyConf(bd).UseLocation(Vec2(0.0f, 6.0f) * Meter);
    const auto lowerBodyConf = BodyConf(bd).UseLocation(Vec2(0.0f, 0.5f) * Meter);

    const auto groundConf = EdgeShapeConf{}
        .Set(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter);
    
    const auto diskConf = DiskShapeConf{}.UseDensity(10_kgpm2);
    const auto smallerDiskConf = DiskShapeConf(diskConf).UseRadius(0.5_m);
    const auto biggerDiskConf = DiskShapeConf(diskConf).UseRadius(5.0_m);
    
    const auto baseStepConf = []() {
        auto step = StepConf{}.SetInvTime(60_Hz);
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
        const auto ground = world.CreateBody();
        ground->CreateFixture(Shape(groundConf));

        const auto lowerBody = world.CreateBody(lowerBodyConf);
        const auto upperBody = world.CreateBody(upperBodyConf);
        ASSERT_LT(GetY(lowerBody->GetLocation()), GetY(upperBody->GetLocation()));

        lowerBody->CreateFixture(Shape(smallerDiskConf));
        upperBody->CreateFixture(Shape(biggerDiskConf));
        ASSERT_LT(GetMass(*lowerBody), GetMass(*upperBody));
        
        auto upperBodysLowestPoint = GetY(upperBody->GetLocation());
        auto numSteps = 0ul;
        while (GetAwakeCount(world) > 0)
        {
            world.Step(largerStepConf);
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(upperBody->GetLocation()));
            ++numSteps;
        }
        
        // The least num steps is 145
        EXPECT_EQ(numSteps, 145ul);
        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9475154876708984, 0.001);
    }
    
    // Create upper body, then lower body using the larger step conf
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = world.CreateBody();
        ground->CreateFixture(Shape(groundConf));
        
        const auto upperBody = world.CreateBody(upperBodyConf);
        const auto lowerBody = world.CreateBody(lowerBodyConf);
        ASSERT_LT(GetY(lowerBody->GetLocation()), GetY(upperBody->GetLocation()));

        lowerBody->CreateFixture(Shape(smallerDiskConf));
        upperBody->CreateFixture(Shape(biggerDiskConf));
        ASSERT_LT(GetMass(*lowerBody), GetMass(*upperBody));
        
        auto upperBodysLowestPoint = GetY(upperBody->GetLocation());
        auto numSteps = 0ul;
        while (GetAwakeCount(world) > 0)
        {
            world.Step(largerStepConf);
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(upperBody->GetLocation()));
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
        const auto ground = world.CreateBody();
        ground->CreateFixture(Shape(groundConf));
        
        const auto lowerBody = world.CreateBody(lowerBodyConf);
        const auto upperBody = world.CreateBody(upperBodyConf);
        ASSERT_LT(GetY(lowerBody->GetLocation()), GetY(upperBody->GetLocation()));

        lowerBody->CreateFixture(Shape(smallerDiskConf));
        upperBody->CreateFixture(Shape(biggerDiskConf));
        ASSERT_LT(GetMass(*lowerBody), GetMass(*upperBody));
        
        auto upperBodysLowestPoint = GetY(upperBody->GetLocation());
        auto numSteps = 0ul;
        while (GetAwakeCount(world) > 0)
        {
            world.Step(smallerStepConf);
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(upperBody->GetLocation()));
            ++numSteps;
        }
        
        // This here is the highest step count.
        // XXX Is this a bug or did the algorithm just work least well here?
        switch (sizeof(Real))
        {
            case 4: EXPECT_EQ(numSteps, 736ul); break;
            case 8: EXPECT_EQ(numSteps, 736ul); break;
        }

        // Here we see that the upper body at some point sunk into most of the lower body.
        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9473052024841309, 0.001);
    }
    
    // Create upper body, then lower body using the smaller step conf
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = world.CreateBody();
        ground->CreateFixture(Shape(groundConf));
        
        const auto upperBody = world.CreateBody(upperBodyConf);
        const auto lowerBody = world.CreateBody(lowerBodyConf);
        ASSERT_LT(GetY(lowerBody->GetLocation()), GetY(upperBody->GetLocation()));

        lowerBody->CreateFixture(Shape(smallerDiskConf));
        upperBody->CreateFixture(Shape(biggerDiskConf));
        ASSERT_LT(GetMass(*lowerBody), GetMass(*upperBody));
        
        auto upperBodysLowestPoint = GetY(upperBody->GetLocation());
        auto numSteps = 0ul;
        EXPECT_EQ(GetAwakeCount(world), 2);
        while (GetAwakeCount(world) > 0)
        {
            world.Step(smallerStepConf);
            EXPECT_EQ(GetTouchingCount(world), ContactCounter(2));
            upperBodysLowestPoint = std::min(upperBodysLowestPoint, GetY(upperBody->GetLocation()));
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
        }

        EXPECT_NEAR(static_cast<double>(Real(upperBodysLowestPoint / Meter)), 5.9476470947265625, 0.001);
    }
    
    // Create upper body, then lower body using the smaller step conf, and using sensors
    {
        auto world = World{WorldConf{}.UseMinVertexRadius(SmallerLinearSlop)};
        const auto ground = world.CreateBody();
        ground->CreateFixture(Shape(groundConf));
        
        const auto upperBody = world.CreateBody(upperBodyConf);
        const auto lowerBody = world.CreateBody(lowerBodyConf);
        ASSERT_LT(GetY(lowerBody->GetLocation()), GetY(upperBody->GetLocation()));
        
        lowerBody->CreateFixture(Shape(smallerDiskConf), FixtureConf{}.UseIsSensor(true));
        upperBody->CreateFixture(Shape(biggerDiskConf), FixtureConf{}.UseIsSensor(true));
        ASSERT_LT(GetMass(*lowerBody), GetMass(*upperBody));
        
        EXPECT_EQ(GetAwakeCount(world), BodyCounter(2));
        world.Step(smallerStepConf);
        EXPECT_EQ(GetTouchingCount(world), ContactCounter(2));
    }
}

TEST(World, PerfectlyOverlappedSameCirclesStayPut)
{
    const auto radius = 1_m;
    const auto shape = Shape{DiskShapeConf{}.UseRadius(radius).UseDensity(1_kgpm2).UseRestitution(Real(1))};

    auto world = World{};
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.location = Length2{0_m, 0_m};

    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation(), body_def.location);
    
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation(), body_def.location);
    
    const auto time_inc = Real(.01);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, 1_s * time_inc);
        EXPECT_EQ(body1->GetLocation(), body_def.location);
        EXPECT_EQ(body2->GetLocation(), body_def.location);
    }
}

TEST(World, PerfectlyOverlappedConcentricCirclesStayPut)
{
    const auto radius1 = 1_m;
    const auto radius2 = 0.6_m;
    const auto shape1 = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius1));
    const auto shape2 = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius2));
    
    World world{};
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.location = Length2{};
    
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape1);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation(), body_def.location);
    
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape2);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation(), body_def.location);
    
    const auto time_inc = Real(.01);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, 1_s * time_inc);
        EXPECT_EQ(body1->GetLocation(), body_def.location);
        EXPECT_EQ(body2->GetLocation(), body_def.location);
    }
}

TEST(World, ListenerCalledForCircleBodyWithinCircleBody)
{
    World world{};
    MyContactListener listener{
        [&](Contact&, const Manifold&) {},
        [&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };
    world.SetContactListener(&listener);

    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{};
    const auto shape = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(1_m));
    for (auto i = 0; i < 2; ++i)
    {
        const auto body = world.CreateBody(body_def);
        ASSERT_NE(body, nullptr);
        ASSERT_NE(body->CreateFixture(shape), nullptr);
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
        [&](Contact&, const Manifold&) {},
        [&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };
    world.SetContactListener(&listener);
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{};
    auto conf = PolygonShapeConf{};
    conf.UseVertexRadius(1_m);
    conf.SetAsBox(2_m, 2_m);
    conf.UseDensity(1_kgpm2);
    conf.UseRestitution(Real(1));
    const auto shape = Shape{conf};
    for (auto i = 0; i < 2; ++i)
    {
        const auto body = world.CreateBody(body_def);
        ASSERT_NE(body, nullptr);
        ASSERT_NE(body->CreateFixture(shape), nullptr);
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

TEST(World, PartiallyOverlappedSameCirclesSeparate)
{
    const auto radius = Real(1);
    
    World world{};
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false; // separation is faster if true.
    const auto shape = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius * Meter));
    const auto body1pos = Length2{(-radius/4) * Meter, 0_m};
    body_def.location = body1pos;
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation(), body_def.location);
    
    const auto body2pos = Length2{(+radius/4) * Meter, 0_m};
    body_def.location = body2pos;
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation(), body_def.location);
    
    auto position_diff = body2pos - body1pos;
    auto distance = GetMagnitude(position_diff);

    const auto angle = GetAngle(position_diff);
    ASSERT_EQ(angle, 0_deg);

    auto lastpos1 = body1->GetLocation();
    auto lastpos2 = body2->GetLocation();

    const auto time_inc = .01_s;
    StepConf step;
    step.SetTime(time_inc);

    // Solver won't separate more than -step.linearSlop.
    const auto full_separation = radius * 2_m - Length{step.linearSlop};
    for (auto i = 0; i < 100; ++i)
    {
        world.Step(step);

        const auto new_pos_diff = body2->GetLocation() - body1->GetLocation();
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
                EXPECT_LT(GetX(body1->GetLocation()), GetX(lastpos1));
                EXPECT_GT(GetX(body2->GetLocation()), GetX(lastpos2));
            }
            if (sin(angle) != 0)
            {
                EXPECT_LT(GetY(body1->GetLocation()), GetY(lastpos1));
                EXPECT_GT(GetY(body2->GetLocation()), GetY(lastpos2));
            }
        }

        ASSERT_NE(body1->GetLocation(), lastpos1);
        ASSERT_NE(body2->GetLocation(), lastpos2);
        
        lastpos1 = body1->GetLocation();
        lastpos2 = body2->GetLocation();

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
    const auto shape = Shape(PolygonShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).SetAsBox(1_m, 1_m));
    
    World world{};
    
    auto body_def = BodyConf{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.location = Length2{};
    
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation(), body_def.location);
    
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation(), body_def.location);
    
    auto lastpos1 = body1->GetLocation();
    auto lastpos2 = body2->GetLocation();

    auto stepConf = StepConf{};
    const auto time_inc = .01_s;
    stepConf.SetTime(time_inc);
    stepConf.maxLinearCorrection = Real{0.0001f * 40} * Meter;
    for (auto i = 0; i < 100; ++i)
    {
        world.Step(stepConf);
        
        // body1 moves left only
        EXPECT_LT(GetX(body1->GetLocation()), GetX(lastpos1));
        EXPECT_EQ(GetY(body1->GetLocation()), GetY(lastpos1));

        // body2 moves right only
        EXPECT_GT(GetX(body2->GetLocation()), GetX(lastpos2));
        EXPECT_EQ(GetY(body2->GetLocation()), GetY(lastpos2));
        
        // body1 and body2 move away from each other equally.
        EXPECT_EQ(GetX(body1->GetLocation()), -GetX(body2->GetLocation()));
        EXPECT_EQ(GetY(body1->GetLocation()), -GetY(body2->GetLocation()));
        
        lastpos1 = body1->GetLocation();
        lastpos2 = body2->GetLocation();
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
    const auto shape = Shape(PolygonShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).SetAsBox(half_dim * Meter, half_dim * Meter));
    
    const auto body1pos = Length2{Real(half_dim/2) * Meter, 0_m}; // 0 causes additional y-axis separation
    body_def.location = body1pos;
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation(), body1pos);
    
    const auto body2pos = Length2{-Real(half_dim/2) * Meter, 0_m}; // 0 causes additional y-axis separation
    body_def.location = body2pos;
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation(), body2pos);

    ASSERT_EQ(body1->GetAngle(), 0_deg);
    ASSERT_EQ(body2->GetAngle(), 0_deg);
    auto last_angle_1 = body1->GetAngle();
    auto last_angle_2 = body2->GetAngle();

    ASSERT_EQ(world.GetBodies().size(), World::Bodies::size_type(2));
    ASSERT_EQ(world.GetContacts().size(), World::Contacts::size_type(0));

    auto position_diff = body1pos - body2pos;
    auto distance = GetMagnitude(position_diff);
    
    auto angle = GetAngle(position_diff);
    EXPECT_TRUE(AlmostEqual(Real{angle / 1_rad}, Real{0}));
    
    auto lastpos1 = body1->GetLocation();
    auto lastpos2 = body2->GetLocation();
    
    const auto velocity_iters = 10u;
    const auto position_iters = 10u;
    
    const auto time_inc = Real(.01);
    StepConf step;
    step.SetTime(1_s * time_inc);
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
            const auto c = GetPtr(contact.second);

            const auto fa = c->GetFixtureA();
            const auto fb = c->GetFixtureB();
            const auto body_a = fa->GetBody();
            const auto body_b = fb->GetBody();
            EXPECT_EQ(body_a, body1);
            EXPECT_EQ(body_b, body2);
            
            const auto& manifold = c->GetManifold();
            EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
            EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
        }
        ASSERT_EQ(count, decltype(world.GetContacts().size())(1));

        const auto v1 = body1->GetVelocity();
        EXPECT_EQ(v1.angular, 0_deg / 1_s);
        EXPECT_EQ(GetX(v1.linear), 0_mps);
        EXPECT_EQ(GetY(v1.linear), 0_mps);

        const auto v2 = body2->GetVelocity();
        EXPECT_EQ(v2.angular, 0_deg / 1_s);
        EXPECT_EQ(GetX(v2.linear), 0_mps);
        EXPECT_EQ(GetY(v2.linear), 0_mps);

        EXPECT_TRUE(AlmostEqual(Real{body1->GetAngle() / 1_rad}, Real{last_angle_1 / 1_rad}));
        EXPECT_TRUE(AlmostEqual(Real{body2->GetAngle() / 1_rad}, Real{last_angle_2 / 1_rad}));
        last_angle_1 = body1->GetAngle();
        last_angle_2 = body2->GetAngle();

        const auto new_pos_diff = body1->GetLocation() - body2->GetLocation();
        const auto new_distance = GetMagnitude(new_pos_diff);
        
        if (AlmostEqual(Real{new_distance / Meter}, Real{full_separation / Meter}) || new_distance > full_separation)
        {
            break;
        }
        
        if (new_distance == distance)
        {
            if (cos(angle) != 0)
            {
                EXPECT_NE(GetX(body1->GetLocation()), GetX(lastpos1));
                EXPECT_NE(GetX(body2->GetLocation()), GetX(lastpos2));
            }
            if (sin(angle) != 0)
            {
                EXPECT_NE(GetY(body1->GetLocation()), GetY(lastpos1));
                EXPECT_NE(GetY(body2->GetLocation()), GetY(lastpos2));
            }
            ASSERT_GE(new_distance, 2_m);
            break;
        }
        
        ASSERT_NE(body1->GetLocation(), lastpos1);
        ASSERT_NE(body2->GetLocation(), lastpos2);
        
        // Body 1 moves right only.
        EXPECT_GT(GetX(body1->GetLocation()), GetX(lastpos1));
        EXPECT_TRUE(AlmostEqual(Real{GetY(body1->GetLocation()) / Meter}, Real{GetY(lastpos1) / Meter}));

        // Body 2 moves left only.
        EXPECT_LT(GetX(body2->GetLocation()), GetX(lastpos2));
        EXPECT_TRUE(AlmostEqual(Real{GetY(body2->GetLocation()) / Meter}, Real{GetY(lastpos2) / Meter}));

        lastpos1 = body1->GetLocation();
        lastpos2 = body2->GetLocation();
        
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
    
    MyContactListener listener{
        [](Contact&, const Manifold&) {},
        [](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };

    World world{};
    world.SetContactListener(&listener);
    
    const auto shape = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(radius));

    body_def.location = Length2{-(x + 1) * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{+x * 1_mps, 0_mps};
    const auto body_a = world.CreateBody(body_def);
    ASSERT_NE(body_a, nullptr);
    ASSERT_EQ(body_a->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body_a->IsSpeedable());
    ASSERT_TRUE(body_a->IsAccelerable());
    
    const auto fixture1 = body_a->CreateFixture(shape);
    ASSERT_NE(fixture1, nullptr);

    body_def.location = Length2{+(x + 1) * Meter, 0_m};
    body_def.linearVelocity = LinearVelocity2{-x * 1_mps, 0_mps};
    const auto body_b = world.CreateBody(body_def);
    ASSERT_NE(body_b, nullptr);
    ASSERT_EQ(body_b->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body_b->IsSpeedable());
    ASSERT_TRUE(body_b->IsAccelerable());

    const auto fixture2 = body_b->CreateFixture(shape);
    ASSERT_NE(fixture2, nullptr);

    EXPECT_EQ(GetX(GetLinearVelocity(*body_a)), +x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body_a)), 0_mps);
    EXPECT_EQ(GetX(GetLinearVelocity(*body_b)), -x * 1_mps);
    EXPECT_EQ(GetY(GetLinearVelocity(*body_b)), 0_mps);
    
    const auto time_collision = Real(1.0099994); // only valid for x >= around 4.214
    const auto time_inc = Real(.01);
    
    auto elapsed_time = Real(0);
    for (;;)
    {
        Step(world, 1_s * time_inc);
        elapsed_time += time_inc;
        if (listener.contacting)
        {
            break;
        }
    }
    
    // Call Refilter and SetSensor to add some unit test coverage of these Fixture methods.
    EXPECT_FALSE(body_a->GetContacts().empty());
    for (auto&& ci: body_a->GetContacts())
    {
        EXPECT_FALSE(ci.second->NeedsFiltering());
        EXPECT_TRUE(ci.second->NeedsUpdating());
    }
    fixture1->Refilter();
    EXPECT_FALSE(fixture1->IsSensor());
    fixture1->SetSensor(true);
    EXPECT_TRUE(fixture1->IsSensor());
    fixture1->SetSensor(false);
    EXPECT_FALSE(fixture1->IsSensor());
    EXPECT_FALSE(body_a->GetContacts().empty());
    for (auto&& ci: body_a->GetContacts())
    {
        EXPECT_TRUE(ci.second->NeedsFiltering());
        EXPECT_TRUE(ci.second->NeedsUpdating());
    }

    const auto time_contacting = elapsed_time;

    EXPECT_TRUE(listener.touching);
    EXPECT_NEAR(double(time_contacting), double(time_collision), 0.02);
    EXPECT_EQ(GetY(body_a->GetLocation()), 0_m);
    EXPECT_EQ(GetY(body_b->GetLocation()), 0_m);

    const auto tolerance = x / 100;
    
    // x position for body1 depends on restitution but it should be around -1
    EXPECT_GE(GetX(body_a->GetLocation()) / Meter, Real(-1) - tolerance);
    EXPECT_LT(GetX(body_a->GetLocation()) / Meter, Real(-1) + tolerance);

    // x position for body2 depends on restitution but it should be around +1
    EXPECT_LE(GetX(body_b->GetLocation()) / Meter, Real(+1) + tolerance);
    EXPECT_GT(GetX(body_b->GetLocation()) / Meter, Real(+1) - tolerance);
    
    // and their deltas from -1 and +1 should be about equal.
    EXPECT_TRUE(AlmostEqual(Real{(GetX(body_a->GetLocation()) + 1_m) / Meter},
                            Real{(1_m - GetX(body_b->GetLocation())) / Meter}));

    EXPECT_GE(GetX(listener.body_a[0]), -1_m);
    EXPECT_LE(GetX(listener.body_b[0]), +1_m);

    for (;;)
    {
        Step(world, 1_s * time_inc);
        elapsed_time += time_inc;
        if (!listener.contacting && !listener.touching)
        {
            break;
        }
    }
    EXPECT_FALSE(listener.touching);
    
    EXPECT_TRUE(AlmostEqual(elapsed_time, time_contacting + time_inc));
    
    // collision should be fully resolved now...
    EXPECT_LT(GetX(body_a->GetLocation()), -1_m);
    EXPECT_GT(GetX(body_b->GetLocation()), +1_m);
    
    // and their deltas from -1 and +1 should be about equal.
    EXPECT_TRUE(AlmostEqual(Real{(GetX(body_a->GetLocation()) + 1_m) / Meter},
                            Real{(1_m - GetX(body_b->GetLocation())) / Meter}));

    EXPECT_LT(GetX(listener.body_a[1]), -1_m);
    EXPECT_GT(GetX(listener.body_b[1]), +1_m);
    
    // confirm conservation of momentum:
    // velocities should now be same magnitude but in opposite directions
    EXPECT_NEAR(double(Real{GetX(GetLinearVelocity(*body_a)) / 1_mps}),
                double(-x), 0.0001);
    EXPECT_EQ(GetY(GetLinearVelocity(*body_a)), 0_mps);
    EXPECT_NEAR(double(Real{GetX(GetLinearVelocity(*body_b)) / 1_mps}),
                double(+x), 0.0001);
    EXPECT_EQ(GetY(GetLinearVelocity(*body_b)), 0_mps);
}

TEST(World_Longer, TilesComesToRest)
{
    PLAYRHO_CONSTEXPR const auto LinearSlop = Meter / 1000;
    PLAYRHO_CONSTEXPR const auto AngularSlop = (Pi * 2 * 1_rad) / 180;
    PLAYRHO_CONSTEXPR const auto VertexRadius = LinearSlop * 2;
    auto conf = PolygonShapeConf{}.UseVertexRadius(VertexRadius);
    const auto m_world = std::make_unique<World>(WorldConf{}.UseMinVertexRadius(VertexRadius));
    
    constexpr const auto e_count = 36;
    
    {
        const auto a = Real{0.5f};
        const auto ground = m_world->CreateBody(BodyConf{}.UseLocation(Length2{0, -a * Meter}));
        
        const auto N = 200;
        const auto M = 10;
        Length2 position;
        GetY(position) = 0.0_m;
        for (auto j = 0; j < M; ++j)
        {
            GetX(position) = -N * a * Meter;
            for (auto i = 0; i < N; ++i)
            {
                conf.SetAsBox(a * Meter, a * Meter, position, 0_deg);
                ground->CreateFixture(Shape{conf});
                GetX(position) += 2.0f * a * Meter;
            }
            GetY(position) -= 2.0f * a * Meter;
        }
    }
    
    {
        const auto a = Real{0.5f};
        conf.UseDensity(5_kgpm2);
        conf.SetAsBox(a * Meter, a * Meter);
        const auto shape = Shape(conf);
        
        Length2 x(-7.0_m, 0.75_m);
        Length2 y;
        const auto deltaX = Length2(0.5625_m, 1.25_m);
        const auto deltaY = Length2(1.125_m, 0.0_m);
        
        for (auto i = 0; i < e_count; ++i)
        {
            y = x;
            
            for (auto j = i; j < e_count; ++j)
            {
                const auto body = m_world->CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(y).UseLinearAcceleration(EarthlyGravity));
                body->CreateFixture(shape);
                y += deltaY;
            }
            
            x += deltaX;
        }
    }
    
    StepConf step;
    step.SetTime(1_s / 60);
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
    while (GetAwakeCount(*m_world) > 0)
    {
        const auto stats = m_world->Step(step);
        sumRegPosIters += stats.reg.sumPosIters;
        sumRegVelIters += stats.reg.sumVelIters;
        sumToiPosIters += stats.toi.sumPosIters;
        sumToiVelIters += stats.toi.sumVelIters;
        ++numSteps;
    }
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
    //   4.85618s with Real=float and NDEBUG defined.
    //   5.32973s with Real=double and NDEBUG defined.
    
    EXPECT_EQ(GetAwakeCount(*m_world), 0u);

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
    //   does an early exit from its velocityIterations looping.
    
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
            EXPECT_EQ(numSteps, 1796ul);
            EXPECT_EQ(sumRegPosIters, 36503ul);
            EXPECT_EQ(sumRegVelIters, 46923ul);
            EXPECT_EQ(sumToiPosIters, 43913ul);
            EXPECT_EQ(sumToiVelIters, 113000ul);

            // From commit 0b049bd28d1bbb01d1750ec1fc9498105f13d192 onward:
            //EXPECT_EQ(numSteps, 1912ul);
            //EXPECT_EQ(sumRegPosIters, 36657ul);
            //EXPECT_EQ(sumRegVelIters, 47843ul);
            //EXPECT_EQ(sumToiPosIters, 43931ul);
            //EXPECT_EQ(sumToiVelIters, 113034ul);

            // From commit ee74290c17422ccbd6a73f07d6fd9abe960da84a onward:
            //EXPECT_EQ(numSteps, 1802ul);
            //EXPECT_EQ(sumRegPosIters, 36524ul);
            //EXPECT_EQ(sumRegVelIters, 46981ul);
            //EXPECT_EQ(sumToiPosIters, 44084ul);
            //EXPECT_EQ(sumToiVelIters, 114366ul);
#else
            // From commit ee74290c17422ccbd6a73f07d6fd9abe960da84a onward:
            EXPECT_EQ(numSteps, 1003ul);
            EXPECT_EQ(sumRegPosIters, 52909ul);
            EXPECT_EQ(sumRegVelIters, 103896ul);
            EXPECT_EQ(sumToiPosIters, 20616ul);
            EXPECT_EQ(sumToiVelIters, 30175ul);
#endif
            
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            //EXPECT_EQ(numSteps, 1801ul);
            //EXPECT_EQ(sumRegPosIters, 36523ul);
            //EXPECT_EQ(sumRegVelIters, 46973ul);
            //EXPECT_EQ(sumToiPosIters, 44044ul);
            //EXPECT_EQ(sumToiVelIters, 114344ul);

            // From commit 04f9188c47961cafe76c55eb6b766a608593ee08 onward.
            //EXPECT_EQ(numSteps, 1856ul);
            //EXPECT_EQ(sumRegPosIters, 36720ul);
            //EXPECT_EQ(sumRegVelIters, 47656ul);
            //EXPECT_EQ(sumToiPosIters, 44263ul);
            //EXPECT_EQ(sumToiVelIters, 112833ul);
            
            // From commit d361c51d6aca13079e9d44b701715e62cec18a63 onward.
            //EXPECT_EQ(numSteps, 1856ul);
            //EXPECT_EQ(sumRegPosIters, 36720ul);
            //EXPECT_EQ(sumRegVelIters, 264376ul);
            //EXPECT_EQ(sumToiPosIters, 44263ul);
            //EXPECT_EQ(sumToiVelIters, 145488ul);
            
            // Pre commit d361c51d6aca13079e9d44b701715e62cec18a63
            //EXPECT_EQ(numSteps, 1814ul);
            //EXPECT_EQ(sumRegPosIters, 36600ul);
            //EXPECT_EQ(sumRegVelIters, 264096ul);
            //EXPECT_EQ(sumToiPosIters, 45022ul);
            //EXPECT_EQ(sumToiVelIters, 148560ul);
            break;
        }
        case  8:
        {
            EXPECT_EQ(numSteps,         1828ul);
            EXPECT_EQ(sumRegPosIters,  36540ul);
            EXPECT_EQ(sumRegVelIters,  47173ul);
            EXPECT_EQ(sumToiPosIters,  44005ul);
            EXPECT_EQ(sumToiVelIters, 114427ul);

            // From commit 0b049bd28d1bbb01d1750ec1fc9498105f13d192 onward:
            //EXPECT_EQ(numSteps,         1828ul);
            //EXPECT_EQ(sumRegPosIters,  36540ul);
            //EXPECT_EQ(sumRegVelIters,  47173ul);
            //EXPECT_EQ(sumToiPosIters,  44005ul);
            //EXPECT_EQ(sumToiVelIters, 114490ul);
            
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            //EXPECT_EQ(numSteps,         1807ul);
            //EXPECT_EQ(sumRegPosIters,  36584ul);
            //EXPECT_EQ(sumRegVelIters,  47380ul);
            //EXPECT_EQ(sumToiPosIters,  44552ul);
            //EXPECT_EQ(sumToiVelIters, 115392ul);

            // From commit 04f9188c47961cafe76c55eb6b766a608593ee08 onward.
            //EXPECT_EQ(numSteps, 1808ul);
            //EXPECT_EQ(sumRegPosIters, 36684ul);
            //EXPECT_EQ(sumRegVelIters, 48087ul);
            //EXPECT_EQ(sumToiPosIters, 45116ul);
            //EXPECT_EQ(sumToiVelIters, 118984ul);

            //EXPECT_EQ(numSteps, 1808ul);
            //EXPECT_EQ(sumRegPosIters, 36684ul);
            //EXPECT_EQ(sumRegVelIters, 264856ul);
            //EXPECT_EQ(sumToiPosIters, 45116ul);
            //EXPECT_EQ(sumToiVelIters, 149392ul);
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
            // From commit 0b049bd28d1bbb01d1750ec1fc9498105f13d192 onward:
            EXPECT_EQ(numSteps,         1768ul);
            EXPECT_EQ(sumRegPosIters,  36419ul);
            EXPECT_EQ(sumRegVelIters,  46684ul);
            EXPECT_EQ(sumToiPosIters,  43814ul);
            EXPECT_EQ(sumToiVelIters, 113452ul);

            // From commit 0b049bd28d1bbb01d1750ec1fc9498105f13d192 onward:
            //EXPECT_EQ(numSteps,         1799ul);
            //EXPECT_EQ(sumRegPosIters,  36515ul);
            //EXPECT_EQ(sumRegVelIters,  46964ul);
            //EXPECT_EQ(sumToiPosIters,  43999ul);
            //EXPECT_EQ(sumToiVelIters, 113153ul);
            
            // From commit ee74290c17422ccbd6a73f07d6fd9abe960da84a onward:
            //EXPECT_EQ(numSteps,         1803ul);
            //EXPECT_EQ(sumRegPosIters,  36673ul);
            //EXPECT_EQ(sumRegVelIters,  48148ul);
            //EXPECT_EQ(sumToiPosIters,  43959ul);
            //EXPECT_EQ(sumToiVelIters, 113189ul);

            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            //EXPECT_EQ(numSteps,         1803ul);
            //EXPECT_EQ(sumRegPosIters,  36528ul);
            //EXPECT_EQ(sumRegVelIters,  46988ul);
            //EXPECT_EQ(sumToiPosIters,  44178ul);
            //EXPECT_EQ(sumToiVelIters, 114936ul);

            // From commit 04f9188c47961cafe76c55eb6b766a608593ee08 onward.
            //EXPECT_EQ(numSteps, 1855ul);
            //EXPECT_EQ(sumRegPosIters, 36737ul);
            //EXPECT_EQ(sumRegVelIters, 47759ul);
            //EXPECT_EQ(sumToiPosIters, 44698ul);
            //EXPECT_EQ(sumToiVelIters, 114840ul);

            break;
        }
        case  8:
        {
            // From commit 0b049bd28d1bbb01d1750ec1fc9498105f13d192 onward:
            EXPECT_EQ(numSteps,         1828ul);
            EXPECT_EQ(sumRegPosIters,  36540ul);
            EXPECT_EQ(sumRegVelIters,  47173ul);
            EXPECT_EQ(sumToiPosIters,  44005ul);
            EXPECT_EQ(sumToiVelIters, 114462ul);

            // From commit 0b049bd28d1bbb01d1750ec1fc9498105f13d192 onward:
            //EXPECT_EQ(numSteps,         1828ul);
            //EXPECT_EQ(sumRegPosIters,  36540ul);
            //EXPECT_EQ(sumRegVelIters,  47173ul);
            //EXPECT_EQ(sumToiPosIters,  44005ul);
            //EXPECT_EQ(sumToiVelIters, 114259ul);
            
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            //EXPECT_EQ(numSteps,         1807ul);
            //EXPECT_EQ(sumRegPosIters,  36584ul);
            //EXPECT_EQ(sumRegVelIters,  47380ul);
            //EXPECT_EQ(sumToiPosIters,  44552ul);
            //EXPECT_EQ(sumToiVelIters, 115406ul);
            
            // From commit 04f9188c47961cafe76c55eb6b766a608593ee08 onward.
            //EXPECT_EQ(numSteps, 1808ul);
            //EXPECT_EQ(sumRegPosIters, 36684ul);
            //EXPECT_EQ(sumRegVelIters, 48087ul);
            //EXPECT_EQ(sumToiPosIters, 45116ul);
            //EXPECT_EQ(sumToiVelIters, 118830ul);

            break;
        }
    }

    // From commit d361c51d6aca13079e9d44b701715e62cec18a63 onward.
    //EXPECT_EQ(numSteps, 1855ul);
    //EXPECT_EQ(sumRegPosIters, 36737ul);
    //EXPECT_EQ(sumRegVelIters, 264528ul);
    //EXPECT_EQ(sumToiPosIters, 44698ul);
    //EXPECT_EQ(sumToiVelIters, 147544ul);
    
    // Pre commit d361c51d6aca13079e9d44b701715e62cec18a63
    //EXPECT_EQ(numSteps, 1822ul);
    //EXPECT_EQ(sumRegPosIters, 36616ul);
    //EXPECT_EQ(sumRegVelIters, 264096ul);
    //EXPECT_EQ(sumToiPosIters, 44415ul);
    //EXPECT_EQ(sumToiVelIters, 146800ul);
#elif defined(_WIN64) // This is likely wrong as the results are more likely arch dependent
    EXPECT_EQ(numSteps, 1800ul);
    EXPECT_EQ(sumRegPosIters, 36516ul);
    EXPECT_EQ(sumRegVelIters, 46948ul);
    EXPECT_EQ(sumToiPosIters, 43970ul);
    EXPECT_EQ(sumToiVelIters, 112904ul);
#elif defined(_WIN32)
    EXPECT_EQ(numSteps, 1793ul);
    EXPECT_EQ(sumRegPosIters, 36495ul);
    EXPECT_EQ(sumRegVelIters, 46884ul);
    EXPECT_EQ(sumToiPosIters, 43982ul);
    EXPECT_EQ(sumToiVelIters, 112969ul);
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
    PLAYRHO_CONSTEXPR const auto LinearSlop = playrho::Meter / playrho::Real(1000);
    PLAYRHO_CONSTEXPR const auto AngularSlop = (Pi * Real{2} * 1_rad) / Real{180};
    PLAYRHO_CONSTEXPR const auto VertexRadius = playrho::Length{LinearSlop * playrho::Real(2)};
    
    World world{WorldConf{}.UseMinVertexRadius(VertexRadius)};

    MyContactListener listener{
        [](Contact&, const Manifold&) {},
        [](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };
    world.SetContactListener(&listener);

    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    const auto left_edge_x = -0.1_m;
    const auto right_edge_x = +0.1_m;

    const auto edgeConf = EdgeShapeConf{}
        .UseVertexRadius(VertexRadius)
        .UseRestitution(Real(1))
        .Set(Length2{0_m, +10_m}, Length2{0_m, -10_m});
    const auto edge_shape = Shape(edgeConf);

    BodyConf body_def;
    body_def.type = BodyType::Static;

    body_def.location = Length2{left_edge_x, 0_m};
    const auto left_wall_body = world.CreateBody(body_def);
    ASSERT_NE(left_wall_body, nullptr);
    {
        const auto wall_fixture = left_wall_body->CreateFixture(edge_shape);
        ASSERT_NE(wall_fixture, nullptr);
    }

    body_def.location = Length2{right_edge_x, 0_m};
    const auto right_wall_body = world.CreateBody(body_def);
    ASSERT_NE(right_wall_body, nullptr);
    {
        const auto wall_fixture = right_wall_body->CreateFixture(edge_shape);
        ASSERT_NE(wall_fixture, nullptr);
    }
    
    const auto begin_x = Real(0);

    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{begin_x * Meter, 0_m};
    body_def.bullet = false;
    const auto ball_body = world.CreateBody(body_def);
    ASSERT_NE(ball_body, nullptr);
    
    const auto ball_radius = 0.01_m;
    const auto circle_shape = Shape(DiskShapeConf{}.UseDensity(1_kgpm2).UseRestitution(Real(1)).UseRadius(ball_radius));
    const auto ball_fixture = ball_body->CreateFixture(circle_shape);
    ASSERT_NE(ball_fixture, nullptr);

    const auto velocity = LinearVelocity2{+1_mps, 0_mps};
    ball_body->SetVelocity(Velocity{velocity, 0_deg / 1_s});

    const auto time_inc = .01_s;
    auto step = StepConf{};
    step.SetTime(time_inc);
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

    EXPECT_GT(GetX(ball_body->GetLocation()) / Meter, begin_x);

    EXPECT_EQ(GetLinearVelocity(*ball_body), velocity);
    
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

            EXPECT_LT(GetX(ball_body->GetLocation()), right_edge_x - (ball_radius/Real{2}));
            EXPECT_GT(GetX(ball_body->GetLocation()), left_edge_x + (ball_radius/Real{2}));

            if (GetX(ball_body->GetVelocity().linear) >= max_velocity)
            {
                return;
            }

            if (listener.begin_contacts % 2 != 0) // direction switched
            {
                EXPECT_LT(GetX(ball_body->GetVelocity().linear), 0_mps);
                break; // going left now
            }
            else if (listener.begin_contacts > last_contact_count)
            {
                ++increments;
                ball_body->SetVelocity(Velocity{
                    LinearVelocity2{
                        static_cast<Real>(increments) * GetX(velocity),
                        GetY(ball_body->GetVelocity().linear)
                    }, ball_body->GetVelocity().angular});
            }
            else
            {
                EXPECT_TRUE(AlmostEqual(Real{GetX(ball_body->GetVelocity().linear) / 1_mps},
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
            
            EXPECT_LT(GetX(ball_body->GetLocation()), right_edge_x - (ball_radius/Real{2}));
            EXPECT_GT(GetX(ball_body->GetLocation()), left_edge_x + (ball_radius/Real{2}));

            if (GetX(ball_body->GetVelocity().linear) <= -max_velocity)
            {
                return;
            }

            if (listener.begin_contacts % 2 != 0) // direction switched
            {
                EXPECT_GT(GetX(ball_body->GetVelocity().linear), 0_mps);
                break; // going right now
            }
            else if (listener.begin_contacts > last_contact_count)
            {
                ++increments;
                ball_body->SetVelocity(Velocity{
                    LinearVelocity2{
                        -static_cast<Real>(increments) * GetX(velocity),
                        GetY(ball_body->GetVelocity().linear)
                    }, ball_body->GetVelocity().angular});
            }
            else
            {
                EXPECT_TRUE(AlmostEqual(Real{GetX(ball_body->GetVelocity().linear) / 1_mps},
                                        Real{-static_cast<Real>(increments) * GetX(velocity) / 1_mps}));
            }
        }
        
        ++increments;
        ball_body->SetVelocity(Velocity{
            LinearVelocity2{
                static_cast<Real>(increments) * GetX(velocity),
                GetY(ball_body->GetVelocity().linear)
            }, ball_body->GetVelocity().angular});
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
        const auto left_wall_body = world.CreateBody(body_def);
        ASSERT_NE(left_wall_body, nullptr);
        {
            const auto wall_fixture = left_wall_body->CreateFixture(Shape(edgeConf));
            ASSERT_NE(wall_fixture, nullptr);
        }
        Include(container_aabb, ComputeAABB(*left_wall_body));
    }
    
    body_def.location = Length2{right_edge_x * Meter, 0_m};
    {
        const auto right_wall_body = world.CreateBody(body_def);
        ASSERT_NE(right_wall_body, nullptr);
        {
            const auto wall_fixture = right_wall_body->CreateFixture(Shape(edgeConf));
            ASSERT_NE(wall_fixture, nullptr);
        }
        Include(container_aabb, ComputeAABB(*right_wall_body));
    }

    // Setup horizontal bounderies
    edgeConf.Set(Length2{-half_box_width * 2_m, 0_m}, Length2{+half_box_width * 2_m, 0_m});
    
    body_def.location = Length2{0, btm_edge_y * Meter};
    {
        const auto btm_wall_body = world.CreateBody(body_def);
        ASSERT_NE(btm_wall_body, nullptr);
        {
            const auto wall_fixture = btm_wall_body->CreateFixture(Shape(edgeConf));
            ASSERT_NE(wall_fixture, nullptr);
        }
        Include(container_aabb, ComputeAABB(*btm_wall_body));
    }
    
    body_def.location = Length2{0, top_edge_y * Meter};
    {
        const auto top_wall_body = world.CreateBody(body_def);
        ASSERT_NE(top_wall_body, nullptr);
        {
            const auto wall_fixture = top_wall_body->CreateFixture(Shape(edgeConf));
            ASSERT_NE(wall_fixture, nullptr);
        }
        Include(container_aabb, ComputeAABB(*top_wall_body));
    }

    body_def.type = BodyType::Dynamic;
    body_def.location = Length2{};
    body_def.bullet = true;
    
    const auto ball_body = world.CreateBody(body_def);
    ASSERT_NE(ball_body, nullptr);
    ASSERT_EQ(GetX(ball_body->GetLocation()), 0_m);
    ASSERT_EQ(GetY(ball_body->GetLocation()), 0_m);
    
    const auto ball_radius = Real(half_box_width / 4) * Meter;
    const auto object_shape = Shape(PolygonShapeConf{}.UseDensity(10_kgpm2).SetAsBox(ball_radius, ball_radius));
    {
        const auto ball_fixture = ball_body->CreateFixture(object_shape);
        ASSERT_NE(ball_fixture, nullptr);
    }

    constexpr const unsigned numBodies = 1;
    Length2 last_opos[numBodies];
    Body *bodies[numBodies];
    for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
    {
        const auto angle = i * 2 * Pi / numBodies;
        const auto x = ball_radius * Real(2.1) * cos(angle);
        const auto y = ball_radius * Real(2.1) * sin(angle);
        body_def.location = Length2{x, y};
        bodies[i] = world.CreateBody(body_def);
        ASSERT_NE(bodies[i], nullptr);
        ASSERT_EQ(GetX(bodies[i]->GetLocation()), x);
        ASSERT_EQ(GetY(bodies[i]->GetLocation()), y);
        last_opos[i] = bodies[i]->GetLocation();
        {
            const auto fixture = bodies[i]->CreateFixture(object_shape);
            ASSERT_NE(fixture, nullptr);
        }
    }

    BodyConf bodyConf;
    const auto spare_body = world.CreateBody(bodyConf);

    const auto target_joint = [&]() {
        TargetJointConf mjd;
        mjd.bodyA = spare_body;
        mjd.bodyB = ball_body;
        const auto ball_body_pos = ball_body->GetLocation();
        mjd.target = Length2{
            GetX(ball_body_pos) - ball_radius / Real{2},
            GetY(ball_body_pos) + ball_radius / Real{2}
        };
        mjd.maxForce = Real(1000) * GetMass(*ball_body) * MeterPerSquareSecond;
        return static_cast<TargetJoint*>(world.CreateJoint(mjd));
    }();
    ASSERT_NE(target_joint, nullptr);

    ball_body->SetAwake();

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

    MyContactListener listener{
        [&](Contact& contact, const Manifold& old_manifold)
        {
            // PreSolve...
            const auto new_manifold = contact.GetManifold();
            const auto pointStates = GetPointStates(old_manifold, new_manifold);
            const auto oldPointCount = old_manifold.GetPointCount();
            switch (oldPointCount)
            {
                case 0:
                    ASSERT_EQ(pointStates.state1[0], PointState::NullState);
                    ASSERT_EQ(pointStates.state1[1], PointState::NullState);
                    break;
                case 1:
                    ASSERT_NE(pointStates.state1[0], PointState::NullState);
                    ASSERT_EQ(pointStates.state1[1], PointState::NullState);
                    break;
                case 2:
                    ASSERT_NE(pointStates.state1[0], PointState::NullState);
                    ASSERT_NE(pointStates.state1[1], PointState::NullState);
                    break;
                default:
                    ASSERT_LE(oldPointCount, 2);
                    break;
            }
            const auto newPointCount = new_manifold.GetPointCount();
            switch (newPointCount)
            {
                case 0:
                    ASSERT_EQ(pointStates.state2[0], PointState::NullState);
                    ASSERT_EQ(pointStates.state2[1], PointState::NullState);
                    break;
                case 1:
                    ASSERT_NE(pointStates.state2[0], PointState::NullState);
                    ASSERT_EQ(pointStates.state2[1], PointState::NullState);
                    break;
                case 2:
                    ASSERT_NE(pointStates.state2[0], PointState::NullState);
                    ASSERT_NE(pointStates.state2[1], PointState::NullState);
                    break;
                default:
                    ASSERT_LE(newPointCount, 2);
                    break;
            }
            ASSERT_THROW(world.Destroy(target_joint), WrongState);
        },
        [&](Contact& contact, const ContactImpulsesList& impulse, ContactListener::iteration_type solved)
        {
            const auto fA = contact.GetFixtureA();
            const auto fB = contact.GetFixtureB();

            ASSERT_NE(fA, nullptr);
            ASSERT_NE(fB, nullptr);

            const auto body_a = fA->GetBody();
            const auto body_b = fB->GetBody();

            ASSERT_NE(body_a, nullptr);
            ASSERT_NE(body_b, nullptr);

            auto fail_count = unsigned{0};
            for (auto&& body: {body_a, body_b})
            {
                if (!body->IsSpeedable())
                {
                    continue;
                }
                const auto bpos = body->GetLocation();
                const auto lt = Length2{right_edge_x * Meter, top_edge_y * Meter} - bpos;
                const auto gt = bpos - Length2{left_edge_x * Meter, btm_edge_y * Meter};
                
                if (GetX(lt) <= 0_m || GetY(lt) <= 0_m || GetX(gt) <= 0_m || GetY(gt) <= 0_m)
                {
                    if (!TestOverlap(container_aabb, ComputeAABB(*body)))
                    {
                        // Body out of bounds and no longer even overlapping container!
                        EXPECT_LT(GetX(body->GetLocation()), right_edge_x * Meter);
                        EXPECT_LT(GetY(body->GetLocation()), top_edge_y * Meter);
                        EXPECT_GT(GetX(body->GetLocation()), left_edge_x * Meter);
                        EXPECT_GT(GetY(body->GetLocation()), btm_edge_y * Meter);
                        ++fail_count;
                    }
                }
            }
            if (fail_count > 0)
            {
                std::cout << " angl=" << angle;
                std::cout << " ctoi=" << 0 + contact.GetToiCount();
                std::cout << " solv=" << 0 + solved;
                std::cout << " targ=(" << distance * cos(angle) << "," << distance * sin(angle) << ")";
                std::cout << " maxv=" << max_velocity;
                std::cout << " rang=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
                std::cout << " bpos=(" << GetX(ball_body->GetLocation()) << "," << GetY(ball_body->GetLocation()) << ")";
                std::cout << std::endl;
                for (auto i = decltype(impulse.GetCount()){0}; i < impulse.GetCount(); ++i)
                {
                    std::cout << " i#" << (0 + i) << "={n" << impulse.GetEntryNormal(i) << ",t" << impulse.GetEntryTanget(i) << "}";
                }
                std::cout << std::endl;

                std::cout << " bodyA=(" << GetX(body_a->GetLocation()) << "," << GetY(body_a->GetLocation()) << ")";
                if (body_a == ball_body) std::cout << " ball";
                if (!body_a->IsSpeedable()) std::cout << " wall";
                std::cout << " " << body_a;
                std::cout << std::endl;
                std::cout << " bodyB=(" << GetX(body_b->GetLocation()) << "," << GetY(body_b->GetLocation()) << ")";
                if (body_b == ball_body) std::cout << " ball";
                if (!body_b->IsSpeedable()) std::cout << " wall";
                std::cout << " " << body_b;
                std::cout << std::endl;

                //GTEST_FATAL_FAILURE_("");                
            }
        },
        [&](Contact& contact) {
            const auto fA = contact.GetFixtureA();
            const auto fB = contact.GetFixtureB();
            const auto body_a = fA->GetBody();
            const auto body_b = fB->GetBody();

            auto escaped = false;
            for (auto&& body: {body_a, body_b})
            {
                if (!body->IsSpeedable())
                {
                    continue;
                }

                if (GetX(body->GetLocation()) >= right_edge_x * Meter)
                {
                    escaped = true;
                }
                if (GetY(body->GetLocation()) >= top_edge_y * Meter)
                {
                    escaped = true;
                }
                if (GetX(body->GetLocation()) <= left_edge_x * Meter)
                {
                    escaped = true;
                }
                if (GetY(body->GetLocation()) <= btm_edge_y * Meter)
                {
                    escaped = true;                    
                }
            }
            if (escaped && !contact.IsTouching())
            {
                std::cout << "Escaped at EndContact[" << &contact << "]:";
                std::cout << " toiSteps=" << static_cast<unsigned>(contact.GetToiCount());
                std::cout << " toiValid=" << contact.HasValidToi();
                std::cout << " a[" << body_a << "]@(" << GetX(body_a->GetLocation()) << "," << GetY(body_a->GetLocation()) << ")";
                std::cout << " b[" << body_b << "]@(" << GetX(body_b->GetLocation()) << "," << GetY(body_b->GetLocation()) << ")";
                std::cout << std::endl;
                //exit(1);
            }
        },
    };
    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    world.SetContactListener(&listener);
    
    for (auto outer = unsigned{0}; outer < 2000; ++outer)
    {
        auto last_pos = ball_body->GetLocation();
        for (auto loops = unsigned{0};; ++loops)
        {
            target_joint->SetTarget(Length2{distance * cos(angle) * Meter, distance * sin(angle) * Meter});
            angle += anglular_speed;
            distance += distance_speed;

            Step(world, 1_s * time_inc, 8, 3);
            
            ASSERT_LT(GetX(ball_body->GetLocation()), right_edge_x * Meter);
            ASSERT_LT(GetY(ball_body->GetLocation()), top_edge_y * Meter);
            ASSERT_GT(GetX(ball_body->GetLocation()), left_edge_x * Meter);
            ASSERT_GT(GetY(ball_body->GetLocation()), btm_edge_y * Meter);
            for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
            {
                ASSERT_LT(GetX(bodies[i]->GetLocation()), right_edge_x * Meter);
                ASSERT_LT(GetY(bodies[i]->GetLocation()), top_edge_y * Meter);
                ASSERT_GT(GetX(bodies[i]->GetLocation()), left_edge_x * Meter);
                ASSERT_GT(GetY(bodies[i]->GetLocation()), btm_edge_y * Meter);
            }

            max_x = std::max(Real{GetX(ball_body->GetLocation()) / Meter}, max_x);
            min_x = std::min(Real{GetX(ball_body->GetLocation()) / Meter}, min_x);

            max_y = std::max(Real{GetY(ball_body->GetLocation()) / Meter}, max_y);
            min_y = std::min(Real{GetY(ball_body->GetLocation()) / Meter}, min_y);

            const auto linVel = ball_body->GetVelocity().linear;
            max_velocity = std::max(GetMagnitude(GetVec2(linVel)), max_velocity);

            if (loops > 50)
            {
                if (GetX(target_joint->GetTarget()) < 0_m)
                {
                    if (GetX(ball_body->GetLocation()) >= GetX(last_pos))
                        break;                    
                }
                else
                {
                    if (GetX(ball_body->GetLocation()) <= GetX(last_pos))
                        break;
                }
                if (GetY(target_joint->GetTarget()) < 0_m)
                {
                    if (GetY(ball_body->GetLocation()) >= GetY(last_pos))
                        break;
                }
                else
                {
                    if (GetY(ball_body->GetLocation()) <= GetY(last_pos))
                        break;
                }
            }
            last_pos = ball_body->GetLocation();
        }
        anglular_speed *= anglular_accel;
        distance_speed *= distance_accel;

        ASSERT_NE(ball_body->GetLocation(), (Length2{}));
#if 0
        if (outer > 100)
        {
            for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
            {
                // a sanity check to ensure the other bodies are getting moved
                EXPECT_NE(last_opos[i], bodies[i]->GetLocation());
                last_opos[i] = bodies[i]->GetLocation();
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

    const auto target0 = target_joint->GetTarget();
    const auto shift = Length2{2_m, 2_m};
    world.ShiftOrigin(shift);
    const auto target1 = target_joint->GetTarget();
    EXPECT_EQ(target0 - shift, target1);
}

#if 0
static void smaller_still_conserves_momentum(bool bullet, Real multiplier, Real time_inc)
{
    const auto radius = Real(1);
    const auto start_distance = Real(10);
    
    auto scale = Real(1);
    for (;;)
    {
        const auto gravity = Vec2{};
        World world{WorldConf{}.UseGravity(gravity)};
        ASSERT_EQ(GetX(world.GetGravity()), 0);
        ASSERT_EQ(GetY(world.GetGravity()), 0);

        auto maxNormalImpulse = Real(0);
        auto maxTangentImpulse = Real(0);
        auto maxPoints = 0u;
        auto numSteps = 0u;
        auto failed = false;
        auto preB1 = Vec2{};
        auto preB2 = Vec2{};
        
        MyContactListener listener{
            [&](Contact& contact, const Manifold&)
            {
                const auto fA = contact.GetFixtureA();
                const auto fB = contact.GetFixtureB();
                const auto bA = fA->GetBody();
                const auto bB = fB->GetBody();
                preB1 = bA->GetLocation();
                preB2 = bB->GetLocation();
            },
            [&](Contact& /* contact */, const ContactImpulsesList& impulse, ContactListener::iteration_type /* solved */)
            {
                {
                    const auto count = impulse.GetCount();
                    maxPoints = std::max(maxPoints, decltype(maxPoints){count});
                    for (auto i = decltype(count){0}; i < count; ++i)
                    {
                        maxNormalImpulse = std::max(maxNormalImpulse, impulse.GetEntryNormal(i));
                        maxTangentImpulse = std::max(maxTangentImpulse, impulse.GetEntryTanget(i));
                    }
                }
                if (maxNormalImpulse == 0 && maxTangentImpulse == 0)
                {
                    failed = true;
#if 0
                    const auto& manifold = contact.GetManifold();
                    std::cout << " solved=" << unsigned(solved);
                    std::cout << " numstp=" << numSteps;
                    std::cout << " type=" << unsigned(manifold.GetType());
                    std::cout << " lp.x=" << manifold.GetLocalPoint().x;
                    std::cout << " lp.y=" << manifold.GetLocalPoint().y;
                    const auto count = manifold.GetPointCount();
                    std::cout << " points=" << unsigned(count);
                    for (auto i = decltype(count){0}; i < count; ++i)
                    {
                        std::cout << " ni[" << unsigned(i) << "]=" << manifold.GetPoint(i).normalImpulse;
                        std::cout << " ti[" << unsigned(i) << "]=" << manifold.GetPoint(i).tangentImpulse;
                        std::cout << " lp[" << unsigned(i) << "].x=" << manifold.GetPoint(i).localPoint.x;
                        std::cout << " lp[" << unsigned(i) << "].y=" << manifold.GetPoint(i).localPoint.y;
                    }
                    std::cout << std::endl;
#endif
                }
            },
            [=](Contact&) {
            }
        };
        world.SetContactListener(&listener);

        const auto shape = DiskShapeConf{}.UseRadius(scale * radius * Meter);
        ASSERT_EQ(shape->GetRadius(), scale * radius);
        
        auto fixture_def = FixtureConf{}.UseDensity(1);
        fixture_def.friction = 0;
        fixture_def.restitution = 1;
        
        auto body_def = BodyConf{};
        body_def.type = BodyType::Dynamic;
        body_def.bullet = bullet;
        
        body_def.position = Vec2{+(scale * start_distance), 0};
        body_def.linearVelocity = Vec2{-start_distance, 0};
        const auto body_1 = world.CreateBody(body_def);
        ASSERT_EQ(body_1->GetLocation().x, body_def.position.x);
        ASSERT_EQ(body_1->GetLocation().y, body_def.position.y);
        ASSERT_EQ(GetLinearVelocity(*body_1).x, body_def.linearVelocity.x);
        ASSERT_EQ(GetLinearVelocity(*body_1).y, body_def.linearVelocity.y);
        body_1->CreateFixture(shape, fixture_def);
        
        body_def.position = Vec2{-(scale * start_distance), 0};
        body_def.linearVelocity = Vec2{+start_distance, 0};
        const auto body_2 = world.CreateBody(body_def);
        ASSERT_EQ(body_2->GetLocation().x, body_def.position.x);
        ASSERT_EQ(body_2->GetLocation().y, body_def.position.y);
        ASSERT_EQ(GetLinearVelocity(*body_2).x, body_def.linearVelocity.x);
        ASSERT_EQ(GetLinearVelocity(*body_2).y, body_def.linearVelocity.y);
        body_2->CreateFixture(shape, fixture_def);
        
        for (;;)
        {
            const auto relative_velocity = GetLinearVelocity(*body_1) - GetLinearVelocity(*body_2);
            if (relative_velocity.x >= 0)
            {
                EXPECT_NEAR(double(relative_velocity.x), double(abs(body_def.linearVelocity.x) * +2), 0.0001);
                break;
            }
            if (failed)
            {
                std::cout << " scale=" << scale;
                std::cout << " dist0=" << (scale * start_distance * 2);
                std::cout << " bcont=" << listener.begin_contacts;
                std::cout << " econt=" << listener.end_contacts;
                std::cout << " pre-#=" << listener.pre_solves;
                std::cout << " post#=" << listener.post_solves;
                std::cout << " normi=" << maxNormalImpulse;
                std::cout << " tangi=" << maxTangentImpulse;
                std::cout << " n-pts=" << maxPoints;
                std::cout << std::endl;
                std::cout << " pre1.x=" << preB1.x;
                std::cout << " pre2.x=" << preB2.x;
                std::cout << " pos1.x=" << body_1->GetLocation().x;
                std::cout << " pos2.x=" << body_2->GetLocation().x;
                std::cout << " preDel=" << (preB1.x - preB2.x);
                std::cout << " posDel=" << (body_1->GetLocation().x - body_2->GetLocation().x);
                std::cout << " travel=" << (body_1->GetLocation().x - preB1.x);
                std::cout << std::endl;
                ASSERT_FALSE(failed);
            }
            
            EXPECT_TRUE(AlmostEqual(relative_velocity.x, abs(body_def.linearVelocity.x) * -2));
            Step(world, time_inc);
            ++numSteps;
        }
        
        scale *= multiplier;
    }
}

TEST(World, SmallerStillConservesMomemtum)
{
    // smaller_still_conserves_momentum(false, Real(0.999), Real(0.01));
    // fails around scale=0.0899796 dist0=1.79959
    // goin to smaller time increment fails nearly same point.
    smaller_still_conserves_momentum(false, Real(0.999), Real(0.01));
}

TEST(World, SmallerBulletStillConservesMomemtum)
{
    // smaller_still_conserves_momentum(true, Real(0.999), Real(0.01))
    // fails around scale=4.99832e-05 dist0=0.000999664
    // goin to smaller time increment fails nearly same point.
// smaller_still_conserves_momentum(true, Real(0.999), Real(0.01));
}
#endif

class VerticalStackTest: public ::testing::TestWithParam<Real>
{
public:
    virtual void SetUp()
    {
        const auto hw_ground = 40.0_m;
        const auto ground = world.CreateBody();
        ground->CreateFixture(Shape{EdgeShapeConf{}.Set(Length2{-hw_ground, 0_m}, Length2{hw_ground, 0_m})});
        const auto numboxes = boxes.size();
        original_x = GetParam();
        
        const auto boxShape = Shape{PolygonShapeConf{}.UseDensity(1_kgpm2).UseFriction(Real(0.3f)).SetAsBox(hdim, hdim)};
        for (auto i = decltype(numboxes){0}; i < numboxes; ++i)
        {
            // (hdim + 0.05f) + (hdim * 2 + 0.1f) * i
            const auto location = Length2{original_x * Meter, (i + Real{1}) * hdim * Real{4}};
            const auto box = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location));
            box->CreateFixture(boxShape);
            boxes[i] = box;
        }
        
        SetAccelerations(world, Acceleration{LinearAcceleration2{
            Real(0) * MeterPerSquareSecond, -Real(10) * MeterPerSquareSecond
        }, 0 * RadianPerSquareSecond});
        const auto stepConf = StepConf{}.SetTime(1_s / 60);
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
    std::vector<Body*> boxes{10};
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
        EXPECT_EQ(GetX(box->GetLocation()), original_x * Meter);
    }
}

TEST_P(VerticalStackTest, EachBoxAboveLast)
{
    auto lasty = 0_m;
    for (auto&& box: boxes)
    {
        EXPECT_GT(GetY(box->GetLocation()), lasty + hdim);
        lasty = GetY(box->GetLocation());
    }
}

TEST_P(VerticalStackTest, EachBodyLevel)
{
    for (auto&& box: boxes)
    {
        EXPECT_EQ(box->GetAngle(), 0_deg);
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
