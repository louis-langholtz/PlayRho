/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Collision.hpp>
#include <Box2D/Dynamics/Joints/MouseJoint.hpp>
#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <chrono>
#include <type_traits>

using namespace box2d;

TEST(World, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
        {
            // Size is OS dependent.
            // Seems linux containers are bigger in size...
#ifdef __APPLE__
            EXPECT_EQ(sizeof(World), std::size_t(368));
#endif
#ifdef __linux__
            EXPECT_EQ(sizeof(World), std::size_t(392));
#endif
            break;
        }
        case  8:
        {
#ifdef __APPLE__
            EXPECT_EQ(sizeof(World), std::size_t(392));
#endif
#ifdef __linux__
            EXPECT_EQ(sizeof(World), std::size_t(416));
#endif
            break;
        }
        case 16: EXPECT_EQ(sizeof(World), std::size_t(432)); break;
        default: FAIL(); break;
    }
}

TEST(World, Def)
{
    const auto worldDef = WorldDef{};
    const auto defaultDef = GetDefaultWorldDef();
    
    EXPECT_EQ(defaultDef.gravity, worldDef.gravity);
    EXPECT_EQ(defaultDef.maxVertexRadius, worldDef.maxVertexRadius);
    EXPECT_EQ(defaultDef.minVertexRadius, worldDef.minVertexRadius);
    const auto stepConf = StepConf{};

    const auto v = Real(1);
    const auto n = std::nextafter(v, Real(0));
    const auto time_inc = (v - n) * Second;
    ASSERT_GT(time_inc, Real(0) * Second);
    ASSERT_LT(time_inc, Real(1) * Second);
    const auto max_inc = time_inc * stepConf.maxTranslation;
    EXPECT_GT(max_inc, Real(0) * Meter * Second);
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

TEST(World, DefaultInit)
{
    World world;

    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    EXPECT_EQ(world.GetProxyCount(), World::proxy_size_type(0));
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_EQ(GetContactCount(world), ContactCounter(0));
    EXPECT_EQ(world.GetTreeHeight(), World::proxy_size_type(0));
    EXPECT_EQ(world.GetTreeQuality(), Real(0));

    EXPECT_EQ(world.GetGravity(), EarthlyGravity);

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
    const auto gravity = LinearAcceleration2D{
        Real(-4.2) * MeterPerSquareSecond,
        Real(3.4) * MeterPerSquareSecond
    };
    World world{WorldDef{}.UseGravity(gravity)};
    EXPECT_EQ(world.GetGravity(), gravity);
    EXPECT_FALSE(world.IsLocked());
}

TEST(World, CopyConstruction)
{
    auto world = World{};

    {
        const auto copy = World{world};
        EXPECT_EQ(world.GetGravity(), copy.GetGravity());
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(world.GetTreeHeight(), copy.GetTreeHeight());
        EXPECT_EQ(world.GetProxyCount(), copy.GetProxyCount());
        EXPECT_EQ(world.GetTreeBalance(), copy.GetTreeBalance());
    }
    
    const auto shape = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                   .UseDensity(Real(1) * KilogramPerSquareMeter)
                                                   .UseVertexRadius(Real(1) * Meter));
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    b1->CreateFixture(shape);
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    b2->CreateFixture(shape);

    world.CreateJoint(RevoluteJointDef{b1, b2, Length2D(0, 0)});
    world.CreateJoint(PrismaticJointDef{b1, b2, Length2D(0, 0), UnitVec2::GetRight()});
    world.CreateJoint(PulleyJointDef{b1, b2, Length2D(0, 0), Length2D(0, 0),
        Length2D(0, 0), Length2D(0, 0), Real(1)});
    
    auto stepConf = StepConf{};
    world.Step(stepConf);

    {
        const auto copy = World{world};
        EXPECT_EQ(world.GetGravity(), copy.GetGravity());
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        const auto minJoints = std::min(world.GetJoints().size(), copy.GetJoints().size());
        
        auto worldJointIter = world.GetJoints().begin();
        auto copyJointIter = copy.GetJoints().begin();
        for (auto i = decltype(minJoints){0}; i < minJoints; ++i)
        {
            EXPECT_EQ((*worldJointIter)->GetType(), (*copyJointIter)->GetType());
            ++worldJointIter;
            ++copyJointIter;
        }
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(world.GetTreeHeight(), copy.GetTreeHeight());
        EXPECT_EQ(world.GetProxyCount(), copy.GetProxyCount());
        EXPECT_EQ(world.GetTreeBalance(), copy.GetTreeBalance());
    }
}

TEST(World, CopyAssignment)
{
    auto world = World{};
    
    {
        auto copy = World{};
        copy = world;
        EXPECT_EQ(world.GetGravity(), copy.GetGravity());
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(world.GetTreeHeight(), copy.GetTreeHeight());
        EXPECT_EQ(world.GetProxyCount(), copy.GetProxyCount());
        EXPECT_EQ(world.GetTreeBalance(), copy.GetTreeBalance());
    }
    
    const auto shape = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                   .UseDensity(Real(1) * KilogramPerSquareMeter)
                                                   .UseVertexRadius(Real(1) * Meter));
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    b1->CreateFixture(shape);
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    b2->CreateFixture(shape);
    
    world.CreateJoint(RevoluteJointDef{b1, b2, Length2D(0, 0)});
    world.CreateJoint(PrismaticJointDef{b1, b2, Length2D(0, 0), UnitVec2::GetRight()});
    world.CreateJoint(PulleyJointDef{b1, b2, Length2D(0, 0), Length2D(0, 0),
        Length2D(0, 0), Length2D(0, 0), Real(1)});
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    
    {
        auto copy = World{};
        copy = world;
        EXPECT_EQ(world.GetGravity(), copy.GetGravity());
        EXPECT_EQ(world.GetMinVertexRadius(), copy.GetMinVertexRadius());
        EXPECT_EQ(world.GetMaxVertexRadius(), copy.GetMaxVertexRadius());
        EXPECT_EQ(world.GetJoints().size(), copy.GetJoints().size());
        const auto minJoints = std::min(world.GetJoints().size(), copy.GetJoints().size());
        
        auto worldJointIter = world.GetJoints().begin();
        auto copyJointIter = copy.GetJoints().begin();
        for (auto i = decltype(minJoints){0}; i < minJoints; ++i)
        {
            EXPECT_EQ((*worldJointIter)->GetType(), (*copyJointIter)->GetType());
            ++worldJointIter;
            ++copyJointIter;
        }
        EXPECT_EQ(world.GetBodies().size(), copy.GetBodies().size());
        EXPECT_EQ(world.GetContacts().size(), copy.GetContacts().size());
        EXPECT_EQ(world.GetTreeHeight(), copy.GetTreeHeight());
        EXPECT_EQ(world.GetProxyCount(), copy.GetProxyCount());
        EXPECT_EQ(world.GetTreeBalance(), copy.GetTreeBalance());
    }
}

TEST(World, SetGravity)
{
    const auto gravity = LinearAcceleration2D{
        Real(-4.2) * MeterPerSquareSecond,
        Real(3.4) * MeterPerSquareSecond
    };
    World world;
    EXPECT_NE(world.GetGravity(), gravity);
    world.SetGravity(gravity);
    EXPECT_EQ(world.GetGravity(), gravity);    
    world.SetGravity(-gravity);
    EXPECT_NE(world.GetGravity(), gravity);
}

TEST(World, CreateAndDestroyBody)
{
    World world;
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));

    const auto body = world.CreateBody();
    EXPECT_NE(body, nullptr);
    EXPECT_EQ(body->GetType(), BodyType::Static);
    EXPECT_FALSE(body->IsSpeedable());
    EXPECT_FALSE(body->IsAccelerable());
    EXPECT_TRUE(body->IsImpenetrable());

    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    const auto& bodies1 = world.GetBodies();
    EXPECT_FALSE(bodies1.empty());
    EXPECT_EQ(bodies1.size(), BodyCounter(1));
    EXPECT_NE(bodies1.begin(), bodies1.end());
    const auto& first = *(bodies1.begin());
    EXPECT_EQ(body, &first);

    world.Destroy(body);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    const auto& bodies0 = world.GetBodies();
    EXPECT_TRUE(bodies0.empty());
    EXPECT_EQ(bodies0.size(), BodyCounter(0));
    EXPECT_EQ(bodies0.begin(), bodies0.end());
}

TEST(World, ClearForcesFreeFunction)
{
    World world;
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body->IsSpeedable());
    ASSERT_TRUE(body->IsAccelerable());
    ASSERT_FALSE(body->IsImpenetrable());
    ASSERT_EQ(body->GetLinearAcceleration().x, world.GetGravity().x);
    ASSERT_EQ(body->GetLinearAcceleration().y, world.GetGravity().y);
    
    const auto v1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = Real{1} * Meter;
    conf.density = Real{1} * KilogramPerSquareMeter;
    const auto shape = std::make_shared<EdgeShape>(v1, v2, conf);
    const auto fixture = body->CreateFixture(shape);
    ASSERT_NE(fixture, nullptr);

    ApplyForceToCenter(*body, Force2D(Real(2) * Newton, Real(4) * Newton));
    ASSERT_NE(body->GetLinearAcceleration().x, world.GetGravity().x);
    ASSERT_NE(body->GetLinearAcceleration().y, world.GetGravity().y);
    
    ClearForces(world);
    EXPECT_EQ(body->GetLinearAcceleration().x, world.GetGravity().x);
    EXPECT_EQ(body->GetLinearAcceleration().y, world.GetGravity().y);
}

TEST(World, GetShapeCountFreeFunction)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetShapeCount(world), std::size_t(0));
    
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    
    const auto shapeConf = EdgeShape::Conf{}
        .UseVertexRadius(Real{1} * Meter)
        .UseDensity(Real{1} * KilogramPerSquareMeter);
    const auto v1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(1) * Meter, Real(0) * Meter};

    const auto shape1 = std::make_shared<EdgeShape>(v1, v2, shapeConf);
    
    const auto fixture1 = body->CreateFixture(shape1);
    ASSERT_NE(fixture1, nullptr);
    EXPECT_EQ(GetShapeCount(world), std::size_t(1));

    const auto fixture2 = body->CreateFixture(shape1);
    ASSERT_NE(fixture2, nullptr);
    EXPECT_EQ(GetShapeCount(world), std::size_t(1));
    
    const auto shape2 = std::make_shared<EdgeShape>(v1, v2, shapeConf);
    
    const auto fixture3 = body->CreateFixture(shape2);
    ASSERT_NE(fixture3, nullptr);
    EXPECT_EQ(GetShapeCount(world), std::size_t(2));
}

TEST(World, GetFixtureCountFreeFunction)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    ASSERT_EQ(GetFixtureCount(world), std::size_t(0));
    
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    
    const auto shapeConf = EdgeShape::Conf{}
        .UseVertexRadius(Real{1} * Meter)
        .UseDensity(Real{1} * KilogramPerSquareMeter);
    const auto v1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    
    const auto shape = std::make_shared<EdgeShape>(v1, v2, shapeConf);
    
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
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    ASSERT_EQ(GetBodyCount(world), BodyCounter(0));
    
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    ASSERT_TRUE(body->IsSpeedable());
    ASSERT_TRUE(body->IsAccelerable());
    ASSERT_FALSE(body->IsImpenetrable());
    ASSERT_EQ(body->GetLinearAcceleration().x, Real(0) * MeterPerSquareSecond);
    ASSERT_EQ(body->GetLinearAcceleration().y, Real(0) * MeterPerSquareSecond);
    
    const auto v1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto shape = std::make_shared<EdgeShape>(v1, v2,
                                                   EdgeShape::Conf{}
                                                   .UseVertexRadius(Real{1} * Meter)
                                                   .UseDensity(Real{1} * KilogramPerSquareMeter));
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

TEST(World, DynamicEdgeBodyHasCorrectMass)
{
    World world;
    
    auto bodyDef = BodyDef{};
    bodyDef.type = BodyType::Dynamic;
    const auto body = world.CreateBody(bodyDef);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    
    const auto v1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = Real{1} * Meter;
    const auto shape = std::make_shared<EdgeShape>(v1, v2, conf);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    ASSERT_EQ(shape->GetVertexRadius(), Real(1) * Meter);

    const auto fixture = body->CreateFixture(shape);
    ASSERT_NE(fixture, nullptr);
    ASSERT_EQ(fixture->GetDensity(), Real{1} * KilogramPerSquareMeter);

    const auto circleMass = Mass{fixture->GetDensity() * (Pi * Square(shape->GetVertexRadius()))};
    const auto rectMass = Mass{fixture->GetDensity() * (shape->GetVertexRadius() * Real{2} * GetLength(v2 - v1))};
    const auto totalMass = Mass{circleMass + rectMass};
    
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    EXPECT_EQ(body->GetInvMass(), Real(1) / totalMass);

    ASSERT_NE(fixture->GetShape(), nullptr);
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
    
    const auto anchorA = Length2D{Real(+0.4) * Meter, Real(-1.2) * Meter};
    const auto anchorB = Length2D{Real(-2.3) * Meter, Real(+0.7) * Meter};
    const auto joint = world.CreateJoint(DistanceJointDef{body1, body2, anchorA, anchorB});
    EXPECT_EQ(GetJointCount(world), JointCounter(1));
    EXPECT_FALSE(world.GetJoints().empty());
    EXPECT_NE(world.GetJoints().begin(), world.GetJoints().end());
    const auto first = *world.GetJoints().begin();
    EXPECT_EQ(joint, first);
    EXPECT_EQ(joint->GetType(), JointType::Distance);
    EXPECT_EQ(joint->GetBodyA(), body1);
    EXPECT_EQ(joint->GetBodyB(), body2);
    EXPECT_EQ(joint->GetAnchorA(), anchorA);
    EXPECT_EQ(joint->GetAnchorB(), anchorB);
    EXPECT_FALSE(joint->GetCollideConnected());

    world.Destroy(joint);
    EXPECT_EQ(GetJointCount(world), JointCounter(0));
    EXPECT_TRUE(world.GetJoints().empty());
    EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
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
        const auto body = world.CreateBody();
        EXPECT_EQ(body, nullptr);        
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
        const auto joint = world.CreateJoint(RopeJointDef{body1, body2});
        ASSERT_NE(joint, nullptr);
    }
    {
        const auto joint = world.CreateJoint(RopeJointDef{body1, body2});
        EXPECT_EQ(joint, nullptr);
    }
}

TEST(World, StepZeroTimeDoesNothing)
{
    const auto gravity = EarthlyGravity;
    
    World world{WorldDef{}.UseGravity(gravity)};
    
    BodyDef def;
    def.position = Length2D{Real(31.9) * Meter, Real(-19.24) * Meter};
    def.type = BodyType::Dynamic;
    
    const auto body = world.CreateBody(def);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->GetLocation().x, def.position.x);
    EXPECT_EQ(body->GetLocation().y, def.position.y);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real(0) * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body).y, Real(0) * MeterPerSecond);
    EXPECT_EQ(body->GetLinearAcceleration().x, Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
    
    const auto time_inc = Time{Second * Real{0}};
    
    auto pos = body->GetLocation();
    auto vel = GetLinearVelocity(*body);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, time_inc);
        
        EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
        
        EXPECT_EQ(body->GetLocation().x, def.position.x);
        EXPECT_EQ(body->GetLocation().y, pos.y);
        pos = body->GetLocation();
        
        EXPECT_EQ(GetLinearVelocity(*body).x, Real(0) * MeterPerSecond);
        EXPECT_TRUE(almost_equal(Real{GetLinearVelocity(*body).y / MeterPerSecond}, vel.y / MeterPerSecond));
        vel = GetLinearVelocity(*body);
    }
}

TEST(World, GravitationalBodyMovement)
{
    auto p0 = Length2D{Real(0) * Meter, Real(1) * Meter};

    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.position = p0;

    const auto a = Real(-10);
    const auto gravity = LinearAcceleration2D{0, a * MeterPerSquareSecond};
    const auto t = Real(.01) * Second;
    
    World world{WorldDef{}.UseGravity(gravity)};

    const auto body = world.CreateBody(body_def);
    ASSERT_NE(body, nullptr);
    EXPECT_FALSE(body->IsImpenetrable());
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real{0.0f} * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body).y, Real{0.0f} * MeterPerSecond);
    EXPECT_EQ(body->GetLocation().x, p0.x);
    EXPECT_EQ(body->GetLocation().y, p0.y);

    Step(world, t);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real{0.0f} * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * Real{1}) * MeterPerSquareSecond);
    EXPECT_EQ(body->GetLocation().x, p0.x);
    EXPECT_EQ(body->GetLocation().y, p0.y + GetLinearVelocity(*body).y * t);

    p0 = body->GetLocation();
    Step(world, t);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real{0.0f} * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * Real{2}) * MeterPerSquareSecond);
    EXPECT_EQ(body->GetLocation().x, p0.x);
    EXPECT_EQ(body->GetLocation().y, p0.y + GetLinearVelocity(*body).y * t);
    
    p0 = body->GetLocation();
    Step(world, t);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real{0.0f} * MeterPerSecond);
    EXPECT_NEAR(double(Real{GetLinearVelocity(*body).y / MeterPerSecond}),
                double(Real{a * (t * Real{3}) / Second}), 0.00001);
    EXPECT_EQ(body->GetLocation().x, p0.x);
    EXPECT_EQ(body->GetLocation().y, p0.y + GetLinearVelocity(*body).y * t);
}

TEST(World, BodyAccelPerSpecWithNoVelOrPosIterations)
{
    const auto gravity = EarthlyGravity;
    
    World world{WorldDef{}.UseGravity(gravity)};
    
    BodyDef def;
    def.position = Length2D{Real(31.9) * Meter, Real(-19.24) * Meter};
    def.type = BodyType::Dynamic;
    
    const auto body = world.CreateBody(def);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->GetLocation().x, def.position.x);
    EXPECT_EQ(body->GetLocation().y, def.position.y);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real(0) * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body).y, Real(0) * MeterPerSecond);
    EXPECT_EQ(body->GetLinearAcceleration().x, Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
    
    const auto time_inc = Real(0.01) * Second;
    
    auto pos = body->GetLocation();
    auto vel = GetLinearVelocity(*body);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, time_inc, 0, 0);
        
        EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
        
        EXPECT_EQ(body->GetLocation().x, def.position.x);
        EXPECT_LT(body->GetLocation().y, pos.y);
        EXPECT_EQ(body->GetLocation().y, pos.y + ((vel.y + gravity.y * time_inc) * time_inc));
        pos = body->GetLocation();
        
        EXPECT_EQ(GetLinearVelocity(*body).x, Real(0) * MeterPerSecond);
        EXPECT_LT(GetLinearVelocity(*body).y, vel.y);
        EXPECT_TRUE(almost_equal(GetLinearVelocity(*body).y / MeterPerSecond, (vel.y + gravity.y * time_inc) / MeterPerSecond));
        vel = GetLinearVelocity(*body);
    }
}


TEST(World, BodyAccelRevPerSpecWithNegativeTimeAndNoVelOrPosIterations)
{
    const auto gravity = EarthlyGravity;
    
    World world{WorldDef{}.UseGravity(gravity)};
    
    BodyDef def;
    def.position = Length2D{Real(31.9) * Meter, Real(-19.24) * Meter};
    def.linearVelocity = LinearVelocity2D{0, Real(-9.8) * MeterPerSecond};
    def.type = BodyType::Dynamic;
    
    const auto body = world.CreateBody(def);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->GetLocation().x, def.position.x);
    EXPECT_EQ(body->GetLocation().y, def.position.y);
    EXPECT_EQ(GetLinearVelocity(*body).x, Real(0) * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body).y, Real(-9.8) * MeterPerSecond);
    EXPECT_EQ(body->GetLinearAcceleration().x, Real{0.0f} * MeterPerSquareSecond);
    EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
    
    const auto time_inc = Real{-0.01f} * Second;
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
        
        EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
        
        EXPECT_EQ(body->GetLocation().x, def.position.x);
        EXPECT_GT(body->GetLocation().y, pos.y);
        EXPECT_EQ(body->GetLocation().y, pos.y + ((vel.y + gravity.y * time_inc) * time_inc));
        pos = body->GetLocation();
        
        EXPECT_EQ(GetLinearVelocity(*body).x, Real(0) * MeterPerSecond);
        EXPECT_GT(GetLinearVelocity(*body).y, vel.y);
        EXPECT_TRUE(almost_equal(GetLinearVelocity(*body).y / MeterPerSecond, (vel.y + gravity.y * time_inc) / MeterPerSecond));
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
        
        body_a[0] = contact.GetFixtureA()->GetBody()->GetLocation();
        body_b[0] = contact.GetFixtureB()->GetBody()->GetLocation();
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
    Length2D body_a[2] = {Length2D(0, 0), Length2D(0, 0)};
    Length2D body_b[2] = {Length2D(0, 0), Length2D(0, 0)};
    PreSolver presolver;
    PostSolver postsolver;
    Ender ender;
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

    const auto gravity = LinearAcceleration2D{0, 0};
    World world{WorldDef{}.UseGravity(gravity)};
    world.SetContactListener(&listener);
    
    ASSERT_EQ(listener.begin_contacts, unsigned(0));
    ASSERT_EQ(listener.end_contacts, unsigned(0));
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = true;
    
    const auto shape = std::make_shared<DiskShape>(Real{1} * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1));
    
    body_def.position = Length2D{-x * Meter, Real(0) * Meter};
    body_def.linearVelocity = LinearVelocity2D{+x * MeterPerSecond, 0};
    const auto body_a = world.CreateBody(body_def);
    ASSERT_NE(body_a, nullptr);
    EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body_a->IsSpeedable());
    EXPECT_TRUE(body_a->IsAccelerable());
    const auto fixture1 = body_a->CreateFixture(shape);
    ASSERT_NE(fixture1, nullptr);
    
    body_def.position = Length2D{+x * Meter, Real(0) * Meter};
    body_def.linearVelocity = LinearVelocity2D{-x * MeterPerSecond, 0};
    const auto body_b = world.CreateBody(body_def);
    ASSERT_NE(body_b, nullptr);
    const auto fixture2 = body_b->CreateFixture(shape);
    ASSERT_NE(fixture2, nullptr);
    EXPECT_EQ(body_b->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body_b->IsSpeedable());
    EXPECT_TRUE(body_b->IsAccelerable());

    EXPECT_EQ(GetLinearVelocity(*body_a).x, +x * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body_a).y, Real{0.0f} * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body_b).x, -x * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body_b).y, Real{0.0f} * MeterPerSecond);

    const auto time_inc = Real(.01) * Second;

    auto pos_a = body_a->GetLocation();
    auto pos_b = body_b->GetLocation();
    ASSERT_LT(pos_a.x, pos_b.x);

    auto conf = StepConf{};
    conf.SetTime(time_inc);
    conf.regPositionIterations = 0;
    conf.regVelocityIterations = 0;
    conf.toiPositionIterations = 0;
    conf.toiVelocityIterations = 0;
    conf.tolerance = std::nextafter(StripUnit(conf.targetDepth), Real{0}) * Meter;

    auto steps = unsigned{0};
    while (pos_a.x < (x * Meter) && pos_b.x > (-x * Meter))
    {
        world.Step(conf);
        ++steps;
        
        EXPECT_TRUE(almost_equal(body_a->GetLocation().x / Meter, (pos_a.x + x * time_inc * MeterPerSecond) / Meter));
        EXPECT_EQ(body_a->GetLocation().y, Real{0} * Meter);
        EXPECT_TRUE(almost_equal(body_b->GetLocation().x / Meter, (pos_b.x - x * time_inc * MeterPerSecond) / Meter));
        EXPECT_EQ(body_b->GetLocation().y, Real{0} * Meter);

        EXPECT_EQ(GetLinearVelocity(*body_a).x, +x * MeterPerSecond);
        EXPECT_EQ(GetLinearVelocity(*body_a).y, Real{0.0f} * MeterPerSecond);
        EXPECT_EQ(GetLinearVelocity(*body_b).x, -x * MeterPerSecond);
        EXPECT_EQ(GetLinearVelocity(*body_b).y, Real{0.0f} * MeterPerSecond);

        pos_a = body_a->GetLocation();
        pos_b = body_b->GetLocation();
    }
    
    // d = v * t
    // d = 20, v = 10:
    // 20 = 10 * t, t = d/v = 20 / 10 = 2
    // steps = t / time_inc = 200
    EXPECT_GE(steps, 199u);
    EXPECT_LE(steps, 201u);
    //EXPECT_EQ(int64_t(steps), static_cast<int64_t>(std::round(((x * 2) / x) / time_inc)));
}

TEST(World, PerfectlyOverlappedSameCirclesStayPut)
{
    const auto radius = Real(1) * Meter;
    const auto shape = std::make_shared<DiskShape>(radius);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1)); // changes where bodies will be after collision
    const auto gravity = LinearAcceleration2D{0, 0};

    World world{WorldDef{}.UseGravity(gravity)};
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.position = Length2D{Real(0) * Meter, Real(0) * Meter};

    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
    
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
    
    const auto time_inc = Real(.01);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, Time{Second * time_inc});
        EXPECT_EQ(body1->GetLocation().x, body_def.position.x);
        EXPECT_EQ(body1->GetLocation().y, body_def.position.y);
        EXPECT_EQ(body2->GetLocation().x, body_def.position.x);
        EXPECT_EQ(body2->GetLocation().y, body_def.position.y);
    }
}

TEST(World, PerfectlyOverlappedConcentricCirclesStayPut)
{
    const auto radius1 = Real(1) * Meter;
    const auto radius2 = Real(0.6) * Meter;
    
    const auto shape1 = std::make_shared<DiskShape>(radius1);
    shape1->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape1->SetRestitution(Real(1)); // changes where bodies will be after collision
    
    const auto shape2 = std::make_shared<DiskShape>(radius2);
    shape2->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape2->SetRestitution(Real(1)); // changes where bodies will be after collision

    const auto gravity = LinearAcceleration2D{0, 0};
    
    World world{WorldDef{}.UseGravity(gravity)};
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.position = Length2D{0, 0};
    
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape1);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
    
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape2);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
    
    const auto time_inc = Real(.01);
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, Time{Second * time_inc});
        EXPECT_EQ(body1->GetLocation().x, body_def.position.x);
        EXPECT_EQ(body1->GetLocation().y, body_def.position.y);
        EXPECT_EQ(body2->GetLocation().x, body_def.position.x);
        EXPECT_EQ(body2->GetLocation().y, body_def.position.y);
    }
}

TEST(World, ListenerCalledForCircleBodyWithinCircleBody)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    MyContactListener listener{
        [&](Contact&, const Manifold&) {},
        [&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };
    world.SetContactListener(&listener);

    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.position = Length2D{0, 0};
    const auto shape = std::make_shared<DiskShape>(Real{1} * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1));
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

    Step(world, Second * Real(1));

    EXPECT_NE(listener.begin_contacts, 0u);
    EXPECT_EQ(listener.end_contacts, 0u);
    EXPECT_NE(listener.pre_solves, 0u);
    EXPECT_NE(listener.post_solves, 0u);
}

TEST(World, ListenerCalledForSquareBodyWithinSquareBody)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    MyContactListener listener{
        [&](Contact&, const Manifold&) {},
        [&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };
    world.SetContactListener(&listener);
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.position = Length2D{0, 0};
    auto shape = std::make_shared<PolygonShape>();
    shape->SetVertexRadius(Real{1} * Meter);
    shape->SetAsBox(Real{2} * Meter, Real{2} * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1));
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
    
    Step(world, Second * Real(1));
    
    EXPECT_NE(listener.begin_contacts, 0u);
    EXPECT_EQ(listener.end_contacts, 0u);
    EXPECT_NE(listener.pre_solves, 0u);
    EXPECT_NE(listener.post_solves, 0u);
}

TEST(World, PartiallyOverlappedSameCirclesSeparate)
{
    const auto radius = Real(1);
    
    const auto gravity = LinearAcceleration2D{0, 0};
    World world{WorldDef{}.UseGravity(gravity)};
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false; // separation is faster if true.
    
    const auto shape = std::make_shared<DiskShape>(radius * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1)); // changes where bodies will be after collision
    
    const auto body1pos = Length2D{-radius/Real(4) * Meter, 0};
    body_def.position = body1pos;
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
    
    const auto body2pos = Length2D{+radius/Real(4) * Meter, 0};
    body_def.position = body2pos;
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
    
    auto position_diff = body2pos - body1pos;
    auto distance = GetLength(position_diff);

    const auto angle = GetAngle(position_diff);
    ASSERT_EQ(angle, Angle{0});

    auto lastpos1 = body1->GetLocation();
    auto lastpos2 = body2->GetLocation();

    const auto time_inc = Real(.01f) * Second;
    StepConf step;
    step.SetTime(time_inc);

    // Solver won't separate more than -step.linearSlop.
    const auto full_separation = radius * Real{2} * Meter - Length{step.linearSlop};
    for (auto i = 0; i < 100; ++i)
    {
        world.Step(step);

        const auto new_pos_diff = body2->GetLocation() - body1->GetLocation();
        const auto new_distance = GetLength(new_pos_diff);
        
        if (almost_equal(new_distance / Meter, full_separation / Meter) || new_distance > full_separation)
        {
            break;
        }
        
        ASSERT_GE(new_distance, distance);

        if (new_distance == distance)
        {
            // position resolution has come to tolerance
            ASSERT_GE(new_distance, radius * Real{2} * Meter - Length{step.linearSlop} * Real{4});
            break;
        }
        else // new_distance > distance
        {
            if (std::cos(angle / Radian) != 0)
            {
                EXPECT_LT(body1->GetLocation().x, lastpos1.x);
                EXPECT_GT(body2->GetLocation().x, lastpos2.x);
            }
            if (std::sin(angle / Radian) != 0)
            {
                EXPECT_LT(body1->GetLocation().y, lastpos1.y);
                EXPECT_GT(body2->GetLocation().y, lastpos2.y);
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
    const auto shape = std::make_shared<PolygonShape>(Real{1} * Meter, Real{1} * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1)); // changes where bodies will be after collision

    const auto gravity = LinearAcceleration2D{0, 0};
    
    World world{WorldDef{}.UseGravity(gravity)};
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false;
    body_def.position = Length2D{0, 0};
    
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
    
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
    ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
    
    auto lastpos1 = body1->GetLocation();
    auto lastpos2 = body2->GetLocation();

    auto stepConf = StepConf{};
    const auto time_inc = Real(.01) * Second;
    stepConf.SetTime(time_inc);
    stepConf.maxLinearCorrection = Real{0.0001f * 40} * Meter;
    for (auto i = 0; i < 100; ++i)
    {
        world.Step(stepConf);
        
        // body1 moves left only
        EXPECT_LT(body1->GetLocation().x, lastpos1.x);
        EXPECT_EQ(body1->GetLocation().y, lastpos1.y);

        // body2 moves right only
        EXPECT_GT(body2->GetLocation().x, lastpos2.x);
        EXPECT_EQ(body2->GetLocation().y, lastpos2.y);
        
        // body1 and body2 move away from each other equally.
        EXPECT_EQ(body1->GetLocation().x, -body2->GetLocation().x);
        EXPECT_EQ(body1->GetLocation().y, -body2->GetLocation().y);
        
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

    const auto gravity = LinearAcceleration2D{0, 0};
    World world{WorldDef{}.UseGravity(gravity)};
    
    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    body_def.bullet = false; // separation is faster if true.
    
    const auto half_dim = Real(64); // 1 causes additional y-axis separation
    const auto shape = std::make_shared<PolygonShape>(half_dim * Meter, half_dim * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1)); // changes where bodies will be after collision
    
    const auto body1pos = Length2D{Real(half_dim/2) * Meter, Real(0) * Meter}; // 0 causes additional y-axis separation
    body_def.position = body1pos;
    const auto body1 = world.CreateBody(body_def);
    {
        const auto fixture = body1->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body1->GetLocation().x, body1pos.x);
    ASSERT_EQ(body1->GetLocation().y, body1pos.y);
    
    const auto body2pos = Length2D{-Real(half_dim/2) * Meter, Real(0) * Meter}; // 0 causes additional y-axis separation
    body_def.position = body2pos;
    const auto body2 = world.CreateBody(body_def);
    {
        const auto fixture = body2->CreateFixture(shape);
        ASSERT_NE(fixture, nullptr);
    }
    ASSERT_EQ(body2->GetLocation().x, body2pos.x);
    ASSERT_EQ(body2->GetLocation().y, body2pos.y);

    ASSERT_EQ(body1->GetAngle(), Angle{0});
    ASSERT_EQ(body2->GetAngle(), Angle{0});
    auto last_angle_1 = body1->GetAngle();
    auto last_angle_2 = body2->GetAngle();

    ASSERT_EQ(world.GetBodies().size(), World::Bodies::size_type(2));
    ASSERT_EQ(world.GetContacts().size(), World::Contacts::size_type(0));

    auto position_diff = body1pos - body2pos;
    auto distance = GetLength(position_diff);
    
    auto angle = GetAngle(position_diff);
    EXPECT_TRUE(almost_equal(angle / Radian, Real{0}));
    
    auto lastpos1 = body1->GetLocation();
    auto lastpos2 = body2->GetLocation();
    
    const auto velocity_iters = 10u;
    const auto position_iters = 10u;
    
    const auto time_inc = Real(.01);
    StepConf step;
    step.SetTime(Time{Second * time_inc});
    // Solver won't separate more than -step.linearSlop.
    const auto full_separation = half_dim * Real{2} * Meter - Length{step.linearSlop};
    for (auto i = 0; i < 100; ++i)
    {
        Step(world, Time{Second * time_inc}, velocity_iters, position_iters);
        
        ASSERT_EQ(world.GetContacts().size(), decltype(world.GetContacts().size())(1));

        auto count = decltype(world.GetContacts().size())(0);
        const auto& contacts = world.GetContacts();
        for (auto&& contact: contacts)
        {
            ++count;
            const auto c = GetContactPtr(contact);

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
        EXPECT_EQ(v1.angular, Angle{0} / Second);
        EXPECT_EQ(v1.linear.x, Real(0) * MeterPerSecond);
        EXPECT_EQ(v1.linear.y, Real(0) * MeterPerSecond);

        const auto v2 = body2->GetVelocity();
        EXPECT_EQ(v2.angular, Angle{0} / Second);
        EXPECT_EQ(v2.linear.x, Real(0) * MeterPerSecond);
        EXPECT_EQ(v2.linear.y, Real(0) * MeterPerSecond);

        EXPECT_TRUE(almost_equal(body1->GetAngle() / Radian, last_angle_1 / Radian));
        EXPECT_TRUE(almost_equal(body2->GetAngle() / Radian, last_angle_2 / Radian));
        last_angle_1 = body1->GetAngle();
        last_angle_2 = body2->GetAngle();

        const auto new_pos_diff = body1->GetLocation() - body2->GetLocation();
        const auto new_distance = GetLength(new_pos_diff);
        
        if (almost_equal(new_distance / Meter, full_separation / Meter) || new_distance > full_separation)
        {
            break;
        }
        
        if (new_distance == distance)
        {
            if (std::cos(angle / Radian) != 0)
            {
                EXPECT_NE(body1->GetLocation().x, lastpos1.x);
                EXPECT_NE(body2->GetLocation().x, lastpos2.x);
            }
            if (std::sin(angle / Radian) != 0)
            {
                EXPECT_NE(body1->GetLocation().y, lastpos1.y);
                EXPECT_NE(body2->GetLocation().y, lastpos2.y);
            }
            ASSERT_GE(new_distance, Real(2) * Meter);
            break;
        }
        
        ASSERT_NE(body1->GetLocation(), lastpos1);
        ASSERT_NE(body2->GetLocation(), lastpos2);
        
        // Body 1 moves right only.
        EXPECT_GT(body1->GetLocation().x, lastpos1.x);
        EXPECT_TRUE(almost_equal(body1->GetLocation().y / Meter, lastpos1.y / Meter));

        // Body 2 moves left only.
        EXPECT_LT(body2->GetLocation().x, lastpos2.x);
        EXPECT_TRUE(almost_equal(body2->GetLocation().y / Meter, lastpos2.y / Meter));

        lastpos1 = body1->GetLocation();
        lastpos2 = body2->GetLocation();
        
        ASSERT_NE(new_pos_diff, position_diff);
        position_diff = new_pos_diff;
        
        ASSERT_NE(new_distance, distance);
        distance = new_distance;
        
        const auto new_angle = GetAngle(new_pos_diff);
        EXPECT_TRUE(almost_equal(angle / Radian, new_angle / Radian));
        
        angle = new_angle;
    }
}

TEST(World, CollidingDynamicBodies)
{
    const auto radius = Real(1) * Meter;
    const auto x = Real(10); // other test parameters tuned to this value being 10

    auto body_def = BodyDef{};
    body_def.type = BodyType::Dynamic;
    
    MyContactListener listener{
        [](Contact&, const Manifold&) {},
        [](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };

    const auto gravity = LinearAcceleration2D{0, 0};
    World world{WorldDef{}.UseGravity(gravity)};
    EXPECT_EQ(world.GetGravity(), gravity);
    world.SetContactListener(&listener);
    
    const auto shape = std::make_shared<DiskShape>(radius);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    shape->SetRestitution(Real(1)); // changes where bodies will be after collision

    body_def.position = Length2D{-(x + 1) * Meter, Real(0) * Meter};
    body_def.linearVelocity = LinearVelocity2D{+x * MeterPerSecond, 0};
    const auto body_a = world.CreateBody(body_def);
    ASSERT_NE(body_a, nullptr);
    EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body_a->IsSpeedable());
    EXPECT_TRUE(body_a->IsAccelerable());
    const auto fixture1 = body_a->CreateFixture(shape);
    ASSERT_NE(fixture1, nullptr);

    body_def.position = Length2D{+(x + 1) * Meter, Real(0) * Meter};
    body_def.linearVelocity = LinearVelocity2D{-x * MeterPerSecond, 0};
    const auto body_b = world.CreateBody(body_def);
    ASSERT_NE(body_b, nullptr);
    const auto fixture2 = body_b->CreateFixture(shape);
    ASSERT_NE(fixture2, nullptr);
    EXPECT_EQ(body_b->GetType(), BodyType::Dynamic);
    EXPECT_TRUE(body_b->IsSpeedable());
    EXPECT_TRUE(body_b->IsAccelerable());

    EXPECT_EQ(GetLinearVelocity(*body_a).x, +x * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body_a).y, Real{0.0f} * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body_b).x, -x * MeterPerSecond);
    EXPECT_EQ(GetLinearVelocity(*body_b).y, Real{0.0f} * MeterPerSecond);
    
    const auto time_collision = Real(1.0099994); // only valid for x >= around 4.214
    const auto time_inc = Real(.01);
    
    auto elapsed_time = Real(0);
    for (;;)
    {
        Step(world, Time{Second * time_inc});
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
    EXPECT_EQ(body_a->GetLocation().y, Length{0});
    EXPECT_EQ(body_b->GetLocation().y, Length{0});

    const auto tolerance = x / 100;
    
    // x position for body1 depends on restitution but it should be around -1
    EXPECT_GE(body_a->GetLocation().x / Meter, Real(-1) - tolerance);
    EXPECT_LT(body_a->GetLocation().x / Meter, Real(-1) + tolerance);

    // x position for body2 depends on restitution but it should be around +1
    EXPECT_LE(body_b->GetLocation().x / Meter, Real(+1) + tolerance);
    EXPECT_GT(body_b->GetLocation().x / Meter, Real(+1) - tolerance);
    
    // and their deltas from -1 and +1 should be about equal.
    EXPECT_TRUE(almost_equal((body_a->GetLocation().x + Real{1} * Meter) / Meter, (Real{1} * Meter - body_b->GetLocation().x) / Meter));

    EXPECT_GE(listener.body_a[0].x, Real{-1} * Meter);
    EXPECT_LE(listener.body_b[0].x, Real{+1} * Meter);

    for (;;)
    {
        Step(world, Time{Second * time_inc});
        elapsed_time += time_inc;
        if (!listener.contacting && !listener.touching)
        {
            break;
        }
    }
    EXPECT_FALSE(listener.touching);
    
    EXPECT_TRUE(almost_equal(elapsed_time, time_contacting + time_inc));
    
    // collision should be fully resolved now...
    EXPECT_LT(body_a->GetLocation().x, Real(-1) * Meter);
    EXPECT_GT(body_b->GetLocation().x, Real(+1) * Meter);
    
    // and their deltas from -1 and +1 should be about equal.
    EXPECT_TRUE(almost_equal((body_a->GetLocation().x + Real{1} * Meter) / Meter, (Real{1} * Meter - body_b->GetLocation().x) / Meter));

    EXPECT_LT(listener.body_a[1].x, Real{-1} * Meter);
    EXPECT_GT(listener.body_b[1].x, Real{+1} * Meter);
    
    // confirm conservation of momentum:
    // velocities should now be same magnitude but in opposite directions
    EXPECT_NEAR(double(Real{GetLinearVelocity(*body_a).x / MeterPerSecond}),
                double(-x), 0.0001);
    EXPECT_EQ(GetLinearVelocity(*body_a).y, Real{0.0f} * MeterPerSecond);
    EXPECT_NEAR(double(Real{GetLinearVelocity(*body_b).x / MeterPerSecond}),
                double(+x), 0.0001);
    EXPECT_EQ(GetLinearVelocity(*body_b).y, Real{0.0f} * MeterPerSecond);
}

TEST(World, TilesComesToRest)
{
    const auto m_world = std::make_unique<World>();
    
    constexpr auto e_count = 36;
    
    {
        const auto a = Real{0.5f};
        const auto ground = m_world->CreateBody(BodyDef{}.UseLocation(Length2D{0, -a * Meter}));
        
        const auto N = 200;
        const auto M = 10;
        Length2D position;
        position.y = 0.0f * Meter;
        for (auto j = 0; j < M; ++j)
        {
            position.x = -N * a * Meter;
            for (auto i = 0; i < N; ++i)
            {
                PolygonShape shape;
                SetAsBox(shape, a * Meter, a * Meter, position, Angle{0});
                ground->CreateFixture(std::make_shared<PolygonShape>(shape));
                position.x += 2.0f * a * Meter;
            }
            position.y -= 2.0f * a * Meter;
        }
    }
    
    {
        const auto a = Real{0.5f};
        const auto shape = std::make_shared<PolygonShape>(a * Meter, a * Meter);
        shape->SetDensity(Real{5} * KilogramPerSquareMeter);
        
        Length2D x(-7.0f * Meter, 0.75f * Meter);
        Length2D y;
        const auto deltaX = Length2D(0.5625f * Meter, 1.25f * Meter);
        const auto deltaY = Length2D(1.125f * Meter, 0.0f * Meter);
        
        for (auto i = 0; i < e_count; ++i)
        {
            y = x;
            
            for (auto j = i; j < e_count; ++j)
            {
                const auto body = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(y));
                body->CreateFixture(shape);
                y += deltaY;
            }
            
            x += deltaX;
        }
    }
    
    StepConf step;
    step.SetTime(Time{Second / Real{60}});

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
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            EXPECT_EQ(numSteps, 1801ul);
            EXPECT_EQ(sumRegPosIters, 36523ul);
            EXPECT_EQ(sumRegVelIters, 46973ul);
            EXPECT_EQ(sumToiPosIters, 44044ul);
            EXPECT_EQ(sumToiVelIters, 114344ul);

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
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            EXPECT_EQ(numSteps,         1807ul);
            EXPECT_EQ(sumRegPosIters,  36584ul);
            EXPECT_EQ(sumRegVelIters,  47380ul);
            EXPECT_EQ(sumToiPosIters,  44552ul);
            EXPECT_EQ(sumToiVelIters, 115392ul);

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
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            EXPECT_EQ(numSteps,         1803ul);
            EXPECT_EQ(sumRegPosIters,  36528ul);
            EXPECT_EQ(sumRegVelIters,  46988ul);
            EXPECT_EQ(sumToiPosIters,  44338ul);
            EXPECT_EQ(sumToiVelIters, 115317ul);

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
            // From commit 6b16f3722d5daac80ebaefd1dfda424939498dd4 onward:
            EXPECT_EQ(numSteps,         1807ul);
            EXPECT_EQ(sumRegPosIters,  36584ul);
            EXPECT_EQ(sumRegVelIters,  47380ul);
            EXPECT_EQ(sumToiPosIters,  44552ul);
            EXPECT_EQ(sumToiVelIters, 115406ul);
            
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
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};

    MyContactListener listener{
        [](Contact&, const Manifold&) {},
        [](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
        [&](Contact&) {},
    };
    world.SetContactListener(&listener);

    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    const auto left_edge_x = Real(-0.1) * Meter;
    const auto right_edge_x = Real(+0.1) * Meter;

    BodyDef body_def;
    const auto edge_shape = std::make_shared<EdgeShape>(Length2D{Real(0) * Meter, +Real(10) * Meter}, Length2D{Real(0) * Meter, -Real(10) * Meter});
    edge_shape->SetRestitution(Real(1));

    body_def.type = BodyType::Static;

    body_def.position = Length2D{left_edge_x, Real{0} * Meter};
    const auto left_wall_body = world.CreateBody(body_def);
    ASSERT_NE(left_wall_body, nullptr);
    {
        const auto wall_fixture = left_wall_body->CreateFixture(edge_shape);
        ASSERT_NE(wall_fixture, nullptr);
    }

    body_def.position = Length2D{right_edge_x, Real{0} * Meter};
    const auto right_wall_body = world.CreateBody(body_def);
    ASSERT_NE(right_wall_body, nullptr);
    {
        const auto wall_fixture = right_wall_body->CreateFixture(edge_shape);
        ASSERT_NE(wall_fixture, nullptr);
    }
    
    const auto begin_x = Real(0);

    body_def.type = BodyType::Dynamic;
    body_def.position = Length2D{begin_x * Meter, 0};
    body_def.bullet = false;
    const auto ball_body = world.CreateBody(body_def);
    ASSERT_NE(ball_body, nullptr);
    
    const auto ball_radius = Real(.01) * Meter;
    const auto circle_shape = std::make_shared<DiskShape>(ball_radius);
    circle_shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    circle_shape->SetRestitution(Real(1)); // changes where bodies will be after collision
    const auto ball_fixture = ball_body->CreateFixture(circle_shape);
    ASSERT_NE(ball_fixture, nullptr);

    const auto velocity = LinearVelocity2D{+Real(1) * MeterPerSecond, Real(0) * MeterPerSecond};
    ball_body->SetVelocity(Velocity{velocity, Angle{0} / Second});

    const auto time_inc = Real(.01) * Second;
    auto stepConf = StepConf{};
    stepConf.SetTime(time_inc);
    const auto max_velocity = stepConf.maxTranslation / time_inc;
    world.Step(stepConf);

    ASSERT_EQ(listener.begin_contacts, unsigned{0});

    EXPECT_GT(ball_body->GetLocation().x / Meter, begin_x);

    EXPECT_EQ(GetLinearVelocity(*ball_body).x, velocity.x);
    EXPECT_EQ(GetLinearVelocity(*ball_body).y, velocity.y);
    
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
            world.Step(stepConf);

            EXPECT_LT(ball_body->GetLocation().x, right_edge_x - (ball_radius/Real{2}));
            EXPECT_GT(ball_body->GetLocation().x, left_edge_x + (ball_radius/Real{2}));

            if (ball_body->GetVelocity().linear.x >= max_velocity)
            {
                return;
            }

            if (listener.begin_contacts % 2 != 0) // direction switched
            {
                EXPECT_LT(ball_body->GetVelocity().linear.x, Real{0.0f} * MeterPerSecond);
                break; // going left now
            }
            else if (listener.begin_contacts > last_contact_count)
            {
                ++increments;
                ball_body->SetVelocity(Velocity{{static_cast<Real>(increments) * velocity.x, ball_body->GetVelocity().linear.y}, ball_body->GetVelocity().angular});
            }
            else
            {
                EXPECT_TRUE(almost_equal(ball_body->GetVelocity().linear.x / MeterPerSecond, static_cast<Real>(increments) * velocity.x / MeterPerSecond));
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
            world.Step(stepConf);
            
            EXPECT_LT(ball_body->GetLocation().x, right_edge_x - (ball_radius/Real{2}));
            EXPECT_GT(ball_body->GetLocation().x, left_edge_x + (ball_radius/Real{2}));

            if (ball_body->GetVelocity().linear.x <= -max_velocity)
            {
                return;
            }

            if (listener.begin_contacts % 2 != 0) // direction switched
            {
                EXPECT_GT(ball_body->GetVelocity().linear.x, Real{0.0f} * MeterPerSecond);
                break; // going right now
            }
            else if (listener.begin_contacts > last_contact_count)
            {
                ++increments;
                ball_body->SetVelocity(Velocity{{-static_cast<Real>(increments) * velocity.x, ball_body->GetVelocity().linear.y}, ball_body->GetVelocity().angular});
            }
            else
            {
                EXPECT_TRUE(almost_equal(ball_body->GetVelocity().linear.x / MeterPerSecond, -static_cast<Real>(increments) * velocity.x / MeterPerSecond));
            }
        }
        
        ++increments;
        ball_body->SetVelocity(Velocity{{static_cast<Real>(increments) * velocity.x, ball_body->GetVelocity().linear.y}, ball_body->GetVelocity().angular});
    }
}

TEST(World, MouseJointWontCauseTunnelling)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    
    const auto half_box_width = Real(0.2);
    const auto left_edge_x = -half_box_width;
    const auto right_edge_x = +half_box_width;

    const auto half_box_height = Real(0.2);
    const auto btm_edge_y = -half_box_height;
    const auto top_edge_y = +half_box_height;

    AABB container_aabb;

    BodyDef body_def;
    EdgeShape edge_shape;
    edge_shape.SetFriction(Real(0.4f));
    edge_shape.SetRestitution(Real(0.94f)); // changes where bodies will be after collision
    body_def.type = BodyType::Static;
    
    // Setup vertical bounderies
    edge_shape.Set(Length2D{0, +half_box_height * Real(2) * Meter}, Length2D{0, -half_box_height * Real(2) * Meter});

    body_def.position = Length2D{left_edge_x * Meter, 0};
    {
        const auto left_wall_body = world.CreateBody(body_def);
        ASSERT_NE(left_wall_body, nullptr);
        {
            const auto wall_fixture = left_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
            ASSERT_NE(wall_fixture, nullptr);
        }
        container_aabb.Include(ComputeAABB(*left_wall_body));
    }
    
    body_def.position = Length2D{right_edge_x * Meter, 0};
    {
        const auto right_wall_body = world.CreateBody(body_def);
        ASSERT_NE(right_wall_body, nullptr);
        {
            const auto wall_fixture = right_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
            ASSERT_NE(wall_fixture, nullptr);
        }
        container_aabb.Include(ComputeAABB(*right_wall_body));
    }

    // Setup horizontal bounderies
    edge_shape.Set(Length2D{-half_box_width * Real(2) * Meter, 0}, Length2D{+half_box_width * Real(2) * Meter, 0});
    
    body_def.position = Length2D{0, btm_edge_y * Meter};
    {
        const auto btm_wall_body = world.CreateBody(body_def);
        ASSERT_NE(btm_wall_body, nullptr);
        {
            const auto wall_fixture = btm_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
            ASSERT_NE(wall_fixture, nullptr);
        }
        container_aabb.Include(ComputeAABB(*btm_wall_body));
    }
    
    body_def.position = Length2D{0, top_edge_y * Meter};
    {
        const auto top_wall_body = world.CreateBody(body_def);
        ASSERT_NE(top_wall_body, nullptr);
        {
            const auto wall_fixture = top_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
            ASSERT_NE(wall_fixture, nullptr);
        }
        container_aabb.Include(ComputeAABB(*top_wall_body));
    }

    body_def.type = BodyType::Dynamic;
    body_def.position = Length2D(0, 0);
    body_def.bullet = true;
    
    const auto ball_body = world.CreateBody(body_def);
    ASSERT_NE(ball_body, nullptr);
    ASSERT_EQ(ball_body->GetLocation().x, Length{0});
    ASSERT_EQ(ball_body->GetLocation().y, Length{0});
    
    const auto ball_radius = Real(half_box_width / 4) * Meter;
    const auto object_shape = std::make_shared<PolygonShape>(ball_radius, ball_radius);
    object_shape->SetDensity(Real{10} * KilogramPerSquareMeter);
    {
        const auto ball_fixture = ball_body->CreateFixture(object_shape);
        ASSERT_NE(ball_fixture, nullptr);
    }

    constexpr unsigned numBodies = 1;
    Length2D last_opos[numBodies];
    Body *bodies[numBodies];
    for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
    {
        const auto angle = i * 2 * Pi / numBodies;
        const auto x = ball_radius * Real(2.1) * Real(std::cos(angle));
        const auto y = ball_radius * Real(2.1) * Real(std::sin(angle));
        body_def.position = Length2D{x, y};
        bodies[i] = world.CreateBody(body_def);
        ASSERT_NE(bodies[i], nullptr);
        ASSERT_EQ(bodies[i]->GetLocation().x, x);
        ASSERT_EQ(bodies[i]->GetLocation().y, y);
        last_opos[i] = bodies[i]->GetLocation();
        {
            const auto fixture = bodies[i]->CreateFixture(object_shape);
            ASSERT_NE(fixture, nullptr);
        }
    }

    BodyDef bodyDef;
    const auto spare_body = world.CreateBody(bodyDef);

    const auto mouse_joint = [&]() {
        MouseJointDef mjd;
        mjd.bodyA = spare_body;
        mjd.bodyB = ball_body;
        const auto ball_body_pos = ball_body->GetLocation();
        mjd.target = Length2D{ball_body_pos.x - ball_radius / Real{2}, ball_body_pos.y + ball_radius / Real{2}};
        mjd.maxForce = Real(1000) * GetMass(*ball_body) * MeterPerSquareSecond;
        return static_cast<MouseJoint*>(world.CreateJoint(mjd));
    }();
    ASSERT_NE(mouse_joint, nullptr);

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
                const auto lt = Length2D{right_edge_x * Meter, top_edge_y * Meter} - bpos;
                const auto gt = bpos - Length2D{left_edge_x * Meter, btm_edge_y * Meter};
                
                if (lt.x <= Length{0} || lt.y <= Length{0} || gt.x <= Length{0} || gt.y <= Length{0})
                {
                    if (!TestOverlap(container_aabb, ComputeAABB(*body)))
                    {
                        // Body out of bounds and no longer even overlapping container!
                        EXPECT_LT(body->GetLocation().x, right_edge_x * Meter);
                        EXPECT_LT(body->GetLocation().y, top_edge_y * Meter);
                        EXPECT_GT(body->GetLocation().x, left_edge_x * Meter);
                        EXPECT_GT(body->GetLocation().y, btm_edge_y * Meter);
                        ++fail_count;
                    }
                }
            }
            if (fail_count > 0)
            {
                std::cout << " angl=" << angle;
                std::cout << " ctoi=" << 0 + contact.GetToiCount();
                std::cout << " solv=" << 0 + solved;
                std::cout << " targ=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
                std::cout << " maxv=" << max_velocity;
                std::cout << " rang=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
                std::cout << " bpos=(" << ball_body->GetLocation().x << "," << ball_body->GetLocation().y << ")";
                std::cout << std::endl;
                for (auto i = decltype(impulse.GetCount()){0}; i < impulse.GetCount(); ++i)
                {
                    std::cout << " i#" << (0 + i) << "={n" << impulse.GetEntryNormal(i) << ",t" << impulse.GetEntryTanget(i) << "}";
                }
                std::cout << std::endl;

                std::cout << " bodyA=(" << body_a->GetLocation().x << "," << body_a->GetLocation().y << ")";
                if (body_a == ball_body) std::cout << " ball";
                if (!body_a->IsSpeedable()) std::cout << " wall";
                std::cout << " " << body_a;
                std::cout << std::endl;
                std::cout << " bodyB=(" << body_b->GetLocation().x << "," << body_b->GetLocation().y << ")";
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

                if (body->GetLocation().x >= right_edge_x * Meter)
                {
                    escaped = true;
                }
                if (body->GetLocation().y >= top_edge_y * Meter)
                {
                    escaped = true;
                }
                if (body->GetLocation().x <= left_edge_x * Meter)
                {
                    escaped = true;
                }
                if (body->GetLocation().y <= btm_edge_y * Meter)
                {
                    escaped = true;                    
                }
            }
            if (escaped && !contact.IsTouching())
            {
                std::cout << "Escaped at EndContact[" << &contact << "]:";
                std::cout << " toiSteps=" << static_cast<unsigned>(contact.GetToiCount());
                std::cout << " toiValid=" << contact.HasValidToi();
                std::cout << " a[" << body_a << "]@(" << body_a->GetLocation().x << "," << body_a->GetLocation().y << ")";
                std::cout << " b[" << body_b << "]@(" << body_b->GetLocation().x << "," << body_b->GetLocation().y << ")";
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
            mouse_joint->SetTarget(Length2D{distance * std::cos(angle) * Meter, distance * std::sin(angle) * Meter});
            angle += anglular_speed;
            distance += distance_speed;

            Step(world, Time{Second * time_inc}, 8, 3);
            
            ASSERT_LT(ball_body->GetLocation().x, right_edge_x * Meter);
            ASSERT_LT(ball_body->GetLocation().y, top_edge_y * Meter);
            ASSERT_GT(ball_body->GetLocation().x, left_edge_x * Meter);
            ASSERT_GT(ball_body->GetLocation().y, btm_edge_y * Meter);
            for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
            {
                ASSERT_LT(bodies[i]->GetLocation().x, right_edge_x * Meter);
                ASSERT_LT(bodies[i]->GetLocation().y, top_edge_y * Meter);
                ASSERT_GT(bodies[i]->GetLocation().x, left_edge_x * Meter);
                ASSERT_GT(bodies[i]->GetLocation().y, btm_edge_y * Meter);
            }

            max_x = Max(Real{ball_body->GetLocation().x / Meter}, max_x);
            min_x = Min(Real{ball_body->GetLocation().x / Meter}, min_x);

            max_y = Max(Real{ball_body->GetLocation().y / Meter}, max_y);
            min_y = Min(Real{ball_body->GetLocation().y / Meter}, min_y);

            const auto linVel = ball_body->GetVelocity().linear;
            max_velocity = Max(GetLength(Vec2{linVel.x / MeterPerSecond, linVel.y / MeterPerSecond}), max_velocity);

            if (loops > 50)
            {
                if (mouse_joint->GetTarget().x < Length{0})
                {
                    if (ball_body->GetLocation().x >= last_pos.x)
                        break;                    
                }
                else
                {
                    if (ball_body->GetLocation().x <= last_pos.x)
                        break;
                }
                if (mouse_joint->GetTarget().y < Length{0})
                {
                    if (ball_body->GetLocation().y >= last_pos.y)
                        break;
                }
                else
                {
                    if (ball_body->GetLocation().y <= last_pos.y)
                        break;
                }
            }
            last_pos = ball_body->GetLocation();
        }
        anglular_speed *= anglular_accel;
        distance_speed *= distance_accel;

        ASSERT_NE(ball_body->GetLocation(), Length2D(0, 0));
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
    std::cout << " target=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
    std::cout << " maxvel=" << max_velocity;
    std::cout << " range=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
    std::cout << std::endl;
#endif
}

#if 0
static void smaller_still_conserves_momentum(bool bullet, Real multiplier, Real time_inc)
{
    const auto radius = Real(1);
    const auto start_distance = Real(10);
    
    auto scale = Real(1);
    for (;;)
    {
        const auto gravity = Vec2_zero;
        World world{WorldDef{}.UseGravity(gravity)};
        ASSERT_EQ(world.GetGravity().x, 0);
        ASSERT_EQ(world.GetGravity().y, 0);

        auto maxNormalImpulse = Real(0);
        auto maxTangentImpulse = Real(0);
        auto maxPoints = 0u;
        auto numSteps = 0u;
        auto failed = false;
        auto preB1 = Vec2_zero;
        auto preB2 = Vec2_zero;
        
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
                    maxPoints = Max(maxPoints, decltype(maxPoints){count});
                    for (auto i = decltype(count){0}; i < count; ++i)
                    {
                        maxNormalImpulse = Max(maxNormalImpulse, impulse.GetEntryNormal(i));
                        maxTangentImpulse = Max(maxTangentImpulse, impulse.GetEntryTanget(i));
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

        const auto shape = std::make_shared<DiskShape>(scale * radius * Meter);
        ASSERT_EQ(shape->GetRadius(), scale * radius);
        
        auto fixture_def = FixtureDef{}.UseDensity(1);
        fixture_def.friction = 0;
        fixture_def.restitution = 1;
        
        auto body_def = BodyDef{};
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
                EXPECT_NEAR(double(relative_velocity.x), double(Abs(body_def.linearVelocity.x) * +2), 0.0001);
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
            
            EXPECT_TRUE(almost_equal(relative_velocity.x, Abs(body_def.linearVelocity.x) * -2));
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
        const auto hw_ground = 40.0f * Meter;
        const auto ground = world.CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Length2D{-hw_ground, 0}, Length2D{hw_ground, 0}));
        
        const auto numboxes = boxes.size();
        
        original_x = GetParam();
        
        const auto boxShape = std::make_shared<PolygonShape>(hdim, hdim);
        boxShape->SetDensity(Real{1} * KilogramPerSquareMeter);
        boxShape->SetFriction(Real(0.3f));
        for (auto i = decltype(numboxes){0}; i < numboxes; ++i)
        {
            // (hdim + 0.05f) + (hdim * 2 + 0.1f) * i
            const auto location = Length2D{original_x * Meter, (i + Real{1}) * hdim * Real{4}};
            const auto box = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location));
            box->CreateFixture(boxShape);
            boxes[i] = box;
        }
        
        const auto stepConf = StepConf{}.SetTime(Time{Second / Real{60}});
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
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{
        Real(0) * MeterPerSquareSecond, -Real(10) * MeterPerSquareSecond
    })};
    std::size_t loopsTillSleeping = 0;
    const std::size_t maxLoops = 10000;
    std::vector<Body*> boxes{10};
    Real original_x = 0;
    const Length hdim = Real{0.1f} * Meter;
};

TEST_P(VerticalStackTest, EndsBeforeMaxLoops)
{
    EXPECT_LT(loopsTillSleeping, maxLoops);
}

TEST_P(VerticalStackTest, BoxesAtOriginalX)
{
    for (auto&& box: boxes)
    {
        EXPECT_EQ(box->GetLocation().x, original_x * Meter);
    }
}

TEST_P(VerticalStackTest, EachBoxAboveLast)
{
    auto lasty = Length{0};
    for (auto&& box: boxes)
    {
        EXPECT_GT(box->GetLocation().y, lasty + hdim);
        lasty = box->GetLocation().y;
    }
}

TEST_P(VerticalStackTest, EachBodyLevel)
{
    for (auto&& box: boxes)
    {
        EXPECT_EQ(box->GetAngle(), Angle{0});
    }
}

static std::string test_suffix_generator(::testing::TestParamInfo<Real> param_info)
{
    std::stringstream strbuf;
    strbuf << param_info.index;
    return strbuf.str();
}

extern ::testing::internal::ParamGenerator<VerticalStackTest::ParamType> gtest_WorldVerticalStackTest_EvalGenerator_();
extern ::std::string gtest_WorldVerticalStackTest_EvalGenerateName_(const ::testing::TestParamInfo<VerticalStackTest::ParamType>& info);

INSTANTIATE_TEST_CASE_P(World, VerticalStackTest, ::testing::Values(Real(0), Real(5)), test_suffix_generator);
