/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>

#include <chrono>

using namespace box2d;

TEST(Body, ContactsByteSize)
{
    // Size is C++ library dependent.
    // Some platforms it's 24-bytes. Others 16.
    EXPECT_TRUE(sizeof(Body::Contacts) == std::size_t(24)
                || sizeof(Body::Contacts) == std::size_t(16));
}

TEST(Body, JointsByteSize)
{
    // Size is arch, os, or library dependent.
#ifdef __APPLE__
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(24));
#endif
#ifdef __linux__
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(24));
#endif
}

TEST(Body, FixturesByteSize)
{
    // Size is arch, os, or library dependent.
#ifdef __linux__
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#endif
}

TEST(Body, ByteSize)
{
    const auto contactsSize = sizeof(Body::Contacts);
    const auto jointsSize = sizeof(Body::Joints);
    const auto fixturesSize = sizeof(Body::Fixtures);
    const auto allSize = contactsSize + jointsSize + fixturesSize;

    // architecture dependent...
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Body), std::size_t(120 + allSize)); break;
        case  8: EXPECT_EQ(sizeof(Body), std::size_t(216 + allSize)); break;
        case 16: EXPECT_EQ(sizeof(Body), std::size_t(496)); break;
        default: FAIL(); break;
    }
}

TEST(Body, Traits)
{
    EXPECT_FALSE(std::is_default_constructible<Body>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<Body>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<Body>::value);
    
    EXPECT_FALSE(std::is_constructible<Body>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<Body>::value);
    EXPECT_FALSE(std::is_trivially_constructible<Body>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<Body>::value);
    EXPECT_FALSE(std::is_nothrow_copy_constructible<Body>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<Body>::value);
    
    EXPECT_FALSE(std::is_copy_assignable<Body>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<Body>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<Body>::value);
    
    EXPECT_TRUE(std::is_destructible<Body>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Body>::value);
    EXPECT_FALSE(std::is_trivially_destructible<Body>::value);
}

TEST(Body, GetFlagsStatic)
{
    EXPECT_TRUE(Body::GetFlags(BodyDef{}.UseFixedRotation(true)) & Body::e_fixedRotationFlag);
    EXPECT_TRUE(Body::GetFlags(BodyDef{}
                               .UseAwake(false)
                               .UseAllowSleep(false)
                               .UseType(BodyType::Dynamic)) & Body::e_awakeFlag);
}

TEST(Body, WorldCreated)
{
    World world;
    
    auto body = world.CreateBody();
    ASSERT_NE(body, nullptr);

    EXPECT_EQ(body->GetWorld(), &world);
    EXPECT_EQ(body->GetUserData(), nullptr);
    EXPECT_TRUE(body->IsEnabled());
    EXPECT_FALSE(body->IsAwake());
    EXPECT_FALSE(body->IsSpeedable());
    EXPECT_FALSE(body->IsAccelerable());
    
    EXPECT_TRUE(body->GetFixtures().empty());
    {
        int i = 0;
        for (auto&& fixture: body->GetFixtures())
        {
            EXPECT_EQ(fixture.GetBody(), body);
            ++i;
        }
        EXPECT_EQ(i, 0);
    }

    EXPECT_TRUE(body->GetJoints().empty());
    {
        int i = 0;
        for (auto&& joint: body->GetJoints())
        {
            NOT_USED(joint);
            ++i;
        }
        EXPECT_EQ(i, 0);        
    }
    
    EXPECT_TRUE(body->GetContacts().empty());
    {
        int i = 0;
        for (auto&& ce: body->GetContacts())
        {
            NOT_USED(ce);
            ++i;
        }
        EXPECT_EQ(i, 0);        
    }
}

TEST(Body, CreateFixture)
{
    World world;
    const auto body = world.CreateBody();
    EXPECT_EQ(GetFixtureCount(*body), std::size_t(0));

    const auto valid_shape = std::make_shared<DiskShape>(Real{1} * Meter);
    EXPECT_NE(body->CreateFixture(valid_shape, FixtureDef{}), nullptr);

    EXPECT_EQ(GetFixtureCount(*body), std::size_t(1));
}

TEST(Body, SetEnabled)
{
    World world;
    const auto body = world.CreateBody();
    const auto valid_shape = std::make_shared<DiskShape>(Real{1} * Meter);
    ASSERT_NE(body->CreateFixture(valid_shape, FixtureDef{}), nullptr);
    
    EXPECT_TRUE(body->IsEnabled());
    body->SetEnabled(false);
    EXPECT_FALSE(body->IsEnabled());
    body->SetEnabled(true);
    EXPECT_TRUE(body->IsEnabled());
}

TEST(Body, SetFixedRotation)
{
    World world;
    const auto body = world.CreateBody();
    const auto valid_shape = std::make_shared<DiskShape>(Real{1} * Meter);
    ASSERT_NE(body->CreateFixture(valid_shape, FixtureDef{}), nullptr);
    
    EXPECT_FALSE(body->IsFixedRotation());
    body->SetFixedRotation(true);
    EXPECT_TRUE(body->IsFixedRotation());
    body->SetFixedRotation(false);
    EXPECT_FALSE(body->IsFixedRotation());
}

TEST(Body, CreateAndDestroyFixture)
{
    World world;

    auto body = world.CreateBody();
    ASSERT_NE(body, nullptr);
    EXPECT_TRUE(body->GetFixtures().empty());
    EXPECT_FALSE(body->IsMassDataDirty());

    auto conf = DiskShape::Conf{};
    conf.vertexRadius = Real{2.871f} * Meter;
    conf.location = Vec2{1.912f, -77.31f} * (Real(1) * Meter);
    conf.density = Real{1} * KilogramPerSquareMeter;
    const auto shape = std::make_shared<DiskShape>(conf);
    
    {
        auto fixture = body->CreateFixture(shape, FixtureDef{}, false);
        const auto fshape = fixture->GetShape();
        ASSERT_NE(fshape, nullptr);
        EXPECT_EQ(typeid(fshape.get()), typeid(const Shape*));
        EXPECT_EQ(GetVertexRadius(*fshape), GetVertexRadius(*shape));
        EXPECT_EQ(static_cast<const DiskShape*>(fshape.get())->GetLocation().x, shape->GetLocation().x);
        EXPECT_EQ(static_cast<const DiskShape*>(fshape.get())->GetLocation().y, shape->GetLocation().y);
        EXPECT_FALSE(body->GetFixtures().empty());
        {
            int i = 0;
            for (auto&& f: body->GetFixtures())
            {
                EXPECT_EQ(&f, fixture);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(body->IsMassDataDirty());
        body->ResetMassData();
        EXPECT_FALSE(body->IsMassDataDirty());

        body->DestroyFixture(fixture, false);
        EXPECT_TRUE(body->GetFixtures().empty());
        EXPECT_TRUE(body->IsMassDataDirty());
        
        body->ResetMassData();
        EXPECT_FALSE(body->IsMassDataDirty());
        
        body->DestroyFixtures();
        EXPECT_TRUE(body->GetFixtures().empty());
    }
    
    {
        auto fixture = body->CreateFixture(shape, FixtureDef{}, false);
        const auto fshape = fixture->GetShape();
        ASSERT_NE(fshape, nullptr);
        EXPECT_EQ(typeid(fshape.get()), typeid(const Shape*));
        EXPECT_EQ(GetVertexRadius(*fshape), GetVertexRadius(*shape));
        EXPECT_EQ(static_cast<const DiskShape*>(fshape.get())->GetLocation().x, shape->GetLocation().x);
        EXPECT_EQ(static_cast<const DiskShape*>(fshape.get())->GetLocation().y, shape->GetLocation().y);
        EXPECT_FALSE(body->GetFixtures().empty());
        {
            int i = 0;
            for (auto&& f: body->GetFixtures())
            {
                EXPECT_EQ(&f, fixture);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(body->IsMassDataDirty());
        body->ResetMassData();
        EXPECT_FALSE(body->IsMassDataDirty());
        EXPECT_FALSE(body->GetFixtures().empty());
        
        body->DestroyFixtures();
        EXPECT_TRUE(body->GetFixtures().empty());
        EXPECT_FALSE(body->IsMassDataDirty());
    }
}

TEST(Body, SetType)
{
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    World world;
    const auto body = world.CreateBody(bd);
    ASSERT_EQ(body->GetType(), BodyType::Dynamic);
    body->SetType(BodyType::Static);
    EXPECT_EQ(body->GetType(), BodyType::Static);
    body->SetType(BodyType::Kinematic);
    EXPECT_EQ(body->GetType(), BodyType::Kinematic);
    body->SetType(BodyType::Dynamic);
    EXPECT_EQ(body->GetType(), BodyType::Dynamic);
}

TEST(Body, SetTransform)
{
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    World world;
    const auto body = world.CreateBody(bd);
    const auto xfm1 = Transformation{Vec2_zero * (Real(1) * Meter), UnitVec2::GetRight()};
    ASSERT_EQ(body->GetTransformation(), xfm1);
    const auto xfm2 = Transformation{Vec2(10, -12) * (Real(1) * Meter), UnitVec2::GetLeft()};
    body->SetTransform(xfm2.p, GetAngle(xfm2.q));
    EXPECT_EQ(body->GetTransformation().p, xfm2.p);
    EXPECT_NEAR(static_cast<double>(GetX(body->GetTransformation().q)),
                static_cast<double>(GetX(xfm2.q)),
                0.001);
    EXPECT_NEAR(static_cast<double>(GetY(body->GetTransformation().q)),
                static_cast<double>(GetY(xfm2.q)),
                0.001);
}

TEST(Body, CreateLotsOfFixtures)
{
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = Real{2.871f} * Meter;
    conf.location = Vec2{1.912f, -77.31f} * (Real(1) * Meter);
    conf.density = Real{1.3f} * KilogramPerSquareMeter;
    const auto shape = std::make_shared<DiskShape>(conf);
    const auto num = 5000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    
    start = std::chrono::system_clock::now();
    {
        World world;

        auto body = world.CreateBody(bd);
        ASSERT_NE(body, nullptr);
        EXPECT_TRUE(body->GetFixtures().empty());
        
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            auto fixture = body->CreateFixture(shape, FixtureDef{}, false);
            ASSERT_NE(fixture, nullptr);
        }
        body->ResetMassData();
        
        EXPECT_FALSE(body->GetFixtures().empty());
        {
            int i = decltype(num){0};
            for (auto&& f: body->GetFixtures())
            {
                NOT_USED(f);
                ++i;
            }
            EXPECT_EQ(i, num);
        }
    }
    end = std::chrono::system_clock::now();
    const std::chrono::duration<double> elapsed_secs_resetting_at_end = end - start;

    start = std::chrono::system_clock::now();
    {
        World world;
        
        auto body = world.CreateBody(bd);
        ASSERT_NE(body, nullptr);
        EXPECT_TRUE(body->GetFixtures().empty());
        
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            auto fixture = body->CreateFixture(shape, FixtureDef{}, true);
            ASSERT_NE(fixture, nullptr);
        }
        
        EXPECT_FALSE(body->GetFixtures().empty());
        {
            int i = decltype(num){0};
            for (auto&& f: body->GetFixtures())
            {
                NOT_USED(f);
                ++i;
            }
            EXPECT_EQ(i, num);
        }
    }
    end = std::chrono::system_clock::now();
    const std::chrono::duration<double> elapsed_secs_resetting_in_create = end - start;

    EXPECT_LT(elapsed_secs_resetting_at_end.count(), elapsed_secs_resetting_in_create.count());
}

TEST(Body, GetWorldIndex)
{
    World world;
    ASSERT_EQ(world.GetBodies().size(), std::size_t(0));
    const auto body0 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(1));
    EXPECT_EQ(GetWorldIndex(body0), std::size_t(0));
    const auto body1 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    EXPECT_EQ(GetWorldIndex(body1), std::size_t(1));
    const auto body2 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(3));
    EXPECT_EQ(GetWorldIndex(body2), std::size_t(2));
}
