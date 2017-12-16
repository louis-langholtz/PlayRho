/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

#include <chrono>

using namespace playrho;

TEST(Body, ContactsByteSize)
{
#if defined(__APPLE__)
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(24));
#elif defined(__linux__)
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(24));
#elif defined(_WIN64)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(32));
#else
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(24));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(12));
#endif
#else
    // Intentionally fail for unknown platform...
    EXPECT_EQ(sizeof(Body::Contacts), std::size_t(0));
#endif
}

TEST(Body, JointsByteSize)
{
#ifdef __APPLE__
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(24));
#elif __linux__
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(24));
#elif _WIN64
#if defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(24));
#else
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(32));
#endif
#elif _WIN32
#if defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(12));
#else
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(16));
#endif
#else // !__APPLE__ && !__linux__ && !_WIN64 && !_WIN32
    // Intentionally fail for unknown platform...
    EXPECT_EQ(sizeof(Body::Joints), std::size_t(0));
#endif
}

TEST(Body, FixturesByteSize)
{
    // Size is arch, os, or library dependent.
#ifdef __APPLE__
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#elif __linux__
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#elif _WIN64
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(32));
#else
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#endif
#elif _WIN32
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(12));
#endif
#else
    // Intentionally fail for unknown platform...
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(0));
#endif
}

TEST(Body, ByteSize)
{
    const auto contactsSize = sizeof(Body::Contacts);
    const auto jointsSize = sizeof(Body::Joints);
    const auto fixturesSize = sizeof(Body::Fixtures);
    const auto allSize = contactsSize + jointsSize + fixturesSize;

#if defined(_WIN64)
#if !defined(NDEBUG)
    EXPECT_EQ(allSize, std::size_t(96));
#else
    EXPECT_EQ(allSize, std::size_t(72));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
    EXPECT_EQ(allSize, std::size_t(48));
#else
    EXPECT_EQ(allSize, std::size_t(36));
#endif
#else
    EXPECT_EQ(allSize, std::size_t(72));
#endif

    // architecture dependent...
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(Body), std::size_t(108 + allSize));
#else
            EXPECT_EQ(sizeof(Body), std::size_t(120 + allSize));
#endif
            break;
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
    
    EXPECT_FALSE(Awaken(*body));

    EXPECT_TRUE(body->GetFixtures().empty());
    {
        int i = 0;
        for (auto&& fixture: body->GetFixtures())
        {
            EXPECT_EQ(GetRef(fixture).GetBody(), body);
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

TEST(Body, SetVelocityDoesNothingToStatic)
{
    const auto zeroVelocity = Velocity2D{
        LinearVelocity2{0_mps, 0_mps},
        AngularVelocity{Real(0) * RadianPerSecond}
    };

    World world;

    auto body = world.CreateBody();
    ASSERT_NE(body, nullptr);
    ASSERT_FALSE(body->IsAwake());
    ASSERT_FALSE(body->IsSpeedable());
    ASSERT_FALSE(body->IsAccelerable());
    ASSERT_EQ(body->GetVelocity(), zeroVelocity);
    
    const auto velocity = Velocity2D{
        LinearVelocity2{1.1_mps, 1.1_mps},
        AngularVelocity{Real(1.1) * RadianPerSecond}
    };
    body->SetVelocity(velocity);
    EXPECT_NE(body->GetVelocity(), velocity);
    EXPECT_EQ(body->GetVelocity(), zeroVelocity);
}

TEST(Body, CreateFixture)
{
    World world;
    const auto body = world.CreateBody();
    EXPECT_EQ(GetFixtureCount(*body), std::size_t(0));

    const auto valid_shape = DiskShapeConf(1_m);
    EXPECT_NE(body->CreateFixture(valid_shape, FixtureDef{}), nullptr);

    EXPECT_EQ(GetFixtureCount(*body), std::size_t(1));
}

TEST(Body, DestroyFixture)
{
    World world;
    const auto bodyA = world.CreateBody();
    const auto bodyB = world.CreateBody();
    ASSERT_EQ(GetFixtureCount(*bodyA), std::size_t(0));
    ASSERT_EQ(GetFixtureCount(*bodyB), std::size_t(0));

    const auto fixtureA = bodyA->CreateFixture(DiskShapeConf(1_m), FixtureDef{});
    ASSERT_NE(fixtureA, nullptr);
    ASSERT_EQ(GetFixtureCount(*bodyA), std::size_t(1));

    EXPECT_FALSE(bodyB->DestroyFixture(fixtureA));
    EXPECT_EQ(GetFixtureCount(*bodyA), std::size_t(1));
    EXPECT_TRUE(bodyA->DestroyFixture(fixtureA));
    EXPECT_EQ(GetFixtureCount(*bodyA), std::size_t(0));
}

TEST(Body, SetEnabled)
{
    auto stepConf = StepConf{};
    World world;
    const auto body = world.CreateBody();
    const auto valid_shape = DiskShapeConf(1_m);

    const auto fixture = body->CreateFixture(valid_shape, FixtureDef{});
    ASSERT_NE(fixture, nullptr);
    ASSERT_TRUE(body->IsEnabled());
    ASSERT_EQ(fixture->GetProxyCount(), 0u);

    world.Step(stepConf);
    EXPECT_EQ(fixture->GetProxyCount(), 1u);

    // Test that set enabled to flag already set is not a toggle
    body->SetEnabled(true);
    EXPECT_TRUE(body->IsEnabled());
    EXPECT_EQ(fixture->GetProxyCount(), 1u);

    body->SetEnabled(false);
    EXPECT_FALSE(body->IsEnabled());
    EXPECT_EQ(fixture->GetProxyCount(), 1u);

    world.Step(stepConf);
    EXPECT_EQ(fixture->GetProxyCount(), 0u);
    
    body->SetEnabled(true);
    EXPECT_TRUE(body->IsEnabled());

    world.Step(stepConf);
    EXPECT_EQ(fixture->GetProxyCount(), 1u);
}

TEST(Body, SetFixedRotation)
{
    World world;
    const auto body = world.CreateBody();
    const auto valid_shape = DiskShapeConf(1_m);

    ASSERT_NE(body->CreateFixture(valid_shape, FixtureDef{}), nullptr);
    ASSERT_FALSE(body->IsFixedRotation());

    // Test that set fixed rotation to flag already set is not a toggle
    body->SetFixedRotation(false);
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

    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1_kgpm2;
    const auto shape = Shape(conf);
    
    {
        auto fixture = body->CreateFixture(shape, FixtureDef{}, false);
        const auto fshape = fixture->GetShape();
        EXPECT_EQ(GetVertexRadius(fshape), GetVertexRadius(shape));
        EXPECT_EQ(static_cast<const DiskShapeConf*>(GetData(fshape))->GetLocation(), conf.GetLocation());
        EXPECT_FALSE(body->GetFixtures().empty());
        {
            int i = 0;
            for (auto&& f: body->GetFixtures())
            {
                EXPECT_EQ(GetPtr(f), fixture);
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
        EXPECT_EQ(GetVertexRadius(fshape), GetVertexRadius(shape));
        EXPECT_EQ(static_cast<const DiskShapeConf*>(GetData(fshape))->GetLocation(), conf.GetLocation());
        EXPECT_FALSE(body->GetFixtures().empty());
        {
            int i = 0;
            for (auto&& f: body->GetFixtures())
            {
                EXPECT_EQ(GetPtr(f), fixture);
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
    const auto xfm1 = Transformation{Length2{}, UnitVec2::GetRight()};
    ASSERT_EQ(body->GetTransformation(), xfm1);
    const auto xfm2 = Transformation{Vec2(10, -12) * 1_m, UnitVec2::GetLeft()};
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
    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1.3_kgpm2;
    const auto shape = Shape(conf);
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
    EXPECT_EQ(GetWorldIndex(body0), BodyCounter(0));
    const auto body1 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    EXPECT_EQ(GetWorldIndex(body1), BodyCounter(1));
    const auto body2 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(3));
    EXPECT_EQ(GetWorldIndex(body2), BodyCounter(2));
    EXPECT_EQ(GetWorldIndex(static_cast<const Body*>(nullptr)), BodyCounter(-1));
}

TEST(Body, ApplyLinearAccelDoesNothingToStatic)
{
    World world;
    
    auto body = world.CreateBody();
    ASSERT_NE(body, nullptr);
    ASSERT_FALSE(body->IsAwake());
    ASSERT_FALSE(body->IsSpeedable());
    ASSERT_FALSE(body->IsAccelerable());
    
    const auto zeroAccel = LinearAcceleration2{
        Real(0) * MeterPerSquareSecond, Real(0) * MeterPerSquareSecond
    };
    const auto linAccel = LinearAcceleration2{
        Real(2) * MeterPerSquareSecond, Real(2) * MeterPerSquareSecond
    };
    ApplyLinearAcceleration(*body, linAccel);
    EXPECT_NE(body->GetLinearAcceleration(), linAccel);
    EXPECT_EQ(body->GetLinearAcceleration(), zeroAccel);
}

TEST(Body, GetAccelerationFF)
{
    World world;
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    body->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
    
    ASSERT_EQ(body->GetLinearAcceleration(), LinearAcceleration2{});
    ASSERT_EQ(body->GetAngularAcceleration(), AngularAcceleration{});
    
    EXPECT_EQ(GetAcceleration(*body), Acceleration{});
}

TEST(Body, SetAccelerationFF)
{
    World world;
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    body->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
    
    ASSERT_EQ(body->GetLinearAcceleration(), LinearAcceleration2{});
    ASSERT_EQ(body->GetAngularAcceleration(), AngularAcceleration{});
 
    const auto newAccel = Acceleration{
        LinearAcceleration2{2_mps2, 3_mps2}, AngularAcceleration{1.2f * RadianPerSquareSecond}
    };
    SetAcceleration(*body, newAccel);
    EXPECT_EQ(GetAcceleration(*body), newAccel);
}

TEST(Body, CalcGravitationalAcceleration)
{
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2{})};

    const auto l1 = Length2{-8_m, 0_m};
    const auto l2 = Length2{+8_m, 0_m};
    const auto l3 = Length2{+16_m, 0_m};
    const auto shape = DiskShapeConf{}.UseRadius(2_m).UseDensity(1e10_kgpm2);
    
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(l1));
    b1->CreateFixture(shape);
    EXPECT_EQ(CalcGravitationalAcceleration(*b1), Acceleration{});
    
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(l2));
    b2->CreateFixture(shape);
    const auto accel = CalcGravitationalAcceleration(*b1);
    EXPECT_NEAR(static_cast<double>(Real(GetX(accel.linear)/MeterPerSquareSecond)),
                0.032761313021183014, 0.032761313021183014/100);
    EXPECT_EQ(GetY(accel.linear), 0 * MeterPerSquareSecond);
    EXPECT_EQ(accel.angular, 0 * RadianPerSquareSecond);
    
    const auto b3 = world.CreateBody(BodyDef{}.UseType(BodyType::Static).UseLocation(l3));
    EXPECT_EQ(CalcGravitationalAcceleration(*b3), Acceleration{});
}

TEST(Body, RotateAboutWorldPointFF)
{
    auto world = World{};
    const auto body = world.CreateBody();
    const auto locationA = body->GetLocation();
    ASSERT_EQ(locationA, Length2(0_m, 0_m));
    RotateAboutWorldPoint(*body, 90_deg, Length2{2_m, 0_m});
    const auto locationB = body->GetLocation();
    EXPECT_NEAR(static_cast<double>(Real(GetX(locationB)/Meter)), +2.0, 0.001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(locationB)/Meter)), -2.0, 0.001);
}

TEST(Body, RotateAboutLocalPointFF)
{
    auto world = World{};
    const auto body = world.CreateBody();
    const auto locationA = body->GetLocation();
    ASSERT_EQ(locationA, Length2(0_m, 0_m));
    RotateAboutLocalPoint(*body, 90_deg, Length2{2_m, 0_m});
    const auto locationB = body->GetLocation();
    EXPECT_NEAR(static_cast<double>(Real(GetX(locationB)/Meter)), +2.0, 0.001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(locationB)/Meter)), -2.0, 0.001);
}
