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
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldFixture.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <chrono>

using namespace playrho;
using namespace playrho::d2;

TEST(BodyConf, UsePosition)
{
    const auto p = Position{Length2{3_m, -4_m}, 22_deg};
    EXPECT_EQ(BodyConf{}.Use(p).location, p.linear);
    EXPECT_EQ(BodyConf{}.Use(p).angle, p.angular);
}

TEST(BodyConf, UseVelocity)
{
    const auto v = Velocity{LinearVelocity2{3_mps, -4_mps}, 22_rad / 1_s};
    EXPECT_EQ(BodyConf{}.Use(v).linearVelocity, v.linear);
    EXPECT_EQ(BodyConf{}.Use(v).angularVelocity, v.angular);
}

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
#if defined(_WIN64)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(Body), std::size_t(216));
#else
            EXPECT_EQ(sizeof(Body), std::size_t(192));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
            // Win32 debug
            EXPECT_EQ(sizeof(Body), std::size_t(192));
#else
            // Win32 release
            EXPECT_EQ(sizeof(Body), std::size_t(144));
#endif
#else
            EXPECT_EQ(sizeof(Body), std::size_t(184));
#endif
            break;
        case  8:
            EXPECT_EQ(sizeof(Body), std::size_t(288));
            break;
        case 16:
            EXPECT_EQ(sizeof(Body), std::size_t(496));
            break;
        default: FAIL(); break;
    }
}

TEST(Body, WorldCreated)
{
    auto world = World{};

    const auto body = world.CreateBody();
    ASSERT_NE(body, InvalidBodyID);

    EXPECT_EQ(GetUserData(world, body), nullptr);
    EXPECT_TRUE(IsEnabled(world, body));
    EXPECT_FALSE(IsAwake(world, body));
    EXPECT_FALSE(IsSpeedable(world, body));
    EXPECT_FALSE(IsAccelerable(world, body));
    EXPECT_FALSE(Awaken(world, body));

    EXPECT_TRUE(GetFixtures(world, body).empty());
    {
        auto i = 0;
        for (auto&& fixture: GetFixtures(world, body))
        {
            EXPECT_EQ(GetBody(world, fixture), body);
            ++i;
        }
        EXPECT_EQ(i, 0);
    }

    EXPECT_TRUE(GetJoints(world, body).empty());
    {
        auto i = 0;
        for (auto&& joint: GetJoints(world, body))
        {
            NOT_USED(joint);
            ++i;
        }
        EXPECT_EQ(i, 0);        
    }

    EXPECT_TRUE(GetContacts(world, body).empty());
    {
        auto i = 0;
        for (auto&& ce: GetContacts(world, body))
        {
            NOT_USED(ce);
            ++i;
        }
        EXPECT_EQ(i, 0);        
    }
}

TEST(Body, SetVelocityDoesNothingToStatic)
{
    const auto zeroVelocity = Velocity{
        LinearVelocity2{0_mps, 0_mps},
        AngularVelocity{Real(0) * RadianPerSecond}
    };

    auto world = World{};

    const auto body = world.CreateBody();
    ASSERT_NE(body, InvalidBodyID);
    ASSERT_FALSE(IsAwake(world, body));
    ASSERT_FALSE(IsSpeedable(world, body));
    ASSERT_FALSE(IsAccelerable(world, body));
    ASSERT_EQ(GetVelocity(world, body), zeroVelocity);

    const auto velocity = Velocity{
        LinearVelocity2{1.1_mps, 1.1_mps},
        AngularVelocity{Real(1.1) * RadianPerSecond}
    };
    SetVelocity(world, body, velocity);
    EXPECT_NE(GetVelocity(world, body), velocity);
    EXPECT_EQ(GetVelocity(world, body), zeroVelocity);
}

TEST(Body, CreateFixture)
{
    auto world = World{};
    const auto body = world.CreateBody();
    EXPECT_EQ(GetFixtureCount(world, body), std::size_t(0));

    const auto valid_shape = Shape{DiskShapeConf(1_m)};
    EXPECT_NE(world.CreateFixture(body, valid_shape, FixtureConf{}), InvalidFixtureID);

    EXPECT_EQ(GetFixtureCount(world, body), std::size_t(1));
    
    const auto minRadius = world.GetMinVertexRadius();
    EXPECT_THROW(world.CreateFixture(body, Shape{DiskShapeConf{minRadius / 2}}), InvalidArgument);
    
    const auto maxRadius = world.GetMaxVertexRadius();
    EXPECT_THROW(world.CreateFixture(body, Shape{DiskShapeConf{maxRadius + maxRadius / 10}}), InvalidArgument);
}

TEST(Body, Destroy)
{
    auto world = World{};
    const auto bodyA = world.CreateBody();
    const auto bodyB = world.CreateBody();
    ASSERT_EQ(GetFixtureCount(world, bodyA), std::size_t(0));
    ASSERT_EQ(GetFixtureCount(world, bodyB), std::size_t(0));

    const auto fixtureA = world.CreateFixture(bodyA, Shape{DiskShapeConf(1_m)}, FixtureConf{});
    ASSERT_NE(fixtureA, InvalidFixtureID);
    ASSERT_EQ(GetFixtureCount(world, bodyA), std::size_t(1));

    EXPECT_TRUE(world.Destroy(fixtureA));
    EXPECT_EQ(GetFixtureCount(world, bodyA), std::size_t(0));
}

TEST(Body, SetEnabledCausesIsEnabled)
{
    auto world = World{};
    const auto body = world.CreateBody();
    ASSERT_TRUE(IsEnabled(world, body));
    auto value = true;
    for (auto i = 0; i < 4; ++i) {
        // Set and check twice to ensure same behavior if state already same.
        // Inlined to help match state with line number of any reports.
        EXPECT_NO_THROW(world.SetEnabled(body, value));
        EXPECT_EQ(IsEnabled(world, body), value);
        EXPECT_NO_THROW(world.SetEnabled(body, value));
        EXPECT_EQ(IsEnabled(world, body), value);
        value = !value;
    }
}

TEST(Body, SetEnabled)
{
    auto stepConf = StepConf{};

    auto world = World{};
    ASSERT_EQ(world.GetFixturesForProxies().size(), 0u);
    ASSERT_EQ(world.GetBodiesForProxies().size(), 0u);

    const auto body0 = world.CreateBody();
    const auto body1 = world.CreateBody();
    const auto valid_shape = Shape{DiskShapeConf(1_m)};

    const auto fixture0 = world.CreateFixture(body0, valid_shape, FixtureConf{});
    const auto fixture1 = world.CreateFixture(body1, valid_shape, FixtureConf{});
    ASSERT_NE(fixture0, InvalidFixtureID);
    ASSERT_NE(fixture1, InvalidFixtureID);

    ASSERT_TRUE(IsEnabled(world, body0));
    ASSERT_EQ(GetProxyCount(world, fixture0), 0u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 2u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.Step(stepConf));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 0u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    // Test that set enabled to flag already set is not a toggle
    EXPECT_NO_THROW(world.SetEnabled(body0, true));
    EXPECT_TRUE(IsEnabled(world, body0));
    EXPECT_NO_THROW(world.SetEnabled(body1, false));
    EXPECT_FALSE(IsEnabled(world, body1));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 1u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.SetEnabled(body0, false));
    EXPECT_FALSE(IsEnabled(world, body0));
    EXPECT_NO_THROW(world.SetEnabled(body1, true));
    EXPECT_TRUE(IsEnabled(world, body1));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 3u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.SetEnabled(body0, true));
    EXPECT_TRUE(IsEnabled(world, body0));
    EXPECT_NO_THROW(world.SetEnabled(body1, false));
    EXPECT_FALSE(IsEnabled(world, body1));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 5u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.SetEnabled(body0, false));
    EXPECT_FALSE(IsEnabled(world, body0));
    EXPECT_NO_THROW(world.SetEnabled(body1, true));
    EXPECT_TRUE(IsEnabled(world, body1));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 7u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.Step(stepConf));
    EXPECT_EQ(GetProxyCount(world, fixture0), 0u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 0u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.SetEnabled(body0, true));
    EXPECT_TRUE(IsEnabled(world, body0));
    EXPECT_EQ(world.GetFixturesForProxies().size(), 1u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);

    EXPECT_NO_THROW(world.Step(stepConf));
    EXPECT_EQ(GetProxyCount(world, fixture0), 1u);
    EXPECT_EQ(world.GetFixturesForProxies().size(), 0u);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);
}

TEST(Body, SetFixedRotation)
{
    auto world = World{};
    const auto body = world.CreateBody();
    const auto valid_shape = Shape{DiskShapeConf(1_m)};

    ASSERT_NE(world.CreateFixture(body, valid_shape, FixtureConf{}), InvalidFixtureID);
    ASSERT_FALSE(IsFixedRotation(world, body));

    // Test that set fixed rotation to flag already set is not a toggle
    SetFixedRotation(world, body, false);
    EXPECT_FALSE(IsFixedRotation(world, body));

    SetFixedRotation(world, body, true);
    EXPECT_TRUE(IsFixedRotation(world, body));
    SetFixedRotation(world, body, false);
    EXPECT_FALSE(IsFixedRotation(world, body));
}

TEST(Body, CreateAndDestroyFixture)
{
    auto world = World{};

    auto body = world.CreateBody();
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_TRUE(GetFixtures(world, body).empty());
    EXPECT_FALSE(IsMassDataDirty(world, body));

    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1_kgpm2;
    const auto shape = Shape(conf);
    
    {
        auto fixture = world.CreateFixture(body, shape, FixtureConf{}, false);
        const auto fshape = GetShape(world, fixture);
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(ShapeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
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
        EXPECT_TRUE(IsMassDataDirty(world, body));
        ResetMassData(world, body);
        EXPECT_FALSE(IsMassDataDirty(world, body));

        ASSERT_EQ(world.GetFixturesForProxies().size(), std::size_t{1});
        EXPECT_EQ(*world.GetFixturesForProxies().begin(), fixture);

        EXPECT_TRUE(world.Destroy(fixture, false));
        EXPECT_TRUE(GetFixtures(world, body).empty());
        EXPECT_TRUE(IsMassDataDirty(world, body));

        EXPECT_EQ(GetFixturesForProxies(world).size(), std::size_t(0));

        ResetMassData(world, body);
        EXPECT_FALSE(IsMassDataDirty(world, body));

        DestroyFixtures(world, body);
        EXPECT_TRUE(GetFixtures(world, body).empty());
    }
    {
        auto fixture = world.CreateFixture(body, shape, FixtureConf{}, false);
        const auto fshape = GetShape(world, fixture);
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(ShapeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
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
        EXPECT_TRUE(IsMassDataDirty(world, body));
        ResetMassData(world, body);
        EXPECT_FALSE(IsMassDataDirty(world, body));
        EXPECT_FALSE(GetFixtures(world, body).empty());
        
        world.DestroyFixtures(body);
        EXPECT_TRUE(GetFixtures(world, body).empty());
        EXPECT_FALSE(IsMassDataDirty(world, body));
    }
}

TEST(Body, SetType)
{
    auto bd = BodyConf{};
    bd.type = BodyType::Dynamic;
    auto world = World{};

    const auto body = world.CreateBody(bd);
    ASSERT_EQ(world.GetBodiesForProxies().size(), 0u);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);

    SetType(world, body, BodyType::Static);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 1u);
    EXPECT_EQ(GetType(world, body), BodyType::Static);

    SetType(world, body, BodyType::Kinematic);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 1u);
    EXPECT_EQ(GetType(world, body), BodyType::Kinematic);

    SetType(world, body, BodyType::Dynamic);
    EXPECT_EQ(GetType(world, body), BodyType::Dynamic);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 1u);
}

TEST(Body, StaticIsExpected)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Static));
    EXPECT_FALSE(IsAccelerable(world, body));
    EXPECT_FALSE(IsSpeedable(world, body));
    EXPECT_TRUE(IsImpenetrable(world, body));
}

TEST(Body, KinematicIsExpected)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Kinematic));
    EXPECT_FALSE(IsAccelerable(world, body));
    EXPECT_TRUE( IsSpeedable(world, body));
    EXPECT_TRUE( IsImpenetrable(world, body));
}

TEST(Body, DynamicIsExpected)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    EXPECT_TRUE(IsAccelerable(world, body));
    EXPECT_TRUE(IsSpeedable(world, body));
    EXPECT_FALSE(IsImpenetrable(world, body));
}

TEST(Body, SetMassData)
{
    const auto center = Length2{0_m, 0_m};
    const auto mass = 32_kg;
    const auto rotInertiaUnits = SquareMeter * Kilogram / SquareRadian;
    const auto rotInertia = 3 * rotInertiaUnits; // L^2 M QP^-2
    const auto massData = MassData{center, mass, rotInertia};
    
    // has effect on dynamic bodies...
    {
        auto world = World{};
        const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
        EXPECT_EQ(GetMass(world, body), 1_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
        SetMassData(world, body, massData);
        EXPECT_EQ(GetMass(world, body), mass);
        EXPECT_EQ(GetRotInertia(world, body), rotInertia);
    }
    
    // has no rotational effect on fixed rotation dynamic bodies...
    {
        auto world = World{};
        const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseFixedRotation(true));
        EXPECT_EQ(GetMass(world, body), 1_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
        SetMassData(world, body, massData);
        EXPECT_EQ(GetMass(world, body), mass);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
    }

    // has no effect on static bodies...
    {
        auto world = World{};
        const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Static));
        EXPECT_EQ(GetMass(world, body), 0_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
        SetMassData(world, body, massData);
        EXPECT_EQ(GetMass(world, body), 0_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
    }
}

TEST(Body, SetTransform)
{
    auto bd = BodyConf{};
    bd.type = BodyType::Dynamic;
    auto world = World{};
    ASSERT_EQ(world.GetBodiesForProxies().size(), 0u);
    
    const auto body = world.CreateBody(bd);
    const auto xfm1 = Transformation{Length2{}, UnitVec::GetRight()};
    ASSERT_EQ(GetTransformation(world, body), xfm1);
    ASSERT_EQ(world.GetBodiesForProxies().size(), 0u);

    const auto xfm2 = Transformation{Vec2(10, -12) * 1_m, UnitVec::GetLeft()};
    SetTransform(world, body, xfm2.p, GetAngle(xfm2.q));
    EXPECT_EQ(GetTransformation(world, body).p, xfm2.p);
    EXPECT_NEAR(static_cast<double>(GetX(GetTransformation(world, body).q)),
                static_cast<double>(GetX(xfm2.q)),
                0.001);
    EXPECT_NEAR(static_cast<double>(GetY(GetTransformation(world, body).q)),
                static_cast<double>(GetY(xfm2.q)),
                0.001);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 1u);
    
    world.Destroy(body);
    EXPECT_EQ(world.GetBodiesForProxies().size(), 0u);
}

TEST(Body, SetAcceleration)
{
    const auto someLinearAccel = LinearAcceleration2{2 * MeterPerSquareSecond, 3 * MeterPerSquareSecond};
    const auto someAngularAccel = 2 * RadianPerSquareSecond;

    {
        auto world = World{};
        const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Static));
        ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        ASSERT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        ASSERT_FALSE(IsAwake(world, body));
        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        
        SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));

        SetAcceleration(world, body, LinearAcceleration2{}, someAngularAccel);
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));

        SetAcceleration(world, body, someLinearAccel, AngularAcceleration{});
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));
    }
    
    // Kinematic and dynamic bodies awake at creation...
    {
        auto world = World{};
        const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Kinematic));
        ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        ASSERT_TRUE(IsAwake(world, body));
        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        
        SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));
        
        SetAcceleration(world, body, LinearAcceleration2{}, someAngularAccel);
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));
        
        SetAcceleration(world, body, someLinearAccel, AngularAcceleration{});
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));
    }
    
    // Dynamic bodies take a non-zero linear or angular acceleration.
    {
        auto world = World{};
        const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
        ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        ASSERT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        ASSERT_TRUE(IsAwake(world, body));
        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        
        SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_FALSE(IsAwake(world, body));
        
        SetAcceleration(world, body, LinearAcceleration2{}, someAngularAccel);
        EXPECT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel);
        EXPECT_TRUE(IsAwake(world, body));
        
        SetAcceleration(world, body, someLinearAccel, AngularAcceleration{});
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel);
        EXPECT_EQ(GetAngularAcceleration(world, body), 0 * RadianPerSquareSecond);
        EXPECT_TRUE(IsAwake(world, body));
        
        SetAcceleration(world, body, someLinearAccel, someAngularAccel);
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel);
        EXPECT_TRUE(IsAwake(world, body));

        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel);
        
        // Reseting to same acceleration shouldn't change asleep status...
        SetAcceleration(world, body, someLinearAccel, someAngularAccel);
        EXPECT_FALSE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel);
        
        // Seting to lower acceleration shouldn't change asleep status...
        SetAcceleration(world, body, someLinearAccel * 0.5f, someAngularAccel * 0.9f);
        EXPECT_FALSE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel * 0.5f);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel * 0.9f);

        // Seting to higher acceleration or new direction should awaken...
        SetAcceleration(world, body, someLinearAccel * 1.5f, someAngularAccel * 1.9f);
        EXPECT_TRUE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel * 1.5f);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel * 1.9f);
        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        SetAcceleration(world, body, someLinearAccel * 1.5f, someAngularAccel * 2.0f);
        EXPECT_TRUE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel * 1.5f);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel * 2.0f);
        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        SetAcceleration(world, body, someLinearAccel * 2.0f, someAngularAccel * 2.0f);
        EXPECT_TRUE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel * 2.0f);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel * 2.0f);
        UnsetAwake(world, body);
        ASSERT_FALSE(IsAwake(world, body));
        SetAcceleration(world, body, someLinearAccel * -1.0f, someAngularAccel * 2.0f);
        EXPECT_TRUE(IsAwake(world, body));
        EXPECT_EQ(GetLinearAcceleration(world, body), someLinearAccel * -1.0f);
        EXPECT_EQ(GetAngularAcceleration(world, body), someAngularAccel * 2.0f);
    }
}

TEST(Body, CreateLotsOfFixtures)
{
    auto bd = BodyConf{};
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
        auto world = World{};

        auto body = world.CreateBody(bd);
        ASSERT_NE(body, InvalidBodyID);
        EXPECT_TRUE(GetFixtures(world, body).empty());
        
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            auto fixture = world.CreateFixture(body, shape, FixtureConf{}, false);
            ASSERT_NE(fixture, InvalidFixtureID);
        }
        ResetMassData(world, body);
        
        EXPECT_FALSE(GetFixtures(world, body).empty());
        {
            int i = decltype(num){0};
            for (auto&& f: GetFixtures(world, body))
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
        auto world = World{};
        
        auto body = world.CreateBody(bd);
        ASSERT_NE(body, InvalidBodyID);
        EXPECT_TRUE(GetFixtures(world, body).empty());
        
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            auto fixture = world.CreateFixture(body, shape, FixtureConf{}, true);
            ASSERT_NE(fixture, InvalidFixtureID);
        }
        
        EXPECT_FALSE(GetFixtures(world, body).empty());
        {
            auto i = decltype(num){0};
            for (auto&& f: GetFixtures(world, body))
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
    auto world = World{};
    ASSERT_EQ(world.GetBodies().size(), std::size_t(0));
    const auto body0 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(1));
    EXPECT_EQ(GetWorldIndex(world, body0), BodyCounter(0));
    const auto body1 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(2));
    EXPECT_EQ(GetWorldIndex(world, body1), BodyCounter(1));
    const auto body2 = world.CreateBody();
    ASSERT_EQ(world.GetBodies().size(), std::size_t(3));
    EXPECT_EQ(GetWorldIndex(world, body2), BodyCounter(2));
    EXPECT_EQ(GetWorldIndex(world, InvalidBodyID), BodyCounter(-1));
}

TEST(Body, ApplyLinearAccelDoesNothingToStatic)
{
    auto world = World{};
    
    const auto body = world.CreateBody();
    ASSERT_NE(body, InvalidBodyID);
    ASSERT_FALSE(IsAwake(world, body));
    ASSERT_FALSE(IsSpeedable(world, body));
    ASSERT_FALSE(IsAccelerable(world, body));
    
    const auto zeroAccel = LinearAcceleration2{
        Real(0) * MeterPerSquareSecond, Real(0) * MeterPerSquareSecond
    };
    const auto linAccel = LinearAcceleration2{
        Real(2) * MeterPerSquareSecond, Real(2) * MeterPerSquareSecond
    };
    SetAcceleration(world, body, GetLinearAcceleration(world, body) + linAccel);
    EXPECT_NE(GetLinearAcceleration(world, body), linAccel);
    EXPECT_EQ(GetLinearAcceleration(world, body), zeroAccel);
}

TEST(Body, GetAccelerationFF)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
    
    ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
    ASSERT_EQ(GetAngularAcceleration(world, body), AngularAcceleration{});
    EXPECT_EQ(GetAcceleration(world, body), Acceleration());
}

TEST(Body, SetAccelerationFF)
{
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
    
    ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
    ASSERT_EQ(GetAngularAcceleration(world, body), AngularAcceleration{});
 
    const auto newAccel = Acceleration{
        LinearAcceleration2{2_mps2, 3_mps2}, AngularAcceleration{1.2f * RadianPerSquareSecond}
    };
    SetAcceleration(world, body, newAccel);
    EXPECT_EQ(GetAcceleration(world, body), newAccel);
}

TEST(Body, CalcGravitationalAcceleration)
{
    auto world = World{};

    const auto l1 = Length2{-8_m, 0_m};
    const auto l2 = Length2{+8_m, 0_m};
    const auto l3 = Length2{+16_m, 0_m};
    const auto shape = Shape{DiskShapeConf{}.UseRadius(2_m).UseDensity(1e10_kgpm2)};
    
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    world.CreateFixture(b1, shape);
    EXPECT_EQ(CalcGravitationalAcceleration(world, b1).linear, LinearAcceleration2());
    EXPECT_EQ(CalcGravitationalAcceleration(world, b1).angular, AngularAcceleration());

    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l2));
    world.CreateFixture(b2, shape);
    const auto accel = CalcGravitationalAcceleration(world, b1);
    EXPECT_NEAR(static_cast<double>(Real(GetX(accel.linear)/MeterPerSquareSecond)),
                0.032761313021183014, 0.032761313021183014/100);
    EXPECT_EQ(GetY(accel.linear), 0 * MeterPerSquareSecond);
    EXPECT_EQ(accel.angular, 0 * RadianPerSquareSecond);
    
    const auto b3 = world.CreateBody(BodyConf{}.UseType(BodyType::Static).UseLocation(l3));
    EXPECT_EQ(CalcGravitationalAcceleration(world, b3), Acceleration{});
}

TEST(Body, RotateAboutWorldPointFF)
{
    auto world = World{};
    const auto body = world.CreateBody();
    const auto locationA = GetLocation(world, body);
    ASSERT_EQ(locationA, Length2(0_m, 0_m));
    RotateAboutWorldPoint(world, body, 90_deg, Length2{2_m, 0_m});
    const auto locationB = GetLocation(world, body);
    EXPECT_NEAR(static_cast<double>(Real(GetX(locationB)/Meter)), +2.0, 0.001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(locationB)/Meter)), -2.0, 0.001);
}

TEST(Body, RotateAboutLocalPointFF)
{
    auto world = World{};
    const auto body = world.CreateBody();
    const auto locationA = GetLocation(world, body);
    ASSERT_EQ(locationA, Length2(0_m, 0_m));
    RotateAboutLocalPoint(world, body, 90_deg, Length2{2_m, 0_m});
    const auto locationB = GetLocation(world, body);
    EXPECT_NEAR(static_cast<double>(Real(GetX(locationB)/Meter)), +2.0, 0.001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(locationB)/Meter)), -2.0, 0.001);
}

TEST(Body, GetCentripetalForce)
{
    const auto l1 = Length2{-8_m, 0_m};
    auto world = World{};
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    const auto shape = Shape{DiskShapeConf{}.UseRadius(2_m).UseDensity(1_kgpm2)};
    world.CreateFixture(body, shape);
    SetVelocity(world, body, LinearVelocity2{2_mps, 3_mps});
    EXPECT_EQ(GetLinearVelocity(world, body), LinearVelocity2(2_mps, 3_mps));

    const auto force = GetCentripetalForce(world, body, Length2{1_m, 10_m});
    EXPECT_NEAR(static_cast<double>(Real(GetX(force)/Newton)), 8.1230141222476959, 0.01);
    EXPECT_NEAR(static_cast<double>(Real(GetY(force)/Newton)), 9.0255714952945709, 0.01);
}

TEST(Body, GetPositionFF)
{
    const auto position = Position{Length2{-33_m, +4_m}, 10_deg};
    auto world = World{};
    auto body = world.CreateBody();
    EXPECT_NE(GetPosition(world, body), position);
    SetLocation(world, body, position.linear);
    SetAngle(world, body, position.angular);
    EXPECT_EQ(GetPosition(world, body).linear, position.linear);
    EXPECT_NEAR(static_cast<double>(Real(GetPosition(world, body).angular / Degree)),
                static_cast<double>(Real(position.angular) / Degree),
                0.0001);
}

TEST(Body, GetSetTransformationFF)
{
    const auto xfm0 = Transformation{Length2{-33_m, +4_m}, UnitVec::GetTopRight()};
    auto world = World{};
    auto body = world.CreateBody();
    EXPECT_NE(GetTransformation(world, body), xfm0);
    SetTransformation(world, body, xfm0);
    const auto xfm1 = GetTransformation(world, body);
    EXPECT_EQ(xfm1.p, xfm0.p);
    EXPECT_NEAR(static_cast<double>(GetX(xfm1.q)), static_cast<double>(GetX(xfm0.q)), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(xfm1.q)), static_cast<double>(GetY(xfm0.q)), 0.0001);
}
