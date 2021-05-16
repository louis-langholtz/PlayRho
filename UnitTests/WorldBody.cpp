/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/World.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>

#include <chrono>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldBody, WorldCreated)
{
    auto world = World{};
    const auto body = CreateBody(world);
    ASSERT_NE(body, InvalidBodyID);
    EXPECT_TRUE(IsEnabled(world, body));
    EXPECT_FALSE(IsAwake(world, body));
    EXPECT_FALSE(IsSpeedable(world, body));
    EXPECT_FALSE(IsAccelerable(world, body));
    EXPECT_FALSE(Awaken(world, body));
    EXPECT_TRUE(GetShapes(world, body).empty());
    EXPECT_TRUE(GetJoints(world, body).empty());
    EXPECT_TRUE(GetContacts(world, body).empty());
}

TEST(WorldBody, SetVelocityDoesNothingToStatic)
{
    const auto zeroVelocity = Velocity{
        LinearVelocity2{0_mps, 0_mps},
        AngularVelocity{Real(0) * RadianPerSecond}
    };

    auto world = World{};

    const auto body = CreateBody(world);
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

TEST(WorldBody, CreateAttachShape)
{
    auto world = World{};
    const auto body = CreateBody(world);
    EXPECT_EQ(size(GetShapes(world, body)), std::size_t(0));

    const auto valid_shape = CreateShape(world, DiskShapeConf(1_m));
    ASSERT_NE(valid_shape, InvalidShapeID);
    EXPECT_NO_THROW(Attach(world, body, valid_shape));
    EXPECT_EQ(size(GetShapes(world, body)), std::size_t(1));

    const auto minRadius = GetMinVertexRadius(world);
    EXPECT_THROW(CreateShape(world, DiskShapeConf{minRadius / 2}), InvalidArgument);

    const auto maxRadius = GetMaxVertexRadius(world);
    EXPECT_THROW(CreateShape(world, DiskShapeConf{maxRadius + maxRadius / 10}), InvalidArgument);
}

TEST(WorldBody, Destroy)
{
    auto world = World{};
    EXPECT_THROW(Destroy(world, InvalidBodyID), std::out_of_range);
    auto bodyID = InvalidBodyID;
    EXPECT_NO_THROW(bodyID = CreateBody(world));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    EXPECT_NO_THROW(Destroy(world, bodyID));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));

    const auto bodyA = CreateBody(world);
    const auto bodyB = CreateBody(world);
    EXPECT_EQ(GetBodyCount(world), BodyCounter(2));
    ASSERT_EQ(GetShapeCount(world, bodyA), std::size_t(0));
    ASSERT_EQ(GetShapeCount(world, bodyB), std::size_t(0));

    const auto shapeId = CreateShape(world, Shape{DiskShapeConf(1_m)});
    ASSERT_NE(shapeId, InvalidShapeID);
    ASSERT_NO_THROW(Attach(world, bodyA, shapeId));
    ASSERT_EQ(GetShapeCount(world, bodyA), std::size_t(1));
    ASSERT_TRUE(Detach(world, bodyA, shapeId));
    EXPECT_EQ(GetShapeCount(world, bodyA), std::size_t(0));
}

TEST(WorldBody, SetEnabledCausesIsEnabled)
{
    auto world = World{};
    const auto body = CreateBody(world);
    ASSERT_TRUE(IsEnabled(world, body));
    auto value = true;
    for (auto i = 0; i < 4; ++i) {
        // Set and check twice to ensure same behavior if state already same.
        // Inlined to help match state with line number of any reports.
        EXPECT_NO_THROW(SetEnabled(world, body, value));
        EXPECT_EQ(IsEnabled(world, body), value);
        EXPECT_NO_THROW(SetEnabled(world, body, value));
        EXPECT_EQ(IsEnabled(world, body), value);
        value = !value;
    }
}

TEST(WorldBody, SetFixedRotation)
{
    auto world = World{};
    const auto body = CreateBody(world);
    const auto valid_shape = CreateShape(world, DiskShapeConf(1_m));

    ASSERT_NO_THROW(Attach(world, body, valid_shape));
    ASSERT_FALSE(IsFixedRotation(world, body));

    // Test that set fixed rotation to flag already set is not a toggle
    SetFixedRotation(world, body, false);
    EXPECT_FALSE(IsFixedRotation(world, body));

    SetFixedRotation(world, body, true);
    EXPECT_TRUE(IsFixedRotation(world, body));
    SetFixedRotation(world, body, false);
    EXPECT_FALSE(IsFixedRotation(world, body));
}

TEST(WorldBody, CreateAndDestroyFixture)
{
    auto world = World{};

    auto bodyId = CreateBody(world);
    ASSERT_NE(bodyId, InvalidBodyID);
    EXPECT_TRUE(GetShapes(world, bodyId).empty());
    EXPECT_FALSE(IsMassDataDirty(world, bodyId));

    auto conf = DiskShapeConf{};
    conf.vertexRadius = 2.871_m;
    conf.location = Vec2{1.912f, -77.31f} * 1_m;
    conf.density = 1_kgpm2;
    const auto shape = Shape(conf);
    
    {
        auto shapeId = CreateShape(world, shape);
        Attach(world, bodyId, shapeId, false);
        const auto fshape = GetShape(world, shapeId);
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(TypeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
        EXPECT_FALSE(GetShapes(world, bodyId).empty());
        {
            auto i = 0;
            for (const auto& f: GetShapes(world, bodyId))
            {
                EXPECT_EQ(f, shapeId);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(IsMassDataDirty(world, bodyId));
        ResetMassData(world, bodyId);
        EXPECT_FALSE(IsMassDataDirty(world, bodyId));

        EXPECT_NO_THROW(world.Destroy(shapeId));
        EXPECT_TRUE(GetShapes(world, bodyId).empty());
        EXPECT_TRUE(IsMassDataDirty(world, bodyId));

        ResetMassData(world, bodyId);
        EXPECT_FALSE(IsMassDataDirty(world, bodyId));

        Detach(world, bodyId);
        EXPECT_TRUE(GetShapes(world, bodyId).empty());
    }
    {
        auto shapeId = CreateShape(world, shape);
        Attach(world, bodyId, shapeId, false);
        const auto fshape = GetShape(world, shapeId);
        EXPECT_EQ(GetVertexRadius(fshape, 0), GetVertexRadius(shape, 0));
        EXPECT_EQ(TypeCast<DiskShapeConf>(fshape).GetLocation(), conf.GetLocation());
        EXPECT_FALSE(GetShapes(world, bodyId).empty());
        {
            auto i = 0;
            for (const auto& f: GetShapes(world, bodyId)) {
                EXPECT_EQ(f, shapeId);
                ++i;
            }
            EXPECT_EQ(i, 1);
        }
        EXPECT_TRUE(IsMassDataDirty(world, bodyId));
        ResetMassData(world, bodyId);
        EXPECT_FALSE(IsMassDataDirty(world, bodyId));
        EXPECT_FALSE(GetShapes(world, bodyId).empty());
        
        Detach(world, bodyId);
        EXPECT_TRUE(GetShapes(world, bodyId).empty());
        EXPECT_FALSE(IsMassDataDirty(world, bodyId));
    }
}

TEST(WorldBody, SetType)
{
    auto world = World{};

    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);
    ASSERT_EQ(GetType(world, body), BodyType::Dynamic);

    SetType(world, body, BodyType::Static);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    EXPECT_EQ(GetType(world, body), BodyType::Static);

    SetType(world, body, BodyType::Kinematic);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    EXPECT_EQ(GetType(world, body), BodyType::Kinematic);

    SetType(world, body, BodyType::Dynamic);
    EXPECT_EQ(GetType(world, body), BodyType::Dynamic);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
}

TEST(WorldBody, StaticIsExpected)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Static));
    EXPECT_FALSE(IsAccelerable(world, body));
    EXPECT_FALSE(IsSpeedable(world, body));
    EXPECT_TRUE(IsImpenetrable(world, body));
}

TEST(WorldBody, KinematicIsExpected)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Kinematic));
    EXPECT_FALSE(IsAccelerable(world, body));
    EXPECT_TRUE( IsSpeedable(world, body));
    EXPECT_TRUE( IsImpenetrable(world, body));
}

TEST(WorldBody, DynamicIsExpected)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    EXPECT_TRUE(IsAccelerable(world, body));
    EXPECT_TRUE(IsSpeedable(world, body));
    EXPECT_FALSE(IsImpenetrable(world, body));
}

TEST(WorldBody, SetMassData)
{
    const auto center = Length2{0_m, 0_m};
    const auto mass = 32_kg;
    const auto rotInertiaUnits = SquareMeter * Kilogram / SquareRadian;
    const auto rotInertia = 3 * rotInertiaUnits; // L^2 M QP^-2
    const auto massData = MassData{center, mass, rotInertia};
    
    // has effect on dynamic bodies...
    {
        auto world = World{};
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
        EXPECT_EQ(GetMass(world, body), 1_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
        SetMassData(world, body, massData);
        EXPECT_EQ(GetMass(world, body), mass);
        EXPECT_EQ(GetRotInertia(world, body), rotInertia);
    }
    
    // has no rotational effect on fixed rotation dynamic bodies...
    {
        auto world = World{};
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseFixedRotation(true));
        EXPECT_EQ(GetMass(world, body), 1_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
        SetMassData(world, body, massData);
        EXPECT_EQ(GetMass(world, body), mass);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
    }

    // has no effect on static bodies...
    {
        auto world = World{};
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Static));
        EXPECT_EQ(GetMass(world, body), 0_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
        SetMassData(world, body, massData);
        EXPECT_EQ(GetMass(world, body), 0_kg);
        EXPECT_EQ(GetRotInertia(world, body), std::numeric_limits<Real>::infinity() * rotInertiaUnits);
    }
}

TEST(WorldBody, SetTransform)
{
    auto bd = BodyConf{};
    bd.type = BodyType::Dynamic;
    auto world = World{};
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);
    
    const auto body = CreateBody(world, bd);
    const auto xfm1 = Transformation{Length2{}, UnitVec::GetRight()};
    ASSERT_EQ(GetTransformation(world, body), xfm1);
    ASSERT_EQ(GetBodiesForProxies(world).size(), 0u);

    const auto xfm2 = Transformation{Vec2(10, -12) * 1_m, UnitVec::GetLeft()};
    SetTransform(world, body, xfm2.p, GetAngle(xfm2.q));
    EXPECT_EQ(GetTransformation(world, body).p, xfm2.p);
    EXPECT_NEAR(static_cast<double>(GetX(GetTransformation(world, body).q)),
                static_cast<double>(GetX(xfm2.q)),
                0.001);
    EXPECT_NEAR(static_cast<double>(GetY(GetTransformation(world, body).q)),
                static_cast<double>(GetY(xfm2.q)),
                0.001);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 1u);
    
    world.Destroy(body);
    EXPECT_EQ(GetBodiesForProxies(world).size(), 0u);
}

TEST(WorldBody, SetAcceleration)
{
    const auto someLinearAccel = LinearAcceleration2{2 * MeterPerSquareSecond, 3 * MeterPerSquareSecond};
    const auto someAngularAccel = 2 * RadianPerSquareSecond;

    {
        auto world = World{};
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Static));
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
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Kinematic));
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
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
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

TEST(WorldBody, SetAngularAcceleration)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    auto acceleration = AngularAcceleration{};
    acceleration = Real(2) * RadianPerSquareSecond;
    EXPECT_NO_THROW(SetAcceleration(world, body, acceleration));
    EXPECT_EQ(GetAngularAcceleration(world, body), acceleration);
    acceleration = Real(3) * RadianPerSquareSecond;
    EXPECT_NO_THROW(SetAcceleration(world, body, acceleration));
    EXPECT_EQ(GetAngularAcceleration(world, body), acceleration);
}

TEST(WorldBody, SetAngularVelocity)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    auto value = AngularVelocity{};
    value = Real(4) * RadianPerSecond;
    EXPECT_NO_THROW(SetVelocity(world, body, value));
    EXPECT_EQ(GetAngularVelocity(world, body), value);
    value = Real(5) * RadianPerSecond;
    EXPECT_NO_THROW(SetVelocity(world, body, value));
    EXPECT_EQ(GetAngularVelocity(world, body), value);
}

TEST(WorldBody, ApplyForce)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, PolygonShapeConf(1_m, 1_m).UseDensity(1_kgpm2));
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, body, shapeId);
    ASSERT_EQ(GetMass(world, body), 4_kg);
    auto value = Force2{};
    value = Force2{4_N, 4_N};
    EXPECT_NO_THROW(ApplyForce(world, body, value, GetWorldCenter(world, body)));
    EXPECT_EQ(GetX(GetAcceleration(world, body).linear), LinearAcceleration(1_mps2));
    EXPECT_EQ(GetY(GetAcceleration(world, body).linear), LinearAcceleration(1_mps2));
    EXPECT_EQ(GetAcceleration(world, body).angular, AngularAcceleration());
}

TEST(WorldBody, ApplyTorque)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, PolygonShapeConf(1_m, 1_m).UseDensity(1_kgpm2));
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, body, shapeId);
    ASSERT_EQ(GetMass(world, body), 4_kg);
    auto value = Torque{};
    value = 4_kg * SquareMeter / SquareSecond / Radian;
    EXPECT_NO_THROW(ApplyTorque(world, body, value));
    EXPECT_EQ(GetX(GetAcceleration(world, body).linear), LinearAcceleration(0_mps2));
    EXPECT_EQ(GetY(GetAcceleration(world, body).linear), LinearAcceleration(0_mps2));
    EXPECT_EQ(GetAcceleration(world, body).angular,
              AngularAcceleration(Real(1.5) * Radian / SquareSecond));
}

TEST(WorldBody, ApplyLinearImpulse)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, PolygonShapeConf(1_m, 1_m).UseDensity(1_kgpm2));
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, body, shapeId);
    ASSERT_EQ(GetMass(world, body), 4_kg);
    auto value = Momentum2{40_Ns, 0_Ns};
    EXPECT_NO_THROW(ApplyLinearImpulse(world, body, value, GetWorldCenter(world, body)));
    EXPECT_EQ(GetX(GetVelocity(world, body).linear), LinearVelocity(10_mps));
    EXPECT_EQ(GetY(GetVelocity(world, body).linear), LinearVelocity(0_mps));
    EXPECT_EQ(GetVelocity(world, body).angular, AngularVelocity(0_rpm));
}

TEST(WorldBody, ApplyAngularImpulse)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, PolygonShapeConf(1_m, 1_m).UseDensity(1_kgpm2));
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, body, shapeId);
    ASSERT_EQ(GetMass(world, body), 4_kg);
    auto value = AngularMomentum{Real(8) * NewtonMeterSecond};
    EXPECT_NO_THROW(ApplyAngularImpulse(world, body, value));
    EXPECT_EQ(GetX(GetVelocity(world, body).linear), LinearVelocity(0_mps));
    EXPECT_EQ(GetY(GetVelocity(world, body).linear), LinearVelocity(0_mps));
    EXPECT_EQ(GetVelocity(world, body).angular, AngularVelocity(Real(3) * RadianPerSecond));
}

TEST(WorldBody, CreateLotsOfFixtures)
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

        const auto shapeId = CreateShape(world, shape);
        auto body = CreateBody(world, bd);
        ASSERT_NE(body, InvalidBodyID);
        EXPECT_TRUE(GetShapes(world, body).empty());
        
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            ASSERT_NO_THROW(Attach(world, body, shapeId, false));
        }
        ResetMassData(world, body);
        
        EXPECT_FALSE(GetShapes(world, body).empty());
        {
            int i = decltype(num){0};
            for (auto&& f: GetShapes(world, body))
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
        
        const auto shapeId = CreateShape(world, shape);
        auto body = CreateBody(world, bd);
        ASSERT_NE(body, InvalidBodyID);
        EXPECT_TRUE(GetShapes(world, body).empty());
        
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            ASSERT_NO_THROW(Attach(world, body, shapeId, true));
        }
        
        EXPECT_FALSE(GetShapes(world, body).empty());
        {
            auto i = decltype(num){0};
            for (auto&& f: GetShapes(world, body))
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

TEST(WorldBody, GetWorldIndex)
{
    auto world = World{};
    ASSERT_EQ(GetBodies(world).size(), std::size_t(0));
    const auto body0 = CreateBody(world);
    ASSERT_EQ(GetBodies(world).size(), std::size_t(1));
    EXPECT_EQ(GetWorldIndex(world, body0), BodyCounter(0));
    const auto body1 = CreateBody(world);
    ASSERT_EQ(GetBodies(world).size(), std::size_t(2));
    EXPECT_EQ(GetWorldIndex(world, body1), BodyCounter(1));
    const auto body2 = CreateBody(world);
    ASSERT_EQ(GetBodies(world).size(), std::size_t(3));
    EXPECT_EQ(GetWorldIndex(world, body2), BodyCounter(2));
    EXPECT_EQ(GetWorldIndex(world, InvalidBodyID), BodyCounter(-1));
}

TEST(WorldBody, ApplyLinearAccelDoesNothingToStatic)
{
    auto world = World{};
    
    const auto body = CreateBody(world);
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

TEST(WorldBody, GetAccelerationFF)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
    
    ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
    ASSERT_EQ(GetAngularAcceleration(world, body), AngularAcceleration{});
    EXPECT_EQ(GetAcceleration(world, body), Acceleration());
}

TEST(WorldBody, SetAccelerationFF)
{
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    SetAcceleration(world, body, LinearAcceleration2{}, AngularAcceleration{});
    
    ASSERT_EQ(GetLinearAcceleration(world, body), LinearAcceleration2{});
    ASSERT_EQ(GetAngularAcceleration(world, body), AngularAcceleration{});
 
    const auto newAccel = Acceleration{
        LinearAcceleration2{2_mps2, 3_mps2}, AngularAcceleration{1.2f * RadianPerSquareSecond}
    };
    SetAcceleration(world, body, newAccel);
    EXPECT_EQ(GetAcceleration(world, body), newAccel);
}

TEST(WorldBody, CalcGravitationalAcceleration)
{
    auto world = World{};

    const auto l1 = Length2{-8_m, 0_m};
    const auto l2 = Length2{+8_m, 0_m};
    const auto l3 = Length2{+16_m, 0_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(2_m).UseDensity(1e10_kgpm2));
    
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    Attach(world, b1, shapeId);
    EXPECT_EQ(CalcGravitationalAcceleration(world, b1).linear, LinearAcceleration2());
    EXPECT_EQ(CalcGravitationalAcceleration(world, b1).angular, AngularAcceleration());

    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l2));
    Attach(world, b2, shapeId);
    const auto accel = CalcGravitationalAcceleration(world, b1);
    EXPECT_NEAR(static_cast<double>(Real(GetX(accel.linear)/MeterPerSquareSecond)),
                0.032761313021183014, 0.032761313021183014/100);
    EXPECT_EQ(GetY(accel.linear), 0 * MeterPerSquareSecond);
    EXPECT_EQ(accel.angular, 0 * RadianPerSquareSecond);
    
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Static).UseLocation(l3));
    EXPECT_EQ(CalcGravitationalAcceleration(world, b3), Acceleration{});
}

TEST(WorldBody, RotateAboutWorldPointFF)
{
    auto world = World{};
    const auto body = CreateBody(world);
    const auto locationA = GetLocation(world, body);
    ASSERT_EQ(locationA, Length2(0_m, 0_m));
    RotateAboutWorldPoint(world, body, 90_deg, Length2{2_m, 0_m});
    const auto locationB = GetLocation(world, body);
    EXPECT_NEAR(static_cast<double>(Real(GetX(locationB)/Meter)), +2.0, 0.001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(locationB)/Meter)), -2.0, 0.001);
}

TEST(WorldBody, RotateAboutLocalPointFF)
{
    auto world = World{};
    const auto body = CreateBody(world);
    const auto locationA = GetLocation(world, body);
    ASSERT_EQ(locationA, Length2(0_m, 0_m));
    RotateAboutLocalPoint(world, body, 90_deg, Length2{2_m, 0_m});
    const auto locationB = GetLocation(world, body);
    EXPECT_NEAR(static_cast<double>(Real(GetX(locationB)/Meter)), +2.0, 0.001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(locationB)/Meter)), -2.0, 0.001);
}

TEST(WorldBody, GetCentripetalForce)
{
    const auto l1 = Length2{-8_m, 0_m};
    auto world = World{};
    const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(l1));
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(2_m).UseDensity(1_kgpm2));
    Attach(world, body, shapeId);
    SetVelocity(world, body, LinearVelocity2{2_mps, 3_mps});
    EXPECT_EQ(GetLinearVelocity(world, body), LinearVelocity2(2_mps, 3_mps));

    const auto force = GetCentripetalForce(world, body, Length2{1_m, 10_m});
    EXPECT_NEAR(static_cast<double>(Real(GetX(force)/Newton)), 8.1230141222476959, 0.01);
    EXPECT_NEAR(static_cast<double>(Real(GetY(force)/Newton)), 9.0255714952945709, 0.01);
}

TEST(WorldBody, GetPositionFF)
{
    const auto position = Position{Length2{-33_m, +4_m}, 10_deg};
    auto world = World{};
    auto body = CreateBody(world);
    EXPECT_NE(GetPosition(world, body), position);
    SetLocation(world, body, position.linear);
    SetAngle(world, body, position.angular);
    EXPECT_EQ(GetPosition(world, body).linear, position.linear);
    EXPECT_NEAR(static_cast<double>(Real(GetPosition(world, body).angular / Degree)),
                static_cast<double>(Real(position.angular / Degree)),
                0.0001);
}

TEST(WorldBody, GetSetTransformationFF)
{
    const auto xfm0 = Transformation{Length2{-33_m, +4_m}, UnitVec::GetTopRight()};
    auto world = World{};
    auto body = CreateBody(world);
    EXPECT_NE(GetTransformation(world, body), xfm0);
    SetTransformation(world, body, xfm0);
    const auto xfm1 = GetTransformation(world, body);
    EXPECT_EQ(xfm1.p, xfm0.p);
    EXPECT_NEAR(static_cast<double>(GetX(xfm1.q)), static_cast<double>(GetX(xfm0.q)), 0.0001);
    EXPECT_NEAR(static_cast<double>(GetY(xfm1.q)), static_cast<double>(GetY(xfm0.q)), 0.0001);
}

TEST(WorldBody, SetAwake)
{
    {
        World world;
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
        EXPECT_NO_THROW(UnsetAwake(world, body));
        EXPECT_FALSE(IsAwake(world, body));
        EXPECT_NO_THROW(SetAwake(world, body));
        EXPECT_TRUE(IsAwake(world, body));
    }
    {
        World world;
        const auto body = CreateBody(world, BodyConf{}.UseType(BodyType::Static));
        EXPECT_NO_THROW(UnsetAwake(world, body));
        EXPECT_FALSE(IsAwake(world, body));
        EXPECT_NO_THROW(SetAwake(world, body));
        EXPECT_FALSE(IsAwake(world, body)); // because Static, !IsSpeedable
    }
}

TEST(WorldBody, GetBodyRange)
{
    auto world = World{};
    auto body = InvalidBodyID;
    EXPECT_EQ(GetBodyRange(world), BodyCounter(0));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));

    ASSERT_NO_THROW(body = CreateBody(world));
    EXPECT_EQ(GetBodyRange(world), BodyCounter(1));
    for (auto i = 1; i < 10; ++i) {
        ASSERT_NO_THROW(CreateBody(world));
    }
    EXPECT_EQ(GetBodyRange(world), BodyCounter(10));

    ASSERT_NO_THROW(Destroy(world, body));
    EXPECT_EQ(GetBodyRange(world), BodyCounter(10));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(9));

    ASSERT_NO_THROW(body = CreateBody(world));
    EXPECT_EQ(GetBodyRange(world), BodyCounter(10));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(10));

    ASSERT_NO_THROW(Clear(world));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    EXPECT_EQ(GetBodyRange(world), BodyCounter(0));

    ASSERT_NO_THROW(body = CreateBody(world));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(1));
    EXPECT_EQ(GetBodyRange(world), BodyCounter(1));

    ASSERT_NO_THROW(Destroy(world, body));
    EXPECT_EQ(GetBodyCount(world), BodyCounter(0));
    EXPECT_EQ(GetBodyRange(world), BodyCounter(1));
}
