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

#include <playrho/d2/Body.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Body, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.

    // architecture dependent...
    switch (sizeof(Real)) {
    case 4:
#if defined(_WIN64)
#if !defined(NDEBUG)
        EXPECT_EQ(sizeof(Body), std::size_t(136));
#else
        EXPECT_EQ(sizeof(Body), std::size_t(128));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
        // Win32 debug
        EXPECT_EQ(sizeof(Body), std::size_t(116));
#else
        // Win32 release
        EXPECT_EQ(sizeof(Body), std::size_t(112));
#endif
#else
        EXPECT_EQ(sizeof(Body), std::size_t(128));
#endif
        break;
    case 8:
        EXPECT_EQ(sizeof(Body), std::size_t(224));
        break;
    case 16:
        EXPECT_EQ(sizeof(Body), std::size_t(432));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(Body, DefaultConstruction)
{
    EXPECT_EQ(Body().GetType(), BodyType::Static);
    EXPECT_TRUE(Body().IsEnabled());
    EXPECT_FALSE(Body().IsAwake());
    EXPECT_FALSE(Body().IsSpeedable());
    EXPECT_EQ(Body().GetLinearDamping(), Body::DefaultLinearDamping);
    EXPECT_EQ(Body().GetAngularDamping(), Body::DefaultAngularDamping);
}

TEST(Body, GetFlagsForBodyType)
{
    EXPECT_EQ(Body::GetFlags(BodyType::Static), (Body::e_impenetrableFlag));
    EXPECT_EQ(Body::GetFlags(BodyType::Kinematic),
              (Body::e_impenetrableFlag | Body::e_velocityFlag));
    EXPECT_EQ(Body::GetFlags(BodyType::Dynamic), (Body::e_accelerationFlag | Body::e_velocityFlag));
}

TEST(Body, GetFlagsForBodyConf)
{
    EXPECT_TRUE(Body::GetFlags(BodyConf{}.UseFixedRotation(true)) & Body::e_fixedRotationFlag);
    EXPECT_TRUE(
        Body::GetFlags(BodyConf{}.UseAwake(false).UseAllowSleep(false).UseType(BodyType::Dynamic)) &
        Body::e_awakeFlag);
}

TEST(Body, ShapeOnConstruction)
{
    const auto shapeId = ShapeID(1u);
    ASSERT_TRUE(!empty(Body(BodyConf{}.Use(shapeId)).GetShapes()));
    ASSERT_EQ(size(Body(BodyConf{}.Use(shapeId)).GetShapes()), 1u);
    EXPECT_EQ(Body(BodyConf{}.Use(shapeId)).GetShapes()[0], shapeId);
}

TEST(Body, LinearDampingOnConstruction)
{
    EXPECT_EQ(Body(BodyConf{}.UseLinearDamping(0_Hz)).GetLinearDamping(), 0_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseLinearDamping(20_Hz)).GetLinearDamping(), 20_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseLinearDamping(30_Hz)).GetLinearDamping(), 30_Hz);
}

TEST(Body, AngularDampingOnConstruction)
{
    EXPECT_EQ(Body(BodyConf{}.UseAngularDamping(0_Hz)).GetAngularDamping(), 0_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseAngularDamping(20_Hz)).GetAngularDamping(), 20_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseAngularDamping(30_Hz)).GetAngularDamping(), 30_Hz);
}

TEST(Body, InvMassOnConstruction)
{
    EXPECT_EQ(Body(BodyConf{}.UseType(BodyType::Dynamic)).GetInvMass(), Real(1) / 1_kg);
    EXPECT_EQ(Body(BodyConf{}.UseType(BodyType::Kinematic)).GetInvMass(), Real(0) / 1_kg);
    EXPECT_EQ(Body(BodyConf{}.UseType(BodyType::Static)).GetInvMass(), Real(0) / 1_kg);
}

TEST(Body, TransformationOnConstruction)
{
    EXPECT_EQ(
        Body(BodyConf{}.UseLocation(Length2{10_m, 12_m}).UseAngle(90_deg)).GetTransformation(),
        ::playrho::d2::GetTransformation(
            BodyConf{}.UseLocation(Length2{10_m, 12_m}).UseAngle(90_deg)));
    EXPECT_EQ(
        Body(BodyConf{}.UseLocation(Length2{4_m, -3_m}).UseAngle(-32_deg)).GetTransformation(),
        ::playrho::d2::GetTransformation(
            BodyConf{}.UseLocation(Length2{4_m, -3_m}).UseAngle(-32_deg)));
}

TEST(Body, VelocityOnConstruction)
{
    auto body = Body{};
    body.SetVelocity(Velocity{LinearVelocity2{1_mps, 2_mps}, 3_rpm});
    EXPECT_EQ(
        Body(BodyConf{}.Use(Velocity{LinearVelocity2{1_mps, 2_mps}, 3_rpm})).GetVelocity().linear,
        body.GetVelocity().linear);
    EXPECT_EQ(
        Body(BodyConf{}.Use(Velocity{LinearVelocity2{1_mps, 2_mps}, 3_rpm})).GetVelocity().angular,
        body.GetVelocity().angular);
}

TEST(Body, AccelerationOnConstruction)
{
    auto body = Body{};
    body.SetAcceleration(LinearAcceleration2{2_mps2, 3_mps2}, Real(4) * RadianPerSquareSecond);
    EXPECT_EQ(Body(BodyConf{}
                       .UseLinearAcceleration(LinearAcceleration2{2_mps2, 3_mps2})
                       .UseAngularAcceleration(Real(4) * RadianPerSquareSecond))
                  .GetLinearAcceleration(),
              body.GetLinearAcceleration());
    EXPECT_EQ(Body(BodyConf{}
                       .UseLinearAcceleration(LinearAcceleration2{2_mps2, 3_mps2})
                       .UseAngularAcceleration(Real(4) * RadianPerSquareSecond))
                  .GetAngularAcceleration(),
              body.GetAngularAcceleration());
}

TEST(Body, EqualsOperator)
{
    EXPECT_TRUE(Body() == Body());
    {
        auto body = Body{};
        EXPECT_TRUE(body == Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_TRUE(body1 == body2);
    }
    {
        auto body = Body{};
        SetTransformation(body, Transformation{Length2{2_m, 0_m}, UnitVec{}});
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetSweep(Sweep{Position{Length2{}, 2_deg}});
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        body.JustSetVelocity(Velocity{LinearVelocity2{}, 2_rpm});
        EXPECT_FALSE(body == Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetAcceleration(LinearAcceleration2{}, Real(2) * RadianPerSquareSecond);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_FALSE(body1 == body2);
    }
    {
        auto body = Body{};
        SetMass(body, 3.2_kg);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetInvMassData(body.GetInvMass(), (Real(2) * SquareRadian) / (2_m2 * 1.2_kg));
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        SetLinearDamping(body, 2_Hz);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        SetAngularDamping(body, 2_Hz);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetUnderActiveTime(2_s);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_FALSE(body1 == body2);
    }
}

TEST(Body, NotEqualsOperator)
{
    EXPECT_FALSE(Body() != Body());
    {
        auto body = Body{};
        EXPECT_FALSE(body != Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_FALSE(body1 != body2);
    }
    {
        auto body = Body{};
        SetTransformation(body, Transformation{Length2{2_m, 0_m}, UnitVec{}});
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetSweep(Sweep{Position{Length2{}, 2_deg}});
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        body.JustSetVelocity(Velocity{LinearVelocity2{}, 2_rpm});
        EXPECT_TRUE(body != Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetAcceleration(LinearAcceleration2{}, Real(2) * RadianPerSquareSecond);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_TRUE(body1 != body2);
    }
    {
        auto body = Body{};
        SetMass(body, 3.2_kg);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetInvMassData(body.GetInvMass(), (Real(2) * SquareRadian) / (2_m2 * 1.2_kg));
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        SetLinearDamping(body, 2_Hz);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        SetAngularDamping(body, 2_Hz);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetUnderActiveTime(2_s);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_TRUE(body1 != body2);
    }
}
