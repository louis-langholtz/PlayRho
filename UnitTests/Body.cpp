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

#include <PlayRho/Dynamics/Body.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Body, ByteSize)
{
    // architecture dependent...
    switch (sizeof(Real)) {
    case 4:
#if defined(_WIN64)
#if !defined(NDEBUG)
        EXPECT_EQ(sizeof(Body), std::size_t(216));
#else
        EXPECT_EQ(sizeof(Body), std::size_t(100));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
        // Win32 debug
        EXPECT_EQ(sizeof(Body), std::size_t(192));
#else
        // Win32 release
        EXPECT_EQ(sizeof(Body), std::size_t(100));
#endif
#else
        EXPECT_EQ(sizeof(Body), std::size_t(100));
#endif
        break;
    case 8:
        EXPECT_EQ(sizeof(Body), std::size_t(200));
        break;
    case 16:
        EXPECT_EQ(sizeof(Body), std::size_t(400));
        break;
    default:
        FAIL();
        break;
    }
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
