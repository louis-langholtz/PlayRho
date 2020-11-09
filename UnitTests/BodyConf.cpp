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

#include <PlayRho/Dynamics/BodyConf.hpp>

#include <PlayRho/Dynamics/Body.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(BodyConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(BodyConf), 60u);
        break;
    case 8:
        EXPECT_EQ(sizeof(BodyConf), 112u);
        break;
    case 16:
        EXPECT_EQ(sizeof(BodyConf), 224u);
        break;
    }
}

TEST(BodyConf, UseType)
{
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Static).type, BodyType::Static);
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Dynamic).type, BodyType::Dynamic);
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Kinematic).type, BodyType::Kinematic);
}

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

static void IsSame(const BodyConf& conf, const BodyConf& conf2)
{
    EXPECT_EQ(conf.type, conf2.type);
    EXPECT_EQ(conf.location, conf2.location);
    EXPECT_EQ(conf.angle, conf2.angle);
    EXPECT_EQ(conf.linearVelocity, conf2.linearVelocity);
    EXPECT_EQ(conf.angularVelocity, conf2.angularVelocity);
    EXPECT_EQ(conf.linearAcceleration, conf2.linearAcceleration);
    EXPECT_EQ(conf.angularAcceleration, conf2.angularAcceleration);
    EXPECT_EQ(conf.linearDamping, conf2.linearDamping);
    EXPECT_EQ(conf.angularDamping, conf2.angularDamping);
    EXPECT_EQ(conf.underActiveTime, conf2.underActiveTime);
    EXPECT_EQ(conf.allowSleep, conf2.allowSleep);
    EXPECT_EQ(conf.awake, conf2.awake);
    EXPECT_EQ(conf.fixedRotation, conf2.fixedRotation);
    EXPECT_EQ(conf.bullet, conf2.bullet);
    EXPECT_EQ(conf.enabled, conf2.enabled);
}

TEST(BodyConf, GetBodyConf1)
{
    auto conf = BodyConf{};
    conf.type = BodyType::Static;
    conf.awake = false;
    SCOPED_TRACE("checking");
    EXPECT_NO_THROW(IsSame(conf, GetBodyConf(Body(conf))));
}

TEST(BodyConf, GetBodyConf2)
{
    auto conf = BodyConf{};
    conf.type = BodyType::Dynamic;
    conf.location = Length2{2_m, 3_m};
    conf.angle = 30_deg;
    conf.linearVelocity = LinearVelocity2{2_mps, 0_mps};
    conf.angularVelocity = 4_rpm;
    conf.linearAcceleration = LinearAcceleration2{2_mps2, 0_mps2};
    conf.angularAcceleration = 2_rpm / Second;
    conf.linearDamping = 2_Hz;
    conf.angularDamping = 3_Hz;
    conf.underActiveTime = 50_s;
    conf.allowSleep = false;
    conf.awake = true;
    conf.fixedRotation = true;
    conf.bullet = true;
    conf.enabled = false;
    SCOPED_TRACE("checking");
    EXPECT_NO_THROW(IsSame(conf, GetBodyConf(Body(conf))));
}
