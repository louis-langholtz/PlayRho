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

#include <playrho/d2/Body.hpp>
#include <playrho/d2/BodyConf.hpp>

#include "gtest/gtest.h"

using namespace playrho;
using namespace playrho::d2;

namespace {

void IsSame(const BodyConf& conf, const BodyConf& conf2)
{
    EXPECT_EQ(conf.type, conf2.type);
    EXPECT_EQ(conf.sweep, conf2.sweep);
    EXPECT_EQ(conf.invMass, conf2.invMass);
    EXPECT_EQ(conf.invRotI, conf2.invRotI);
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

}

TEST(BodyConf, DefaultConstruction)
{
    EXPECT_EQ(BodyConf().type, BodyConf::DefaultBodyType);
    EXPECT_EQ(BodyConf().sweep, BodyConf::DefaultSweep);
    EXPECT_EQ(BodyConf().invMass, BodyConf::DefaultInvMass);
    EXPECT_EQ(BodyConf().invRotI, BodyConf::DefaultInvRotI);
    EXPECT_EQ(BodyConf().linearVelocity, BodyConf::DefaultLinearVelocity);
    EXPECT_EQ(BodyConf().angularVelocity, BodyConf::DefaultAngularVelocity);
    EXPECT_EQ(BodyConf().linearAcceleration, BodyConf::DefaultLinearAcceleration);
    EXPECT_EQ(BodyConf().angularAcceleration, BodyConf::DefaultAngularAcceleration);
    EXPECT_EQ(BodyConf().linearDamping, BodyConf::DefaultLinearDamping);
    EXPECT_EQ(BodyConf().angularDamping, BodyConf::DefaultAngularDamping);
    EXPECT_EQ(BodyConf().underActiveTime, BodyConf::DefaultUnderActiveTime);
    EXPECT_EQ(BodyConf().shapes.size(), 0u);
    EXPECT_EQ(BodyConf().allowSleep, BodyConf::DefaultAllowSleep);
    EXPECT_EQ(BodyConf().awake, BodyConf::DefaultAwake);
    EXPECT_EQ(BodyConf().fixedRotation, BodyConf::DefaultFixedRotation);
    EXPECT_EQ(BodyConf().bullet, BodyConf::DefaultBullet);
    EXPECT_EQ(BodyConf().enabled, BodyConf::DefaultEnabled);
}

TEST(BodyConf, UseType)
{
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Static).type, BodyType::Static);
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Dynamic).type, BodyType::Dynamic);
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Kinematic).type, BodyType::Kinematic);
}

TEST(BodyConf, UseInvMass)
{
    const auto v = InvMass{Real(2) / 1_kg};
    EXPECT_EQ(BodyConf{}.UseInvMass(v).invMass, v);
}

TEST(BodyConf, UseInvRotI)
{
    const auto v = InvRotInertia{Real{4} * SquareRadian / (SquareMeter * 1_kg)};
    EXPECT_EQ(BodyConf{}.UseInvRotI(v).invRotI, v);
}

TEST(BodyConf, UsePosition)
{
    const auto p = Position{Length2{3_m, -4_m}, 22_deg};
    EXPECT_EQ(BodyConf{}.Use(p).sweep.pos0, p);
}

TEST(BodyConf, UseVelocity)
{
    const auto v = Velocity{LinearVelocity2{3_mps, -4_mps}, 22_rad / 1_s};
    EXPECT_EQ(BodyConf{}.Use(v).linearVelocity, v.linear);
    EXPECT_EQ(BodyConf{}.Use(v).angularVelocity, v.angular);
}

TEST(BodyConf, UseShapes)
{
    const auto shapes = ArrayList<ShapeID, 3u>{ShapeID(0), ShapeID(1), ShapeID(2)};
    EXPECT_EQ(BodyConf{}.Use(shapes).shapes.size(), shapes.size());
    EXPECT_EQ(BodyConf{}.Use(shapes).shapes, shapes);
    EXPECT_EQ(BodyConf{}.Use(shapes).Use(shapes).shapes.size(), shapes.size() * 2u);
    const auto toomany = std::vector<ShapeID>(BodyConf::MaxShapes * 2u);
    EXPECT_THROW(BodyConf{}.Use(toomany), LengthError);
}

TEST(BodyConf, GetBodyConf1)
{
    auto conf = BodyConf{};
    conf.invMass = {};
    conf.invRotI = {};
    conf.type = BodyType::Static;
    conf.awake = false;
    SCOPED_TRACE("checking for static");
    EXPECT_NO_THROW(IsSame(conf, GetBodyConf(Body(conf))));
}

TEST(BodyConf, GetBodyConf2)
{
    auto conf = BodyConf{};
    conf.type = BodyType::Dynamic;
    conf.sweep = Sweep{Position{Length2{1_m, 2_m}, 20_deg},
                       Position{Length2{2_m, 3_m}, 30_deg},
                       Length2{3_m, 4_m},
                       Real(0.75)};
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
    conf.invMass = InvMass{Real(1) / 2_kg};
    conf.invRotI = InvRotInertia{Real(4) * SquareRadian / (SquareMeter * 1_kg)};
    SCOPED_TRACE("checking for dynamic");
    EXPECT_NO_THROW(IsSame(conf, GetBodyConf(Body(conf))));
}

TEST(BodyConf, EqualsOperator)
{
    EXPECT_TRUE(BodyConf() == BodyConf());
    EXPECT_FALSE(BodyConf().UseType(BodyType::Dynamic) == BodyConf());
    EXPECT_FALSE(BodyConf().UseLocation(Length2(2_m, 3_m)) == BodyConf());
    EXPECT_FALSE(BodyConf().UseAngle(15_deg) == BodyConf());
    EXPECT_FALSE(BodyConf().UseLinearVelocity(LinearVelocity2{2_mps, 3_mps}) == BodyConf());
    EXPECT_FALSE(BodyConf().UseAngularVelocity(3_rpm) == BodyConf());
    EXPECT_FALSE(BodyConf().Use(Position{Length2(2_m, 3_m), 3_deg}) == BodyConf());
    EXPECT_FALSE(BodyConf().Use(Velocity{LinearVelocity2{2_mps, 3_mps}, 3_rpm}) == BodyConf());
    EXPECT_FALSE(BodyConf().UseLinearAcceleration(LinearAcceleration2{3_mps2, 0_mps2}) ==
                 BodyConf());
    EXPECT_FALSE(BodyConf().UseAngularAcceleration(Real(2) * RadianPerSquareSecond) == BodyConf());
    EXPECT_FALSE(BodyConf().UseLinearDamping(1_Hz) == BodyConf());
    EXPECT_FALSE(BodyConf().UseAngularDamping(1_Hz) == BodyConf());
    EXPECT_FALSE(BodyConf().UseUnderActiveTime(1_s) == BodyConf());
    EXPECT_FALSE(BodyConf().UseAllowSleep(!BodyConf{}.allowSleep) == BodyConf());
    EXPECT_FALSE(BodyConf().UseAwake(!BodyConf{}.awake) == BodyConf());
    EXPECT_FALSE(BodyConf().UseFixedRotation(!BodyConf{}.fixedRotation) == BodyConf());
    EXPECT_FALSE(BodyConf().UseBullet(!BodyConf{}.bullet) == BodyConf());
    EXPECT_FALSE(BodyConf().UseEnabled(!BodyConf{}.enabled) == BodyConf());
}

TEST(BodyConf, NotEqualsOperator)
{
    EXPECT_FALSE(BodyConf() != BodyConf());
    EXPECT_TRUE(BodyConf().UseType(BodyType::Dynamic) != BodyConf());
    EXPECT_TRUE(BodyConf().UseLocation(Length2(2_m, 3_m)) != BodyConf());
    EXPECT_TRUE(BodyConf().UseAngle(15_deg) != BodyConf());
    EXPECT_TRUE(BodyConf().UseLinearVelocity(LinearVelocity2{2_mps, 3_mps}) != BodyConf());
    EXPECT_TRUE(BodyConf().UseAngularVelocity(3_rpm) != BodyConf());
    EXPECT_TRUE(BodyConf().Use(Position{Length2(2_m, 3_m), 3_deg}) != BodyConf());
    EXPECT_TRUE(BodyConf().Use(Velocity{LinearVelocity2{2_mps, 3_mps}, 3_rpm}) != BodyConf());
    EXPECT_TRUE(BodyConf().UseLinearAcceleration(LinearAcceleration2{3_mps2, 0_mps2}) !=
                 BodyConf());
    EXPECT_TRUE(BodyConf().UseAngularAcceleration(Real(2) * RadianPerSquareSecond) != BodyConf());
    EXPECT_TRUE(BodyConf().UseLinearDamping(1_Hz) != BodyConf());
    EXPECT_TRUE(BodyConf().UseAngularDamping(1_Hz) != BodyConf());
    EXPECT_TRUE(BodyConf().UseUnderActiveTime(1_s) != BodyConf());
    EXPECT_TRUE(BodyConf().UseAllowSleep(!BodyConf{}.allowSleep) != BodyConf());
    EXPECT_TRUE(BodyConf().UseAwake(!BodyConf{}.awake) != BodyConf());
    EXPECT_TRUE(BodyConf().UseFixedRotation(!BodyConf{}.fixedRotation) != BodyConf());
    EXPECT_TRUE(BodyConf().UseBullet(!BodyConf{}.bullet) != BodyConf());
    EXPECT_TRUE(BodyConf().UseEnabled(!BodyConf{}.enabled) != BodyConf());
}
