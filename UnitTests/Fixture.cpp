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
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>

using namespace box2d;

TEST(Fixture, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(Fixture), size_t(56)); break;
        case  8: EXPECT_EQ(sizeof(Fixture), size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(Fixture), size_t(56)); break;
        default: FAIL(); break;
    }
}

TEST(Fixture, CreateMatchesDef)
{
    const auto density = RealNum{2} * KilogramPerSquareMeter;
    int variable;
    const auto userData = &variable;
    const auto friction = RealNum(0.5);
    const auto restitution = RealNum(0.4);
    const auto isSensor = true;
    const auto shapeA = std::make_shared<DiskShape>();
    shapeA->SetFriction(friction);
    shapeA->SetRestitution(restitution);
    shapeA->SetDensity(density);

    auto def = FixtureDef{};
    def.userData = userData;
    def.isSensor = isSensor;

    World world;
    const auto body = world.CreateBody();
    const auto fixture = body->CreateFixture(shapeA, def);
    
    EXPECT_EQ(fixture->GetBody(), body);
    EXPECT_EQ(fixture->GetShape(), shapeA);

    EXPECT_EQ(fixture->GetDensity(), density);
    EXPECT_EQ(fixture->GetFriction(), friction);
    EXPECT_EQ(fixture->GetUserData(), userData);
    EXPECT_EQ(fixture->GetRestitution(), restitution);
    EXPECT_EQ(fixture->IsSensor(), isSensor);
}

TEST(Fixture, SetSensor)
{
    const auto shapeA = std::make_shared<DiskShape>();
    const auto bodyCtrPos = Vec2(1, 2) * Meter;
    
    World world;
    const auto body = world.CreateBody(BodyDef{}.UseLocation(bodyCtrPos));
    const auto fixture = body->CreateFixture(shapeA);
    fixture->SetSensor(true);
    EXPECT_TRUE(fixture->IsSensor());
    fixture->SetSensor(true);
    EXPECT_TRUE(fixture->IsSensor());
    fixture->SetSensor(false);
    EXPECT_FALSE(fixture->IsSensor());
}

TEST(Fixture, TestPointFreeFunction)
{
    const auto shapeA = std::make_shared<DiskShape>();
    const auto bodyCtrPos = Vec2(1, 2) * Meter;

    World world;
    const auto body = world.CreateBody(BodyDef{}.UseLocation(bodyCtrPos));
    const auto fixture = body->CreateFixture(shapeA);
    EXPECT_TRUE(TestPoint(*fixture, bodyCtrPos));
    EXPECT_FALSE(TestPoint(*fixture, Vec2_zero * Meter));
}

TEST(Fixture, SetAwakeFreeFunction)
{
    const auto shapeA = std::make_shared<DiskShape>();
    
    World world;
    const auto body = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
    body->UnsetAwake();
    ASSERT_FALSE(body->IsAwake());
    const auto fixture = body->CreateFixture(shapeA);
    SetAwake(*fixture);
    EXPECT_TRUE(body->IsAwake());
}
