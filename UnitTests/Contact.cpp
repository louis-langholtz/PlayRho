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

#include <PlayRho/Dynamics/Contacts/Contact.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldFixture.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldContact.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/FixtureConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Contact, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Contact), std::size_t(36)); break;
        case  8: EXPECT_EQ(sizeof(Contact), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(Contact), std::size_t(96)); break;
        default: FAIL(); break;
    }
}

TEST(Contact, Enabled)
{
    const auto shape = DiskShapeConf{};
    auto world = World{};
    const auto bA = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto fA = world.CreateFixture(bA, Shape{shape});
    const auto fB = world.CreateFixture(bB, Shape{shape});
    auto c = Contact{bA, fA, 0u, bB, fB, 0u};
    EXPECT_TRUE(c.IsEnabled());
    c.UnsetEnabled();
    EXPECT_FALSE(c.IsEnabled());
    c.SetEnabled();
    EXPECT_TRUE(c.IsEnabled());
}

TEST(Contact, SetAwake)
{
    const auto shape = DiskShapeConf{};
    auto world = World{};
    const auto bA = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto fA = world.CreateFixture(bA, Shape{shape});
    const auto fB = world.CreateFixture(bB, Shape{shape});

    ASSERT_TRUE(world.GetContacts().empty());
    world.Step(StepConf{});
    const auto contacts = world.GetContacts();
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    ASSERT_EQ(GetFixtureA(world, c), fA);
    ASSERT_EQ(GetFixtureB(world, c), fB);
    ASSERT_TRUE(IsAwake(world, c));

    ASSERT_NO_THROW(UnsetAwake(world, bA));
    ASSERT_FALSE(IsAwake(world, bA));

    ASSERT_NO_THROW(UnsetAwake(world, bB));
    ASSERT_FALSE(IsAwake(world, bB));

    EXPECT_NO_THROW(SetAwake(world, c));
    EXPECT_TRUE(IsAwake(world, c));
    EXPECT_TRUE(IsAwake(world, bA));
    EXPECT_TRUE(IsAwake(world, bB));
}

TEST(Contact, ResetFriction)
{
    const auto shape = DiskShapeConf{};
    auto world = World{};
    const auto bA = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto fA = world.CreateFixture(bA, Shape{shape});
    const auto fB = world.CreateFixture(bB, Shape{shape});

    ASSERT_TRUE(world.GetContacts().empty());
    world.Step(StepConf{});
    const auto contacts = world.GetContacts();
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    ASSERT_EQ(GetFixtureA(world, c), fA);
    ASSERT_EQ(GetFixtureB(world, c), fB);

    ASSERT_GT(GetFriction(shape), Real(0));
    ASSERT_NEAR(static_cast<double>(GetFriction(world, c)),
                static_cast<double>(Real{GetFriction(shape)}),
                0.01);
    SetFriction(world, c, GetFriction(shape) * Real(2));
    ASSERT_NE(GetFriction(world, c), GetFriction(shape));
    ResetFriction(world, c);
    EXPECT_NEAR(static_cast<double>(GetFriction(world, c)),
                static_cast<double>(Real{GetFriction(shape)}),
                0.01);
}

TEST(Contact, ResetRestitution)
{
    const auto shape = DiskShapeConf{};
    auto world = World{};
    const auto bA = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto fA = world.CreateFixture(bA, Shape{shape});
    const auto fB = world.CreateFixture(bB, Shape{shape});

    ASSERT_TRUE(world.GetContacts().empty());
    world.Step(StepConf{});
    const auto contacts = world.GetContacts();
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    ASSERT_EQ(GetFixtureA(world, c), fA);
    ASSERT_EQ(GetFixtureB(world, c), fB);

    ASSERT_EQ(GetRestitution(shape), Real(0));
    ASSERT_EQ(GetRestitution(world, c), GetRestitution(shape));
    SetRestitution(world, c, Real(2));
    ASSERT_NE(GetRestitution(world, c), GetRestitution(shape));
    ResetRestitution(world, c);
    EXPECT_EQ(GetRestitution(world, c), GetRestitution(shape));
}
