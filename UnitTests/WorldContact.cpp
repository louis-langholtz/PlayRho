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

#include <PlayRho/Dynamics/WorldContact.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>

#include <PlayRho/Dynamics/Contacts/Contact.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldContact, SetAwake)
{
    auto world = World{};
    const auto s1 = CreateShape(world, DiskShapeConf{});
    const auto s2 = CreateShape(world, DiskShapeConf{});
    const auto bA = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, bA, s1);
    Attach(world, bB, s2);

    ASSERT_TRUE(GetContacts(world).empty());
    Step(world, StepConf{});
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    ASSERT_EQ(GetShapeA(world, c), s1);
    ASSERT_EQ(GetShapeB(world, c), s2);
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

TEST(WorldContact, ResetFriction)
{
    const auto shape = DiskShapeConf{};
    auto world = World{};
    const auto sA = CreateShape(world, Shape{shape});
    const auto sB = CreateShape(world, Shape{shape});
    const auto bA = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, bA, sA);
    Attach(world, bB, sB);

    ASSERT_TRUE(GetContacts(world).empty());
    Step(world, StepConf{});
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    ASSERT_EQ(GetShapeA(world, c), sA);
    ASSERT_EQ(GetShapeB(world, c), sB);

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

TEST(WorldContact, ResetRestitution)
{
    const auto shape = DiskShapeConf{};
    auto world = World{};
    const auto sA = CreateShape(world, Shape{shape});
    const auto sB = CreateShape(world, Shape{shape});
    const auto bA = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, bA, sA);
    Attach(world, bB, sB);

    ASSERT_TRUE(GetContacts(world).empty());
    Step(world, StepConf{});
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    ASSERT_EQ(GetShapeA(world, c), sA);
    ASSERT_EQ(GetShapeB(world, c), sB);

    ASSERT_EQ(GetRestitution(shape), Real(0));
    ASSERT_EQ(GetRestitution(world, c), GetRestitution(shape));
    SetRestitution(world, c, Real(2));
    ASSERT_NE(GetRestitution(world, c), GetRestitution(shape));
    ResetRestitution(world, c);
    EXPECT_EQ(GetRestitution(world, c), GetRestitution(shape));
}

TEST(WorldContact, SetUnsetEnabled)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    const auto bA = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, bA, shapeId);
    Attach(world, bB, shapeId);
    ASSERT_NO_THROW(Step(world, StepConf{}));
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    EXPECT_NO_THROW(SetEnabled(world, c));
    EXPECT_TRUE(IsEnabled(world, c));
    EXPECT_NO_THROW(UnsetEnabled(world, c));
    EXPECT_FALSE(IsEnabled(world, c));
    EXPECT_NO_THROW(SetEnabled(world, c));
    EXPECT_TRUE(IsEnabled(world, c));
}

TEST(WorldContact, SetIsActive)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).Use(shapeId));
    CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).Use(shapeId));
    ASSERT_NO_THROW(Step(world, StepConf{}));
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    auto contact = GetContact(world, c);
    ASSERT_TRUE(IsActive(contact));
    EXPECT_NO_THROW(SetIsActive(contact));
    EXPECT_NO_THROW(SetContact(world, c, contact));
    EXPECT_NO_THROW(UnsetIsActive(contact));
    EXPECT_THROW(SetContact(world, c, contact), InvalidArgument);
}

TEST(WorldContact, SetImpenetrable)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).Use(shapeId));
    CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).Use(shapeId));
    ASSERT_NO_THROW(Step(world, StepConf{}));
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    auto contact = GetContact(world, c);
    ASSERT_FALSE(IsImpenetrable(contact));
    EXPECT_NO_THROW(UnsetImpenetrable(contact));
    EXPECT_NO_THROW(SetContact(world, c, contact));
    EXPECT_NO_THROW(SetImpenetrable(contact));
    EXPECT_THROW(SetContact(world, c, contact), InvalidArgument);
}

TEST(WorldContact, SetSensor)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).Use(shapeId));
    CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).Use(shapeId));
    ASSERT_NO_THROW(Step(world, StepConf{}));
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    auto contact = GetContact(world, c);
    ASSERT_FALSE(IsSensor(contact));
    EXPECT_NO_THROW(UnsetIsSensor(contact));
    EXPECT_NO_THROW(SetContact(world, c, contact));
    EXPECT_NO_THROW(SetSensor(contact));
    EXPECT_THROW(SetContact(world, c, contact), InvalidArgument);
}

TEST(WorldContact, SetTangentSpeed)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    const auto bA = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, bA, shapeId);
    Attach(world, bB, shapeId);
    ASSERT_NO_THROW(Step(world, StepConf{}));
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    const auto c = contacts.begin()->second;
    {
        const auto linearVelocity = LinearVelocity{5.6_mps};
        EXPECT_NO_THROW(SetTangentSpeed(world, c, linearVelocity));
        EXPECT_EQ(GetTangentSpeed(world, c), linearVelocity);
    }
    {
        const auto linearVelocity = LinearVelocity{0.2_mps};
        EXPECT_NO_THROW(SetTangentSpeed(world, c, linearVelocity));
        EXPECT_EQ(GetTangentSpeed(world, c), linearVelocity);
    }
}

TEST(WorldContact, WorldManifoldAndMore)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    const auto bA = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto bB = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    Attach(world, bA, shapeId);
    Attach(world, bB, shapeId);
    ASSERT_NO_THROW(Step(world, StepConf{}));
    const auto contacts = GetContacts(world);
    ASSERT_EQ(contacts.size(), ContactCounter(1));
    EXPECT_EQ(GetContactRange(world), 1u);
    const auto c = contacts.begin()->second;
    auto count = ContactCounter(0);
    EXPECT_EQ(count = GetTouchingCount(world), ContactCounter(1));
    if (count > ContactCounter(0)) {
        if (HasValidToi(world, c)) {
            auto toi = Real(0);
            EXPECT_NO_THROW(toi = GetToi(world, c));
            EXPECT_GE(toi, Real(0));
            EXPECT_LE(toi, Real(1));
        }
    }
    {
        auto manifold = WorldManifold{};
        EXPECT_NO_THROW(manifold = GetWorldManifold(world, c));
        EXPECT_EQ(manifold.GetPointCount(), 1u);
        EXPECT_EQ(manifold.GetNormal(), UnitVec::GetRight());
    }
    EXPECT_EQ(GetToiCount(world, c), 0u);
    EXPECT_FALSE(HasValidToi(world, c));
    EXPECT_EQ(GetChildIndexA(world, c), ChildCounter(0));
    EXPECT_EQ(GetChildIndexB(world, c), ChildCounter(0));
}
