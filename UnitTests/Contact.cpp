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

#include <playrho/Contact.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Contact, Enabled)
{
    const auto bA = BodyID(0u);
    const auto bB = BodyID(1u);
    const auto shapeId = ShapeID(0u);
    auto c = Contact{{bA, shapeId, 0u}, {bB, shapeId, 0u}};
    EXPECT_TRUE(c.IsEnabled());
    c.UnsetEnabled();
    EXPECT_FALSE(c.IsEnabled());
    c.SetEnabled();
    EXPECT_TRUE(c.IsEnabled());
}

TEST(Contact, DefaultConstruction)
{
    {
        const auto c = Contact();
        EXPECT_EQ(c.GetContactableA(), Contact::DefaultContactable);
        EXPECT_EQ(c.GetContactableB(), Contact::DefaultContactable);
    }
    EXPECT_EQ(GetBodyA(Contact()), InvalidBodyID);
    EXPECT_EQ(GetBodyB(Contact()), InvalidBodyID);
    EXPECT_EQ(GetShapeA(Contact()), InvalidShapeID);
    EXPECT_EQ(GetShapeB(Contact()), InvalidShapeID);
    EXPECT_EQ(GetChildIndexA(Contact()), 0u);
    EXPECT_EQ(GetChildIndexB(Contact()), 0u);
    EXPECT_EQ(Contact().GetFriction(), Real(0));
    EXPECT_EQ(Contact().GetRestitution(), Real(0));
    EXPECT_EQ(Contact().GetTangentSpeed(), LinearVelocity());
    EXPECT_EQ(Contact().GetToiCount(), Contact::substep_type(0));
    EXPECT_FALSE(Contact().IsEnabled());
    EXPECT_FALSE(Contact().NeedsUpdating());
    EXPECT_FALSE(Contact().IsTouching());
    EXPECT_FALSE(Contact().HasValidToi());
    EXPECT_FALSE(Contact().NeedsFiltering());
    EXPECT_FALSE(Contact().IsSensor());
    EXPECT_FALSE(Contact().IsImpenetrable());
    EXPECT_FALSE(Contact().IsActive());
}

TEST(Contact, InitializingConstructor)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    const auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});
    EXPECT_EQ(GetBodyA(contact), bodyIdA);
    EXPECT_EQ(GetBodyB(contact), bodyIdB);
    EXPECT_EQ(GetShapeA(contact), shapeIdA);
    EXPECT_EQ(GetShapeB(contact), shapeIdB);
    EXPECT_EQ(GetChildIndexA(contact), childIndexA);
    EXPECT_EQ(GetChildIndexB(contact), childIndexB);
    EXPECT_EQ(contact.GetFriction(), Real(0));
    EXPECT_EQ(contact.GetRestitution(), Real(0));
    EXPECT_EQ(contact.GetTangentSpeed(), LinearVelocity());
    EXPECT_EQ(contact.GetToiCount(), Contact::substep_type(0));
    EXPECT_TRUE(contact.IsEnabled());
    EXPECT_TRUE(contact.NeedsUpdating());
    EXPECT_FALSE(contact.IsTouching());
    EXPECT_FALSE(contact.HasValidToi());
    EXPECT_FALSE(contact.NeedsFiltering());
    EXPECT_FALSE(contact.IsSensor());
    EXPECT_FALSE(contact.IsImpenetrable());
    EXPECT_FALSE(contact.IsActive());
}

TEST(Contact, InitializingConstructorFF)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});

    EXPECT_EQ(GetBodyA(contact), bodyIdA);
    EXPECT_EQ(GetBodyB(contact), bodyIdB);
    EXPECT_EQ(GetShapeA(contact), shapeIdA);
    EXPECT_EQ(GetShapeB(contact), shapeIdB);
    EXPECT_EQ(GetChildIndexA(contact), childIndexA);
    EXPECT_EQ(GetChildIndexB(contact), childIndexB);
    EXPECT_EQ(GetFriction(contact), Real(0));
    EXPECT_EQ(GetRestitution(contact), Real(0));
    EXPECT_EQ(GetTangentSpeed(contact), LinearVelocity());
    EXPECT_EQ(GetToiCount(contact), Contact::substep_type(0));
    EXPECT_TRUE(IsEnabled(contact));
    EXPECT_TRUE(NeedsUpdating(contact));
    EXPECT_FALSE(IsTouching(contact));
    EXPECT_FALSE(HasValidToi(contact));
    EXPECT_FALSE(NeedsFiltering(contact));
    EXPECT_FALSE(IsSensor(contact));
    EXPECT_FALSE(IsImpenetrable(contact));
    EXPECT_FALSE(IsActive(contact));

    const auto toi = std::optional<UnitInterval<Real>>{Real(0.5)};
    EXPECT_NO_THROW(SetToi(contact, toi));
    EXPECT_TRUE(HasValidToi(contact));
    EXPECT_EQ(GetToi(contact), toi);
}

TEST(Contact, GetContactableA)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    const auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});
    EXPECT_EQ(contact.GetContactableA().bodyId, bodyIdA);
    EXPECT_EQ(contact.GetContactableA().shapeId, shapeIdA);
    EXPECT_EQ(contact.GetContactableA().childId, childIndexA);
}

TEST(Contact, GetContactableB)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    const auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});
    EXPECT_EQ(contact.GetContactableB().bodyId, bodyIdB);
    EXPECT_EQ(contact.GetContactableB().shapeId, shapeIdB);
    EXPECT_EQ(contact.GetContactableB().childId, childIndexB);
}

TEST(Contact, IsForBodyShape)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    const auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});
    EXPECT_TRUE(IsFor(contact, bodyIdA, shapeIdA));
    EXPECT_TRUE(IsFor(contact, bodyIdB, shapeIdB));
    EXPECT_FALSE(IsFor(contact, bodyIdA, shapeIdB));
    EXPECT_FALSE(IsFor(contact, bodyIdB, shapeIdA));
    EXPECT_FALSE(IsFor(contact, BodyID(0u), ShapeID(0)));
}

TEST(Contact, IsForShape)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    const auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});
    const auto shapeIdC = ShapeID(42u);
    EXPECT_TRUE(IsFor(contact, shapeIdA));
    EXPECT_TRUE(IsFor(contact, shapeIdB));
    EXPECT_FALSE(IsFor(contact, shapeIdC));
}

TEST(Contact, GetOtherBody)
{
    const auto bodyIdA = BodyID(1u);
    const auto shapeIdA = ShapeID(2u);
    const auto childIndexA = ChildCounter(3u);
    const auto bodyIdB = BodyID(4u);
    const auto shapeIdB = ShapeID(5u);
    const auto childIndexB = ChildCounter(6u);
    const auto contact = Contact({bodyIdA, shapeIdA, childIndexA}, {bodyIdB, shapeIdB, childIndexB});
    EXPECT_EQ(GetOtherBody(contact, bodyIdA), bodyIdB);
    EXPECT_EQ(GetOtherBody(contact, bodyIdB), bodyIdA);
}

TEST(Contact, Equality)
{
    EXPECT_TRUE(Contact() == Contact());
    EXPECT_TRUE(Contact() == Contact(Contact::DefaultContactable, Contact::DefaultContactable));
    {
        auto c = Contact();
        c.SetTouching();
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetEnabled();
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetFriction(NonNegative<Real>(2));
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetRestitution(Real(42));
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetTangentSpeed(42_mps);
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetToiCount(42u);
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetToi(std::optional<UnitIntervalFF<Real>>(0.5));
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.FlagForFiltering();
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.FlagForUpdating();
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetSensor();
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetImpenetrable();
        EXPECT_FALSE(Contact() == c);
    }
    {
        auto c = Contact();
        c.SetIsActive();
        EXPECT_FALSE(Contact() == c);
    }
    EXPECT_FALSE(Contact() ==
                 Contact(Contactable{BodyID(1u), ShapeID(0u), ChildCounter(0u)},
                         Contact::DefaultContactable));
    EXPECT_FALSE(Contact() ==
                 Contact(Contactable{BodyID(0u), ShapeID(1u), ChildCounter(0u)},
                         Contact::DefaultContactable));
    EXPECT_FALSE(Contact() ==
                 Contact(Contactable{BodyID(0u), ShapeID(0u), ChildCounter(1u)},
                         Contact::DefaultContactable));
}
