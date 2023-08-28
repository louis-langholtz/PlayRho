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

TEST(Contact, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
            EXPECT_EQ(alignof(Contact), 4u);
            EXPECT_EQ(sizeof(Contact), std::size_t(36));
            break;
        case  8:
            EXPECT_EQ(alignof(Contact), 8u);
            EXPECT_EQ(sizeof(Contact), std::size_t(56));
            break;
        case 16:
            EXPECT_EQ(sizeof(Contact), std::size_t(96));
            break;
        default:
            FAIL();
            break;
    }
}

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
    EXPECT_EQ(Contact().GetBodyA(), InvalidBodyID);
    EXPECT_EQ(Contact().GetBodyB(), InvalidBodyID);
    EXPECT_EQ(Contact().GetShapeA(), InvalidShapeID);
    EXPECT_EQ(Contact().GetShapeB(), InvalidShapeID);
    EXPECT_EQ(Contact().GetChildIndexA(), 0u);
    EXPECT_EQ(Contact().GetChildIndexB(), 0u);
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
    EXPECT_EQ(contact.GetBodyA(), bodyIdA);
    EXPECT_EQ(contact.GetBodyB(), bodyIdB);
    EXPECT_EQ(contact.GetShapeA(), shapeIdA);
    EXPECT_EQ(contact.GetShapeB(), shapeIdB);
    EXPECT_EQ(contact.GetChildIndexA(), childIndexA);
    EXPECT_EQ(contact.GetChildIndexB(), childIndexB);
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

    const auto toi = Real(0.5);
    EXPECT_NO_THROW(SetToi(contact, toi));
    EXPECT_TRUE(HasValidToi(contact));
    EXPECT_EQ(GetToi(contact), toi);
}
