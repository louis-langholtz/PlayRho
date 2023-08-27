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

#include <PlayRho/Island.hpp>

#include <type_traits>

using namespace playrho;

static Island foo()
{
    Island island;
    Reserve(island, 10, 10, 10);
    return island;
}

TEST(Island, DefaultConstructor)
{
    Island island;
    EXPECT_EQ(island.bodies.size(), 0u);
    EXPECT_EQ(island.contacts.size(), 0u);
    EXPECT_EQ(island.joints.size(), 0u);
}

TEST(Island, Reserve)
{
    const auto bodyCapacity = 2u;
    const auto contactCapacity = 3u;
    const auto jointCapacity = 4u;
    Island island;
    Reserve(island, bodyCapacity, contactCapacity, jointCapacity);
    EXPECT_GE(island.bodies.capacity(), bodyCapacity);
    EXPECT_GE(island.contacts.capacity(), contactCapacity);
    EXPECT_GE(island.joints.capacity(), jointCapacity);
}

TEST(Island, Clear)
{
    Island island;

    island.bodies.push_back(InvalidBodyID);
    ASSERT_EQ(island.bodies.size(), 1u);
    ASSERT_GE(island.bodies.capacity(), 1u);
    island.contacts.push_back(InvalidContactID);
    ASSERT_EQ(island.contacts.size(), 1u);
    ASSERT_GE(island.contacts.capacity(), 1u);
    island.joints.push_back(InvalidJointID);
    ASSERT_EQ(island.joints.size(), 1u);
    ASSERT_GE(island.joints.capacity(), 1u);

    EXPECT_NO_THROW(Clear(island));
    EXPECT_EQ(island.bodies.size(), 0u);
    EXPECT_GE(island.bodies.capacity(), 1u);
    EXPECT_EQ(island.contacts.size(), 0u);
    EXPECT_GE(island.contacts.capacity(), 1u);
    EXPECT_EQ(island.joints.size(), 0u);
    EXPECT_GE(island.joints.capacity(), 1u);
}

TEST(Island, IsReturnableByValue)
{
    // This should be possible due to C++ copy elision (regardless of move construction or copy
    // construction support). For information on copy elision see:
    //   http://en.cppreference.com/w/cpp/language/copy_elision

    {
        const auto island = foo();
        
        EXPECT_EQ(island.bodies.capacity(), decltype(island.bodies.max_size()){10});
        EXPECT_EQ(island.contacts.capacity(), decltype(island.contacts.max_size()){10});
        EXPECT_EQ(island.joints.capacity(), decltype(island.joints.max_size()){10});
    }
}

TEST(Island, Count)
{
    auto island = Island();
    Reserve(island, 4, 4, 4);
    EXPECT_EQ(Count(island, InvalidBodyID), std::size_t(0));
    EXPECT_EQ(Count(island, InvalidContactID), std::size_t(0));
    EXPECT_EQ(Count(island, InvalidJointID), std::size_t(0));
}

TEST(Island, Sort)
{
    auto island = Island();
    island.bodies.push_back(BodyID(3u));
    island.bodies.push_back(BodyID(8u));
    island.bodies.push_back(BodyID(0u));
    island.contacts.push_back(ContactID(1u));
    island.contacts.push_back(ContactID(9u));
    island.contacts.push_back(ContactID(2u));
    island.contacts.push_back(ContactID(8u));
    island.contacts.push_back(ContactID(3u));
    island.contacts.push_back(ContactID(0u));
    island.joints.push_back(JointID(3u));
    island.joints.push_back(JointID(2u));
    island.joints.push_back(JointID(4u));
    island.joints.push_back(JointID(0u));
    ASSERT_EQ(size(island.bodies), 3u);
    ASSERT_EQ(size(island.contacts), 6u);
    ASSERT_EQ(size(island.joints), 4u);
    ASSERT_EQ(island.bodies[0], BodyID(3u));
    ASSERT_EQ(island.contacts[0], ContactID(1u));
    ASSERT_EQ(island.joints[0], JointID(3u));
    EXPECT_NO_THROW(Sort(island));
    EXPECT_EQ(island.bodies[0], BodyID(0u));
    EXPECT_EQ(island.contacts[0], ContactID(0u));
    EXPECT_EQ(island.joints[0], JointID(0u));
    EXPECT_EQ(island.joints[1], JointID(2u));
    EXPECT_EQ(island.joints[2], JointID(3u));
    EXPECT_EQ(island.joints[3], JointID(4u));
}
