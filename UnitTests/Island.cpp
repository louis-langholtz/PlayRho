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
#include <PlayRho/Dynamics/Island.hpp>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(IslandBodyContainer, BytesSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
#if defined(_WIN64)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island::Bodies), std::size_t(32));
#else
    EXPECT_EQ(sizeof(Island::Bodies), std::size_t(24));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island::Bodies), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Island::Bodies), std::size_t(12));
#endif
#else
    EXPECT_EQ(sizeof(Island::Bodies), std::size_t(24));
#endif
}

TEST(IslandContactContainer, BytesSize)
{
#if defined(_WIN64)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island::Contacts), std::size_t(32));
#else
    EXPECT_EQ(sizeof(Island::Contacts), std::size_t(24));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island::Contacts), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Island::Contacts), std::size_t(12));
#endif
#else
    EXPECT_EQ(sizeof(Island::Contacts), std::size_t(24));
#endif
}

TEST(IslandJointContainer, BytesSize)
{
#if defined(_WIN64)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island::Joints), std::size_t(32));
#else
    EXPECT_EQ(sizeof(Island::Joints), std::size_t(24));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island::Joints), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Island::Joints), std::size_t(12));
#endif
#else
    EXPECT_EQ(sizeof(Island::Joints), std::size_t(24));
#endif
}

TEST(Island, ByteSize)
{
#if defined(_WIN64)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island), std::size_t(96));
#else
    EXPECT_EQ(sizeof(Island), std::size_t(72));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Island), std::size_t(48));
#else
    EXPECT_EQ(sizeof(Island), std::size_t(36));
#endif
#else
    EXPECT_EQ(sizeof(Island), std::size_t(72));
#endif
}

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
