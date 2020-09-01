/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Dynamics/Island.hpp>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(IslandBodyContainer, BytesSize)
{
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

TEST(Island, NotDefaultConstructible)
{
    EXPECT_FALSE(std::is_default_constructible<Island>::value);
}

TEST(Island, IsCopyConstructible)
{
    EXPECT_TRUE(std::is_copy_constructible<Island>::value);
}

TEST(Island, IsNothrowMoveConstructible)
{
    EXPECT_TRUE(std::is_nothrow_move_constructible<Island>::value);
}

TEST(Island, IsMoveAssignable)
{
    EXPECT_TRUE(std::is_move_assignable<Island>::value);
}

TEST(Island, CopyAssignable)
{
    EXPECT_TRUE(std::is_copy_assignable<Island>::value);
}

TEST(Island, IsNothrowDestructible)
{
    EXPECT_TRUE(std::is_nothrow_destructible<Island>::value);
}

static Island foo()
{
    return Island(10, 10, 10);
}

TEST(Island, IsReturnableByValue)
{
    // This should be possible due to C++ copy elision (regardless of move construction or copy
    // construction support). For information on copy elision see:
    //   http://en.cppreference.com/w/cpp/language/copy_elision

    {
        const auto island = foo();
        
        EXPECT_EQ(island.m_bodies.capacity(), decltype(island.m_bodies.max_size()){10});
        EXPECT_EQ(island.m_contacts.capacity(), decltype(island.m_contacts.max_size()){10});
        EXPECT_EQ(island.m_joints.capacity(), decltype(island.m_joints.max_size()){10});
    }
}

TEST(Island, Count)
{
    const auto island = Island(4, 4, 4);
    EXPECT_EQ(Count(island, static_cast<Body*>(nullptr)), std::size_t(0));
    EXPECT_EQ(Count(island, static_cast<Contact*>(nullptr)), std::size_t(0));
    EXPECT_EQ(Count(island, static_cast<Joint*>(nullptr)), std::size_t(0));
}
