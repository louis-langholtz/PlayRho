/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Common/StackAllocator.hpp>
#include <Box2D/Dynamics/Island.h>
#include <type_traits>

using namespace box2d;

TEST(IslandBodyContainer, BytesSizeIs32)
{
	EXPECT_EQ(sizeof(Island::BodyContainer), size_t(32));
}

TEST(IslandContactContainer, BytesSizeIs32)
{
	EXPECT_EQ(sizeof(Island::ContactContainer), size_t(32));
}

TEST(IslandJointContainer, BytesSizeIs32)
{
	EXPECT_EQ(sizeof(Island::JointContainer), size_t(32));
}

TEST(Island, ByteSizeIs96)
{
	EXPECT_EQ(sizeof(Island), size_t(96));
}

TEST(Island, NotDefaultConstructible)
{
	EXPECT_FALSE(std::is_default_constructible<Island>::value);
}

TEST(Island, NotCopyConstructible)
{
	EXPECT_FALSE(std::is_copy_constructible<Island>::value);
}

TEST(Island, IsNothrowMoveConstructible)
{
	EXPECT_TRUE(std::is_nothrow_move_constructible<Island>::value);
}

TEST(Island, NotMoveAssignable)
{
	EXPECT_FALSE(std::is_move_assignable<Island>::value);
}

TEST(Island, NotCopyAssignable)
{
	EXPECT_FALSE(std::is_copy_assignable<Island>::value);
}

TEST(Island, IsNothrowDestructible)
{
	EXPECT_TRUE(std::is_nothrow_destructible<Island>::value);
}

static Island foo(StackAllocator& allocator)
{
	return Island(10, 10, 10, allocator);
}

TEST(Island, IsReturnableByValue)
{
	// This should be possible due to C++ copy elision (regardless of move construction or copy
	// construction support). For information on copy elision see:
	//   http://en.cppreference.com/w/cpp/language/copy_elision

	StackAllocator allocator;
	ASSERT_EQ(allocator.GetIndex(), decltype(allocator.GetIndex()){0});
	ASSERT_EQ(allocator.GetEntryCount(), decltype(allocator.GetEntryCount()){0});

	{
		const auto island = foo(allocator);
		
		EXPECT_EQ(allocator.GetEntryCount(), decltype(allocator.GetEntryCount()){3});
		const auto size = 10 * (sizeof(Body*) + sizeof(Contact*) + sizeof(Joint*));
		EXPECT_EQ(allocator.GetIndex(), decltype(allocator.GetIndex()){size});

		EXPECT_EQ(island.m_bodies.max_size(), decltype(island.m_bodies.max_size()){10});
		EXPECT_EQ(island.m_contacts.max_size(), decltype(island.m_contacts.max_size()){10});
		EXPECT_EQ(island.m_joints.max_size(), decltype(island.m_joints.max_size()){10});
	}
	
	EXPECT_EQ(allocator.GetEntryCount(), decltype(allocator.GetEntryCount()){0});
	EXPECT_EQ(allocator.GetIndex(), decltype(allocator.GetIndex()){0});	
}
