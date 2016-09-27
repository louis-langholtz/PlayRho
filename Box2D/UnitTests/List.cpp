//
//  List.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/26/16.
//
//

#include "gtest/gtest.h"
#include <type_traits>
#include <Box2D/Common/List.hpp>
#include <Box2D/Dynamics/Body.h>

using namespace box2d;

TEST(InternalListForBody, ByteSizeIs16)
{
	EXPECT_EQ(sizeof(InternalList<Body>), size_t(16));
}

TEST(InternalListForBody, IsDefaultConstructable)
{
	EXPECT_TRUE(std::is_default_constructible<InternalList<Body>>::value);
}

TEST(InternalListForBody, IsMoveConstructable)
{
	EXPECT_TRUE(std::is_move_constructible<InternalList<Body>>::value);
}

TEST(InternalListForBody, DefaultInit)
{
	InternalList<Body> list;

	EXPECT_TRUE(list.empty());
	EXPECT_EQ(list.size(), size_t(0));
	EXPECT_EQ(list.max_size(), size_t(MaxBodies));
	
	EXPECT_EQ(list.begin(), list.end());
	EXPECT_EQ(list.cbegin(), list.cend());
}

TEST(ListForBody, DefaultInit)
{
	InternalList<Body> internal_list;
	List<Body> list{internal_list};
	
	EXPECT_TRUE(list.empty());
	EXPECT_EQ(list.size(), size_t(0));
	EXPECT_EQ(list.max_size(), size_t(MaxBodies));
	
	EXPECT_EQ(list.begin(), list.end());
	EXPECT_EQ(list.cbegin(), list.cend());
}

TEST(ListNodeForBody, ByteSizeIs176)
{
	EXPECT_EQ(sizeof(ListNode<Body>), size_t(176));
}

TEST(InternalListForBody, PushAndPop)
{
	InternalList<Body> list;

	ASSERT_TRUE(list.empty());
	ASSERT_EQ(list.size(), size_t(0));

	const size_t n = 4;
	auto list_nodes = static_cast<ListNode<Body>*>(alloc(sizeof(ListNode<Body>) * n));
	for (auto i = decltype(n){0}; i < n; ++i)
	{
		list_nodes[i].next = nullptr;
		list_nodes[i].prev = nullptr;
		list.push_front(list_nodes + i);
	}
	
	EXPECT_FALSE(list.empty());
	EXPECT_EQ(list.size(), n);
	
	auto size = size_t(0);
	for (auto&& e: list)
	{
		if (size == 0)
		{
			EXPECT_EQ(e.prev, nullptr);			
		}
		else
		{
			EXPECT_NE(e.prev, nullptr);			
		}
		++size;
		if (size == n)
		{
			EXPECT_EQ(e.next, nullptr);			
		}
		else
		{
			EXPECT_NE(e.next, nullptr);
		}
	}
	EXPECT_EQ(size, n);

	for (auto i = decltype(n){0}; i < n; ++i)
	{
		list.pop_front();
	}
	EXPECT_TRUE(list.empty());
	EXPECT_EQ(list.size(), size_t(0));
	
	box2d::free(list_nodes);
}