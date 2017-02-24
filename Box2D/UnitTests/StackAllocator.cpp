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
#include <Box2D/Common/StackAllocator.hpp>
#include <chrono>

using namespace box2d;

TEST(StackAllocator, ByteSizeIs64)
{
	EXPECT_EQ(sizeof(StackAllocator), size_t(64));
}

TEST(StackAllocator, DefaultConstruction)
{
	const auto config = StackAllocator::GetDefaultConfiguration();
	StackAllocator foo;
	EXPECT_EQ(foo.GetPreallocatedSize(), config.preallocation_size);
	EXPECT_EQ(foo.GetMaxEntries(), config.allocation_records);
	EXPECT_EQ(foo.GetIndex(), decltype(foo.GetIndex()){0});
	EXPECT_EQ(foo.GetAllocation(), decltype(foo.GetAllocation()){0});
}

TEST(StackAllocator, slower_than_mallocfree)
{
	// If this test fails, the question arrises of whether the stack allocator code should be
	// replaced with instead using malloc/free.

	const auto ptr_val = reinterpret_cast<Body*>(0x768ea);
	constexpr auto iterations = unsigned(500000);
	
	std::chrono::duration<double> elapsed_secs_custom;
	std::chrono::duration<double> elapsed_secs_malloc;

	{
		StackAllocator foo;
		
		std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
		start = std::chrono::high_resolution_clock::now();
		{
			for (auto i = decltype(iterations){0}; i < iterations; ++i)
			{
				for (auto num_body_ptrs = size_t(1); num_body_ptrs < 200; ++num_body_ptrs)
				{
					const auto elem_to_poke = num_body_ptrs / 2;
					auto buf = static_cast<Body**>(foo.Allocate(num_body_ptrs * sizeof(Body*)));
					buf[elem_to_poke] = ptr_val;
					ASSERT_EQ(buf[elem_to_poke], ptr_val);
					foo.Free(buf);
				}
			}
		}
		end = std::chrono::high_resolution_clock::now();
		elapsed_secs_custom = end - start;
	}
	
	{
		std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
		start = std::chrono::high_resolution_clock::now();
		for (auto i = decltype(iterations){0}; i < iterations; ++i)
		{
			for (auto num_body_ptrs = size_t(1); num_body_ptrs < 200; ++num_body_ptrs)
			{
				const auto elem_to_poke = num_body_ptrs / 2;
				auto buf = static_cast<Body**>(std::malloc(num_body_ptrs * sizeof(Body*)));
				buf[elem_to_poke] = ptr_val;
				ASSERT_EQ(buf[elem_to_poke], ptr_val);
				std::free(buf);
			}
		}
		end = std::chrono::high_resolution_clock::now();
		elapsed_secs_malloc = end - start;
	}

	EXPECT_GT(elapsed_secs_custom.count(), elapsed_secs_malloc.count());
}

static inline bool is_aligned(void* ptr, std::size_t siz)
{
	return reinterpret_cast<std::uintptr_t>(static_cast<void*>(ptr)) % siz == 0;
}

TEST(StackAllocator, aligns_data)
{
	StackAllocator foo;

	const auto p_char1 = static_cast<char*>(foo.Allocate(sizeof(char)));
	
	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){1});
	EXPECT_EQ(foo.GetIndex(), sizeof(char));
	EXPECT_EQ(foo.GetAllocation(), sizeof(char));

	const auto p_char2 = static_cast<char*>(foo.Allocate(sizeof(char)));

	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){2});
	EXPECT_EQ(foo.GetIndex(), sizeof(char) + sizeof(char));
	EXPECT_EQ(foo.GetAllocation(), sizeof(char) + sizeof(char));

	const auto p_int = static_cast<int*>(foo.Allocate(sizeof(int)));

	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){3});
	EXPECT_EQ(foo.GetIndex(), foo.GetAllocation());
	EXPECT_EQ(foo.GetIndex(), sizeof(int) + sizeof(int));
	EXPECT_EQ(foo.GetAllocation(), sizeof(int) + sizeof(int));

	EXPECT_TRUE(is_aligned(p_char1, alignof(char)));
	EXPECT_TRUE(is_aligned(p_char2, alignof(char)));
	EXPECT_TRUE(is_aligned(p_int, alignof(int)));

	*p_char1 = 'W';
	*p_int = 5;
	
	EXPECT_EQ(*p_char1, 'W');
	EXPECT_EQ(*p_int, 5);
	
	foo.Free(p_int);
	foo.Free(p_char2);
	foo.Free(p_char1);
}

TEST(StackAllocator, uses_malloc_when_full)
{
	StackAllocator foo;	
	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){0});
	
	const auto preallocated_size = foo.GetPreallocatedSize();
	const auto p = foo.Allocate(preallocated_size);
	
	EXPECT_NE(p, nullptr);
	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){1});
	EXPECT_EQ(foo.GetIndex(), preallocated_size);
	EXPECT_EQ(foo.GetAllocation(), preallocated_size);
	
	const auto q = foo.Allocate(sizeof(double));
	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){2});
	EXPECT_EQ(foo.GetIndex(), preallocated_size);
	EXPECT_GT(foo.GetAllocation(), preallocated_size);

	foo.Free(q);
	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){1});

	foo.Free(p);
	EXPECT_EQ(foo.GetEntryCount(), decltype(foo.GetEntryCount()){0});
}
