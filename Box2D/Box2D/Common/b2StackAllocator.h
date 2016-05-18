/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_STACK_ALLOCATOR_H
#define B2_STACK_ALLOCATOR_H

#include <Box2D/Common/b2Settings.h>

namespace box2d {

constexpr auto b2_stackSize = unsigned{100 * 1024};	// 100k
constexpr auto b2_maxStackEntries = unsigned{32};

struct b2StackEntry
{
	using size_type = b2_size_t;

	char* data;
	size_type size;
	bool usedMalloc;
};

// This is a stack allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will assert
// if you try to interleave multiple allocate/free pairs.
class b2StackAllocator
{
public:
	using size_type = b2_size_t;

	b2StackAllocator();
	~b2StackAllocator();

	void* Allocate(size_type size);
	void Free(void* p);

	size_type GetMaxAllocation() const noexcept
	{
		return m_maxAllocation;
	}

private:

	char m_data[b2_stackSize];
	size_type m_index = 0;

	size_type m_allocation = 0;
	size_type m_maxAllocation = 0;

	b2StackEntry m_entries[b2_maxStackEntries];
	std::remove_cv<decltype(b2_maxStackEntries)>::type m_entryCount = 0;
};

} // namespace box2d

#endif
