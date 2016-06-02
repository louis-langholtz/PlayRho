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

#include <Box2D/Common/StackAllocator.h>
#include <Box2D/Common/Math.h>

using namespace box2d;

StackAllocator::StackAllocator() noexcept = default;

StackAllocator::~StackAllocator() noexcept
{
	assert(m_index == 0);
	assert(m_entryCount == 0);
}

void* StackAllocator::Allocate(size_type size) noexcept
{
	assert(m_entryCount < MaxStackEntries);
	assert(m_index <= StackSize);

	auto entry = m_entries + m_entryCount;
	entry->size = size;
	
	const auto available = StackSize - m_index;
	if (size > available)
	{
		entry->data = static_cast<decltype(entry->data)>(alloc(size));
		entry->usedMalloc = true;
	}
	else
	{
		entry->data = m_data + m_index;
		entry->usedMalloc = false;
		m_index += size;
	}

	m_allocation += size;
	m_maxAllocation = Max(m_maxAllocation, m_allocation);
	++m_entryCount;

	return entry->data;
}

void StackAllocator::Free(void* p) noexcept
{
	assert(m_entryCount > 0);
	const auto entry = m_entries + m_entryCount - 1;
	assert(p == entry->data);
	if (entry->usedMalloc)
	{
		free(p);
	}
	else
	{
		assert(m_index >= entry->size);
		m_index -= entry->size;
	}
	assert(m_allocation >= entry->size);
	m_allocation -= entry->size;
	--m_entryCount;
}

void StackAllocator::operator()(void *p) noexcept
{
	Free(p);
}