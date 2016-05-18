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

#include <Box2D/Common/b2BlockAllocator.h>
#include <limits>
#include <cstring>
#include <cstddef>

using namespace box2d;

static constexpr size_t s_blockSizes[b2BlockAllocator::b2_blockSizes] =
{
	16,		// 0
	32,		// 1
	64,		// 2
	96,		// 3
	128,	// 4
	160,	// 5
	192,	// 6
	224,	// 7
	256,	// 8
	320,	// 9
	384,	// 10
	448,	// 11
	512,	// 12
	640,	// 13
};

static constexpr uint8 s_blockSizeLookup[b2BlockAllocator::b2_maxBlockSize + 1] =
{
	0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1-16
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 17-32
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 33-64
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, // 65-96
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, // 97-128
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, // 129-160
	6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, // 161-192
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, // 193-224
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, // 225-256
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, // 257-288
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, // 288-320
	10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, // 321-352
	10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, // 353-384
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, // 385-416
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, // 427-448
	12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, // 449-480
	12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, // 481-512
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, // 513-544
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, // 545-576
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, // 577-608
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, // 608-640
};

struct box2d::b2Chunk
{
	using size_type = size_t;

	size_type blockSize;
	b2Block* blocks;
};

struct box2d::b2Block
{
	b2Block* next;
};

b2BlockAllocator::b2BlockAllocator():
	m_chunks(static_cast<b2Chunk*>(alloc(m_chunkSpace * sizeof(b2Chunk))))
{
	assert(b2_blockSizes < std::numeric_limits<uint8>::max());
	std::memset(m_chunks, 0, m_chunkSpace * sizeof(b2Chunk));
	std::memset(m_freeLists, 0, sizeof(m_freeLists));
}

b2BlockAllocator::~b2BlockAllocator()
{
	for (auto i = decltype(m_chunkCount){0}; i < m_chunkCount; ++i)
	{
		free(m_chunks[i].blocks);
	}

	free(m_chunks);
}

void* b2BlockAllocator::Allocate(size_type size)
{
	if (size == 0)
		return nullptr;

	assert(0 < size);

	if (size > b2_maxBlockSize)
	{
		return alloc(size);
	}

	const auto index = s_blockSizeLookup[size];
	assert((0 <= index) && (index < b2_blockSizes));

	if (m_freeLists[index])
	{
		auto block = m_freeLists[index];
		m_freeLists[index] = block->next;
		return block;
	}

	if (m_chunkCount == m_chunkSpace)
	{
		m_chunkSpace += b2_chunkArrayIncrement;
		m_chunks = static_cast<b2Chunk*>(realloc(m_chunks, m_chunkSpace * sizeof(b2Chunk)));
		std::memset(m_chunks + m_chunkCount, 0, b2_chunkArrayIncrement * sizeof(b2Chunk));
	}

	auto chunk = m_chunks + m_chunkCount;
	chunk->blocks = static_cast<b2Block*>(alloc(b2_chunkSize));
#if defined(_DEBUG)
	std::memset(chunk->blocks, 0xcd, b2_chunkSize);
#endif
	const auto blockSize = s_blockSizes[index];
	assert(blockSize > 0);
	chunk->blockSize = blockSize;
	const auto blockCount = b2_chunkSize / blockSize;
	assert((blockCount * blockSize) <= b2_chunkSize);
	for (auto i = decltype(blockCount){0}; i < blockCount - 1; ++i)
	{
		auto block = (b2Block*)((int8*)chunk->blocks + blockSize * i);
		const auto next = (b2Block*)((int8*)chunk->blocks + blockSize * (i + 1));
		block->next = next;
	}
	auto last = (b2Block*)((int8*)chunk->blocks + blockSize * (blockCount - 1));
	last->next = nullptr;

	m_freeLists[index] = chunk->blocks->next;
	++m_chunkCount;

	return chunk->blocks;
}

void b2BlockAllocator::Free(void* p, size_type size)
{
	if (size == 0)
	{
		return;
	}

	assert(size > 0);

	if (size > b2_maxBlockSize)
	{
		free(p);
		return;
	}

	const auto index = s_blockSizeLookup[size];
	assert((0 <= index) && (index < b2_blockSizes));

#define _DEBUG
#ifdef _DEBUG
	// Verify the memory address and size is valid.
	const auto blockSize = s_blockSizes[index];
	bool found = false;
	for (auto i = decltype(m_chunkCount){0}; i < m_chunkCount; ++i)
	{
		const auto chunk = m_chunks + i;
		if (chunk->blockSize != blockSize)
		{
			assert(	(int8*)p + blockSize <= (int8*)chunk->blocks ||
						(int8*)chunk->blocks + b2_chunkSize <= (int8*)p);
		}
		else
		{
			if ((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + b2_chunkSize)
			{
				found = true;
			}
		}
	}

	assert(found);

	std::memset(p, 0xfd, blockSize);
#endif

	auto block = static_cast<b2Block*>(p);
	block->next = m_freeLists[index];
	m_freeLists[index] = block;
}

void b2BlockAllocator::Clear()
{
	for (auto i = decltype(m_chunkCount){0}; i < m_chunkCount; ++i)
	{
		free(m_chunks[i].blocks);
	}

	m_chunkCount = 0;
	std::memset(m_chunks, 0, m_chunkSpace * sizeof(b2Chunk));
	std::memset(m_freeLists, 0, sizeof(m_freeLists));
}
