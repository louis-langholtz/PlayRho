/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/BlockAllocator.hpp>
#include <limits>
#include <cstring>
#include <cstddef>

using namespace box2d;

static constexpr std::size_t s_blockSizes[BlockAllocator::BlockSizes] =
{
    16,        // 0
    32,        // 1
    64,        // 2
    96,        // 3
    128,    // 4
    160,    // 5
    192,    // 6
    224,    // 7
    256,    // 8
    320,    // 9
    384,    // 10
    448,    // 11
    512,    // 12
    640,    // 13
};

static constexpr std::uint8_t s_blockSizeLookup[BlockAllocator::MaxBlockSize + 1] =
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

struct BlockAllocator::Chunk
{
    using size_type = std::size_t;

    size_type blockSize;
    Block* blocks;
};

struct BlockAllocator::Block
{
    Block* next;
};

BlockAllocator::BlockAllocator():
    m_chunks(Alloc<Chunk>(m_chunkSpace))
{
    assert(BlockSizes < std::numeric_limits<std::uint8_t>::max());
    std::memset(m_chunks, 0, m_chunkSpace * sizeof(Chunk));
    std::memset(m_freeLists, 0, sizeof(m_freeLists));
}

BlockAllocator::~BlockAllocator() noexcept
{
    for (auto i = decltype(m_chunkCount){0}; i < m_chunkCount; ++i)
    {
        ::Free(m_chunks[i].blocks);
    }

    ::Free(m_chunks);
}

void* BlockAllocator::Allocate(size_type n)
{
    if (n == 0)
    {
        return nullptr;
    }

    assert(0 < n);

    if (n > MaxBlockSize)
    {
        return Alloc(n);
    }

    const auto index = s_blockSizeLookup[n];
    assert((0 <= index) && (index < BlockSizes));

    {
        const auto block = m_freeLists[index];
        if (block)
        {
            m_freeLists[index] = block->next;
            return block;
        }
    }

    if (m_chunkCount == m_chunkSpace)
    {
        m_chunkSpace += ChunkArrayIncrement;
        m_chunks = Realloc<Chunk>(m_chunks, m_chunkSpace);
        std::memset(m_chunks + m_chunkCount, 0, ChunkArrayIncrement * sizeof(Chunk));
    }

    const auto chunk = m_chunks + m_chunkCount;
    chunk->blocks = static_cast<Block*>(Alloc(ChunkSize));
#if defined(_DEBUG)
    std::memset(chunk->blocks, 0xcd, ChunkSize);
#endif
    const auto blockSize = s_blockSizes[index];
    assert(blockSize > 0);
    chunk->blockSize = blockSize;
    const auto blockCount = ChunkSize / blockSize;
    assert((blockCount * blockSize) <= ChunkSize);
    for (auto i = decltype(blockCount){0}; i < blockCount - 1; ++i)
    {
        const auto block = (Block*)((std::int8_t*)chunk->blocks + blockSize * i);
        const auto next = (Block*)((std::int8_t*)chunk->blocks + blockSize * (i + 1));
        block->next = next;
    }
    const auto last = (Block*)((std::int8_t*)chunk->blocks + blockSize * (blockCount - 1));
    last->next = nullptr;

    m_freeLists[index] = chunk->blocks->next;
    ++m_chunkCount;

    return chunk->blocks;
}

void BlockAllocator::Free(void* p, size_type n)
{
    if (n == 0)
    {
        return;
    }

    assert(n > 0);

    if (n > MaxBlockSize)
    {
        ::Free(p);
        return;
    }

    const auto index = s_blockSizeLookup[n];
    assert((0 <= index) && (index < BlockSizes));

#ifdef _DEBUG
    // Verify the memory address and size is valid.
    const auto blockSize = s_blockSizes[index];
    bool found = false;
    for (auto i = decltype(m_chunkCount){0}; i < m_chunkCount; ++i)
    {
        const auto chunk = m_chunks + i;
        if (chunk->blockSize != blockSize)
        {
            assert(((std::int8_t*)p + blockSize <= (std::int8_t*)chunk->blocks) || ((std::int8_t*)chunk->blocks + ChunkSize <= (std::int8_t*)p));
        }
        else
        {
            if (((std::int8_t*)chunk->blocks <= (std::int8_t*)p) && ((std::int8_t*)p + blockSize <= (std::int8_t*)chunk->blocks + ChunkSize))
            {
                found = true;
            }
        }
    }

    assert(found);

    std::memset(p, 0xfd, blockSize);
#endif

    const auto block = static_cast<Block*>(p);
    block->next = m_freeLists[index];
    m_freeLists[index] = block;
}

void BlockAllocator::Clear()
{
    for (auto i = decltype(m_chunkCount){0}; i < m_chunkCount; ++i)
    {
        ::Free(m_chunks[i].blocks);
    }

    m_chunkCount = 0;
    std::memset(m_chunks, 0, m_chunkSpace * sizeof(Chunk));
    std::memset(m_freeLists, 0, sizeof(m_freeLists));
}
