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

#ifndef B2_BLOCK_ALLOCATOR_H
#define B2_BLOCK_ALLOCATOR_H

#include <Box2D/Common/Settings.hpp>

namespace box2d {

    /// Block allocator.
    ///
    /// This is a small object allocator used for allocating small
    ///   objects that persist for more than one time step.
    /// @note This data structure is 136-bytes large (on at least one 64-bit platform).
    /// @sa http://www.codeproject.com/useritems/Small_Block_Allocator.asp
    ///
    class BlockAllocator
    {
    public:
        using size_type = size_t;
        
        static constexpr auto ChunkSize = size_type{16 * 1024}; ///< Chunk size.
        static constexpr auto MaxBlockSize = size_type{640}; ///< Max block size (before using external allocator).
        static constexpr auto BlockSizes = size_type{14};
        static constexpr auto ChunkArrayIncrement = size_type{128};
        
        BlockAllocator();
        ~BlockAllocator() noexcept;
        
        /// Allocates memory.
        /// @details Allocates uninitialized storage.
        ///   Uses <code>alloc</code> if the size is larger than <code>MaxBlockSize</code>.
        ///   Otherwise looks for an appropriately sized block from the free list.
        ///   Failing that, <code>alloc</code> is used to grow the free list from which
        ///   memory is returned.
        /// @sa alloc.
        void* Allocate(size_type n);
        
        template <typename T>
        T* AllocateArray(size_type n)
        {
            return static_cast<T*>(Allocate(n * sizeof(T)));
        }
        
        /// Free memory.
        /// @details This will use free if the size is larger than <code>MaxBlockSize</code>.
        void Free(void* p, size_type n);
        
        /// Clears this allocator.
        /// @note This resets the chunk-count back to zero.
        void Clear();
        
        auto GetChunkCount() const noexcept
        {
            return m_chunkCount;
        }

    private:
        struct Chunk;
        struct Block;
        
        size_type m_chunkCount = 0;
        size_type m_chunkSpace = ChunkArrayIncrement;
        Chunk* m_chunks;
        Block* m_freeLists[BlockSizes];
    };
    
    template <typename T>
    inline void Delete(const T* p, BlockAllocator& allocator)
    {
        p->~T();
        allocator.Free(const_cast<T*>(p), sizeof(T));
    }
    
    /// Blockl Deallocator.
    struct BlockDeallocator
    {
        using size_type = BlockAllocator::size_type;
        
        BlockDeallocator() = default;
        
        constexpr BlockDeallocator(BlockAllocator* a, size_type n) noexcept: allocator{a}, nelem{n} {} 
        
        void operator()(void *p) noexcept
        {
            allocator->Free(p, nelem);
        }
        
        BlockAllocator* allocator;
        size_type nelem;
    };
    
    inline bool operator==(const BlockAllocator& a, const BlockAllocator& b)
    {
        return &a == &b;
    }
    
    inline bool operator!=(const BlockAllocator& a, const BlockAllocator& b)
    {
        return &a != &b;
    }
    
} // namespace box2d

#endif
