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

#ifndef B2_BLOCK_ALLOCATOR_H
#define B2_BLOCK_ALLOCATOR_H

#include <Box2D/Common/b2Settings.h>

namespace box2d {

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class BlockAllocator
{
public:
	using size_type = size_t;

	static constexpr auto ChunkSize = size_type{16 * 1024};
	static constexpr auto MaxBlockSize = size_type{640};
	static constexpr auto BlockSizes = size_type{14};
	static constexpr auto ChunkArrayIncrement = size_type{128};

	BlockAllocator();
	~BlockAllocator();

	/// Allocate memory. This will use alloc if the size is larger than MaxBlockSize.
	void* Allocate(size_type size);

	/// Free memory. This will use free if the size is larger than MaxBlockSize.
	void Free(void* p, size_type size);

	void Clear();

private:
	struct Chunk;
	struct Block;
	
	size_type m_chunkCount = 0;
	size_type m_chunkSpace = ChunkArrayIncrement;
	Chunk* m_chunks;
	Block* m_freeLists[BlockSizes];
};

} // namespace box2d

#endif
