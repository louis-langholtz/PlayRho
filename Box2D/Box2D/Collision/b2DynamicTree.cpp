/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/b2DynamicTree.h>
#include <cstring>

using namespace box2d;

b2DynamicTree::b2DynamicTree():
	m_nodes(static_cast<b2TreeNode*>(alloc(m_nodeCapacity * sizeof(b2TreeNode))))
{
	std::memset(m_nodes, 0, m_nodeCapacity * sizeof(b2TreeNode));

	// Build a linked list for the free list.
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = NullNode;
	}
	m_nodes[m_nodeCapacity-1].next = NullNode;
	m_nodes[m_nodeCapacity-1].height = NullNode;
}

b2DynamicTree::~b2DynamicTree()
{
	// This frees the entire tree in one shot.
	free(m_nodes);
}

// Allocate a node from the pool. Grow the pool if necessary.
b2DynamicTree::size_type b2DynamicTree::AllocateNode()
{
	// Expand the node pool as needed.
	if (m_freeList == NullNode)
	{
		assert(m_nodeCount == m_nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		m_nodeCapacity *= 2;
		m_nodes = static_cast<b2TreeNode*>(realloc(m_nodes, m_nodeCapacity * sizeof(b2TreeNode)));

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for (auto i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = NullNode;
		}
		m_nodes[m_nodeCapacity-1].next = NullNode;
		m_nodes[m_nodeCapacity-1].height = NullNode;
		m_freeList = m_nodeCount;
	}

	// Peel a node off the free list.
	const auto nodeId = m_freeList;
	m_freeList = m_nodes[nodeId].next;
	m_nodes[nodeId].parent = NullNode;
	m_nodes[nodeId].child1 = NullNode;
	m_nodes[nodeId].child2 = NullNode;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].userData = nullptr;
	++m_nodeCount;
	return nodeId;
}

// Return a node to the pool.
void b2DynamicTree::FreeNode(size_type nodeId)
{
	assert(nodeId < m_nodeCapacity);
	assert(m_nodeCount > 0);
	m_nodes[nodeId].next = m_freeList;
	m_nodes[nodeId].height = NullNode;
	m_freeList = nodeId;
	--m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
b2DynamicTree::size_type b2DynamicTree::CreateProxy(const b2AABB& aabb, void* userData)
{
	const auto proxyId = AllocateNode();

	// Fatten the aabb.
	m_nodes[proxyId].aabb = aabb + b2Vec2(AabbExtension, AabbExtension);
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;

	InsertLeaf(proxyId);

	return proxyId;
}

void b2DynamicTree::DestroyProxy(size_type proxyId)
{
	assert((0 <= proxyId) && (proxyId < m_nodeCapacity));
	assert(m_nodes[proxyId].IsLeaf());

	RemoveLeaf(proxyId);
	FreeNode(proxyId);
}

bool b2DynamicTree::MoveProxy(size_type proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
	assert((0 <= proxyId) && (proxyId < m_nodeCapacity));

	assert(m_nodes[proxyId].IsLeaf());

	if (m_nodes[proxyId].aabb.Contains(aabb))
	{
		return false;
	}

	RemoveLeaf(proxyId);

	// Extend AABB.
	auto b = aabb + b2Vec2(AabbExtension, AabbExtension);
	auto lowerBound = b.GetLowerBound();
	auto upperBound = b.GetUpperBound();
	
	// Predict AABB displacement.
	const auto d = AabbMultiplier * displacement;

	if (d.x < float_t{0})
	{
		lowerBound.x += d.x;
	}
	else
	{
		upperBound.x += d.x;
	}

	if (d.y < float_t{0})
	{
		lowerBound.y += d.y;
	}
	else
	{
		upperBound.y += d.y;
	}

	m_nodes[proxyId].aabb = b2AABB{lowerBound, upperBound};

	InsertLeaf(proxyId);
	return true;
}

void b2DynamicTree::InsertLeaf(size_type leaf)
{
	assert(leaf != NullNode);

	++m_insertionCount;

	if (m_root == NullNode)
	{
		m_root = leaf;
		m_nodes[m_root].parent = NullNode;
		return;
	}

	assert(leaf < m_nodeCapacity);

	// Find the best sibling for this node
	const auto leafAABB = m_nodes[leaf].aabb;
	auto index = m_root;
	while (!m_nodes[index].IsLeaf())
	{
		const auto child1 = m_nodes[index].child1;
		const auto child2 = m_nodes[index].child2;

		const auto area = m_nodes[index].aabb.GetPerimeter();

		const auto combinedAABB = m_nodes[index].aabb + leafAABB;
		const auto combinedArea = combinedAABB.GetPerimeter();

		// Cost of creating a new parent for this node and the new leaf
		const auto cost = float_t(2) * combinedArea;

		// Minimum cost of pushing the leaf further down the tree
		const auto inheritanceCost = float_t(2) * (combinedArea - area);

		// Cost of descending into child1
		float_t cost1;
		assert(child1 != NullNode);
		assert(child1 < m_nodeCapacity);
		if (m_nodes[child1].IsLeaf())
		{
			const auto aabb = leafAABB + m_nodes[child1].aabb;
			cost1 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			const auto aabb = leafAABB + m_nodes[child1].aabb;
			const auto oldArea = m_nodes[child1].aabb.GetPerimeter();
			const auto newArea = aabb.GetPerimeter();
			cost1 = (newArea - oldArea) + inheritanceCost;
		}

		// Cost of descending into child2
		float_t cost2;
		if (m_nodes[child2].IsLeaf())
		{
			const auto aabb = leafAABB + m_nodes[child2].aabb;
			cost2 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			const auto aabb = leafAABB + m_nodes[child2].aabb;
			const auto oldArea = m_nodes[child2].aabb.GetPerimeter();
			cost2 = aabb.GetPerimeter() - oldArea + inheritanceCost;
		}

		// Descend according to the minimum cost.
		if ((cost < cost1) && (cost < cost2))
		{
			break;
		}

		// Descend
		if (cost1 < cost2)
		{
			index = child1;
		}
		else
		{
			index = child2;
		}
	}

	const auto sibling = index;

	// Create a new parent.
	const auto oldParent = m_nodes[sibling].parent;
	const auto newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb = leafAABB + m_nodes[sibling].aabb;
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	if (oldParent != NullNode)
	{
		// The sibling was not the root.
		if (m_nodes[oldParent].child1 == sibling)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}

	// Walk back up the tree fixing heights and AABBs
	index = m_nodes[leaf].parent;
	while (index != NullNode)
	{
		index = Balance(index);

		const auto child1 = m_nodes[index].child1;
		const auto child2 = m_nodes[index].child2;

		assert(child1 != NullNode);
		assert(child2 != NullNode);

		m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[index].aabb = m_nodes[child1].aabb + m_nodes[child2].aabb;

		index = m_nodes[index].parent;
	}

	//Validate();
}

void b2DynamicTree::RemoveLeaf(size_type leaf)
{
	if (leaf == m_root)
	{
		m_root = NullNode;
		return;
	}

	assert(leaf < m_nodeCapacity);
	const auto parent = m_nodes[leaf].parent;

	assert(parent < m_nodeCapacity);
	const auto grandParent = m_nodes[parent].parent;
	
	const auto sibling = (m_nodes[parent].child1 == leaf)? m_nodes[parent].child2: m_nodes[parent].child1;

	if (grandParent != NullNode)
	{
		// Destroy parent and connect sibling to grandParent.
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = sibling;
		}
		else
		{
			m_nodes[grandParent].child2 = sibling;
		}
		m_nodes[sibling].parent = grandParent;
		FreeNode(parent);

		// Adjust ancestor bounds.
		auto index = grandParent;
		while (index != NullNode)
		{
			index = Balance(index);

			const auto child1 = m_nodes[index].child1;
			const auto child2 = m_nodes[index].child2;

			assert(child1 != NullNode);
			assert(child1 < m_nodeCapacity);
			assert(child2 != NullNode);
			assert(child2 < m_nodeCapacity);

			m_nodes[index].aabb = m_nodes[child1].aabb + m_nodes[child2].aabb;
			m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);

			index = m_nodes[index].parent;
		}
	}
	else
	{
		m_root = sibling;
		m_nodes[sibling].parent = NullNode;
		FreeNode(parent);
	}

	//Validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
b2DynamicTree::size_type b2DynamicTree::Balance(size_type iA)
{
	assert(iA != NullNode);
	assert(iA < m_nodeCapacity);

	auto A = m_nodes + iA;
	if (A->IsLeaf() || (A->height < 2))
	{
		return iA;
	}

	const auto iB = A->child1;
	const auto iC = A->child2;
	assert(iB < m_nodeCapacity);
	assert(iC < m_nodeCapacity);

	auto B = m_nodes + iB;
	auto C = m_nodes + iC;

	// Rotate C up
	if (C->height > (B->height + 1))
	{
		const auto iF = C->child1;
		const auto iG = C->child2;
		assert((0 <= iF) && (iF < m_nodeCapacity));
		assert((0 <= iG) && (iG < m_nodeCapacity));

		auto F = m_nodes + iF;
		auto G = m_nodes + iG;
		
		// Swap A and C
		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		// A's old parent should point to C
		if (C->parent != NullNode)
		{
			if (m_nodes[C->parent].child1 == iA)
			{
				m_nodes[C->parent].child1 = iC;
			}
			else
			{
				assert(m_nodes[C->parent].child2 == iA);
				m_nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			m_root = iC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb = B->aabb + G->aabb;
			C->aabb = A->aabb + F->aabb;
			A->height = 1 + b2Max(B->height, G->height);
			C->height = 1 + b2Max(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb = B->aabb + F->aabb;
			C->aabb = A->aabb + G->aabb;
			A->height = 1 + b2Max(B->height, F->height);
			C->height = 1 + b2Max(A->height, G->height);
		}

		return iC;
	}
	
	// Rotate B up
	if (B->height > (C->height + 1))
	{
		const auto iD = B->child1;
		const auto iE = B->child2;
		const auto D = m_nodes + iD;
		const auto E = m_nodes + iE;
		assert((0 <= iD) && (iD < m_nodeCapacity));
		assert((0 <= iE) && (iE < m_nodeCapacity));

		// Swap A and B
		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		// A's old parent should point to B
		if (B->parent != NullNode)
		{
			if (m_nodes[B->parent].child1 == iA)
			{
				m_nodes[B->parent].child1 = iB;
			}
			else
			{
				assert(m_nodes[B->parent].child2 == iA);
				m_nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			m_root = iB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb = C->aabb + E->aabb;
			B->aabb = A->aabb + D->aabb;
			A->height = 1 + b2Max(C->height, E->height);
			B->height = 1 + b2Max(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb = C->aabb + D->aabb;
			B->aabb = A->aabb + E->aabb;
			A->height = 1 + b2Max(C->height, D->height);
			B->height = 1 + b2Max(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

float_t b2DynamicTree::GetAreaRatio() const
{
	if (m_root == NullNode)
	{
		return float_t{0};
	}

	const auto root = m_nodes + m_root;
	const auto rootArea = root->aabb.GetPerimeter();

	auto totalArea = float_t{0};
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		const auto node = m_nodes + i;
		if (node->height == NullNode)
		{
			// Free node in pool
			continue;
		}

		totalArea += node->aabb.GetPerimeter();
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
b2DynamicTree::size_type b2DynamicTree::ComputeHeight(size_type nodeId) const
{
	assert((0 <= nodeId) && (nodeId < m_nodeCapacity));
	const auto node = m_nodes + nodeId;

	if (node->IsLeaf())
	{
		return 0;
	}

	const auto height1 = ComputeHeight(node->child1);
	const auto height2 = ComputeHeight(node->child2);
	return 1 + b2Max(height1, height2);
}

b2DynamicTree::size_type b2DynamicTree::ComputeHeight() const
{
	return ComputeHeight(m_root);
}

void b2DynamicTree::ValidateStructure(size_type index) const
{
	if (index == NullNode)
	{
		return;
	}

	if (index == m_root)
	{
		assert(m_nodes[index].parent == NullNode);
	}

	assert(index < m_nodeCapacity);

	const auto node = m_nodes + index;

	const auto child1 = node->child1;
	const auto child2 = node->child2;

	if (node->IsLeaf())
	{
		assert(child1 == NullNode);
		assert(child2 == NullNode);
		assert(node->height == 0);
		return;
	}

	assert((0 <= child1) && (child1 < m_nodeCapacity));
	assert((0 <= child2) && (child2 < m_nodeCapacity));

	assert(m_nodes[child1].parent == index);
	assert(m_nodes[child2].parent == index);

	ValidateStructure(child1);
	ValidateStructure(child2);
}

void b2DynamicTree::ValidateMetrics(size_type index) const
{
	if (index == NullNode)
	{
		return;
	}

	assert(index < m_nodeCapacity);

	const auto node = m_nodes + index;

	const auto child1 = node->child1;
	const auto child2 = node->child2;

	if (node->IsLeaf())
	{
		assert(child1 == NullNode);
		assert(child2 == NullNode);
		assert(node->height == 0);
		return;
	}

	assert((0 <= child1) && (child1 < m_nodeCapacity));
	assert((0 <= child2) && (child2 < m_nodeCapacity));

#if !defined(NDEBUG)
	{
		const auto height1 = m_nodes[child1].height;
		const auto height2 = m_nodes[child2].height;
		const auto height = 1 + b2Max(height1, height2);
		assert(node->height == height);
	}
	{
		const auto aabb = m_nodes[child1].aabb + m_nodes[child2].aabb;
		assert(aabb.GetLowerBound() == node->aabb.GetLowerBound());
		assert(aabb.GetUpperBound() == node->aabb.GetUpperBound());
	}
#endif

	ValidateMetrics(child1);
	ValidateMetrics(child2);
}

void b2DynamicTree::Validate() const
{
	ValidateStructure(m_root);
	ValidateMetrics(m_root);

	auto freeCount = size_type{0};
	auto freeIndex = m_freeList;
	while (freeIndex != NullNode)
	{
		assert((0 <= freeIndex) && (freeIndex < m_nodeCapacity));
		freeIndex = m_nodes[freeIndex].next;
		++freeCount;
	}

	assert(GetHeight() == ComputeHeight());

	assert((m_nodeCount + freeCount) == m_nodeCapacity);
}

b2DynamicTree::size_type b2DynamicTree::GetMaxBalance() const
{
	auto maxBalance = size_type{0};
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		const auto node = m_nodes + i;
		if (node->height <= 1)
		{
			continue;
		}

		assert(!node->IsLeaf());

		const auto child1 = node->child1;
		assert(child1 < m_nodeCapacity);
		const auto child2 = node->child2;
		assert(child2 < m_nodeCapacity);
		const auto balance = b2Abs(m_nodes[child2].height - m_nodes[child1].height);
		maxBalance = b2Max(maxBalance, balance);
	}

	return maxBalance;
}

void b2DynamicTree::RebuildBottomUp()
{
	auto nodes = static_cast<size_type*>(alloc(m_nodeCount * sizeof(size_type)));
	auto count = size_type{0};

	// Build array of leaves. Free the rest.
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		if (m_nodes[i].height == NullNode)
		{
			// free node in pool
			continue;
		}

		if (m_nodes[i].IsLeaf())
		{
			m_nodes[i].parent = NullNode;
			nodes[count] = i;
			++count;
		}
		else
		{
			FreeNode(i);
		}
	}

	while (count > 1)
	{
		auto minCost = MaxFloat;
		auto iMin = NullNode;
		auto jMin = NullNode;
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto& aabbi = m_nodes[nodes[i]].aabb;

			for (auto j = i + 1; j < count; ++j)
			{
				const auto& aabbj = m_nodes[nodes[j]].aabb;
				const auto b = aabbi + aabbj;
				const auto cost = b.GetPerimeter();
				if (minCost > cost)
				{
					iMin = i;
					jMin = j;
					minCost = cost;
				}
			}
		}

		assert((iMin < m_nodeCount) && (jMin < m_nodeCount));

		const auto index1 = nodes[iMin];
		const auto index2 = nodes[jMin];
		auto child1 = m_nodes + index1;
		auto child2 = m_nodes + index2;

		const auto parentIndex = AllocateNode();
		auto parent = m_nodes + parentIndex;
		parent->child1 = index1;
		parent->child2 = index2;
		parent->height = 1 + b2Max(child1->height, child2->height);
		parent->aabb = child1->aabb + child2->aabb;
		parent->parent = NullNode;

		child1->parent = parentIndex;
		child2->parent = parentIndex;

		nodes[jMin] = nodes[count-1];
		nodes[iMin] = parentIndex;
		--count;
	}

	m_root = nodes[0];
	free(nodes);

	Validate();
}

void b2DynamicTree::ShiftOrigin(const b2Vec2& newOrigin)
{
	// Build array of leaves. Free the rest.
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		m_nodes[i].aabb.Move(-newOrigin);
	}
}
