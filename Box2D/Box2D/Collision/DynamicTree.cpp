/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/DynamicTree.hpp>
#include <cstring>

using namespace box2d;

DynamicTree::DynamicTree(const size_type nodeCapacity):
	m_nodeCapacity{nodeCapacity},
	m_nodes{alloc<TreeNode>(nodeCapacity)}
{
	std::memset(m_nodes, 0, nodeCapacity * sizeof(TreeNode));

	// Build a linked list for the free list.
	for (auto i = decltype(nodeCapacity){0}; i < nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = NullNode;
	}
	m_nodes[nodeCapacity - 1].next = NullNode;
	m_nodes[nodeCapacity - 1].height = NullNode;
}

DynamicTree::~DynamicTree() noexcept
{
	// This frees the entire tree in one shot.
	free(m_nodes);
}

// Allocate a node from the pool. Grow the pool if necessary.
DynamicTree::size_type DynamicTree::AllocateNode()
{
	// Expand the node pool as needed.
	if (m_freeList == NullNode)
	{
		assert(m_nodeCount == m_nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		m_nodeCapacity *= 2;
		m_nodes = realloc<TreeNode>(m_nodes, m_nodeCapacity);

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for (auto i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = NullNode;
		}
		m_nodes[m_nodeCapacity - 1].next = NullNode;
		m_nodes[m_nodeCapacity - 1].height = NullNode;
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
void DynamicTree::FreeNode(const size_type nodeId) noexcept
{
	assert(nodeId != NullNode);
	assert(nodeId < m_nodeCapacity);
	assert(m_nodeCount > 0); // nodeId is not ncessarily less than m_nodeCount.
	m_nodes[nodeId].next = m_freeList;
	m_nodes[nodeId].height = NullNode;
	m_freeList = nodeId;
	--m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
DynamicTree::size_type DynamicTree::CreateProxy(const AABB aabb, void* userData)
{
	const auto proxyId = AllocateNode();

	m_nodes[proxyId].aabb = aabb;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;

	InsertLeaf(proxyId);

	return proxyId;
}

void DynamicTree::DestroyProxy(const size_type proxyId)
{
	assert((0 <= proxyId) && (proxyId < m_nodeCapacity));
	assert(m_nodes[proxyId].IsLeaf());

	RemoveLeaf(proxyId);
	FreeNode(proxyId);
}

bool DynamicTree::MoveProxy(const size_type proxyId, const AABB aabb, const Vec2 displacement)
{
	assert((0 <= proxyId) && (proxyId < m_nodeCapacity));
	assert(IsValid(displacement));
	assert(m_nodes[proxyId].IsLeaf());

	if (m_nodes[proxyId].aabb.Contains(aabb))
	{
		return false;
	}

	RemoveLeaf(proxyId);

	auto lowerBound = aabb.GetLowerBound();
	auto upperBound = aabb.GetUpperBound();
	
	// Predict AABB displacement.
	const auto d = AabbMultiplier * displacement;

	if (d.x < RealNum{0})
	{
		lowerBound.x += d.x;
	}
	else
	{
		upperBound.x += d.x;
	}

	if (d.y < RealNum{0})
	{
		lowerBound.y += d.y;
	}
	else
	{
		upperBound.y += d.y;
	}

	m_nodes[proxyId].aabb = AABB{lowerBound, upperBound};

	InsertLeaf(proxyId);
	return true;
}

DynamicTree::size_type DynamicTree::FindLowestCostNode(const AABB leafAABB) const noexcept
{
	assert(m_root != NullNode);

	auto index = m_root;
	while (!m_nodes[index].IsLeaf())
	{
		const auto child1 = m_nodes[index].child1;
		const auto child2 = m_nodes[index].child2;
		
		const auto area = GetPerimeter(m_nodes[index].aabb);
		
		const auto combinedAABB = GetEnclosingAABB(m_nodes[index].aabb, leafAABB);
		const auto combinedArea = GetPerimeter(combinedAABB);
		
		assert(combinedArea >= area);
		
		// Cost of creating a new parent for this node and the new leaf
		const auto cost = combinedArea * 2;
		
		// Minimum cost of pushing the leaf further down the tree
		const auto inheritanceCost = (combinedArea - area) * 2;
		
		assert(child1 != NullNode);
		assert(child1 < m_nodeCapacity);
		
		// Cost of descending into child1
		const auto cost1 = [&]() {
			const auto aabb = GetEnclosingAABB(leafAABB, m_nodes[child1].aabb);
			const auto perimeter = GetPerimeter(aabb);
			return (m_nodes[child1].IsLeaf())?
			perimeter + inheritanceCost:
			perimeter - GetPerimeter(m_nodes[child1].aabb) + inheritanceCost;
		}();
		
		// Cost of descending into child2
		const auto cost2 = [&]() {
			const auto aabb = GetEnclosingAABB(leafAABB, m_nodes[child2].aabb);
			const auto perimeter = GetPerimeter(aabb);
			return (m_nodes[child2].IsLeaf())?
			perimeter + inheritanceCost:
			perimeter - GetPerimeter(m_nodes[child2].aabb) + inheritanceCost;
		}();
		
		// Descend according to the minimum cost.
		if ((cost < cost1) && (cost < cost2))
		{
			break;
		}
		
		// Descend
		index = (cost1 < cost2)? child1: child2;
	}
	return index;
}

void DynamicTree::InsertLeaf(const size_type leaf)
{
	assert(leaf != NullNode);

	if (m_root == NullNode)
	{
		m_root = leaf;
		m_nodes[m_root].parent = NullNode;
		return;
	}

	assert(leaf < m_nodeCapacity);

	const auto leafAABB = m_nodes[leaf].aabb;
	
	// Find the best sibling for this node
	const auto sibling = FindLowestCostNode(leafAABB);

	// Create a new parent.
	const auto oldParent = m_nodes[sibling].parent;
	const auto newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb = GetEnclosingAABB(leafAABB, m_nodes[sibling].aabb);
	assert(m_nodes[sibling].height != NullNode);
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
	auto index = m_nodes[leaf].parent;
	while (index != NullNode)
	{
		index = Balance(index);

		const auto child1 = m_nodes[index].child1;
		const auto child2 = m_nodes[index].child2;

		assert(child1 != NullNode);
		assert(child2 != NullNode);

		m_nodes[index].height = 1 + Max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[index].aabb = GetEnclosingAABB(m_nodes[child1].aabb, m_nodes[child2].aabb);

		index = m_nodes[index].parent;
	}

	//Validate();
}

void DynamicTree::RemoveLeaf(const size_type leaf)
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

			m_nodes[index].aabb = GetEnclosingAABB(m_nodes[child1].aabb, m_nodes[child2].aabb);
			assert(m_nodes[child1].height != NullNode);
			assert(m_nodes[child2].height != NullNode);
			m_nodes[index].height = 1 + Max(m_nodes[child1].height, m_nodes[child2].height);

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
DynamicTree::size_type DynamicTree::Balance(const size_type iA)
{
	assert(iA != NullNode);
	assert(iA < m_nodeCapacity);

	const auto A = m_nodes + iA;
	if (A->IsLeaf() || (A->height == NullNode) || (A->height < 2))
	{
		return iA;
	}

	const auto iB = A->child1;
	const auto iC = A->child2;
	assert(iB != NullNode);
	assert(iB < m_nodeCapacity);
	assert(iC != NullNode);
	assert(iC < m_nodeCapacity);

	const auto B = m_nodes + iB;
	const auto C = m_nodes + iC;

	assert(B->height != NullNode);
	assert(C->height != NullNode);

	// Rotate C up
	if (C->height > (B->height + 1))
	{
		const auto iF = C->child1;
		const auto iG = C->child2;
		assert((0 <= iF) && (iF < m_nodeCapacity));
		assert((0 <= iG) && (iG < m_nodeCapacity));

		const auto F = m_nodes + iF;
		const auto G = m_nodes + iG;
		
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
		assert(F->height != NullNode);
		assert(G->height != NullNode);
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb = GetEnclosingAABB(B->aabb, G->aabb);
			C->aabb = GetEnclosingAABB(A->aabb, F->aabb);
			A->height = 1 + Max(B->height, G->height);
			C->height = 1 + Max(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb = GetEnclosingAABB(B->aabb, F->aabb);
			C->aabb = GetEnclosingAABB(A->aabb, G->aabb);
			A->height = 1 + Max(B->height, F->height);
			C->height = 1 + Max(A->height, G->height);
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

		assert(D->height != NullNode);
		assert(E->height != NullNode);

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb = GetEnclosingAABB(C->aabb, E->aabb);
			B->aabb = GetEnclosingAABB(A->aabb, D->aabb);
			A->height = 1 + Max(C->height, E->height);
			B->height = 1 + Max(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb = GetEnclosingAABB(C->aabb, D->aabb);
			B->aabb = GetEnclosingAABB(A->aabb, E->aabb);
			A->height = 1 + Max(C->height, D->height);
			B->height = 1 + Max(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

RealNum DynamicTree::GetAreaRatio() const noexcept
{
	if (m_root == NullNode)
	{
		return RealNum{0};
	}

	const auto root = m_nodes + m_root;
	const auto rootArea = GetPerimeter(root->aabb);

	auto totalArea = RealNum{0};
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		const auto node = m_nodes + i;
		if (node->height == NullNode)
		{
			// Free node in pool
			continue;
		}

		totalArea += GetPerimeter(node->aabb);
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
DynamicTree::size_type DynamicTree::ComputeHeight(const size_type nodeId) const noexcept
{
	assert((0 <= nodeId) && (nodeId < m_nodeCapacity));
	const auto node = m_nodes + nodeId;

	if (node->IsLeaf())
	{
		return 0;
	}

	const auto height1 = ComputeHeight(node->child1);
	const auto height2 = ComputeHeight(node->child2);
	return 1 + Max(height1, height2);
}

void DynamicTree::Query(std::function<bool(size_type)> callback, const AABB aabb) const
{
	GrowableStack<size_type, 256> stack;
	stack.Push(m_root);
	
	while (stack.GetCount() > 0)
	{
		const auto nodeId = stack.Pop();
		if (nodeId == NullNode)
		{
			continue;
		}
		
		const auto node = m_nodes + nodeId;
		if (TestOverlap(node->aabb, aabb))
		{
			if (node->IsLeaf())
			{
				const auto proceed = callback(nodeId);
				if (!proceed)
				{
					return;
				}
			}
			else
			{
				stack.Push(node->child1);
				stack.Push(node->child2);
			}
		}
	}
}

bool DynamicTree::ValidateStructure(const size_type index) const noexcept
{
	if (index == NullNode)
	{
		return true;
	}

	if (index == m_root)
	{
		if (m_nodes[index].parent != NullNode)
		{
			return false;
		}
	}

	if (index >= m_nodeCapacity)
	{
		return false;
	}

	const auto node = m_nodes + index;

	const auto child1 = node->child1;
	const auto child2 = node->child2;

	if (node->IsLeaf())
	{
		if (child1 != NullNode)
		{
			return false;
		}
		if (child2 != NullNode)
		{
			return false;
		}
		if (node->height != 0)
		{
			return false;
		}
		return true;
	}

	if (child1 >= m_nodeCapacity)
	{
		return false;
	}
	if (child2 >= m_nodeCapacity)
	{
		return false;
	}

	if (m_nodes[child1].parent != index)
	{
		return false;
	}
	if (m_nodes[child2].parent != index)
	{
		return false;
	}

	if (!ValidateStructure(child1))
	{
		return false;
	}
	if (!ValidateStructure(child2))
	{
		return false;
	}
	
	return true;
}

bool DynamicTree::ValidateMetrics(size_type index) const noexcept
{
	if (index == NullNode)
	{
		return true;
	}

	if (index >= m_nodeCapacity)
	{
		return false;
	}

	const auto node = m_nodes + index;

	const auto child1 = node->child1;
	const auto child2 = node->child2;

	if (node->IsLeaf())
	{
		if (child1 != NullNode)
		{
			return false;
		}
		if (child2 != NullNode)
		{
			return false;
		}
		if (node->height != 0)
		{
			return false;
		}
		return true;
	}

	if (child1 >= m_nodeCapacity)
	{
		return false;
	}
	if (child2 >= m_nodeCapacity)
	{
		return false;
	}

	{
		const auto height1 = m_nodes[child1].height;
		const auto height2 = m_nodes[child2].height;
		const auto height = 1 + Max(height1, height2);
		if (node->height != height)
		{
			return false;
		}
	}
	{
		const auto aabb = GetEnclosingAABB(m_nodes[child1].aabb, m_nodes[child2].aabb);
		if (aabb.GetLowerBound() != node->aabb.GetLowerBound())
		{
			return false;
		}
		if (aabb.GetUpperBound() != node->aabb.GetUpperBound())
		{
			return false;
		}
	}

	if (!ValidateMetrics(child1))
	{
		return false;
	}
	if (!ValidateMetrics(child2))
	{
		return false;
	}
	
	return true;
}

bool DynamicTree::Validate() const
{
	if (!ValidateStructure(m_root))
	{
		return false;
	}

	if (!ValidateMetrics(m_root))
	{
		return false;
	}

	auto freeCount = size_type{0};
	auto freeIndex = m_freeList;
	while (freeIndex != NullNode)
	{
		if (freeIndex >= GetNodeCapacity())
		{
			return false;
		}
		freeIndex = m_nodes[freeIndex].next;
		++freeCount;
	}

	if ((m_root != NullNode) && (GetHeight() != ComputeHeight()))
	{
		return false;
	}
	if ((GetNodeCount() + freeCount) != GetNodeCapacity())
	{
		return false;
	}
	
	return true;
}

DynamicTree::size_type DynamicTree::GetMaxBalance() const
{
	auto maxBalance = size_type{0};
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		const auto node = m_nodes + i;
		
		if (node->height == NullNode)
		{
			continue;
		}

		if (node->height <= 1)
		{
			continue;
		}

		assert(!node->IsLeaf());

		const auto child1 = node->child1;
		assert(child1 != NullNode);
		assert(child1 < m_nodeCapacity);
		const auto child2 = node->child2;
		assert(child2 != NullNode);
		assert(child2 < m_nodeCapacity);
		assert(m_nodes[child1].height != NullNode);
		assert(m_nodes[child2].height != NullNode);
		const auto balance = Abs(m_nodes[child2].height - m_nodes[child1].height);
		maxBalance = Max(maxBalance, balance);
	}

	return maxBalance;
}

void DynamicTree::RebuildBottomUp()
{
	const auto nodes = alloc<size_type>(m_nodeCount);
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
		auto minCost = std::numeric_limits<RealNum>::infinity();
		auto iMin = NullNode;
		auto jMin = NullNode;
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto& aabbi = m_nodes[nodes[i]].aabb;

			for (auto j = i + 1; j < count; ++j)
			{
				const auto& aabbj = m_nodes[nodes[j]].aabb;
				const auto b = GetEnclosingAABB(aabbi, aabbj);
				const auto cost = GetPerimeter(b);
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
		assert(child1->height != NullNode);
		assert(child2->height != NullNode);
		parent->height = 1 + Max(child1->height, child2->height);
		parent->aabb = GetEnclosingAABB(child1->aabb, child2->aabb);
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

void DynamicTree::ShiftOrigin(const Vec2 newOrigin)
{
	// Build array of leaves. Free the rest.
	for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
	{
		m_nodes[i].aabb.Move(-newOrigin);
	}
}
