/*
* Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_DYNAMIC_TREE_H
#define B2_DYNAMIC_TREE_H

#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Common/GrowableStack.h>

namespace box2d {

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by AabbMultiplier
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
class DynamicTree
{
public:
	using size_type = std::remove_const<decltype(MaxContacts)>::type;

	/// Null node index value.
	static constexpr auto NullNode = static_cast<size_type>(-1);
	
	/// Constructing the tree initializes the node pool.
	DynamicTree();

	/// Destroys the tree, freeing the node pool.
	~DynamicTree();

	DynamicTree(const DynamicTree& copy) = delete;
	DynamicTree& operator=(const DynamicTree&) = delete;

	/// Creates a proxy. Provide a tight fitting AABB and a userData pointer.
	/// @return ID of the created proxy.
	size_type CreateProxy(const AABB& aabb, void* userData);

	/// Destroys a proxy. This asserts if the id is invalid.
	void DestroyProxy(size_type proxyId);

	/// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
	/// then the proxy is removed from the tree and re-inserted. Otherwise
	/// the function returns immediately.
	/// @param proxyId Proxy ID. Behavior is undefined if this is the null proxy ID.
	/// @param aabb Axis aligned bounding box.
	/// @param displacement Displacement. Behavior is undefined if this is an invalid value.
	/// @return true if the proxy was re-inserted.
	bool MoveProxy(size_type proxyId, const AABB& aabb, const Vec2& displacement);

	/// Gets the user data for the node identified by the given identifier.
	/// @param proxyId Identifier of node to get the user data for.
	/// @return User data for the specified node.
	/// @note Behavior is undefined if the given index is invalid.
	void* GetUserData(size_type proxyId) const;

	/// Gets the fat AABB for a proxy.
	/// @param proxyId Proxy ID. Must be a valid ID.
	/// @warning Behavior is undefined if the given proxy ID is not a valid ID.
	const AABB& GetFatAABB(size_type proxyId) const;

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const AABB& aabb) const;

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const RayCastInput& input) const;

	/// Validate this tree. For testing.
	void Validate() const;

	/// Gets the height of the binary tree.
	/// @return Zero or more.
	size_type GetHeight() const noexcept;

	/// Gets the maximum balance of an node in the tree. The balance is the difference
	/// in height of the two children of a node.
	size_type GetMaxBalance() const;

	/// Gets the ratio of the sum of the perimeters of nodes to the root perimeter.
	/// @note Zero is returned if no proxies exist at the time of the call.
	/// @return Value of zero or more.
	float_t GetAreaRatio() const;

	/// Build an optimal tree. Very expensive. For testing.
	void RebuildBottomUp();

	/// Shifts the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const Vec2& newOrigin);

private:

	/// A node in the dynamic tree. The client does not interact with this directly.
	struct TreeNode
	{
		bool IsLeaf() const noexcept
		{
			return child1 == NullNode;
		}
		
		/// Enlarged AABB
		AABB aabb;
		
		void* userData;
		
		union
		{
			size_type parent;
			size_type next;
		};
		
		size_type child1; ///< Index of child 1 in DynamicTree::m_nodes or NullNode.
		size_type child2; ///< Index of child 2 in DynamicTree::m_nodes or NullNode.
		
		size_type height; ///< Height - for tree balancing. 0 if leaf node. NullNode if free node.
	};

	size_type AllocateNode();
	void FreeNode(size_type node);

	void InsertLeaf(size_type node);
	void RemoveLeaf(size_type node);

	size_type Balance(size_type index);

	size_type ComputeHeight() const;
	size_type ComputeHeight(size_type nodeId) const;

	void ValidateStructure(size_type index) const;
	void ValidateMetrics(size_type index) const;

	size_type m_root = NullNode; ///< Index of root element in m_nodes or NullNode.

	size_type m_nodeCount = 0;
	size_type m_nodeCapacity = 16;

	size_type m_freeList = 0;

	/// Initialized on construction.
	TreeNode* m_nodes;
};

inline void* DynamicTree::GetUserData(size_type proxyId) const
{
	assert(proxyId != NullNode);
	assert(proxyId < m_nodeCapacity);
	return m_nodes[proxyId].userData;
}

inline const AABB& DynamicTree::GetFatAABB(size_type proxyId) const
{
	assert(proxyId != NullNode);
	assert(proxyId < m_nodeCapacity);
	return m_nodes[proxyId].aabb;
}

inline DynamicTree::size_type DynamicTree::GetHeight() const noexcept
{
	return (m_root != NullNode)? m_nodes[m_root].height: 0;
}

template <typename T>
inline void DynamicTree::Query(T* callback, const AABB& aabb) const
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

		const TreeNode* node = m_nodes + nodeId;

		if (TestOverlap(node->aabb, aabb))
		{
			if (node->IsLeaf())
			{
				const auto proceed = callback->QueryCallback(nodeId);
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

template <typename T>
inline void DynamicTree::RayCast(T* callback, const RayCastInput& input) const
{
	const auto p1 = input.p1;
	const auto p2 = input.p2;

	// v is perpendicular to the segment.
	const auto v = GetRevPerpendicular(GetUnitVector(p2 - p1));
	const auto abs_v = Abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	auto maxFraction = input.maxFraction;

	// Build a bounding box for the segment.
	auto segmentAABB = AABB{p1, p1 + maxFraction * (p2 - p1)};

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

		if (!TestOverlap(node->aabb, segmentAABB))
		{
			continue;
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		const auto c = node->aabb.GetCenter();
		const auto h = node->aabb.GetExtents();
		const auto separation = Abs(Dot(v, p1 - c)) - Dot(abs_v, h);
		if (separation > float_t{0})
		{
			continue;
		}

		if (node->IsLeaf())
		{
			const auto subInput = RayCastInput{input.p1, input.p2, maxFraction};

			const auto value = callback->RayCastCallback(subInput, nodeId);

			if (value == float_t{0})
			{
				// The client has terminated the ray cast.
				return;
			}

			if (value > float_t{0})
			{
				// Update segment bounding box.
				maxFraction = value;
				const auto t = p1 + maxFraction * (p2 - p1);
				segmentAABB = AABB(p1, t);
			}
		}
		else
		{
			stack.Push(node->child1);
			stack.Push(node->child2);
		}
	}
}

} /* namespace box2d */

#endif
