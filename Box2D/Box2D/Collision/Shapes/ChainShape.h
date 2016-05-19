/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef B2_CHAIN_SHAPE_H
#define B2_CHAIN_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.h>

namespace box2d {

class EdgeShape;

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
class ChainShape : public Shape
{
public:
	ChainShape(): Shape(e_chain, PolygonRadius) {}

	ChainShape(const ChainShape&) = delete;

	/// The destructor frees the vertices using free.
	~ChainShape();

	ChainShape& operator=(const ChainShape&) = delete;

	/// Clear all data.
	void Clear();

	/// Create a loop. This automatically adjusts connectivity.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	void CreateLoop(const Vec2* vertices, child_count_t count);

	/// Create a chain with isolated end vertices.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	void CreateChain(const Vec2* vertices, child_count_t count);

	/// Establish connectivity to a vertex that precedes the first vertex.
	/// Don't call this for loops.
	void SetPrevVertex(const Vec2& prevVertex) noexcept;

	/// Establish connectivity to a vertex that follows the last vertex.
	/// Don't call this for loops.
	void SetNextVertex(const Vec2& nextVertex) noexcept;

	/// Implement Shape. Vertices are cloned using alloc.
	Shape* Clone(BlockAllocator* allocator) const override;

	/// @see Shape::GetChildCount
	child_count_t GetChildCount() const override;

	/// Get a child edge.
	void GetChildEdge(EdgeShape* edge, child_count_t index) const;

	/// This always return false.
	/// @see Shape::TestPoint
	bool TestPoint(const Transform& transform, const Vec2& p) const override;

	/// Implement Shape.
	bool RayCast(RayCastOutput* output, const RayCastInput& input,
					const Transform& transform, child_count_t childIndex) const override;

	/// @see Shape::ComputeAABB
	AABB ComputeAABB(const Transform& transform, child_count_t childIndex) const override;

	/// Chains have zero mass.
	/// @see Shape::ComputeMass
	MassData ComputeMass(float_t density) const override;

	/// Get the vertex count.
	child_count_t GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index.
	const Vec2& GetVertex(child_count_t index) const;

	bool HasPrevVertex() const noexcept { return m_hasPrevVertex; }
	bool HasNextVertex() const noexcept { return m_hasNextVertex; }

	Vec2 GetPrevVertex() const noexcept { return m_prevVertex; }
	Vec2 GetNextVertex() const noexcept { return m_nextVertex; }

private:
	/// The vertices. Owned by this class.
	Vec2* m_vertices = nullptr;

	/// The vertex count.
	child_count_t m_count = 0;

	Vec2 m_prevVertex, m_nextVertex;
	bool m_hasPrevVertex = false, m_hasNextVertex = false;
};

inline const Vec2& ChainShape::GetVertex(child_count_t index) const
{
	assert((0 <= index) && (index < m_count));
	return m_vertices[index];
}

} // namespace box2d

#endif
