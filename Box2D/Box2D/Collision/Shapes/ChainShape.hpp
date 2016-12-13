/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef B2_CHAIN_SHAPE_H
#define B2_CHAIN_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.hpp>

namespace box2d {

class EdgeShape;

/// Chain shape.
///
/// @detail A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using alloc.
/// Connectivity information is used to create smooth collisions.
///
/// @warning The chain will not collide properly if there are self-intersections.
///
class ChainShape : public Shape
{
public:
	static constexpr float_t GetDefaultVertexRadius() noexcept
	{
		return LinearSlop * 2;
	}
	
	ChainShape(float_t vertexRadius = GetDefaultVertexRadius()):
		Shape{e_chain, vertexRadius}
	{
		// Intentionally empty.
	}

	ChainShape(const ChainShape& other);

	/// The destructor frees the vertices using free.
	~ChainShape();

	ChainShape& operator=(const ChainShape& other);

	/// Clear all data.
	void Clear();

	/// Create a loop. This automatically adjusts connectivity.
	/// @note Behavior is undefined if vertices is null or if count of vertices is less than 3.
	/// @param vertices Non-null array of vertices. These are copied.
	void CreateLoop(Span<const Vec2> vertices);

	/// Create a chain with isolated end vertices.
	/// @param vertices an array of vertices, these are copied
	void CreateChain(Span<const Vec2> vertices);

	/// Establish connectivity to a vertex that precedes the first vertex.
	/// Don't call this for loops.
	void SetPrevVertex(Vec2 prevVertex) noexcept;

	/// Establish connectivity to a vertex that follows the last vertex.
	/// Don't call this for loops.
	void SetNextVertex(Vec2 nextVertex) noexcept;

	/// Get a child edge.
	EdgeShape GetChildEdge(child_count_t index) const;

	/// Get the vertex count.
	child_count_t GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index.
	Vec2 GetVertex(child_count_t index) const;

	bool HasPrevVertex() const noexcept { return IsValid(m_prevVertex); }
	bool HasNextVertex() const noexcept { return IsValid(m_nextVertex); }

	Vec2 GetPrevVertex() const noexcept { return m_prevVertex; }
	Vec2 GetNextVertex() const noexcept { return m_nextVertex; }
	
private:
	/// The vertices. Owned by this class.
	Vec2* m_vertices = nullptr;

	/// The vertex count.
	child_count_t m_count = 0;

	Vec2 m_prevVertex = GetInvalid<Vec2>();
	Vec2 m_nextVertex = GetInvalid<Vec2>();
};

inline Vec2 ChainShape::GetVertex(child_count_t index) const
{
	assert((0 <= index) && (index < m_count));
	return m_vertices[index];
}

inline child_count_t GetNextIndex(const ChainShape& shape, child_count_t index) noexcept
{
	return GetModuloNext(index, shape.GetVertexCount());
}

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const ChainShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const ChainShape& shape, const Transformation& xf, const Vec2& p);

} // namespace box2d

#endif
