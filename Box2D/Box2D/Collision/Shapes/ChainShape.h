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

#include <Box2D/Collision/Shapes/Shape.h>

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
	ChainShape(): Shape{e_chain, PolygonRadius} {}

	ChainShape(const ChainShape& other);

	/// The destructor frees the vertices using free.
	~ChainShape();

	ChainShape& operator=(const ChainShape& other);

	/// Clear all data.
	void Clear();

	/// Create a loop. This automatically adjusts connectivity.
	/// @note Behavior is undefined if vertices is null or if count of vertices is less than 3.
	/// @param vertices Non-null array of vertices. These are copied.
	/// @param count Count of vertices. Must be 3 or more.
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

	/// Get a child edge.
	void GetChildEdge(EdgeShape* edge, child_count_t index) const;

	/// Get the vertex count.
	child_count_t GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index.
	const Vec2& GetVertex(child_count_t index) const;

	bool HasPrevVertex() const noexcept { return IsValid(m_prevVertex); }
	bool HasNextVertex() const noexcept { return IsValid(m_nextVertex); }

	Vec2 GetPrevVertex() const noexcept { return m_prevVertex; }
	Vec2 GetNextVertex() const noexcept { return m_nextVertex; }

	child_count_t GetNextIndex(child_count_t index) const noexcept
	{
		assert(index < m_count);
		return (index + 1) % m_count;
	}
	
private:
	/// The vertices. Owned by this class.
	Vec2* m_vertices = nullptr;

	/// The vertex count.
	child_count_t m_count = 0;

	Vec2 m_prevVertex = Vec2_invalid;
	Vec2 m_nextVertex = Vec2_invalid;
};

inline const Vec2& ChainShape::GetVertex(child_count_t index) const
{
	assert((0 <= index) && (index < m_count));
	return m_vertices[index];
}

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const ChainShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const ChainShape& shape, const Transformation& xf, const Vec2& p);

/// Cast a ray against a child shape.
/// @param input the ray-cast input parameters.
/// @param transform the transform to be applied to the shape.
/// @param childIndex the child shape index
RayCastOutput RayCast(const ChainShape& shape, const RayCastInput& input,
					  const Transformation& transform, child_count_t childIndex);

/// Given a transform, compute the associated axis aligned bounding box for a child shape.
/// @param xf the world transform of the shape.
/// @param childIndex the child shape
/// @return the axis aligned box.
AABB ComputeAABB(const ChainShape& shape, const Transformation& xf, child_count_t childIndex);

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @note Behavior is undefined if the given density is negative.
/// @param density Density in kilograms per meter squared (must be non-negative).
/// @return Mass data for this shape.
MassData ComputeMass(const ChainShape& shape, float_t density);

} // namespace box2d

#endif
