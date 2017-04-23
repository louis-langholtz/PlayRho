/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.hpp>

namespace box2d {

/// Edge shape.
/// @detail
/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
/// @note This data structure is 32-bytes.
class EdgeShape : public Shape
{
public:
	static constexpr Length GetDefaultVertexRadius() noexcept
	{
		return DefaultLinearSlop * RealNum{2};
	}

	struct Conf: public Shape::Conf
	{
		constexpr Conf(): Shape::Conf{Shape::Conf{}.UseVertexRadius(GetDefaultVertexRadius())}
		{
		}
	};
	
	static Conf GetDefaultConf() noexcept
	{
		return Conf{};
	}

	EdgeShape(const Conf& conf = GetDefaultConf()) noexcept:
		Shape{conf}
	{
		// Intentionally empty.
	}

	EdgeShape(Length2D v1, Length2D v2, const Conf& conf = GetDefaultConf()) noexcept:
		Shape{conf},
		m_vertex1{v1},
		m_vertex2{v2},
		m_normal1{GetUnitVector(GetFwdPerpendicular(v2 - v1))},
		m_normal2{-m_normal1}
	{
		assert(IsValid(m_normal1));
		assert(IsValid(m_normal2));
	}

	EdgeShape(const EdgeShape&) = default;

	/// Gets the number of child primitives.
	/// @return Positive non-zero count.
	child_count_t GetChildCount() const noexcept override;

	DistanceProxy GetChild(child_count_t index) const noexcept override;
	
	/// Tests a point for containment in this shape.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
	bool TestPoint(const Transformation& xf, const Length2D p) const noexcept override;

	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the density is negative.
	/// @return Mass data for this shape.
	MassData GetMassData() const noexcept override;

	/// Cast a ray against a child shape.
	/// @param input the ray-cast input parameters.
	/// @param xf the transform to be applied to the shape.
	/// @param childIndex the child shape index
	RayCastOutput RayCast(const RayCastInput& input, const Transformation& xf,
						  child_count_t childIndex) const noexcept override;

	void Accept(Visitor& visitor) const override;

	/// Set this as an isolated edge.
	void Set(const Length2D v1, const Length2D v2);

	Length2D GetVertex1() const noexcept { return m_vertex1; }
	Length2D GetVertex2() const noexcept { return m_vertex2; }

	UnitVec2 GetNormal1() const noexcept { return m_normal1; }
	UnitVec2 GetNormal2() const noexcept { return m_normal2; }

private:
	/// These are the edge vertices
	Length2D m_vertex1;
	Length2D m_vertex2;
	
	UnitVec2 m_normal1;
	UnitVec2 m_normal2;
};

inline child_count_t EdgeShape::GetChildCount() const noexcept
{
	return 1;
}

inline DistanceProxy EdgeShape::GetChild(child_count_t index) const noexcept
{
	assert(index == 0);
	return (index == 0)?
		DistanceProxy{GetVertexRadius(), GetVertex1(), GetVertex2(), GetNormal1(), GetNormal2()}:
		DistanceProxy{};
}

inline void EdgeShape::Accept(box2d::Shape::Visitor &visitor) const
{
	visitor.Visit(*this);
}

} // namespace box2d

#endif
