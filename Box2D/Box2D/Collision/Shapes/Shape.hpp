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

#ifndef B2_SHAPE_H
#define B2_SHAPE_H

#include <Box2D/Common/Math.hpp>

namespace box2d {

class Fixture;

/// Base class for shapes.
/// @detail A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in World are created automatically when a Fixture
/// is created. Shapes may encapsulate one or more child shapes.
/// @note This data structure is 8-bytes large (on at least one 64-bit platform).
class Shape
{
public:
	struct Conf
	{
		constexpr Conf& UseVertexRadius(Length value) noexcept;
		constexpr Conf& UseFriction(RealNum value) noexcept;
		constexpr Conf& UseRestitution(RealNum value) noexcept;
		constexpr Conf& UseDensity(Density value) noexcept;

		Length vertexRadius = DefaultLinearSlop;

		/// Friction coefficient.
		///
		/// @note This must be a value between 0 and +infinity.
		/// @note This is usually in the range [0,1].
		/// @note The square-root of the product of this value multiplied by a touching fixture's
		/// friction becomes the friction coefficient for the contact.
		///
		RealNum friction = RealNum{2} / RealNum{10};
		
		/// Restitution (elasticity) of the associated shape.
		///
		/// @note This should be a valid finite value.
		/// @note This is usually in the range [0,1].
		///
		RealNum restitution = RealNum{0};
		
		/// Density of the associated shape.
		///
		/// @note This is usually in kg/m^2.
		/// @note This must be a non-negative value.
		/// @note Use 0 to indicate that the shape's associated mass should be 0.
		///
		Density density = Density{0};
	};

	enum Type
	{
		e_circle = 0,
		e_edge = 1,
		e_polygon = 2,
		e_chain = 3,
		e_typeCount = 4
	};

	Shape() = delete;

	/// Initializing constructor.
	/// @param type Type of this shape object.
	constexpr Shape(Type type, const Conf& conf) noexcept:
		m_type{type},
		m_vertexRadius{conf.vertexRadius},
		m_density{Max(conf.density, Density{0})},
		m_friction{conf.friction},
		m_restitution{conf.restitution}
	{
		assert(type < e_typeCount);
		assert(conf.vertexRadius >= Length{0});
		assert(conf.density >= Density{0});
		assert(conf.friction >= 0);
		assert(conf.restitution < std::numeric_limits<decltype(conf.restitution)>::infinity());
		assert(conf.restitution > -std::numeric_limits<decltype(conf.restitution)>::infinity());
	}

	Shape(const Shape&) = default;

	~Shape() = default;

	/// Gets the type of this shape.
	/// @note You can use this to down cast to the concrete shape.
	/// @return the shape type.
	Type GetType() const noexcept { return m_type; }
	
	Length GetVertexRadius() const noexcept { return m_vertexRadius; }

	void SetVertexRadius(Length vertexRadius)
	{
		assert(vertexRadius >= Length{0});
		m_vertexRadius = vertexRadius;
	}

	/// Gets the density of this fixture.
	/// @return Non-negative density in kg/m^2.
	Density GetDensity() const noexcept;

	/// Sets the density of this fixture.
	/// @note This will _not_ automatically adjust the mass of the body.
	///   You must call Body::ResetMassData to update the body's mass.
	/// @warning Behavior is undefined if given a negative value.
	/// @param density Non-negative density in kg/m^2.
	void SetDensity(Density density) noexcept;
	
	/// Gets the coefficient of friction.
	RealNum GetFriction() const noexcept;
	
	/// Sets the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	void SetFriction(RealNum friction) noexcept;
	
	/// Gets the coefficient of restitution.
	RealNum GetRestitution() const noexcept;
	
	/// Sets the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	void SetRestitution(RealNum restitution) noexcept;

private:
	Type m_type;
	Length m_vertexRadius;
	Density m_density = KilogramPerSquareMeter * RealNum{0}; ///< Density in kg/m^2. 4-bytes.
	RealNum m_friction = RealNum{2} / RealNum{10}; ///< Friction as a coefficient. 4-bytes.
	RealNum m_restitution = 0; ///< Restitution as a coefficient. 4-bytes.
};

constexpr inline Shape::Conf& Shape::Conf::UseVertexRadius(Length value) noexcept
{
	vertexRadius = value;
	return *this;
}

constexpr inline Shape::Conf& Shape::Conf::UseFriction(RealNum value) noexcept
{
	friction = value;
	return *this;
}

constexpr inline Shape::Conf& Shape::Conf::UseRestitution(RealNum value) noexcept
{
	restitution = value;
	return *this;
}

constexpr inline Shape::Conf& Shape::Conf::UseDensity(Density value) noexcept
{
	density = value;
	return *this;
}

inline Density Shape::GetDensity() const noexcept
{
	return m_density;
}

inline RealNum Shape::GetFriction() const noexcept
{
	return m_friction;
}

inline RealNum Shape::GetRestitution() const noexcept
{
	return m_restitution;
}

inline void Shape::SetDensity(Density density) noexcept
{
	m_density = density;
}


inline void Shape::SetFriction(RealNum friction) noexcept
{
	m_friction = friction;
}

inline void Shape::SetRestitution(RealNum restitution) noexcept
{
	m_restitution = restitution;
}

// Free functions...

/// Gets the vertex radius of the given shape (in meters).
/// @detail Gets the radius (in meters) of every vertex of this shape.
/// This is used for collision handling.
/// @note This value should never be less than zero.
inline Length GetVertexRadius(const Shape& shape) noexcept
{
	return shape.GetVertexRadius();
}

/// Gets the number of child primitives of the shape.
/// @return Positive non-zero count.
child_count_t GetChildCount(const Shape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const Shape& shape, const Transformation& xf, const Length2D p);

/// Determine if two generic shapes overlap.
bool TestOverlap(const Shape& shapeA, child_count_t indexA, const Transformation& xfA,
				 const Shape& shapeB, child_count_t indexB, const Transformation& xfB);

Shape::Type GetType(const Fixture& fixture) noexcept;
	
} // namespace box2d

#endif
