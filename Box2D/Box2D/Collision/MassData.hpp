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

#ifndef B2_MASS_DATA_HPP
#define B2_MASS_DATA_HPP

#include <Box2D/Common/Math.hpp>

namespace box2d {
	
	class Fixture;
	class Shape;
	class PolygonShape;
	class EdgeShape;
	class CircleShape;
	class ChainShape;

	Area GetAreaOfCircle(Length radius);
	Area GetAreaOfPolygon(Span<const Length2D> vertices);

	/// Gets the polar moment of the area enclosed by the given vertices.
	/// @warning Behavior is undefined if given collection has less than 3 vertices.
	/// @param vertices Collection of three or more vertices.
	SecondMomentOfArea GetPolarMoment(Span<const Length2D> vertices);

	/// Mass data.
	/// @detail This holds the mass data computed for a shape.
	/// @note This data structure is 16-bytes large (on at least one 64-bit platform).
	struct MassData
	{
		MassData() = default;
		
		/// Initializing constructor.
		/// @param m Non-negative mass in kg.
		/// @param c Position of the shape's centroid relative to the shape's origin.
		/// @param i Non-negative rotational inertia of the shape about the local origin.
		constexpr MassData(Mass m, Length2D c, RotInertia i) noexcept: mass{m}, center{c}, I{i}
		{
			assert(m >= Mass{0});
			assert(i >= RotInertia{0});
		}
				
		/// The position of the shape's centroid relative to the shape's origin.
		Length2D center;
		
		/// Mass of the shape in kilograms.
		/// This should NEVER be a negative value.
		/// @note Behavior is undefined if this value is negative.
		Mass mass;

		/// Rotational inertia, a.k.a. moment of inertia.
		/// @detail
		/// This is the rotational inertia of the shape about the local origin.
		/// This should NEVER be a negative value.
		/// @note Behavior is undefined if this value is negative.
		/// @sa https://en.wikipedia.org/wiki/Moment_of_inertia
		RotInertia I;
	};
	
	/// Computes the mass data for the given fixture.
	/// @detail
	/// The mass data is based on the density and
	/// the shape of the fixture. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	MassData GetMassData(const Fixture& f);

	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the given density is negative.
	/// @param density Density in kilograms per meter squared (must be non-negative).
	/// @return Mass data for this shape.
	MassData GetMassData(const Shape& shape, Density density);

	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the given density is negative.
	/// @param density Density in kilograms per meter squared (must be non-negative).
	/// @return Mass data for this shape.
	MassData GetMassData(const PolygonShape& shape, Density density);
	
	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the given density is negative.
	/// @param density Density in kilograms per meter squared (must be non-negative).
	/// @return Mass data for this shape.
	MassData GetMassData(const EdgeShape& shape, Density density);
	
	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the given density is negative.
	/// @param density Density in kilograms per meter squared (must be non-negative).
	/// @return Mass data for this shape.
	MassData GetMassData(const ChainShape& shape, Density density);
	
	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the given density is negative.
	/// @param density Density in kilograms per meter squared (must be non-negative).
	/// @return Mass data for this shape.
	MassData GetMassData(const CircleShape& shape, Density density);
	
}

#endif /* B2_MASS_DATA_HPP */
