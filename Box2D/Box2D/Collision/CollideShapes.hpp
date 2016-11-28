/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef CollideShapes_hpp
#define CollideShapes_hpp

#include <Box2D/Collision/Manifold.hpp>

namespace box2d
{
	class CircleShape;
	class PolygonShape;
	class EdgeShape;

	/// Computes the collision manifold between two circles.
	/// @param shapeA Shape A.
	/// @param xfA Transformation for shape A.
	/// @param shapeB Shape B.
	/// @param xfB Transformation for shape B.
	/// @return An unset-type manifold if the shapes aren't touching,
	///   a circles-type manifold with one point otherwise.
	Manifold CollideShapes(const CircleShape& shapeA, const Transformation& xfA,
						   const CircleShape& shapeB, const Transformation& xfB);
	
	/// Computes the collision manifold between a polygon and a circle.
	/// @param shapeA Shape A.
	/// @param xfA Transformation for shape A.
	/// @param shapeB Shape B.
	/// @param xfB Transformation for shape B.
	/// @return An unset-type manifold if the shapes aren't touching,
	///   a face-A-type manifold with one point otherwise.
	Manifold CollideShapes(const PolygonShape& shapeA, const Transformation& xfA,
						   const CircleShape& shapeB, const Transformation& xfB);
	
	/// Computes the collision manifold between two polygons.
	/// @param shapeA Shape A.
	/// @param xfA Transformation for shape A.
	/// @param shapeB Shape B.
	/// @param xfB Transformation for shape B.
	/// @return An unset-type manifold if the shapes aren't touching,
	///   a face-A or face-B type manifold with one or two points otherwise.
	Manifold CollideShapes(const PolygonShape& shapeA, const Transformation& xfA,
						   const PolygonShape& shapeB, const Transformation& xfB);
	
	/// Computes the collision manifold between an edge and a circle.
	/// @param shapeA Shape A.
	/// @param xfA Transformation for shape A.
	/// @param shapeB Shape B.
	/// @param xfB Transformation for shape B.
	/// @return A one-point circle-type manifold if the circle shape is by either end of the edge,
	///   a one-point face-A-type manifold if the circle shape is between the edge's ends, or an
	///   unset-type manifold if the shapes aren't touching.
	Manifold CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
						   const CircleShape& shapeB, const Transformation& xfB);
	
	Manifold CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
						   const EdgeShape& shapeB, const Transformation& xfB);

	/// Computes the collision manifold between an edge and a circle.
	/// @param shapeA Shape A.
	/// @param xfA Transformation for shape A.
	/// @param shapeB Shape B.
	/// @param xfB Transformation for shape B.
	/// @return An unset-type manifold if the shapes aren't touching,
	///   a face-A or face-B type manifold with 0 to 2 points otherwise.
	Manifold CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
						   const PolygonShape& shapeB, const Transformation& xfB);

} // namespace box2d

#endif /* CollideShapes_hpp */
