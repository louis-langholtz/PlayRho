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

#include <Box2D/Collision/MassData.hpp>
#include <Box2D/Collision/Shapes/Shape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Dynamics/Fixture.h>

using namespace box2d;

MassData box2d::ComputeMass(const EdgeShape& shape, float_t density)
{
	BOX2D_NOT_USED(density);
	
	return MassData{float_t{0}, (shape.GetVertex1() + shape.GetVertex2()) / float_t(2), float_t{0}};
}

MassData box2d::ComputeMass(const PolygonShape& shape, float_t density)
{
	assert(density >= 0);
	
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.
	
	const auto count = shape.GetVertexCount();
	assert(count >= 3);
	
	auto center = Vec2_zero;
	auto area = float_t{0};
	auto I = float_t{0};
	
	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	// This code puts the reference point inside the polygon.
	const auto s = Average(shape.GetVertices());
	
	constexpr auto k_inv3 = float_t{1} / float_t{3};
	
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// Triangle vertices.
		const auto e1 = shape.GetVertex(i) - s;
		const auto e2 = shape.GetVertex((i + 1) % count) - s;
		
		const auto D = Cross(e1, e2);
		
		const auto triangleArea = D / 2;
		area += triangleArea;
		
		// Area weighted centroid
		center += triangleArea * k_inv3 * (e1 + e2);
		
		const auto intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
		const auto inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
		
		I += (D * k_inv3 / 4) * (intx2 + inty2);
	}
	
	// Total mass
	const auto mass = density * area;
	
	// Center of mass
	assert((area > 0) && !almost_zero(area));
	center *= float_t{1} / area;
	const auto massDataCenter = center + s;
	
	// Inertia tensor relative to the local origin (point s).
	// Shift to center of mass then to original body origin.
	const auto massDataI = (density * I) + (mass * (GetLengthSquared(massDataCenter) - GetLengthSquared(center)));
	
	return MassData{mass, massDataCenter, massDataI};
}

MassData box2d::ComputeMass(const CircleShape& shape, float_t density)
{
	assert(density >= 0);
	const auto mass = density * Pi * Square(shape.GetRadius());
	const auto I = mass * ((Square(shape.GetRadius()) / float_t{2}) + GetLengthSquared(shape.GetLocation()));
	return MassData{mass, shape.GetLocation(), I};
}

MassData box2d::ComputeMass(const ChainShape& shape, float_t density)
{
	BOX2D_NOT_USED(density);
	
	return MassData{float_t{0}, Vec2_zero, float_t{0}};
}

MassData box2d::ComputeMass(const Shape& shape, float_t density)
{
	assert(shape.GetType() < Shape::e_typeCount);
	switch (shape.GetType())
	{
		case Shape::e_edge: return ComputeMass(static_cast<const EdgeShape&>(shape), density);
		case Shape::e_chain: return ComputeMass(static_cast<const ChainShape&>(shape), density);
		case Shape::e_circle: return ComputeMass(static_cast<const CircleShape&>(shape), density);
		case Shape::e_polygon: return ComputeMass(static_cast<const PolygonShape&>(shape), density);
		case Shape::e_typeCount: return MassData{0, GetInvalid<Vec2>(), 0};
	}
}

MassData box2d::ComputeMassData(const Fixture& f)
{
	return ComputeMass(*f.GetShape(), f.GetDensity());
}
