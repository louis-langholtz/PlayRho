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
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>

using namespace box2d;

float_t box2d::GetAreaOfCircle(float_t radius)
{
	return Pi * Square(radius);
}

float_t box2d::GetAreaOfPolygon(Span<const Vec2> vertices)
{
	// Uses the "Shoelace formula".
	// See: https://en.wikipedia.org/wiki/Shoelace_formula
	auto sum = float_t(0);
	const auto count = vertices.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto last_v = vertices[(i - 1) % count];
		const auto this_v = vertices[i];
		const auto next_v = vertices[(i + 1) % count];
		sum += this_v.x * (next_v.y - last_v.y);
	}
	return sum / 2;
}

float_t box2d::GetPolarMomentOfPolygon(Span<const Vec2> vertices)
{
	// Use formulas Ix and Iy for second moment of area of any simple polygon and apply
	// the perpendicular axis theorem on these to get the desired answer.
	//
	// See:
	// https://en.wikipedia.org/wiki/Second_moment_of_area#Any_polygon
	// https://en.wikipedia.org/wiki/Second_moment_of_area#Perpendicular_axis_theorem
	auto sum_x = float_t(0);
	auto sum_y = float_t(0);
	const auto count = vertices.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto this_v = vertices[i];
		const auto next_v = vertices[(i + 1) % count];
		const auto fact_b = this_v.x * next_v.y - next_v.x * this_v.y;
		sum_x += [&]() {
			const auto fact_a = Square(this_v.y) + this_v.y * next_v.y + Square(next_v.y);
			return fact_a * fact_b;
		}();
		sum_y += [&]() {
			const auto fact_a = Square(this_v.x) + this_v.x * next_v.x + Square(next_v.x);
			return fact_a * fact_b;
		}();
	}
	return (sum_x + sum_y) / 12;
}

MassData box2d::GetMassData(const PolygonShape& shape, float_t density)
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

MassData box2d::GetMassData(const CircleShape& shape, float_t density)
{
	// Uses parallel axis theorem, perpendicular axis theorem, and the second moment of area.
	// See: https://en.wikipedia.org/wiki/Second_moment_of_area
	//
	// Ixp = Ix + A * dx^2
	// Iyp = Iy + A * dy^2
	// Iz = Ixp + Iyp = Ix + A * dx^2 + Iy + A * dy^2
	// Ix = Pi * r^4 / 4
	// Iy = Pi * r^4 / 4
	// Iz = (Pi * r^4 / 4) + (Pi * r^4 / 4) + (A * dx^2) + (A * dy^2)
	//    = (Pi * r^4 / 2) + (A * (dx^2 + dy^2))
	// A = Pi * r^2
	// Iz = (Pi * r^4 / 2) + (2 * (Pi * r^2) * (dx^2 + dy^2))
	// Iz = Pi * r^2 * ((r^2 / 2) + (dx^2 + dy^2))
	assert(density >= 0);
	const auto r = shape.GetRadius();
	const auto r_squared = Square(r);
	const auto area = Pi * r_squared;
	const auto mass = density * area;
	const auto Iz = area * ((r_squared / 2) + GetLengthSquared(shape.GetLocation()));
	return MassData{mass, shape.GetLocation(), Iz * density};
}

MassData box2d::GetMassData(const EdgeShape& shape, float_t density)
{
	assert(density >= 0);
	assert(!shape.HasVertex0());
	assert(!shape.HasVertex3());

	const auto r = shape.GetVertexRadius();
	const auto r_squared = Square(r);
	const auto circle_area = Pi * r_squared;
	const auto circle_mass = density * circle_area;
	const auto d = shape.GetVertex2() - shape.GetVertex1();
	const auto offset = GetRevPerpendicular(GetUnitVector(d, UnitVec2::GetZero())) * r;
	const auto b = GetLength(d);
	const auto h = r * 2;
	const auto rect_mass = density * b * h;
	const auto totalMass = circle_mass + rect_mass;

	/// Use the fixture's areal mass density times the shape's second moment of area to derive I.
	/// @sa https://en.wikipedia.org/wiki/Second_moment_of_area
	const auto I0 = (circle_area / 2) * ((r_squared / 2) + GetLengthSquared(shape.GetVertex1()));
	const auto I1 = (circle_area / 2) * ((r_squared / 2) + GetLengthSquared(shape.GetVertex2()));

	const auto vertices = Span<const Vec2>{
		Vec2{shape.GetVertex1() + offset},
		Vec2{shape.GetVertex1() - offset},
		Vec2{shape.GetVertex2() - offset},
		Vec2{shape.GetVertex2() + offset}
	};
	const auto I_z = GetPolarMomentOfPolygon(vertices);
	
	return MassData{
		totalMass,
		(shape.GetVertex1() + shape.GetVertex2()) / float_t(2),
		(I0 + I1 + I_z) * density
	};
}

MassData box2d::GetMassData(const ChainShape& shape, float_t density)
{
	BOX2D_NOT_USED(density);
	
	return MassData{float_t{0}, Vec2_zero, float_t{0}};
}

MassData box2d::GetMassData(const Shape& shape, float_t density)
{
	assert(shape.GetType() < Shape::e_typeCount);
	switch (shape.GetType())
	{
		case Shape::e_edge: return GetMassData(static_cast<const EdgeShape&>(shape), density);
		case Shape::e_chain: return GetMassData(static_cast<const ChainShape&>(shape), density);
		case Shape::e_circle: return GetMassData(static_cast<const CircleShape&>(shape), density);
		case Shape::e_polygon: return GetMassData(static_cast<const PolygonShape&>(shape), density);
		case Shape::e_typeCount: return MassData{0, GetInvalid<Vec2>(), 0};
	}
}

MassData box2d::GetMassData(const Fixture& f)
{
	return GetMassData(*f.GetShape(), f.GetDensity());
}
