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

namespace
{
	MassData GetMassData(RealNum r, RealNum density, Vec2 location)
	{
		assert(density >= 0);

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
		const auto r_squared = Square(r);
		const auto area = r_squared * Pi;
		const auto mass = density * area;
		const auto Iz = area * ((r_squared / 2) + GetLengthSquared(location));
		return MassData{mass, location, Iz * density};
	}

	MassData GetMassData(RealNum r, RealNum density, Vec2 v0, Vec2 v1)
	{
		assert(density >= 0);

		const auto r_squared = Square(r);
		const auto circle_area = r_squared * Pi;
		const auto circle_mass = density * circle_area;
		const auto d = v1 - v0;
		const auto offset = GetRevPerpendicular(GetUnitVector(d, UnitVec2::GetZero())) * r;
		const auto b = GetLength(d);
		const auto h = r * 2;
		const auto rect_mass = density * b * h;
		const auto totalMass = circle_mass + rect_mass;
		
		/// Use the fixture's areal mass density times the shape's second moment of area to derive I.
		/// @sa https://en.wikipedia.org/wiki/Second_moment_of_area
		const auto I0 = (circle_area / 2) * ((r_squared / 2) + GetLengthSquared(v0));
		const auto I1 = (circle_area / 2) * ((r_squared / 2) + GetLengthSquared(v1));
		
		const auto vertices = Span<const Vec2>{
			Vec2{v0 + offset},
			Vec2{v0 - offset},
			Vec2{v1 - offset},
			Vec2{v1 + offset}
		};
		const auto I_z = GetPolarMoment(vertices);
		
		return MassData{totalMass, (v0 + v1) / 2, (I0 + I1 + I_z) * density};
	}

} // unnamed namespace

RealNum box2d::GetAreaOfCircle(RealNum radius)
{
	return Square(radius) * Pi;
}

RealNum box2d::GetAreaOfPolygon(Span<const Vec2> vertices)
{
	// Uses the "Shoelace formula".
	// See: https://en.wikipedia.org/wiki/Shoelace_formula
	auto sum = RealNum(0);
	const auto count = vertices.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto last_v = vertices[GetModuloPrev(i, count)];
		const auto this_v = vertices[i];
		const auto next_v = vertices[GetModuloNext(i, count)];
		sum += this_v.x * (next_v.y - last_v.y);
	}
	return sum / 2;
}

RealNum box2d::GetPolarMoment(Span<const Vec2> vertices)
{
	assert(vertices.size() > 2);

	// Use formulas Ix and Iy for second moment of area of any simple polygon and apply
	// the perpendicular axis theorem on these to get the desired answer.
	//
	// See:
	// https://en.wikipedia.org/wiki/Second_moment_of_area#Any_polygon
	// https://en.wikipedia.org/wiki/Second_moment_of_area#Perpendicular_axis_theorem
	auto sum_x = RealNum(0);
	auto sum_y = RealNum(0);
	const auto count = vertices.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto this_v = vertices[i];
		const auto next_v = vertices[GetModuloNext(i, count)];
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

MassData box2d::GetMassData(const PolygonShape& shape, RealNum density)
{
	// See: https://en.wikipedia.org/wiki/Centroid#Centroid_of_polygon

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
	switch (count)
	{
		case 0:
			return MassData{GetInvalid<RealNum>(), GetInvalid<Vec2>(), GetInvalid<RealNum>()};
		case 1:
			return ::GetMassData(shape.GetVertexRadius(), density, shape.GetVertex(0));
		case 2:
			return ::GetMassData(shape.GetVertexRadius(), density, shape.GetVertex(0), shape.GetVertex(1));;
		default:
			break;
	}
	
	auto center = Vec2_zero;
	auto area = RealNum{0};
	auto I = RealNum{0};
	
	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	// This code puts the reference point inside the polygon.
	const auto s = Average(shape.GetVertices());
	
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// Triangle vertices.
		const auto e1 = shape.GetVertex(i) - s;
		const auto e2 = shape.GetVertex(GetModuloNext(i, count)) - s;
		
		const auto D = Cross(e1, e2);
		
		const auto triangleArea = D / 2;
		area += triangleArea;
		
		// Area weighted centroid
		center += triangleArea * (e1 + e2) / 3;
		
		const auto intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
		const auto inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
		
		const auto triangleI = D * (intx2 + inty2) / (3 * 4);
		I += triangleI;
	}
	
	// Total mass
	const auto mass = density * area;
	
	// Center of mass
	assert((area > 0) && !almost_zero(area));
	center *= 1 / area;
	const auto massDataCenter = center + s;
	
	// Inertia tensor relative to the local origin (point s).
	// Shift to center of mass then to original body origin.
	const auto massCenterOffset = GetLengthSquared(massDataCenter);
	const auto centerOffset = GetLengthSquared(center);
	const auto intertialLever = massCenterOffset - centerOffset;
	const auto massDataI = (density * I) + (mass * intertialLever);
	
	return MassData{mass, massDataCenter, massDataI};
}

MassData box2d::GetMassData(const CircleShape& shape, RealNum density)
{
	return ::GetMassData(shape.GetVertexRadius(), density, shape.GetLocation());
}

MassData box2d::GetMassData(const EdgeShape& shape, RealNum density)
{
	assert(!shape.HasVertex0());
	assert(!shape.HasVertex3());
	return ::GetMassData(shape.GetVertexRadius(), density, shape.GetVertex1(), shape.GetVertex2());
}

MassData box2d::GetMassData(const ChainShape& shape, RealNum density)
{
	BOX2D_NOT_USED(density);
	
	return MassData{RealNum{0}, Vec2_zero, RealNum{0}};
}

MassData box2d::GetMassData(const Shape& shape, RealNum density)
{
	assert(shape.GetType() < Shape::e_typeCount);
	assert(density >= 0);
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
