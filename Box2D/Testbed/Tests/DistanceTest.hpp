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

#ifndef DISTANCE_TEST_H
#define DISTANCE_TEST_H

#include <sstream>

namespace box2d {

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		{
			m_transformA = Transform_identity;
			m_transformA.p = Vec2(-10.0f, 20.2f); // Vec2(0.0f, -0.2f);
			m_polygonA.SetAsBox(8.0f, 6.0f);
		}

		{
			m_positionB = m_transformA.p + Vec2(19.017401f, 0.13678508f);
			m_angleB = 0_deg; // -0.0109265_rad;
			m_transformB = Transformation{m_positionB, UnitVec2{m_angleB}};
			m_polygonB.SetAsBox(7.2f, 0.8f);
		}
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void ShowManifold(Drawer& drawer, const Manifold& manifold, const char* name)
	{
		std::ostringstream strbuf;
		const auto count = manifold.GetPointCount();
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			strbuf << ", ";
			strbuf << "mp={";
			const auto p = manifold.GetPoint(i);
			strbuf << "lp={" << p.localPoint.x << "," << p.localPoint.y << "}";
			strbuf << ", ";
			strbuf << "cf=" << p.contactFeature;
			strbuf << "}";
		}
		drawer.DrawString(5, m_textLine, "%s %s: lp={%g,%g}, ln={%g,%g}, #=%d%s",
						  GetName(manifold.GetType()),
						  name,
						  GetX(manifold.GetLocalPoint()),
						  GetY(manifold.GetLocalPoint()),
						  GetX(manifold.GetLocalNormal()),
						  GetY(manifold.GetLocalNormal()),
						  count, strbuf.str().c_str());
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	void Draw(Drawer& drawer, const PolygonShape& shape, const Transformation& xfm)
	{
		const auto color = Color(0.9f, 0.9f, 0.9f);
		const auto skinColor = Color(color, 0.1f);
		const auto r = shape.GetVertexRadius();
		Vec2 v[MaxPolygonVertices];
		const auto vertexCount = shape.GetVertexCount();
		for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
		{
			v[i] = Transform(shape.GetVertex(i), xfm);
		}
		drawer.DrawPolygon(v, vertexCount, color);
		for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
		{
			if (i > 0)
			{
				const auto normal0 = shape.GetNormal(i - 1);
				const auto worldNormal0 = Rotate(normal0, xfm.q);
				const auto p0 = v[i-1] + worldNormal0 * r;
				const auto p1 = v[i] + worldNormal0 * r;
				drawer.DrawSegment(p0, p1, skinColor);
				const auto normal1 = shape.GetNormal(i);
				const auto worldNormal1 = Rotate(normal1, xfm.q);
				const auto angle0 = GetAngle(worldNormal0);
				const auto angle1 = GetAngle(worldNormal1);
				const auto angleDiff = GetRevRotationalAngle(angle0, angle1);
				auto lastAngle = 0_deg;
				for (auto angle = 5_deg; angle < angleDiff; angle += 5_deg)
				{
					const auto c0 = v[i] + r * UnitVec2(angle0 + lastAngle);
					const auto c1 = v[i] + r * UnitVec2(angle0 + angle);
					drawer.DrawSegment(c0, c1, skinColor);
					lastAngle = angle;
				}
				{
					const auto c0 = v[i] + r * UnitVec2(angle0 + lastAngle);
					const auto c1 = v[i] + r * UnitVec2(angle1);
					drawer.DrawSegment(c0, c1, skinColor);
				}
			}
		}
		if (vertexCount > 0)
		{
			const auto worldNormal0 = Rotate(shape.GetNormal(vertexCount - 1), xfm.q);
			drawer.DrawSegment(v[vertexCount - 1] + worldNormal0 * r, v[0] + worldNormal0 * r, skinColor);
			const auto worldNormal1 = Rotate(shape.GetNormal(0), xfm.q);
			const auto angle0 = GetAngle(worldNormal0);
			const auto angle1 = GetAngle(worldNormal1);
			const auto angleDiff = GetRevRotationalAngle(angle0, angle1);
			auto lastAngle = 0_deg;
			for (auto angle = 5_deg; angle < angleDiff; angle += 5_deg)
			{
				const auto c0 = v[0] + r * UnitVec2(angle0 + lastAngle);
				const auto c1 = v[0] + r * UnitVec2(angle0 + angle);
				drawer.DrawSegment(c0, c1, skinColor);
				lastAngle = angle;
			}
			{
				const auto c0 = v[0] + r * UnitVec2(angle0 + lastAngle);
				const auto c1 = v[0] + r * UnitVec2(angle1);
				drawer.DrawSegment(c0, c1, skinColor);
			}
		}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		const auto proxyA = GetDistanceProxy(m_polygonA, 0);
		const auto proxyB = GetDistanceProxy(m_polygonB, 0);
		const auto transformA = m_transformA;
		const auto transformB = m_transformB;

		const auto manifold = CollideShapes(m_polygonA, m_transformA, m_polygonB, m_transformB);
		const auto panifold = GetManifold(proxyA, transformA, proxyB, transformB);

		Simplex::Cache cache;
		const auto output = Distance(proxyA, transformA, proxyB, transformB, cache);
		cache = Simplex::GetCache(output.simplex.GetEdges());
		const auto witnessPoints = GetWitnessPoints(output.simplex);
		const auto outputDistance = Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b));
		
		const auto rA = proxyA.GetRadius();
		const auto rB = proxyB.GetRadius();
		const auto totalRadius = rA + rB;
		
		auto adjustedWitnessPoints = witnessPoints;
		auto adjustedDistance = outputDistance;
		if ((outputDistance > totalRadius) && !almost_zero(outputDistance))
		{
			// Shapes are still not overlapped.
			// Move the witness points to the outer surface.
			adjustedDistance -= totalRadius;
			const auto normal = GetUnitVector(witnessPoints.b - witnessPoints.a);
			adjustedWitnessPoints.a += rA * normal;
			adjustedWitnessPoints.b -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			const auto p = (witnessPoints.a + witnessPoints.b) / float_t{2};
			adjustedWitnessPoints.a = p;
			adjustedWitnessPoints.b = p;
			adjustedDistance = 0;
		}
		
		drawer.DrawString(transformA.p, "Shape A");
		drawer.DrawString(transformB.p, "Shape B");

		drawer.DrawString(5, m_textLine, "Press 'A', 'D', 'W', or 'S' to move left, right, up, or down respectively.");
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "Press 'Q', or 'E' to rotate counter-clockwise or clockwise respectively.");
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "Press num-pad '+'/'-' to increase/decrease vertex radiuses (%g & %g).",
						  rA, rB);
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "distance = %g (from %g), iterations = %d",
						  adjustedDistance, outputDistance, output.iterations);
		m_textLine += DRAW_STRING_NEW_LINE;
		
		Draw(drawer, m_polygonA, m_transformA);
		Draw(drawer, m_polygonB, m_transformB);

		{
			const auto size = output.simplex.GetSize();
			drawer.DrawString(5, m_textLine, "Simplex: size=%d, wpt-a={%g,%g}, wpt-b={%g,%g})",
							  size,
							  witnessPoints.a.x, witnessPoints.a.y,
							  witnessPoints.b.x, witnessPoints.b.y);
			m_textLine += DRAW_STRING_NEW_LINE;
			for (auto i = decltype(size){0}; i < size; ++i)
			{
				const auto& edge = output.simplex.GetSimplexEdge(i);
				const auto coef = output.simplex.GetCoefficient(i);
				
				drawer.DrawString(5, m_textLine, "a[%d]={%g,%g} b[%d]={%g,%g} coef=%g",
								  edge.GetIndexA(),
								  edge.GetPointA().x,
								  edge.GetPointA().y,
								  edge.GetIndexB(),
								  edge.GetPointB().x,
								  edge.GetPointB().y,
								  coef);
				m_textLine += DRAW_STRING_NEW_LINE;
				
				drawer.DrawSegment(edge.GetPointA(), edge.GetPointB(), Color(0.0f, 1.0f, 1.0f, 0.1f));
			}
		}

		ShowManifold(drawer, manifold, "manifold");
		ShowManifold(drawer, panifold, "wanifold");

		drawer.DrawPoint(adjustedWitnessPoints.a, 4.0f, Color(1.0f, 0.0f, 0.0f));
		drawer.DrawPoint(adjustedWitnessPoints.b, 4.0f, Color(1.0f, 1.0f, 0.0f));

		drawer.DrawPoint(witnessPoints.a, 4.0f, Color(1.0f, 0.0f, 0.0f, 0.5));
		drawer.DrawPoint(witnessPoints.b, 4.0f, Color(1.0f, 1.0f, 0.0f, 0.5));
		
		for (auto&& edge: output.simplex.GetEdges())
		{
			drawer.DrawPoint(edge.GetPointA(), 6.0f, Color(0.0f, 1.0f, 1.0f, 0.6f));
			drawer.DrawPoint(edge.GetPointB(), 6.0f, Color(0.0f, 1.0f, 1.0f, 0.6f));
			drawer.DrawString(edge.GetPointA(), "%d", edge.GetIndexA());
			drawer.DrawString(edge.GetPointB(), "%d", edge.GetIndexB());
		}
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_positionB.x -= 0.1f;
			break;

		case Key_D:
			m_positionB.x += 0.1f;
			break;

		case Key_S:
			m_positionB.y -= 0.1f;
			break;

		case Key_W:
			m_positionB.y += 0.1f;
			break;

		case Key_Q:
			m_angleB += 5_deg;
			break;

		case Key_E:
			m_angleB -= 5_deg;
			break;

		case Key_Add:
			m_polygonA.SetVertexRadius(m_polygonA.GetVertexRadius() + RadiusIncrement);
			m_polygonB.SetVertexRadius(m_polygonB.GetVertexRadius() + RadiusIncrement);
			break;

		case Key_Subtract:
			m_polygonA.SetVertexRadius(m_polygonA.GetVertexRadius() - RadiusIncrement);
			m_polygonB.SetVertexRadius(m_polygonB.GetVertexRadius() - RadiusIncrement);
			break;

		default:
			break;
		}

		m_transformB = Transformation{m_positionB, UnitVec2{m_angleB}};
	}

private:
	static constexpr auto RadiusIncrement = LinearSlop * 100;

	Vec2 m_positionB;
	Angle m_angleB;

	Transformation m_transformA;
	Transformation m_transformB;
	PolygonShape m_polygonA{RadiusIncrement * 40};
	PolygonShape m_polygonB{RadiusIncrement * 40};
};
	
} // namespace box2d

#endif
