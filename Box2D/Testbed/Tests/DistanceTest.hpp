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
#include <Box2D/Collision/ShapeSeparation.hpp>

namespace box2d {

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		m_world->SetGravity(Vec2{0, 0});

		const auto def = BodyDef{}.UseType(BodyType::Dynamic).UseLinearDamping(float_t(0.5)).UseAngularDamping(float_t(0.5));
		m_bodyA = m_world->CreateBody(def);
		m_bodyB = m_world->CreateBody(def);

		m_bodyA->SetTransform(Vec2(-10.0f, 20.2f), 0_deg);
		m_bodyB->SetTransform(m_bodyA->GetLocation() + Vec2(19.017401f, 0.13678508f), 0_deg);
		
		CreateFixtures();
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void CreateFixtures()
	{
		PolygonShape polygonA{m_radius};
		polygonA.SetAsBox(8.0f, 6.0f);
		m_fixtureA = m_bodyA->CreateFixture(FixtureDef{&polygonA, 1});
		
		PolygonShape polygonB{m_radius};
		polygonB.SetAsBox(7.2f, 0.8f);
		m_fixtureB = m_bodyB->CreateFixture(FixtureDef{&polygonB, 1});
	}

	void DestroyFixtures()
	{
		m_bodyA->DestroyFixture(m_fixtureA);
		m_bodyB->DestroyFixture(m_fixtureB);
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

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		const auto shapeA = static_cast<const PolygonShape*>(m_fixtureA->GetShape());
		const auto shapeB = static_cast<const PolygonShape*>(m_fixtureB->GetShape());

		const auto proxyA = GetDistanceProxy(*shapeA, 0);
		const auto proxyB = GetDistanceProxy(*shapeB, 0);
		const auto transformA = m_bodyA->GetTransformation();
		const auto transformB = m_bodyB->GetTransformation();

		const auto maxIndicesAB = GetMaxSeparation(shapeA->GetVertices(), shapeA->GetNormals(), transformA,
												   shapeB->GetVertices(), transformB);
		const auto maxIndicesBA = GetMaxSeparation(shapeB->GetVertices(), shapeB->GetNormals(), transformB,
												   shapeA->GetVertices(), transformA);

		const auto manifold = CollideShapes(*shapeA, transformA, *shapeB, transformB);
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

		drawer.DrawString(5, m_textLine, "Max distance %g for a[%i] b[%i]",
						  maxIndicesAB.separation, maxIndicesAB.index1, maxIndicesAB.index2);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Max distance %g for b[%i] a[%i]",
						  maxIndicesBA.separation, maxIndicesBA.index1, maxIndicesBA.index2);
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "distance = %g (from %g), iterations = %d",
						  adjustedDistance, outputDistance, output.iterations);
		m_textLine += DRAW_STRING_NEW_LINE;
		
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
				
				drawer.DrawSegment(edge.GetPointA(), edge.GetPointB(), Color{0.0f, 0.5f, 0.5f});
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
			m_bodyB->SetTransform(m_bodyB->GetLocation() - Vec2{float_t(0.1), 0}, m_bodyB->GetAngle());
			m_bodyB->SetAwake();
			break;

		case Key_D:
			m_bodyB->SetTransform(m_bodyB->GetLocation() + Vec2{float_t(0.1), 0}, m_bodyB->GetAngle());
			m_bodyB->SetAwake();
			break;

		case Key_S:
			m_bodyB->SetTransform(m_bodyB->GetLocation() - Vec2{0, float_t(0.1)}, m_bodyB->GetAngle());
			m_bodyB->SetAwake();
			break;

		case Key_W:
			m_bodyB->SetTransform(m_bodyB->GetLocation() + Vec2{0, float_t(0.1)}, m_bodyB->GetAngle());
			m_bodyB->SetAwake();
			break;

		case Key_Q:
			m_bodyB->SetTransform(m_bodyB->GetLocation(), m_bodyB->GetAngle() + 5_deg);
			m_bodyB->SetAwake();
			break;

		case Key_E:
			m_bodyB->SetTransform(m_bodyB->GetLocation(), m_bodyB->GetAngle() - 5_deg);
			m_bodyB->SetAwake();
			break;

		case Key_Add:
			DestroyFixtures();
			m_radius += RadiusIncrement;
			CreateFixtures();
			break;

		case Key_Subtract:
			DestroyFixtures();
			m_radius -= RadiusIncrement;
			CreateFixtures();
			break;

		default:
			break;
		}
	}

private:
	static constexpr auto RadiusIncrement = LinearSlop * 200;

	float_t m_radius = RadiusIncrement * 40;
	Body* m_bodyA;
	Body* m_bodyB;
	Fixture* m_fixtureA;
	Fixture* m_fixtureB;
};
	
} // namespace box2d

#endif
