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

#ifndef DISTANCE_TEST_H
#define DISTANCE_TEST_H

#include <sstream>
#include <Box2D/Collision/ShapeSeparation.hpp>
#include <Box2D/Dynamics/Contacts/PositionSolverManifold.hpp>

namespace box2d {

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		m_world->SetGravity(Vec2{0, 0} * MeterPerSquareSecond);

		const auto def = BodyDef{}.UseType(BodyType::Dynamic).UseLinearDamping(RealNum(0.5)).UseAngularDamping(RealNum(0.5));
		m_bodyA = m_world->CreateBody(def);
		m_bodyB = m_world->CreateBody(def);

		m_bodyA->SetTransform(Vec2(-10.0f, 20.2f), 0.0f * Degree);
		m_bodyB->SetTransform(m_bodyA->GetLocation() + Vec2(19.017401f, 0.13678508f), 0.0f * Degree);
		
		CreateFixtures();
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void CreateFixtures()
	{
		const auto radius = RadiusIncrement * 40;
		auto conf = PolygonShape::Conf{};
		conf.density = RealNum{1} * KilogramPerSquareMeter;

		conf.vertexRadius = radius;
		PolygonShape polygonA{conf};
		//polygonA.SetAsBox(8.0f, 6.0f);
		polygonA.Set(Span<const Vec2>{Vec2{-8, -6}, Vec2{8, -6}, Vec2{0, 6}});
		m_bodyA->CreateFixture(std::make_shared<PolygonShape>(polygonA));
		
		conf.vertexRadius = radius * 2;
		PolygonShape polygonB{conf};
		polygonB.SetAsBox(7.2f, 0.8f);
		//polygonB.Set(Span<const Vec2>{Vec2{float(-7.2), 0}, Vec2{float(7.2), 0}});
		m_bodyB->CreateFixture(std::make_shared<PolygonShape>(polygonB));
	}

	static Fixture* GetFixture(Body* body)
	{
		return (body->GetFixtures().begin() != body->GetFixtures().end())?
			body->GetFixtures().front(): nullptr;
	}

	void DestroyFixtures()
	{
		::box2d::DestroyFixtures(*m_bodyA);
		::box2d::DestroyFixtures(*m_bodyB);
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

	void PostStep(const Settings&, Drawer& drawer) override
	{
		const auto shapeA = static_cast<const PolygonShape*>(GetFixture(m_bodyA)->GetShape());
		const auto shapeB = static_cast<const PolygonShape*>(GetFixture(m_bodyB)->GetShape());

		const auto proxyA = GetDistanceProxy(*shapeA, 0);
		const auto proxyB = GetDistanceProxy(*shapeB, 0);
		const auto xfmA = m_bodyA->GetTransformation();
		const auto xfmB = m_bodyB->GetTransformation();

		const auto maxIndicesAB = GetMaxSeparation(shapeA->GetVertices(), shapeA->GetNormals(), xfmA,
												   shapeB->GetVertices(), xfmB);
		const auto maxIndicesBA = GetMaxSeparation(shapeB->GetVertices(), shapeB->GetNormals(), xfmB,
												   shapeA->GetVertices(), xfmA);

		const auto manifold = CollideShapes(*shapeA, xfmA, *shapeB, xfmB);
		const auto panifold = GetManifold(proxyA, xfmA, proxyB, xfmB);

		DistanceConf distanceConf;
		const auto output = Distance(proxyA, xfmA, proxyB, xfmB, distanceConf);
		distanceConf.cache = Simplex::GetCache(output.simplex.GetEdges());
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
			const auto p = (witnessPoints.a + witnessPoints.b) / RealNum{2};
			adjustedWitnessPoints.a = p;
			adjustedWitnessPoints.b = p;
			adjustedDistance = 0;
		}
		
		drawer.DrawString(xfmA.p, "Shape A");
		drawer.DrawString(xfmB.p, "Shape B");

		drawer.DrawString(5, m_textLine,
						  "Press 'A', 'D', 'W', 'S', 'Q', 'E' to move selected shape left, right, up, down, counter-clockwise, or clockwise.");
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "Press num-pad '+'/'-' to increase/decrease vertex radius of selected shape (%g & %g).",
						  rA, rB);
		m_textLine += DRAW_STRING_NEW_LINE;
		
		drawer.DrawString(5, m_textLine,
						  "Press '=', or '-' to toggle drawing simplex, or manifold info (%s, %s).",
						  m_drawSimplexInfo? "on": "off",
						  m_drawManifoldInfo? "on": "off");
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine,
						  "Max separation: %g for a-face[%i] b-vert[%i]; %g for b-face[%i] a-vert[%i]",
						  maxIndicesAB.separation, maxIndicesAB.index1, maxIndicesAB.index2,
						  maxIndicesBA.separation, maxIndicesBA.index1, maxIndicesBA.index2);
		m_textLine += DRAW_STRING_NEW_LINE;

		if (almost_equal(maxIndicesAB.separation, maxIndicesBA.separation))
		{
			//assert(maxIndicesAB.index1 == maxIndicesBA.index2);
			//assert(maxIndicesAB.index2 == maxIndicesBA.index1);
			const auto ifaceA = maxIndicesAB.index1;
			const auto nA = InverseRotate(Rotate(shapeA->GetNormal(ifaceA), xfmA.q), xfmB.q);
			// shapeA face maxIndicesAB.index1 is coplanar to an edge intersecting shapeB vertex maxIndicesAB.index2
			const auto i1 = maxIndicesAB.index2;
			const auto i0 = GetModuloPrev(i1, shapeB->GetVertexCount());
			const auto n0 = shapeB->GetNormal(i0);
			const auto n1 = shapeB->GetNormal(i1);
			const auto s0 = Dot(nA, n0);
			const auto s1 = Dot(nA, n1);
			assert(s0 != s1);
#if 0
			const auto ifaceB = (s0 < s1)?
				// shapeA face maxIndicesAB.index1 is coplanar to shapeB face i0, and
				// nearest shapeB vertex maxIndicesAB.index2
				i0:
				// shapeA face maxIndicesAB.index1 is coplanar to shapeB face i1, and
				// nearest shapeB vertex maxIndicesAB.index2
				i1;
#endif
		}
		else if (maxIndicesAB.separation > maxIndicesBA.separation)
		{
			// shape A face maxIndicesAB.index1 is least separated from shape B vertex maxIndicesAB.index2
			// Circles or Face-A manifold type.
		}
		else // maxIndicesAB.separation < maxIndicesBA.separation
		{
			// shape B face maxIndicesBA.index1 is least separated from shape A vertex maxIndicesBA.index2
			// Circles or Face-B manifold type.
		}

		drawer.DrawString(5, m_textLine, "distance = %g (from %g), iterations = %d",
						  adjustedDistance, outputDistance, output.iterations);
		m_textLine += DRAW_STRING_NEW_LINE;
		
		{
			const auto size = output.simplex.GetSize();
			drawer.DrawString(5, m_textLine, "Simplex info: size=%d, wpt-a={%g,%g}, wpt-b={%g,%g})",
							  size,
							  witnessPoints.a.x, witnessPoints.a.y,
							  witnessPoints.b.x, witnessPoints.b.y);
			m_textLine += DRAW_STRING_NEW_LINE;
			for (auto i = decltype(size){0}; i < size; ++i)
			{
				const auto& edge = output.simplex.GetSimplexEdge(i);
				const auto coef = output.simplex.GetCoefficient(i);
				
				drawer.DrawString(5, m_textLine, "  a[%d]={%g,%g} b[%d]={%g,%g} coef=%g",
								  edge.GetIndexA(),
								  edge.GetPointA().x,
								  edge.GetPointA().y,
								  edge.GetIndexB(),
								  edge.GetPointB().x,
								  edge.GetPointB().y,
								  coef);
				m_textLine += DRAW_STRING_NEW_LINE;
			}
		}

		ShowManifold(drawer, manifold, "manifold");
		ShowManifold(drawer, panifold, "wanifold");

		if (m_drawManifoldInfo)
		{
			const auto pointCount = manifold.GetPointCount();

			switch (manifold.GetType())
			{
				case Manifold::e_unset:
					break;
				case Manifold::e_circles:
				{
					const auto pA = Transform(manifold.GetLocalPoint(), xfmA);
					const auto pB = Transform(manifold.GetPoint(0).localPoint, xfmB);
					drawer.DrawCircle(pA, rA / 2, Color(1, 1, 1));
					drawer.DrawCircle(pB, rB / 2, Color(1, 1, 1));
					
					const auto psm = GetPSM(manifold, 0, xfmA, xfmB);
					const auto psm_separation = psm.m_separation - totalRadius;
					drawer.DrawCircle(psm.m_point, psm_separation, psmPointColor);

					drawer.DrawSegment(psm.m_point, psm.m_point + psm.m_normal * psm_separation, psmPointColor);

					break;
				}
				case Manifold::e_faceA:
				{
					const auto pA = Transform(manifold.GetLocalPoint(), xfmA);
					drawer.DrawCircle(pA, rA / 2, Color(1, 1, 1));
					for (auto i = decltype(pointCount){0}; i < pointCount; ++i)
					{
						const auto pB = Transform(manifold.GetOpposingPoint(i), xfmB);
						drawer.DrawCircle(pB, rB / 2, Color(1, 1, 1));
						
						const auto psm = GetPSM(manifold, i, xfmA, xfmB);
						const auto psm_separation = psm.m_separation - totalRadius;
						drawer.DrawCircle(psm.m_point, psm_separation, psmPointColor);
						
						drawer.DrawSegment(psm.m_point, psm.m_point + psm.m_normal * psm_separation, psmPointColor);
					}
					break;
				}
				case Manifold::e_faceB:
				{
					const auto pB = Transform(manifold.GetLocalPoint(), xfmB);
					drawer.DrawCircle(pB, rB / 2, Color(1, 1, 1));
					for (auto i = decltype(pointCount){0}; i < pointCount; ++i)
					{
						const auto pA = Transform(manifold.GetOpposingPoint(i), xfmA);
						drawer.DrawCircle(pA, rA / 2, Color(1, 1, 1));
						
						const auto psm = GetPSM(manifold, i, xfmA, xfmB);
						const auto psm_separation = psm.m_separation - totalRadius;
						drawer.DrawCircle(psm.m_point, psm_separation, psmPointColor);

						drawer.DrawSegment(psm.m_point, psm.m_point + psm.m_normal * psm_separation, psmPointColor);
					}
					break;
				}
			}
		}

		if (m_drawSimplexInfo)
		{
			const auto size = output.simplex.GetSize();
			for (auto i = decltype(size){0}; i < size; ++i)
			{
				const auto& edge = output.simplex.GetSimplexEdge(i);
				drawer.DrawSegment(edge.GetPointA(), edge.GetPointB(), simplexSegmentColor);
			}

			if (adjustedWitnessPoints.a != adjustedWitnessPoints.b)
			{
				drawer.DrawPoint(adjustedWitnessPoints.a, 4.0f, adjustedPointColor);
				drawer.DrawPoint(adjustedWitnessPoints.b, 4.0f, adjustedPointColor);
			}
			else
			{
				drawer.DrawPoint(adjustedWitnessPoints.a, 4.0f, matchingPointColor);
			}

			drawer.DrawPoint(witnessPoints.a, 4.0f, witnessPointColor);
			drawer.DrawPoint(witnessPoints.b, 4.0f, witnessPointColor);
			
			for (auto&& edge: output.simplex.GetEdges())
			{
				drawer.DrawPoint(edge.GetPointA(), 6.0f, simplexPointColor);
				drawer.DrawPoint(edge.GetPointB(), 6.0f, simplexPointColor);
				drawer.DrawString(edge.GetPointA(), "%d", edge.GetIndexA());
				drawer.DrawString(edge.GetPointB(), "%d", edge.GetIndexB());
			}
		}
	}

	void KeyboardDown(Key key) override
	{
		const auto fixture = GetSelectedFixture();
		const auto body = fixture? fixture->GetBody(): nullptr;

		switch (key)
		{
		case Key_A:
			if (body)
			{
				body->SetTransform(body->GetLocation() - Vec2{RealNum(0.1), 0}, body->GetAngle());
				body->SetAwake();
			}
			break;

		case Key_D:
			if (body)
			{
				body->SetTransform(body->GetLocation() + Vec2{RealNum(0.1), 0}, body->GetAngle());
				body->SetAwake();
			}
			break;

		case Key_S:
			if (body)
			{
				body->SetTransform(body->GetLocation() - Vec2{0, RealNum(0.1)}, body->GetAngle());
				body->SetAwake();
			}
			break;

		case Key_W:
			if (body)
			{
				body->SetTransform(body->GetLocation() + Vec2{0, RealNum(0.1)}, body->GetAngle());
				body->SetAwake();
			}
			break;

		case Key_Q:
			if (body)
			{
				body->SetTransform(body->GetLocation(), body->GetAngle() + 5.0f * Degree);
				body->SetAwake();
			}
			break;

		case Key_E:
			if (body)
			{
				body->SetTransform(body->GetLocation(), body->GetAngle() - 5.0f * Degree);
				body->SetAwake();
			}
			break;

		case Key_Add:
			if (body && fixture)
			{
				const auto shape = fixture->GetShape();
				auto polygon = *static_cast<const PolygonShape*>(shape);
				polygon.SetVertexRadius(shape->GetVertexRadius() + RadiusIncrement);
				SetSelectedFixture(body->CreateFixture(std::make_shared<PolygonShape>(polygon)));
				body->DestroyFixture(fixture);
			}
			break;

		case Key_Subtract:
			if (body && fixture)
			{
				const auto shape = fixture->GetShape();
				PolygonShape polygon{*static_cast<const PolygonShape*>(shape)};
				polygon.SetVertexRadius(shape->GetVertexRadius() - RadiusIncrement);
				SetSelectedFixture(body->CreateFixture(std::make_shared<PolygonShape>(polygon)));
				body->DestroyFixture(fixture);
			}
			break;

		case Key_Equal:
			m_drawSimplexInfo = !m_drawSimplexInfo;
			break;

		case Key_Minus:
			m_drawManifoldInfo = !m_drawManifoldInfo;
			break;

		default:
			break;
		}
	}

private:
	const RealNum RadiusIncrement = DefaultLinearSlop * 200;
	const Color simplexSegmentColor = Color{0.0f, 0.5f, 0.5f}; // dark cyan
	const Color simplexPointColor = Color{0, 1, 1, 0.6f}; // semi-transparent cyan
	const Color witnessPointColor = Color{1, 1, 0, 0.5}; // semi-transparent yellow
	const Color adjustedPointColor = Color{1, 0.5f, 0, 0.5f}; // semi-transparent light brown
	const Color matchingPointColor = Color{1, 0, 0}; // red
	const Color psmPointColor = Color{0.5f, 1, 1};

	Body* m_bodyA;
	Body* m_bodyB;
	bool m_drawSimplexInfo = true;
	bool m_drawManifoldInfo = true;
};
	
} // namespace box2d

#endif
