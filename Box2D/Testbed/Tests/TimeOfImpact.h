/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef TIME_OF_IMPACT_H
#define TIME_OF_IMPACT_H

namespace box2d {

class TimeOfImpactTest : public Test
{
public:
	TimeOfImpactTest()
	{
		m_shapeA.SetAsBox(25.0f, 5.0f);
		m_shapeB.SetAsBox(2.5f, 2.5f);
	}

	static Test* Create()
	{
		return new TimeOfImpactTest;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		Sweep sweepA;
		sweepA.pos0 = Position{Vec2(24.0f, -60.0f), 2.95f};
		sweepA.pos1 = sweepA.pos0;
		sweepA.localCenter = Vec2_zero;

		Sweep sweepB;
		sweepB.pos0 = Position{Vec2(53.474274f, -50.252514f), 513.36676f}; // - 162.0f * Pi;
		sweepB.pos1 = Position{Vec2(54.595478f, -51.083473f), 513.62781f}; //  - 162.0f * Pi;
		sweepB.localCenter = Vec2_zero;

		//sweepB.a0 -= 300.0f * Pi;
		//sweepB.a -= 300.0f * Pi;

		TOIInput input;
		input.proxyA = DistanceProxy(m_shapeA, 0);
		input.proxyB = DistanceProxy(m_shapeB, 0);
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tMax = 1.0f;

		const auto output = TimeOfImpact(input);

		g_debugDraw.DrawString(5, m_textLine, "toi = %g", output.get_t());
		m_textLine += DRAW_STRING_NEW_LINE;

		extern int32 toiMaxIters, toiMaxRootIters;
		g_debugDraw.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", toiMaxIters, toiMaxRootIters);
		m_textLine += DRAW_STRING_NEW_LINE;

		Vec2 vertices[MaxPolygonVertices];

		const auto transformA = GetTransform(sweepA, 0.0f);
		for (int32 i = 0; i < m_shapeA.GetVertexCount(); ++i)
		{
			vertices[i] = Mul(transformA, m_shapeA.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeA.GetVertexCount(), Color(0.9f, 0.9f, 0.9f));

		auto transformB = GetTransform(sweepB, 0.0f);
		//Vec2 localPoint(2.0f, -0.1f);
		for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
		{
			vertices[i] = Mul(transformB, m_shapeB.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), Color(0.5f, 0.9f, 0.5f));

		transformB = GetTransform(sweepB, output.get_t());
		for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
		{
			vertices[i] = Mul(transformB, m_shapeB.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), Color(0.5f, 0.7f, 0.9f));

		transformB = GetTransform(sweepB, 1.0f);
		for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
		{
			vertices[i] = Mul(transformB, m_shapeB.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), Color(0.9f, 0.5f, 0.5f));

#if 0
		for (float_t t = 0.0f; t < 1.0f; t += 0.1f)
		{
			transformB = sweepB.GetTransform(t);
			for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
			{
				vertices[i] = Mul(transformB, m_shapeB.GetVertex(i));
			}
			g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), Color(0.9f, 0.5f, 0.5f));
		}
#endif
	}

	PolygonShape m_shapeA;
	PolygonShape m_shapeB;
};

} // namespace box2d

#endif
