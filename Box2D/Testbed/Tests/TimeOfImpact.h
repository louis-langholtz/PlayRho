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

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
		m_shapeA.SetAsBox(25.0f, 5.0f);
		m_shapeB.SetAsBox(2.5f, 2.5f);
	}

	static Test* Create()
	{
		return new TimeOfImpact;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		b2Sweep sweepA;
		sweepA.c0 = b2Vec2(24.0f, -60.0f);
		sweepA.a0 = 2.95f;
		sweepA.c = sweepA.c0;
		sweepA.a = sweepA.a0;
		sweepA.localCenter = b2Vec2_zero;

		b2Sweep sweepB;
		sweepB.c0 = b2Vec2(53.474274f, -50.252514f);
		sweepB.a0 = 513.36676f; // - 162.0f * b2_pi;
		sweepB.c = b2Vec2(54.595478f, -51.083473f);
		sweepB.a = 513.62781f; //  - 162.0f * b2_pi;
		sweepB.localCenter = b2Vec2_zero;

		//sweepB.a0 -= 300.0f * b2_pi;
		//sweepB.a -= 300.0f * b2_pi;

		b2TOIInput input;
		input.proxyA = b2DistanceProxy(m_shapeA, 0);
		input.proxyB = b2DistanceProxy(m_shapeB, 0);
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tMax = 1.0f;

		b2TOIOutput output;

		b2TimeOfImpact(output, input);

		g_debugDraw.DrawString(5, m_textLine, "toi = %g", output.t);
		m_textLine += DRAW_STRING_NEW_LINE;

		extern int32 b2_toiMaxIters, b2_toiMaxRootIters;
		g_debugDraw.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", b2_toiMaxIters, b2_toiMaxRootIters);
		m_textLine += DRAW_STRING_NEW_LINE;

		b2Vec2 vertices[b2_maxPolygonVertices];

		b2Transform transformA = sweepA.GetTransform(0.0f);
		for (int32 i = 0; i < m_shapeA.GetVertexCount(); ++i)
		{
			vertices[i] = b2Mul(transformA, m_shapeA.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeA.GetVertexCount(), b2Color(0.9f, 0.9f, 0.9f));

		b2Transform transformB = sweepB.GetTransform(0.0f);
		
		//b2Vec2 localPoint(2.0f, -0.1f);

		for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), b2Color(0.5f, 0.9f, 0.5f));

		transformB = sweepB.GetTransform(output.t);
		for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), b2Color(0.5f, 0.7f, 0.9f));

		transformB = sweepB.GetTransform(1.0f);
		for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.GetVertex(i));
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.GetVertexCount(), b2Color(0.9f, 0.5f, 0.5f));

#if 0
		for (b2Float t = 0.0f; t < 1.0f; t += 0.1f)
		{
			sweepB.GetTransform(&transformB, t);
			for (int32 i = 0; i < m_shapeB.m_count; ++i)
			{
				vertices[i] = b2Mul(transformB, m_shapeB.GetVertex(i));
			}
			g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.9f, 0.5f, 0.5f));
		}
#endif
	}

	b2PolygonShape m_shapeA;
	b2PolygonShape m_shapeB;
};

#endif
