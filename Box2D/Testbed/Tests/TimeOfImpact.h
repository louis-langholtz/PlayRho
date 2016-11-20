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

#ifndef TIME_OF_IMPACT_H
#define TIME_OF_IMPACT_H

#include <Box2D/Collision/TimeOfImpact.h>

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

	void Step(Settings& settings, Drawer& drawer) override
	{
		Test::Step(settings, drawer);

		const auto sweepA = Sweep{Position{Vec2(24.0f, -60.0f), 2.95_rad}};
		const auto sweepB = Sweep{Position{Vec2(53.474274f, -50.252514f), 513.36676_rad}, Position{Vec2(54.595478f, -51.083473f), 513.62781_rad}};

		const auto output = TimeOfImpact(GetDistanceProxy(m_shapeA, 0), sweepA, GetDistanceProxy(m_shapeB, 0), sweepB);

		drawer.DrawString(5, m_textLine, "toi = %g", output.get_t());
		m_textLine += DRAW_STRING_NEW_LINE;

		std::remove_const<decltype(MaxTOIIterations)>::type toiMaxIters = 0;
		int32 toiMaxRootIters = 0;
		drawer.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", toiMaxIters, toiMaxRootIters);
		m_textLine += DRAW_STRING_NEW_LINE;

		Vec2 vertices[MaxPolygonVertices];

		{
			const auto transformA = GetTransformation(sweepA, 0.0f);
			const auto vertexCount = m_shapeA.GetVertexCount();
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeA.GetVertex(i), transformA);
			}
			drawer.DrawPolygon(vertices, vertexCount, Color(0.9f, 0.9f, 0.9f));
		}

		{
			const auto transformB = GetTransformation(sweepB, 0.0f);
			const auto vertexCount = m_shapeB.GetVertexCount();
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(vertices, vertexCount, Color(0.5f, 0.9f, 0.5f));
		}

		{
			const auto transformB = GetTransformation(sweepB, output.get_t());
			const auto vertexCount = m_shapeB.GetVertexCount();
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(vertices, vertexCount, Color(0.5f, 0.7f, 0.9f));
		}

		{
			const auto transformB = GetTransformation(sweepB, 1.0f);
			const auto vertexCount = m_shapeB.GetVertexCount();
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(vertices, vertexCount, Color(0.9f, 0.5f, 0.5f));
		}

#if 0
		for (float_t t = 0.0f; t < 1.0f; t += 0.1f)
		{
			transformB = sweepB.GetTransformation(t);
			for (int32 i = 0; i < m_shapeB.GetVertexCount(); ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(vertices, m_shapeB.GetVertexCount(), Color(0.9f, 0.5f, 0.5f));
		}
#endif
	}

	PolygonShape m_shapeA;
	PolygonShape m_shapeB;
};

} // namespace box2d

#endif
