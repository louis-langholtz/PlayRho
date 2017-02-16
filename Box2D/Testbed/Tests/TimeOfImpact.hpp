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

#include <vector>
#include <Box2D/Collision/TimeOfImpact.hpp>

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

	static const char *GetName(TOIOutput::State state)
	{
		switch (state)
		{
			case TOIOutput::e_failed: return "failed";
			case TOIOutput::e_unknown: return "unknown";
			case TOIOutput::e_touching: return "touching";
			case TOIOutput::e_separated: return "separated";
			case TOIOutput::e_overlapped: return "overlapped";
		}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		const auto offset = Vec2{RealNum(-35), RealNum(70)};
		const auto sweepA = Sweep{
			Position{Vec2(24.0f, -60.0f) + offset, 2.95_rad}
		};
		const auto sweepB = Sweep{
			Position{Vec2(53.474274f, -50.252514f) + offset, 513.36676_rad},
			Position{Vec2(54.595478f, -51.083473f) + offset, 513.62781_rad}
		};

		const auto output = TimeOfImpact(GetDistanceProxy(m_shapeA, 0), sweepA, GetDistanceProxy(m_shapeB, 0), sweepB);

		drawer.DrawString(5, m_textLine, "at toi=%g, state=%s", static_cast<float>(output.get_t()), GetName(output.get_state()));
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "TOI iters = %d, max root iters = %d",
						  output.get_toi_iters(), output.get_max_root_iters());
		m_textLine += DRAW_STRING_NEW_LINE;

		{
			const auto vertexCount = m_shapeA.GetVertexCount();
			auto vertices = std::vector<Vec2>(vertexCount);
			const auto transformA = GetTransformation(sweepA, 0.0f);
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeA.GetVertex(i), transformA);
			}
			drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.9f, 0.9f, 0.9f));
		}

		{
			const auto vertexCount = m_shapeB.GetVertexCount();
			auto vertices = std::vector<Vec2>(vertexCount);
			const auto transformB = GetTransformation(sweepB, 0.0f);
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.5f, 0.9f, 0.5f));
		}

		{
			const auto vertexCount = m_shapeB.GetVertexCount();
			auto vertices = std::vector<Vec2>(vertexCount);
			const auto transformB = GetTransformation(sweepB, output.get_t());
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.5f, 0.7f, 0.9f));
		}

		{
			const auto vertexCount = m_shapeB.GetVertexCount();
			auto vertices = std::vector<Vec2>(vertexCount);
			const auto transformB = GetTransformation(sweepB, 1.0f);
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.9f, 0.5f, 0.5f));
		}

#if 1
		for (auto t = 0.0f; t < 1.0f; t += 0.1f)
		{
			const auto transformB = GetTransformation(sweepB, t);
			const auto vertexCount = m_shapeB.GetVertexCount();
			auto vertices = std::vector<Vec2>(vertexCount);
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
			}
			drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.9f, 0.5f, 0.5f));
		}
#endif
	}

	PolygonShape m_shapeA;
	PolygonShape m_shapeB;
};

} // namespace box2d

#endif
