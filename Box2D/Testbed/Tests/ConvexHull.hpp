/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

namespace box2d {

class ConvexHull : public Test
{
public:
	enum
	{
		e_count = MaxPolygonVertices
	};

	ConvexHull()
	{
		Generate();
		m_auto = false;
	}

	void Generate()
	{
		Vec2 lowerBound(-8.0f, -8.0f);
		Vec2 upperBound(8.0f, 8.0f);

		for (int32 i = 0; i < e_count; ++i)
		{
			float_t x = 10.0f * RandomFloat();
			float_t y = 10.0f * RandomFloat();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			Vec2 v(x, y);
			v = Clamp(v, lowerBound, upperBound);
			m_points[i] = v;
		}

		m_count = e_count;
	}

	static Test* Create()
	{
		return new ConvexHull;
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_auto = !m_auto;
			break;

		case Key_G:
			Generate();
			break;

		default:
			break;
		}
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		Test::Step(settings, drawer);

		const auto shape = PolygonShape(Span<const Vec2>{m_points, m_count});

		drawer.DrawString(5, m_textLine, "Press g to generate a new random convex hull");
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawPolygon(shape.GetVertices().begin(), shape.GetVertexCount(), Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_count; ++i)
		{
			drawer.DrawPoint(m_points[i], 3.0f, Color(0.3f, 0.9f, 0.3f));
			drawer.DrawString(m_points[i] + Vec2(0.05f, 0.05f), "%d", i);
		}

		if (!Validate(shape))
		{
			drawer.DrawString(5, m_textLine, "Note: Invalid convex hull");
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (m_auto)
		{
			Generate();
		}
	}

	Vec2 m_points[MaxPolygonVertices];
	std::remove_cv<decltype(MaxPolygonVertices)>::type m_count;
	bool m_auto;
};

} // namespace box2d

#endif
