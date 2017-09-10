/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_POLYCOLLISION_HPP
#define PLAYRHO_POLYCOLLISION_HPP

#include "../Framework/Test.hpp"
#include <vector>

namespace playrho {

class PolyCollision : public Test
{
public:
    PolyCollision()
    {
        {
            m_polygonA.SetAsBox(Real{0.2f} * Meter, Real{0.4f} * Meter);
            m_transformA = Transformation{Vec2(0.0f, 0.0f) * Meter, UnitVec2::GetRight()};
        }

        {
            m_polygonB.SetAsBox(Real{0.5f} * Meter, Real{0.5f} * Meter);
            m_positionB = Vec2(19.345284f, 1.5632932f) * Meter;
            m_angleB = Real{1.9160721f} * Radian;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        }
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        NOT_USED(settings);

        const auto proxyA = m_polygonA.GetChild(0);
        const auto proxyB = m_polygonB.GetChild(0);

        const auto manifold = CollideShapes(proxyA, m_transformA, proxyB, m_transformB);
        const auto pointCount = manifold.GetPointCount();

        drawer.DrawString(5, m_textLine, "point count = %d", pointCount);
        m_textLine += DRAW_STRING_NEW_LINE;

        {
            const auto color = Color(0.9f, 0.9f, 0.9f);
            {
                const auto vertexCount = m_polygonA.GetVertexCount();
                auto v = std::vector<Length2D>(vertexCount);
                for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
                {
                    v[i] = Transform(m_polygonA.GetVertex(i), m_transformA);
                }
                drawer.DrawPolygon(&v[0], vertexCount, color);
            }

            {
                const auto vertexCount = m_polygonB.GetVertexCount();
                auto v = std::vector<Length2D>(vertexCount);
                for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
                {
                    v[i] = Transform(m_polygonB.GetVertex(i), m_transformB);
                }
                drawer.DrawPolygon(&v[0], vertexCount, color);
            }
        }

        const auto worldManifold = GetWorldManifold(manifold,
                                                    m_transformA, GetVertexRadius(m_polygonA),
                                                    m_transformB, GetVertexRadius(m_polygonB));
        for (auto i = decltype(pointCount){0}; i < pointCount; ++i)
        {
            drawer.DrawPoint(worldManifold.GetPoint(i), 4.0f, Color(0.9f, 0.3f, 0.3f));
        }
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_A:
            GetX(m_positionB) -= Real{0.1f} * Meter;
            break;

        case Key_D:
            GetX(m_positionB) += Real{0.1f} * Meter;
            break;

        case Key_S:
            GetY(m_positionB) -= Real{0.1f} * Meter;
            break;

        case Key_W:
            GetY(m_positionB) += Real{0.1f} * Meter;
            break;

        case Key_Q:
            m_angleB += Real{0.1f} * Radian * Pi;
            break;

        case Key_E:
            m_angleB -= Real{0.1f} * Radian * Pi;
            break;

        default:
            break;
        }

        m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
    }

    PolygonShape m_polygonA;
    PolygonShape m_polygonB;

    Transformation m_transformA;
    Transformation m_transformB;

    Length2D m_positionB;
    Angle m_angleB;
};

} // namespace playrho

#endif
