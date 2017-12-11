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
        m_transformA = Transformation{Length2{}, UnitVec2::GetRight()};
        m_positionB = Vec2(19.345284f, 1.5632932f) * 1_m;
        m_angleB = 1.9160721_rad;
        m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Move Left", [&](KeyActionMods) {
            GetX(m_positionB) -= 0.1_m;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Move Right", [&](KeyActionMods) {
            GetX(m_positionB) += 0.1_m;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Move Down", [&](KeyActionMods) {
            GetY(m_positionB) -= 0.1_m;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        });
        RegisterForKey(GLFW_KEY_W, GLFW_PRESS, 0, "Move Up", [&](KeyActionMods) {
            GetY(m_positionB) += 0.1_m;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        });
        RegisterForKey(GLFW_KEY_Q, GLFW_PRESS, 0, "Rotate Counter Clockwise", [&](KeyActionMods) {
            m_angleB += 0.1_rad * Pi;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        });
        RegisterForKey(GLFW_KEY_E, GLFW_PRESS, 0, "Rotate Clockwise", [&](KeyActionMods) {
            m_angleB -= 0.1_rad * Pi;
            m_transformB = Transformation{m_positionB, UnitVec2::Get(m_angleB)};
        });
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        NOT_USED(settings);

        const auto proxyA = GetChild(m_polygonA, 0);
        const auto proxyB = GetChild(m_polygonB, 0);

        const auto manifold = CollideShapes(proxyA, m_transformA, proxyB, m_transformB);
        const auto pointCount = manifold.GetPointCount();

        std::stringstream stream;
        stream << "Point count: " << unsigned{pointCount} << ".";
        m_status = stream.str();

        {
            const auto color = Color(0.9f, 0.9f, 0.9f);
            {
                const auto vertexCount = m_polygonA.GetVertexCount();
                auto v = std::vector<Length2>(vertexCount);
                for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
                {
                    v[i] = Transform(m_polygonA.GetVertex(i), m_transformA);
                }
                drawer.DrawPolygon(&v[0], vertexCount, color);
            }

            {
                const auto vertexCount = m_polygonB.GetVertexCount();
                auto v = std::vector<Length2>(vertexCount);
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
    
    PolygonShape::Conf m_polygonA{PolygonShape::Conf{}.SetAsBox(0.2_m, 0.4_m)};
    PolygonShape::Conf m_polygonB{PolygonShape::Conf{}.SetAsBox(0.5_m, 0.5_m)};

    Transformation m_transformA;
    Transformation m_transformB;

    Length2 m_positionB;
    Angle m_angleB;
};

} // namespace playrho

#endif
