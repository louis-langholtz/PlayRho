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

#ifndef PLAYRHO_TESTS_TIME_OF_IMPACT_HPP
#define PLAYRHO_TESTS_TIME_OF_IMPACT_HPP

#include "../Framework/Test.hpp"
#include <vector>

namespace testbed {

class TimeOfImpactTest : public Test
{
public:
    TimeOfImpactTest()
    {
    }
    
    void PostStep(const Settings&, Drawer& drawer) override
    {
        const auto offset = Vec2{Real(-35), Real(70)} * 1_m;
        const auto sweepA = Sweep{
            Position2D{Vec2(24.0f, -60.0f) * 1_m + offset, 2.95_rad}
        };
        const auto sweepB = Sweep{
            Position2D{Vec2(53.474274f, -50.252514f) * 1_m + offset, 513.36676_rad},
            Position2D{Vec2(54.595478f, -51.083473f) * 1_m + offset, 513.62781_rad}
        };

        const auto output = GetToiViaSat(GetChild(m_shapeA, 0), sweepA,
                                         GetChild(m_shapeB, 0), sweepB);

        std::stringstream stream;
        stream << "At TOI ";
        stream << static_cast<float>(output.time);
        stream << ", state is ";
        stream << GetName(output.state);
        stream << ". TOI iterations is " << unsigned{output.stats.toi_iters};
        stream << ", max root iterations is " << unsigned{output.stats.max_root_iters};
        stream << ".";
        m_status = stream.str();

        {
            const auto vertexCount = m_shapeA.GetVertexCount();
            auto vertices = std::vector<Length2>(vertexCount);
            const auto transformA = GetTransformation(sweepA, 0.0f);
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeA.GetVertex(i), transformA);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.9f, 0.9f, 0.9f));
        }

        {
            const auto vertexCount = m_shapeB.GetVertexCount();
            auto vertices = std::vector<Length2>(vertexCount);
            const auto transformB = GetTransformation(sweepB, 0.0f);
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.5f, 0.9f, 0.5f));
        }

        {
            const auto vertexCount = m_shapeB.GetVertexCount();
            auto vertices = std::vector<Length2>(vertexCount);
            const auto transformB = GetTransformation(sweepB, output.time);
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.5f, 0.7f, 0.9f));
        }

        {
            const auto vertexCount = m_shapeB.GetVertexCount();
            auto vertices = std::vector<Length2>(vertexCount);
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
            auto vertices = std::vector<Length2>(vertexCount);
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.9f, 0.5f, 0.5f));
        }
#endif
    }
    
    PolygonShapeConf m_shapeA{PolygonShapeConf{}.SetAsBox(25_m, 5_m)};
    PolygonShapeConf m_shapeB{PolygonShapeConf{}.SetAsBox(2.5_m, 2.5_m)};
};

} // namespace testbed

#endif /* PLAYRHO_TESTS_TIME_OF_IMPACT_HPP */
