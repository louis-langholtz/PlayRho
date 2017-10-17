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

namespace playrho {

class TimeOfImpactTest : public Test
{
public:
    TimeOfImpactTest()
    {
        m_shapeA.SetAsBox(Real{25.0f} * Meter, Real{5.0f} * Meter);
        m_shapeB.SetAsBox(Real{2.5f} * Meter, Real{2.5f} * Meter);
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
            default: break;
        }
        return "unknown";
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        const auto offset = Vec2{Real(-35), Real(70)} * Meter;
        const auto sweepA = Sweep{
            Position{Vec2(24.0f, -60.0f) * Meter + offset, Real{2.95f} * Radian}
        };
        const auto sweepB = Sweep{
            Position{Vec2(53.474274f, -50.252514f) * Meter + offset, Real{513.36676f} * Radian},
            Position{Vec2(54.595478f, -51.083473f) * Meter + offset, Real{513.62781f} * Radian}
        };

        const auto output = GetToiViaSat(m_shapeA.GetChild(0), sweepA,
                                         m_shapeB.GetChild(0), sweepB);

        std::stringstream stream;
        stream << "At TOI ";
        stream << static_cast<float>(output.get_t());
        stream << ", state is ";
        stream << GetName(output.get_state());
        stream << ". TOI iterations is " << unsigned{output.get_toi_iters()};
        stream << ", max root iterations is " << unsigned{output.get_max_root_iters()};
        stream << ".";
        m_status = stream.str();

        {
            const auto vertexCount = m_shapeA.GetVertexCount();
            auto vertices = std::vector<Length2D>(vertexCount);
            const auto transformA = GetTransformation(sweepA, 0.0f);
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeA.GetVertex(i), transformA);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.9f, 0.9f, 0.9f));
        }

        {
            const auto vertexCount = m_shapeB.GetVertexCount();
            auto vertices = std::vector<Length2D>(vertexCount);
            const auto transformB = GetTransformation(sweepB, 0.0f);
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.5f, 0.9f, 0.5f));
        }

        {
            const auto vertexCount = m_shapeB.GetVertexCount();
            auto vertices = std::vector<Length2D>(vertexCount);
            const auto transformB = GetTransformation(sweepB, output.get_t());
            for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
            {
                vertices[i] = Transform(m_shapeB.GetVertex(i), transformB);
            }
            drawer.DrawPolygon(&vertices[0], vertexCount, Color(0.5f, 0.7f, 0.9f));
        }

        {
            const auto vertexCount = m_shapeB.GetVertexCount();
            auto vertices = std::vector<Length2D>(vertexCount);
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
            auto vertices = std::vector<Length2D>(vertexCount);
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

} // namespace playrho

#endif /* PLAYRHO_TESTS_TIME_OF_IMPACT_HPP */
