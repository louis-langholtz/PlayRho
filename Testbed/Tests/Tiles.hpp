/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_TILES_HPP
#define PLAYRHO_TILES_HPP

#include "../Framework/Test.hpp"

namespace testbed {

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is smooth due to PlayRho knowing about adjacency.
class Tiles : public Test
{
public:
    enum
    {
        e_count = 20
    };

    Tiles()
    {
        m_fixtureCount = 0;
        const auto start = std::chrono::high_resolution_clock::now();

        {
            const auto a = Real{0.5f};
            BodyConf bd;
            GetY(bd.location) = -a * 1_m;
            const auto ground = CreateBody(GetWorld(), bd);

            const auto N = 200;
            const auto M = 10;
            Vec2 position;
            GetY(position) = 0.0f;
            for (auto j = 0; j < M; ++j)
            {
                GetX(position) = -N * a;
                for (auto i = 0; i < N; ++i)
                {
                    Attach(GetWorld(), ground, CreateShape(GetWorld(), PolygonShapeConf{}.SetAsBox(a * 1_m, a * 1_m, position * 1_m, 0_deg)));
                    ++m_fixtureCount;
                    GetX(position) += 2.0f * a;
                }
                GetY(position) -= 2.0f * a;
            }
        }

        {
            const auto a = Real{0.5f};
            const auto shape = CreateShape(GetWorld(),
                                           PolygonShapeConf{}.UseDensity(5_kgpm2).SetAsBox(a * 1_m, a * 1_m));

            Vec2 x(-7.0f, 0.75f);
            Vec2 y;
            const auto deltaX = Vec2(0.5625f, 1.25f);
            const auto deltaY = Vec2(1.125f, 0.0f);

            for (auto i = 0; i < e_count; ++i)
            {
                y = x;

                for (auto j = i; j < e_count; ++j)
                {
                    BodyConf bd;
                    bd.type = BodyType::Dynamic;
                    bd.location = y * 1_m;
                    bd.linearAcceleration = GetGravity();

                    const auto body = CreateBody(GetWorld(), bd);
                    Attach(GetWorld(), body, shape);
                    ++m_fixtureCount;
                    y += deltaY;
                }

                x += deltaX;
            }
        }

        const auto end = std::chrono::high_resolution_clock::now();
        const auto elapsed_secs = std::chrono::duration<double>{end - start};
        m_createTime = elapsed_secs.count();
        
        RegisterForKey(GLFW_KEY_C, GLFW_PRESS, 0, "Make a snapshot.", [&](KeyActionMods) {
            m_snapshot = GetWorld();
        });
        RegisterForKey(GLFW_KEY_BACKSPACE, GLFW_PRESS, 0, "Restore to snapshot.", [&](KeyActionMods) {
            if (!empty(m_snapshot.GetBodies()))
            {
                ResetWorld(m_snapshot);
            }
        });
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << "Create time = ";
        stream << m_createTime * 1000;
        stream << " ms, fixture count = ";
        stream << m_fixtureCount;
        stream << ".";
        SetStatus(stream.str());
    }
    
    int m_fixtureCount;
    double m_createTime;
    World m_snapshot;
};

} // namespace testbed

#endif
