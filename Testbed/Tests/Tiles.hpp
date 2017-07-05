/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef TILES_H
#define TILES_H

#include "../Framework/Test.hpp"

namespace box2d {

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
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
            BodyDef bd;
            bd.position.y = -a * Meter;
            const auto ground = m_world->CreateBody(bd);

            const auto N = 200;
            const auto M = 10;
            Vec2 position;
            position.y = 0.0f;
            for (auto j = 0; j < M; ++j)
            {
                position.x = -N * a;
                for (auto i = 0; i < N; ++i)
                {
                    PolygonShape shape;
                    SetAsBox(shape, a * Meter, a * Meter, position * Meter, Angle{0});
                    ground->CreateFixture(std::make_shared<PolygonShape>(shape));
                    ++m_fixtureCount;
                    position.x += 2.0f * a;
                }
                position.y -= 2.0f * a;
            }
        }

        {
            const auto a = Real{0.5f};
            const auto shape = std::make_shared<PolygonShape>(a * Meter, a * Meter);
            shape->SetDensity(Real{5} * KilogramPerSquareMeter);

            Vec2 x(-7.0f, 0.75f);
            Vec2 y;
            const auto deltaX = Vec2(0.5625f, 1.25f);
            const auto deltaY = Vec2(1.125f, 0.0f);

            for (auto i = 0; i < e_count; ++i)
            {
                y = x;

                for (auto j = i; j < e_count; ++j)
                {
                    BodyDef bd;
                    bd.type = BodyType::Dynamic;
                    bd.position = y * Meter;

                    const auto body = m_world->CreateBody(bd);
                    body->CreateFixture(shape);
                    ++m_fixtureCount;
                    y += deltaY;
                }

                x += deltaX;
            }
        }

        const auto end = std::chrono::high_resolution_clock::now();
        const auto elapsed_secs = std::chrono::duration<Real>{end - start};
        m_createTime = elapsed_secs.count();
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        const auto height = m_world->GetTreeHeight();
        const auto leafCount = m_world->GetProxyCount();
        if (leafCount > 0)
        {
            const auto minimumNodeCount = 2 * leafCount - 1;
            const auto minimumHeight = ceilf(logf(float(minimumNodeCount)) / logf(2.0f));
            drawer.DrawString(5, m_textLine, "dynamic tree height = %d, min = %d",
                              height, int(minimumHeight));
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        drawer.DrawString(5, m_textLine, "create time = %6.2f ms, fixture count = %d",
            static_cast<double>(m_createTime * Real(1000)), m_fixtureCount);
        m_textLine += DRAW_STRING_NEW_LINE;

        //DynamicTree* tree = &m_world->m_contactManager.m_broadPhase.m_tree;

        //if (GetStepCount() == 400)
        //{
        //    tree->RebuildBottomUp();
        //}
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
            case Key_C:
                m_snapshot = *m_world;
                break;
            case Key_Backspace:
                if (m_snapshot.GetBodies().size() > 0)
                {
	                ResetWorld(m_snapshot);
                }
                break;
            default:
                break;
        }
    }
    
    int m_fixtureCount;
    Real m_createTime;
    World m_snapshot;
};

} // namespace box2d

#endif
