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

#ifndef PLAYRHO_VERTICAL_STACK_HPP
#define PLAYRHO_VERTICAL_STACK_HPP

#include "../Framework/Test.hpp"
#include <array>

namespace testbed {

class VerticalStack : public Test
{
public:

    enum
    {
        e_columnCount = 5,
        e_rowCount = 10
        //e_columnCount = 1,
        //e_rowCount = 1
    };

    VerticalStack()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});
        ground->CreateFixture(Shape{EdgeShapeConf{Vec2(20.0f, 0.0f) * 1_m, Vec2(20.0f, 20.0f) * 1_m}});

        const float xs[] = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};
        assert(e_columnCount <= sizeof(xs)/sizeof(xs[0]));

        const auto hdim = Real{0.1f}; // 0.5f is less stable than 1.0f for boxes not at origin (x of 0)
        const auto shape = Shape{
            PolygonShapeConf{}.UseDensity(1_kgpm2).UseFriction(0.3).SetAsBox(hdim * 1_m, hdim * 1_m)
        };
        for (auto j = 0; j < e_columnCount; ++j)
        {
            for (auto i = 0; i < e_rowCount; ++i)
            {
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.linearAcceleration = m_gravity;

                const auto x = 0.0f;
                //const auto x = RandomFloat(-0.02f, 0.02f);
                //const auto x = i % 2 == 0 ? -0.01f : 0.01f;
                //bd.position = Vec2(xs[j] + x, (hdim - hdim/20) + (hdim * 2 - hdim / 20) * i);
                bd.location = Vec2(xs[j] + x, (i + 1) * hdim * 4) * 1_m;
                m_world.CreateBody(bd)->CreateFixture(shape);
            }
        }

        m_bullet = nullptr;
        
        RegisterForKey(GLFW_KEY_COMMA, GLFW_PRESS, 0, "Launch a bullet.", [&](KeyActionMods) {
            if (m_bullet)
            {
                m_world.Destroy(m_bullet);
                m_bullet = nullptr;
            }
            
            {
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.linearAcceleration = m_gravity;
                bd.bullet = true;
                bd.location = Vec2(-31.0f, 5.0f) * 1_m;
                
                m_bullet = m_world.CreateBody(bd);
                m_bullet->CreateFixture(m_bulletshape);
                m_bullet->SetVelocity(Velocity{Vec2(400.0f, 0.0f) * 1_mps, 0_rpm});
            }
        });
    }

    Body* m_bullet;
    Shape m_bulletshape = Shape{DiskShapeConf{}.UseRadius(0.25_m).UseDensity(20_kgpm2).UseRestitution(0.05)};
};
    
} // namespace testbed

#endif
