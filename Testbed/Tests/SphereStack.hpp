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

#ifndef PLAYRHO_SPHERE_STACK_HPP
#define PLAYRHO_SPHERE_STACK_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class SphereStack : public Test
{
public:

    enum
    {
        e_count = 10
    };

    SphereStack()
    {
        m_world.CreateBody()->CreateFixture(Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});
        const auto shape = Shape{DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2)};
        for (auto i = 0; i < e_count; ++i)
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(0, 4.0f + 3.0f * i) * 1_m;
            m_bodies[i] = m_world.CreateBody(bd);
            m_bodies[i]->CreateFixture(shape);
            m_bodies[i]->SetVelocity(Velocity{Vec2(0.0f, -50.0f) * 1_mps, 0_rpm});
        }
    }

    Body* m_bodies[e_count];
};

} // namespace testbed

#endif
