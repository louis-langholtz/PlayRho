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

#ifndef PLAYRHO_VARYING_RESTITUTION_HPP
#define PLAYRHO_VARYING_RESTITUTION_HPP

#include "../Framework/Test.hpp"

namespace playrho {

// Note: even with a restitution of 1.0, there is some energy change
// due to position correction.
class VaryingRestitution : public Test
{
public:

    VaryingRestitution()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(Shape(GetGroundEdgeConf()));

        Real restitution[7] = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};
        auto shape = DiskShapeConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2);
        for (auto i = 0; i < 7; ++i)
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(Real(-10 + 3 * i), 20) * 1_m;
            const auto body = m_world.CreateBody(bd);
            shape.SetRestitution(restitution[i]);
            body->CreateFixture(Shape(shape));
        }
    }
};

} // namespace playrho

#endif
