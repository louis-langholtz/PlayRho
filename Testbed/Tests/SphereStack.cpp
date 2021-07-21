/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "../Framework/Test.hpp"

namespace testbed {

class SphereStack : public Test
{
public:
    static inline const auto registered = RegisterTest("Sphere Stack", MakeUniqueTest<SphereStack>);
    enum { e_count = 10 };

    SphereStack()
    {
        Attach(GetWorld(), CreateBody(GetWorld()),
               CreateShape(GetWorld(),
                           EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}));
        const auto shape =
            CreateShape(GetWorld(), DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
        for (auto i = 0; i < e_count; ++i) {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.location = Vec2(0, 4.0f + 3.0f * i) * 1_m;
            m_bodies[i] = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), m_bodies[i], shape);
            SetVelocity(GetWorld(), m_bodies[i], Velocity{Vec2(0.0f, -50.0f) * 1_mps, 0_rpm});
        }
    }

    BodyID m_bodies[e_count];
};

} // namespace testbed
