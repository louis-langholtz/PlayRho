/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

class Chain : public Test
{
public:
    static inline const auto registered = RegisterTest("Chain", MakeUniqueTest<Chain>);

    Chain()
    {
        const auto ground = CreateBody(GetWorld());
        Attach(GetWorld(), ground, CreateShape(GetWorld(), GetGroundEdgeConf()));
        const auto shape = CreateShape(
            GetWorld(), PolygonShapeConf{}.UseDensity(20_kgpm2).UseFriction(Real(0.2)).SetAsBox(
                            0.6_m, 0.125_m));
        const auto y = 25.0f;
        auto prevBody = ground;
        for (auto i = 0; i < 30; ++i) {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.location = Vec2(0.5f + i, y) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), body, shape);
            CreateJoint(GetWorld(),
                        GetRevoluteJointConf(GetWorld(), prevBody, body, Vec2(Real(i), y) * 1_m));
            prevBody = body;
        }
    }
};

} // namespace testbed
