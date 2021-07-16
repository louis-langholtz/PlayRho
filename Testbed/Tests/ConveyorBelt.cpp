/*
 * Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#include "../Framework/Test.hpp"

namespace testbed {

class ConveyorBelt : public Test
{
public:
    static inline const auto registered = RegisterTest("Conveyor Belt", MakeUniqueTest<ConveyorBelt>);

    ConveyorBelt()
    {
        SetPreSolveContactListener(GetWorld(), [this](ContactID id, const Manifold& manifold) {
            Test::PreSolve(id, manifold);
            ConveyorBelt::PreSolve(id, manifold);
        });

        // Ground
        Attach(GetWorld(), CreateBody(GetWorld()),
               CreateShape(GetWorld(),
                           EdgeShapeConf{Vec2(-20.0f, 0.0f) * 1_m, Vec2(20.0f, 0.0f) * 1_m}));

        // Platform
        {
            BodyConf bd;
            bd.location = Vec2(-5.0f, 5.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);

            auto conf = PolygonShapeConf{};
            conf.friction = 0.8f;
            conf.SetAsBox(10_m, 0.5_m);
            m_platform = CreateShape(GetWorld(), conf);
            Attach(GetWorld(), body, m_platform);
        }

        // Boxes
        const auto boxshape =
            CreateShape(GetWorld(), PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(0.5_m, 0.5_m));
        for (auto i = 0; i < 5; ++i) {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.location = Vec2(-10.0f + 2.0f * i, 7.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), body, boxshape);
        }
    }

    void PreSolve(ContactID contact, const Manifold& /*oldManifold*/)
    {
        const auto shapeA = GetShapeA(GetWorld(), contact);
        const auto shapeB = GetShapeB(GetWorld(), contact);
        if (shapeA == m_platform) {
            SetTangentSpeed(GetWorld(), contact, 5_mps);
        }
        if (shapeB == m_platform) {
            SetTangentSpeed(GetWorld(), contact, -5_mps);
        }
    }

    ShapeID m_platform;
};

} // namespace testbed
