/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

class EdgeTest : public Test
{
public:
    static inline const auto registered = RegisterTest("Edge Test", MakeUniqueTest<EdgeTest>);

    EdgeTest()
    {
        {
            const auto ground = CreateBody(GetWorld());

            const auto v1 = Vec2(-10.0f, 0.0f) * 1_m;
            const auto v2 = Vec2(-7.0f, -2.0f) * 1_m;
            const auto v3 = Vec2(-4.0f, 0.0f) * 1_m;
            const auto v4 = Length2{};
            const auto v5 = Vec2(4.0f, 0.0f) * 1_m;
            const auto v6 = Vec2(7.0f, 2.0f) * 1_m;
            const auto v7 = Vec2(10.0f, 0.0f) * 1_m;

            auto conf = EdgeShapeConf{};
            conf.Set(v1, v2);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(v2, v3);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(v3, v4);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(v4, v5);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(v5, v6);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(v6, v7);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
        }

        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-0.5f, 0.6f) * 1_m;
            bd.allowSleep = false;
            const auto body = CreateBody(GetWorld(), bd);

            auto conf = DiskShapeConf{};
            conf.density = 1_kgpm2;
            conf.vertexRadius = 0.5_m;
            Attach(GetWorld(), body, CreateShape(GetWorld(), conf));
        }

        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(1.0f, 0.6f) * 1_m;
            bd.allowSleep = false;
            const auto body = CreateBody(GetWorld(), bd);

            auto shape = PolygonShapeConf{};
            shape.UseVertexRadius(1_m);
            shape.SetAsBox(0.5_m, 0.5_m);
            shape.UseDensity(1_kgpm2);
            Attach(GetWorld(), body, CreateShape(GetWorld(), shape));
        }

        SetAccelerations(GetWorld(), GetGravity());
    }
};

} // namespace testbed
