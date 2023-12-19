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

class VaryingFriction : public Test
{
public:
    static inline const auto registered =
        RegisterTest("Varying Friction", MakeUniqueTest<VaryingFriction>);

    VaryingFriction()
    {
        Attach(GetWorld(), CreateBody(GetWorld()), CreateShape(GetWorld(), GetGroundEdgeConf()));
        const auto sliderPlank = CreateShape(GetWorld(), PolygonShapeConf{}.SetAsBox(13_m, 0.25_m));
        const auto sliderWall = CreateShape(GetWorld(), PolygonShapeConf{}.SetAsBox(0.25_m, 1_m));
        Attach(
            GetWorld(),
            CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(-4, 22) * 1_m).UseAngle(-0.25_rad)),
            sliderPlank);
        Attach(GetWorld(), CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(10.5f, 19) * 1_m)),
               sliderWall);
        Attach(GetWorld(),
               CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(4, 14) * 1_m).UseAngle(0.25_rad)),
               sliderPlank);
        Attach(GetWorld(), CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(-10.5f, 11) * 1_m)),
               sliderWall);
        Attach(
            GetWorld(),
            CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(-4, 6) * 1_m).UseAngle(-0.25_rad)),
            sliderPlank);

        auto shape = PolygonShapeConf{}.SetAsBox(0.5_m, 0.5_m).UseDensity(25_kgpm2);
        float friction[5] = {std::sqrt(std::numeric_limits<float>::max()), 0.5f, 0.35f, 0.1f, 0.0f};
        for (auto i = 0; i < 5; ++i) {
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.UseLocation(Vec2(-15.0f + 4.0f * i, 28.0f) * 1_m);
            const auto body = CreateBody(GetWorld(), bd);
            shape.UseFriction(friction[i]);
            Attach(GetWorld(), body, CreateShape(GetWorld(), shape));
        }
    }
};

} // namespace testbed
