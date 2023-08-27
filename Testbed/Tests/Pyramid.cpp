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

class Pyramid : public Test
{
public:
    static inline const auto registered = RegisterTest("Pyramid", MakeUniqueTest<Pyramid>);
    enum { e_count = 20 };

    Pyramid()
    {
        using namespace playrho::d2::part;
        auto ground = Body{};
        ground.Attach(CreateShape(
            GetWorld(), EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}));
        CreateBody(GetWorld(), ground);
        const auto shape = CreateShape(
            GetWorld(),
            Compositor<GeometryIs<StaticRectangle<1, 1>>, DensityIs<StaticAreaDensity<5>>>());
        auto x = Vec2(-7.0f, 0.75f);
        const auto deltaX = Vec2(0.5625f, 1.25f);
        const auto deltaY = Vec2(1.125f, 0.0f);
        Vec2 y;
        for (auto i = 0; i < e_count; ++i) {
            y = x;
            for (auto j = i; j < e_count; ++j) {
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.linearAcceleration = GetGravity();
                bd.location = y * 1_m;
                bd.Use(shape);
                CreateBody(GetWorld(), Body{bd});
                y += deltaY;
            }
            x += deltaX;
        }
    }
};

} // namespace testbed
