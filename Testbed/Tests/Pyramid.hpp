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

#ifndef PLAYRHO_PYRAMID_HPP
#define PLAYRHO_PYRAMID_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class Pyramid : public Test
{
public:
    enum
    {
        e_count = 20
    };

    Pyramid()
    {
        const auto ground = CreateBody(GetWorld());
        CreateFixture(GetWorld(), ground, Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});

        const auto a = 0.5_m;
        const auto shape = Shape{PolygonShapeConf{}.SetAsBox(a, a).UseDensity(5_kgpm2)};
        auto x = Vec2(-7.0f, 0.75f);
        const auto deltaX = Vec2(0.5625f, 1.25f);
        const auto deltaY = Vec2(1.125f, 0.0f);
        Vec2 y;
        for (auto i = 0; i < e_count; ++i)
        {
            y = x;
            for (auto j = i; j < e_count; ++j)
            {
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.linearAcceleration = GetGravity();
                bd.location = y * 1_m;
                const auto body = CreateBody(GetWorld(), bd);
                CreateFixture(GetWorld(), body, shape);
                y += deltaY;
            }
            x += deltaX;
        }
    }
};

} // namespace testbed

#endif
