/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

class Bridge : public Test
{
public:
    static inline const auto registered = RegisterTest("Bridge", MakeUniqueTest<Bridge>);
    static constexpr auto Count = 30;

    Bridge()
    {
        const auto ground = CreateBody(GetWorld());
        Attach(GetWorld(), ground, CreateShape(GetWorld(), GetGroundEdgeConf()));
        {
            auto conf = PolygonShapeConf{};
            conf.density = 20_kgpm2;
            conf.friction = 0.2f;
            conf.SetAsBox(0.5_m, 0.125_m);
            const auto shape = CreateShape(GetWorld(), conf);
            auto prevBody = ground;
            for (auto i = 0; i < Count; ++i) {
                const auto body =
                    CreateBody(GetWorld(), BodyConf{}
                                               .UseType(BodyType::Dynamic)
                                               .UseLinearAcceleration(GetGravity())
                                               .UseLocation(Vec2(-14.5f + i, 5.0f) * 1_m));
                Attach(GetWorld(), body, shape);
                CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), prevBody, body,
                                                             Vec2(-15.0f + i, 5.0f) * 1_m));
                if (i == (Count >> 1)) {
                    m_middle = body;
                }
                prevBody = body;
            }

            CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), prevBody, ground,
                                                         Vec2(-15.0f + Count, 5.0f) * 1_m));
        }

        const auto conf = PolygonShapeConf{}.UseDensity(1_kgpm2).UseVertices(
            {Vec2(-0.5f, 0.0f) * 1_m, Vec2(0.5f, 0.0f) * 1_m, Vec2(0.0f, 1.5f) * 1_m});
        const auto polyshape = CreateShape(GetWorld(), conf);
        for (auto i = 0; i < 2; ++i) {
            const auto body =
                CreateBody(GetWorld(), BodyConf{}
                                           .UseType(BodyType::Dynamic)
                                           .UseLinearAcceleration(GetGravity())
                                           .UseLocation(Vec2(-8.0f + 8.0f * i, 12.0f) * 1_m));
            Attach(GetWorld(), body, polyshape);
        }

        const auto diskShape =
            CreateShape(GetWorld(), DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(0.5_m));
        for (auto i = 0; i < 3; ++i) {
            const auto body =
                CreateBody(GetWorld(), BodyConf{}
                                           .UseType(BodyType::Dynamic)
                                           .UseLinearAcceleration(GetGravity())
                                           .UseLocation(Vec2(-6.0f + 6.0f * i, 10.0f) * 1_m));
            Attach(GetWorld(), body, diskShape);
        }
    }

    BodyID m_middle;
};

} // namespace testbed
