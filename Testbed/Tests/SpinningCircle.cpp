/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

class SpinningCircle : public Test
{
public:
    static inline const auto registered =
        RegisterTest("Spinning Circles", MakeUniqueTest<SpinningCircle>);
    enum { e_count = 10 };

    SpinningCircle()
    {
        SetGravity(LinearAcceleration2{});
        auto bodyConf = BodyConf{};
        bodyConf.type = BodyType::Dynamic;
        bodyConf.angularVelocity = 45_deg / 1_s;
        bodyConf.linearVelocity = LinearVelocity2{};
        bodyConf.linearDamping = 0.8_Hz;
        bodyConf.bullet = true;
        bodyConf.location = Vec2{0, 26} * 1_m;
        const auto body1 = CreateBody(GetWorld(), bodyConf);
        bodyConf.location = Vec2{0, 14} * 1_m;
        const auto body2 = CreateBody(GetWorld(), bodyConf);
        auto shapeConf = DiskShapeConf{};
        shapeConf.density = 10_kgpm2;
        shapeConf.vertexRadius = 2_m;
        shapeConf.location = Length2{};
        shapeConf.vertexRadius = 1.5_m;
        shapeConf.location = Vec2{0, 3} * 1_m;
        auto circleA = CreateShape(GetWorld(), shapeConf);
        shapeConf.vertexRadius = 1.5_m;
        shapeConf.location = Vec2{0, -3} * 1_m;
        auto circleB = CreateShape(GetWorld(), shapeConf);
        Attach(GetWorld(), body1, circleA);
        Attach(GetWorld(), body1, circleB);
        Attach(GetWorld(), body2, circleA);
        Attach(GetWorld(), body2, circleB);
    }
};

} // namespace testbed
