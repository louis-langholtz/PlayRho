/*
 * Original work Copyright (c) 2008-2014 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_HEAVY_ON_LIGHT_HPP
#define PLAYRHO_HEAVY_ON_LIGHT_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class HeavyOnLight : public Test
{
public:
    HeavyOnLight()
    {
        const auto bd = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(GetGravity());
        const auto upperBodyConf = BodyConf(bd).UseLocation(Vec2(0.0f, 6.0f) * 1_m);
        const auto lowerBodyConf = BodyConf(bd).UseLocation(Vec2(0.0f, 0.5f) * 1_m);

        const auto groundConf =
            EdgeShapeConf{}.Set(Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m);

        const auto diskConf = DiskShapeConf{}.UseDensity(10_kgpm2);
        const auto smallerDiskConf = DiskShapeConf(diskConf).UseRadius(0.5_m);
        const auto biggerDiskConf = DiskShapeConf(diskConf).UseRadius(5_m);

        Attach(GetWorld(), CreateBody(GetWorld()), Shape(groundConf));

        const auto lowerBody = CreateBody(GetWorld(), lowerBodyConf);
        const auto upperBody = CreateBody(GetWorld(), upperBodyConf);

        Attach(GetWorld(), lowerBody, Shape(smallerDiskConf));
        m_top = CreateShape(GetWorld(), Shape(biggerDiskConf));
        Attach(GetWorld(), upperBody, m_top);

        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "increase density of top shape",
                       [&](KeyActionMods) { ChangeDensity(+1_kgpm2); });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "decrease density of top shape",
                       [&](KeyActionMods) { ChangeDensity(-1_kgpm2); });
    }

    void ChangeDensity(AreaDensity change)
    {
        const auto oldDensity = GetDensity(GetWorld(), m_top);
        const auto newDensity = std::max(oldDensity + change, 1_kgpm2);
        if (newDensity != oldDensity) {
            auto conf = DiskShapeConf{};
            conf.vertexRadius = 5_m;
            conf.density = newDensity;
            SetShape(GetWorld(), m_top, Shape(conf));
        }
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << "Area density of top shape: ";
        stream << double(Real{GetDensity(GetWorld(), m_top) / 1_kgpm2});
        stream << " kg/m^2.";
        SetStatus(stream.str());
    }

    ShapeID m_top = InvalidShapeID;
};

} // namespace testbed

#endif
