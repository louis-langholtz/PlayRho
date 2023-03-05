/*
 * Original work Copyright (c) 2008-2014 Erin Catto http://www.box2d.org
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

class HeavyOnLightTwo : public Test
{
public:
    static inline const auto registered =
        RegisterTest("Heavy On Light Two", MakeUniqueTest<HeavyOnLightTwo>);

    HeavyOnLightTwo()
    {
        const DiskShapeConf DiskConf = DiskShapeConf{}.UseDensity(10_kgpm2);
        lilDisk = CreateShape(GetWorld(), DiskShapeConf(DiskConf).UseRadius(0.5_m));
        bigDisk = CreateShape(GetWorld(), DiskShapeConf(DiskConf).UseRadius(5.0_m));

        Attach(GetWorld(), CreateBody(GetWorld()), CreateShape(GetWorld(), GetGroundEdgeConf()));
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        Attach(GetWorld(), CreateBody(GetWorld(), BodyConf(DynBD).UseLocation(Length2{0_m, 2.5_m})),
               lilDisk);
        Attach(GetWorld(), CreateBody(GetWorld(), BodyConf(DynBD).UseLocation(Length2{0_m, 3.5_m})),
               lilDisk);
        RegisterForKey(GLFW_KEY_H, GLFW_PRESS, 0, "Toggle Heavy",
                       [&](KeyActionMods) { ToggleHeavy(); });
    }

    void ToggleHeavy()
    {
        if (m_heavy != InvalidBodyID) {
            Destroy(GetWorld(), m_heavy);
            m_heavy = InvalidBodyID;
        }
        else {
            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            m_heavy = CreateBody(GetWorld(), BodyConf(DynBD).UseLocation(Length2{0_m, 9_m}));
            Attach(GetWorld(), m_heavy, bigDisk);
        }
    }

    const BodyConf DynBD =
        BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(GetGravity());
    ShapeID lilDisk = InvalidShapeID;
    ShapeID bigDisk = InvalidShapeID;
    BodyID m_heavy = InvalidBodyID;
};

} // namespace testbed
