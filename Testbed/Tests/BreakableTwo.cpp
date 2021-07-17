/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

class BreakableTwo : public Test
{
public:
    static inline const auto registered =
        RegisterTest("Breakable Two", MakeUniqueTest<BreakableTwo>);

    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Demonstrates how bodies can be assembled into a breakable cluster.";
        return conf;
    }

    BreakableTwo() : Test(GetTestConf())
    {
        constexpr Length vr = 2 * DefaultLinearSlop;
        m_shape = CreateShape(GetWorld(),
                              PolygonShapeConf{}.UseVertexRadius(vr).UseDensity(100_kgpm2).SetAsBox(
                                  0.5_m - vr, 0.5_m - vr));
        SetPostSolveContactListener(GetWorld(),
                                    [this](ContactID id, const ContactImpulsesList& impulses,
                                           unsigned count) { PostSolve(id, impulses, count); });

        SetGravity(LinearAcceleration2{});

        BodyID bodies[20 * 20];
        const auto startLoc = Length2{-10_m, 10_m};
        const auto bd = BodyConf{}.UseType(BodyType::Dynamic);
        for (auto y = 0; y < 20; ++y) {
            for (auto x = 0; x < 20; ++x) {
                const auto location = startLoc + Length2{x * 1_m, y * 1_m};
                // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
                bodies[y * 20 + x] = CreateBody(GetWorld(), BodyConf(bd).UseLocation(location));
                Attach(GetWorld(), bodies[y * 20 + x], m_shape);
                if (x > 0) {
                    const auto jd =
                        GetWeldJointConf(GetWorld(), bodies[y * 20 + x - 1], bodies[y * 20 + x],
                                         location + Length2{-0.5_m, 0_m});
                    CreateJoint(GetWorld(), jd);
                }
                if (y > 0) {
                    const auto jd =
                        GetWeldJointConf(GetWorld(), bodies[(y - 1) * 20 + x],
                                         bodies[(y + 0) * 20 + x], location + Length2{0_m, -0.5_m});
                    CreateJoint(GetWorld(), jd);
                }
            }
        }
    }

    void PostSolve(ContactID contact, const ContactImpulsesList& impulses, unsigned)
    {
        if (m_body == InvalidBodyID) {
            // Should the body break?
            auto maxImpulse = GetMaxNormalImpulse(impulses);
            if (maxImpulse > 60_Ns) {
                const auto shapeIdA = GetShapeA(GetWorld(), contact);
                const auto shapeIdB = GetShapeB(GetWorld(), contact);
                if (shapeIdA == m_shape) {
                    m_body = GetBodyA(GetWorld(), contact);
                }
                else if (shapeIdB == m_shape) {
                    m_body = GetBodyB(GetWorld(), contact);
                }
            }
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        if (m_body != InvalidBodyID) {
            Destroy(GetWorld(), m_body);
            m_body = InvalidBodyID;
        }
    }

private:
    ShapeID m_shape = InvalidShapeID;
    BodyID m_body = InvalidBodyID;
};

} // namespace testbed
