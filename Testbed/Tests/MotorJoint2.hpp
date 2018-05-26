/*
 * Work contributed to Box2D on March 11, 2018 by Mikael Lind https://github.com/elemel
 * See: https://github.com/elemel/Box2D/commit/01bae74c52b9e089b04dda8899d8aba9d901ef22
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

#ifndef PLAYRHO_TESTS_MOTOR_JOINT2_HPP
#define PLAYRHO_TESTS_MOTOR_JOINT2_HPP

#include "../Framework/Test.hpp"

#include <utility>

namespace testbed {

/// @brief Motor joint two.
/// @note Elemel (Mikael Lind) reported seeing that:
///   "A system formed by two dynamic bodies connected by a motor joint doesn't look
///    realistic when e.g. falling to the ground."
/// @see https://github.com/erincatto/Box2D/issues/487
class MotorJoint2 : public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.seeAlso = "https://github.com/elemel/Box2D/commit/01bae74c52b9e089b04dda8899d8aba9d901ef22";
        conf.credits = "Written by Mikael Lind for Box2D. Ported to PlayRho by Louis Langholtz.";
        conf.description = "This test had demonstrated a problem in the MotorJoint code:"
            " after the first disk contacted the edge, the system went to sleep with the other"
            " end still up in the air. This should not happen now as this has been fixed.";
        return conf;
    }
    
    void ToggleJoint()
    {
        m_reversedJoint = !m_reversedJoint;
    }
    
    void ToggleBody()
    {
        m_reversedBody = !m_reversedBody;
    }

    void Setup()
    {
        if (m_joint) m_world.Destroy(m_joint);
        if (m_bodyA) m_world.Destroy(m_bodyA);
        if (m_bodyB) m_world.Destroy(m_bodyB);

        const auto bd = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(m_gravity);
        const auto locations = m_reversedBody?
            std::make_pair(m_locationB, m_locationA): std::make_pair(m_locationA, m_locationB);
        m_bodyA = m_world.CreateBody(BodyConf(bd).UseLocation(std::get<0>(locations)));
        m_bodyB = m_world.CreateBody(BodyConf(bd).UseLocation(std::get<1>(locations)));
        m_bodyA->CreateFixture(m_diskShape);
        m_bodyB->CreateFixture(m_diskShape);
        
        const auto jc = MotorJointConf{
            m_reversedJoint? MotorJointConf{m_bodyB, m_bodyA}: MotorJointConf{m_bodyA, m_bodyB}
        }.UseMaxForce(1000_N).UseMaxTorque(1000_Nm);
        m_joint = m_world.CreateJoint(jc);
    }
    
    MotorJoint2(): Test(GetTestConf())
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(Shape{GetGroundEdgeConf()});
        
        Setup();
        
        RegisterForKey(GLFW_KEY_B, GLFW_PRESS, 0, "Toggle bodies.", [&](KeyActionMods) {
            ToggleBody();
            Setup();
        });
        RegisterForKey(GLFW_KEY_J, GLFW_PRESS, 0, "Toggle joint.", [&](KeyActionMods) {
            ToggleJoint();
            Setup();
        });
    }
    
    static Test* Create()
    {
        return new MotorJoint2;
    }
    
    const Shape m_diskShape{DiskShapeConf{1_m}.UseFriction(0.6f).UseDensity(2_kgpm2)};
    const Length2 m_locationA{0_m, 4_m};
    const Length2 m_locationB{4_m, 8_m};
    bool m_reversedBody{false};
    bool m_reversedJoint{false};
    Body* m_bodyA{nullptr};
    Body* m_bodyB{nullptr};
    Joint* m_joint{nullptr};
};

} // namespace testbed

#endif // PLAYRHO_TESTS_MOTOR_JOINT2_HPP
