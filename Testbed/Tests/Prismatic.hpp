/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_PRISMATIC_HPP
#define PLAYRHO_PRISMATIC_HPP

#include "../Framework/Test.hpp"

namespace playrho {

// The motor in this test gets smoother with higher velocity iterations.
class Prismatic : public Test
{
public:
    Prismatic()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m));

        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-10.0f, 10.0f) * 1_m;
            bd.angle = 0.5_rad * Pi;
            bd.allowSleep = false;
            const auto body = m_world.CreateBody(bd);
            
            auto polygonConf = PolygonShape::Conf{};
            polygonConf.density = 5_kgpm2;
            body->CreateFixture(std::make_shared<PolygonShape>(2_m, 0.5_m, polygonConf));

            // Bouncy limit
            const auto axis = GetUnitVector(Vec2(2.0f, 1.0f));
            PrismaticJointDef pjd(ground, body, Length2{}, axis);

            // Non-bouncy limit
            //pjd.Initialize(ground, body, Vec2(-10.0f, 10.0f), Vec2(1.0f, 0.0f));

            pjd.motorSpeed = 10_rad / 1_s;
            pjd.maxMotorForce = 10000_N;
            pjd.enableMotor = true;
            pjd.lowerTranslation = 0_m;
            pjd.upperTranslation = 20_m;
            pjd.enableLimit = true;

            m_joint = (PrismaticJoint*)m_world.CreateJoint(pjd);
        }
        
        RegisterForKey(GLFW_KEY_L, GLFW_PRESS, 0, "Limits", [&](KeyActionMods) {
            m_joint->EnableLimit(!m_joint->IsLimitEnabled());
        });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Motors", [&](KeyActionMods) {
            m_joint->EnableMotor(!m_joint->IsMotorEnabled());
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Speed", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(-m_joint->GetMotorSpeed());
        });
    }

    void PostStep(const Settings& settings, Drawer&) override
    {
        const auto force = m_joint->GetMotorForce((1.0f / settings.dt) * 1_Hz);
        std::stringstream stream;
        stream << "Motor Force: ";
        stream << static_cast<double>(Real{force / 1_N});
        stream << " N.";
        m_status = stream.str();
    }

    PrismaticJoint* m_joint;
};

} // namespace playrho

#endif
