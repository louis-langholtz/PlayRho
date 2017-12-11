/*
* Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_SHAPE_EDITING_HPP
#define PLAYRHO_SHAPE_EDITING_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class ShapeEditing : public Test
{
public:

    ShapeEditing()
    {
        m_world.CreateBody()->CreateFixture(Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});
        
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.location = Vec2(0.0f, 10.0f) * 1_m;
        m_body = m_world.CreateBody(bd);

        auto shape = PolygonShapeConf{};
        shape.SetAsBox(4_m, 4_m, Length2{}, 0_deg);
        shape.UseDensity(10_kgpm2);
        m_fixture1 = m_body->CreateFixture(Shape(shape));

        m_fixture2 = nullptr;

        m_sensor = false;
        
        RegisterForKey(GLFW_KEY_C, GLFW_PRESS, 0, "Create a shape.", [&](KeyActionMods) {
            if (!m_fixture2)
            {
                auto conf = DiskShapeConf{};
                conf.vertexRadius = 3_m;
                conf.location = Vec2(0.5f, -4.0f) * 1_m;
                conf.density = 10_kgpm2;
                m_fixture2 = m_body->CreateFixture(Shape(conf));
                m_body->SetAwake();
            }
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Destroy a shape.", [&](KeyActionMods) {
            if (m_fixture2)
            {
                m_body->DestroyFixture(m_fixture2);
                m_fixture2 = nullptr;
                m_body->SetAwake();
            }
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Toggle Sensor.", [&](KeyActionMods) {
            if (m_fixture2)
            {
                m_sensor = !m_sensor;
                m_fixture2->SetSensor(m_sensor);
            }
        });
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << "Sensor is " << (m_sensor? "on": "off") << ".";
        m_status = stream.str();
    }

    Body* m_body;
    Fixture* m_fixture1;
    Fixture* m_fixture2;
    bool m_sensor;
};

} // namespace playrho

#endif
