/*
 * Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#include <sstream> // for std::ostringstream
#include <vector>

namespace testbed {

class Tumbler : public Test
{
public:
    static inline const auto registered = RegisterTest("Tumbler", MakeUniqueTest<Tumbler>);
    static constexpr auto Count = 1600;
    static constexpr auto MotorInc = 0.5_rpm;

    enum ShapeType { Square, Disk };

    Tumbler()
    {
        SetupTumblers(1);
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Speed up rotation.", [&](KeyActionMods) {
            for (const auto& id : GetJoints(GetWorld())) {
                if (GetType(GetWorld(), id) == GetTypeID<RevoluteJointConf>()) {
                    SetMotorSpeed(GetWorld(), id, GetMotorSpeed(GetWorld(), id) + MotorInc);
                }
            }
        });
        RegisterForKey(
            GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Slow down rotation.", [&](KeyActionMods) {
                for (const auto& id : GetJoints(GetWorld())) {
                    if (GetType(GetWorld(), id) == GetTypeID<RevoluteJointConf>()) {
                        SetMotorSpeed(GetWorld(), id, GetMotorSpeed(GetWorld(), id) - MotorInc);
                    }
                }
            });
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Stop rotation.", [&](KeyActionMods) {
            for (const auto& id : GetJoints(GetWorld())) {
                if (GetType(GetWorld(), id) == GetTypeID<RevoluteJointConf>()) {
                    SetMotorSpeed(GetWorld(), id, 0_rpm);
                }
            }
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "for newly emitted shapes to be disks.",
                       [&](KeyActionMods) { m_shape = Disk; });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "for newly emitted shapes to be squares.",
                       [&](KeyActionMods) { m_shape = Square; });
        RegisterForKey(GLFW_KEY_1, GLFW_PRESS, 0, "Restart with 1 tumbler.",
                       [&](KeyActionMods) { SetupTumblers(1); });
        RegisterForKey(GLFW_KEY_2, GLFW_PRESS, 0, "Restart with 2 tumblers.",
                       [&](KeyActionMods) { SetupTumblers(2); });
        RegisterForKey(GLFW_KEY_C, GLFW_PRESS, 0, "Clear and re-emit shapes.", [&](KeyActionMods) {
            std::vector<BodyID> bodies;
            for (auto&& b : GetBodies(GetWorld())) {
                const auto shapes = GetShapes(GetWorld(), b);
                if (size(shapes) == 1u) {
                    if (const auto shapeId = shapes.front();
                        shapeId == m_disk || shapeId == m_square) {
                        bodies.push_back(b);
                    }
                }
            }
            for (auto&& b : Reverse(bodies)) {
                Destroy(GetWorld(), b);
            }
            m_count = 0;
        });
    }

    void SetupTumblers(unsigned int num)
    {
        Clear(GetWorld());
        m_count = 0;
        m_square = CreateShape(GetWorld(),
                               PolygonShapeConf{}.SetAsBox(0.125_m, 0.125_m).UseDensity(1_kgpm2));
        m_disk = CreateShape(
            GetWorld(), DiskShapeConf{}.UseRadius(0.125_m).UseFriction(0).UseDensity(0.1_kgpm2));
        const auto width = 30_m;
        const auto halfWidth = width / 2;
        const auto totalWidth = num * width;
        auto ctrX = halfWidth - (totalWidth / 2);
        for (auto i = decltype(num){0}; i < num; ++i) {
            CreateRevoluteJoint(CreateEnclosure(Length2{ctrX, 20_m}));
            ctrX += width;
        }
    }

    BodyID CreateEnclosure(Length2 at)
    {
        const auto b = CreateBody(GetWorld(), BodyConf{}
                                                  .UseType(BodyType::Dynamic)
                                                  .UseLocation(at)
                                                  .UseAllowSleep(false)
                                                  .UseLinearAcceleration(GetGravity()));
        auto shape = PolygonShapeConf{}.UseDensity(5_kgpm2);
        shape.SetAsBox(0.5_m, 10_m, Vec2(10, 0) * 1_m, 0_rad);
        Attach(GetWorld(), b, CreateShape(GetWorld(), shape));
        shape.SetAsBox(0.5_m, 10_m, Vec2(-10, 0) * 1_m, 0_rad);
        Attach(GetWorld(), b, CreateShape(GetWorld(), shape));
        shape.SetAsBox(10_m, 0.5_m, Vec2(0, 10) * 1_m, 0_rad);
        Attach(GetWorld(), b, CreateShape(GetWorld(), shape));
        shape.SetAsBox(10_m, 0.5_m, Vec2(0, -10) * 1_m, 0_rad);
        Attach(GetWorld(), b, CreateShape(GetWorld(), shape));
        return b;
    }

    JointID CreateRevoluteJoint(BodyID turn)
    {
        RevoluteJointConf jd;
        jd.bodyA = CreateBody(GetWorld(), BodyConf{}.UseLocation(GetLocation(GetWorld(), turn)));
        jd.bodyB = turn;
        jd.referenceAngle = 0_rad;
        jd.motorSpeed = 1.5_rpm; // same as Pi*0.05_rad/s = 0.025 rev/s
        jd.maxMotorTorque = 100000_Nm;
        jd.enableMotor = true;
        return CreateJoint(GetWorld(), jd);
    }

    ShapeID GetTumbleeShapeID() const noexcept
    {
        switch (m_shape) {
        case Disk:
            return m_disk;
        case Square:
            break;
        }
        return m_square;
    }

    const char* GetEmittingString() const noexcept
    {
        switch (m_shape) {
        case Disk:
            return "disks";
        case Square:
            break;
        }
        return "squares";
    }

    void CreateTumblee(Length2 at)
    {
        CreateBody(GetWorld(), BodyConf{}
                                   .UseType(BodyType::Dynamic)
                                   .UseLocation(at)
                                   .Use(GetTumbleeShapeID())
                                   .UseLinearAcceleration(GetGravity()));
    }

    void PostStep(const Settings& settings, Drawer&) override
    {
        if ((!settings.pause || settings.singleStep) && (m_count < Count)) {
            for (const auto& id : GetJoints(GetWorld())) {
                if (GetType(GetWorld(), id) == GetTypeID<RevoluteJointConf>()) {
                    CreateTumblee(GetLocation(GetWorld(), GetBodyB(GetWorld(), id)));
                }
            }
            ++m_count;
            std::ostringstream os;
            os << "Newly emitted shapes will be " << GetEmittingString() << ".";
            os << " Total shapes tumbling is " << m_count << ".";
            SetStatus(os.str());
        }
    }

    ShapeID m_square = InvalidShapeID;
    ShapeID m_disk = InvalidShapeID;
    ShapeType m_shape = Square;
    int m_count = 0;
};

} // namespace testbed
