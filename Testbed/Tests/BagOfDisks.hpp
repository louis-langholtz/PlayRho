/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_BAG_OF_DISKS_HPP
#define PLAYRHO_BAG_OF_DISKS_HPP

#include "../Framework/Test.hpp"

namespace testbed {

/// @brief Bag of disks test.
class BagOfDisks: public Test
{
public:
    static constexpr auto Count = 180;

    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Simulates bag of a liquid.";
        return conf;
    }
    
    BagOfDisks(): Test(GetTestConf())
    {
        m_ground = m_world.CreateBody(BodyConf{}.UseType(BodyType::Kinematic));
        
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Increase counter-clockwise angular velocity",
                       [&](KeyActionMods) {
            const auto angularVelocity = GetAngularVelocity(m_world, m_ground);
            SetVelocity(m_world, m_ground, angularVelocity + 0.1_rad / Second);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Increase clockwise angular velocity",
                       [&](KeyActionMods) {
            const auto angularVelocity = GetAngularVelocity(m_world, m_ground);
            SetVelocity(m_world, m_ground, angularVelocity - 0.1_rad / Second);
        });

        auto boundaryConf = ChainShapeConf{}.UseFriction(100);
        boundaryConf.UseVertexRadius(0.04_m);
        boundaryConf.Add(Vec2(-12, +20) * 1_m);
        boundaryConf.Add(Vec2(-12,  +0) * 1_m);
        boundaryConf.Add(Vec2(+12,  +0) * 1_m);
        boundaryConf.Add(Vec2(+12, +20) * 1_m);
        m_world.CreateFixture(m_ground, Shape(boundaryConf));
        
        const auto vertices = GetCircleVertices(10_m, 90);
        const auto halfSegmentLength = GetMagnitude(vertices[1] - vertices[0]) / 2;

        auto conf = EdgeShapeConf{};
        conf.vertexRadius = 0.125_m;
        conf.density = 10_kgpm2;
        conf.friction = 0.2f;
        conf.Set(Length2{-halfSegmentLength, 0_m}, Length2{+halfSegmentLength, 0_m});
        const auto vertexOffset = Vec2(0, 14) * 1_m;
        const auto shape = Shape(conf);
        auto prevBody = InvalidBodyID;
        auto firstBody = InvalidBodyID;
        auto prevVertex = Optional<Length2>{};
        for (const auto& vertex: vertices)
        {
            if (prevVertex.has_value())
            {
                const auto midPoint = (vertex + *prevVertex) / 2;
                const auto angle = GetAngle(vertex - *prevVertex);
                const auto body = m_world.CreateBody(BodyConf{}
                                                     .UseType(BodyType::Dynamic)
                                                     .UseBullet(true)
                                                     .UseLocation(midPoint + vertexOffset)
                                                     .UseAngle(angle)
                                                     .UseLinearAcceleration(m_gravity));
                m_world.CreateFixture(body, shape);
                if (prevBody != InvalidBodyID)
                {
                    m_world.CreateJoint(GetRevoluteJointConf(m_world, body,
                                                             prevBody, *prevVertex + vertexOffset));
                }
                else
                {
                    firstBody = body;
                }
                prevBody = body;
            }
            prevVertex = vertex;
        }
        m_world.CreateJoint(GetRevoluteJointConf(m_world, prevBody, firstBody,
                                                 vertices[0] + vertexOffset));

        const auto diskRadius = 0.15_m;
        const auto diskShape = Shape(DiskShapeConf{}.UseRadius(diskRadius).UseDensity(10_kgpm2).UseFriction(0));
        auto angleIncrement = 90_deg;
        auto angle = 0_deg;
        const auto alpha = diskRadius;
        const auto beta = 0.000125_m / Degree;
        for (auto i = 0; i < 2000; ++i)
        {
            const auto radius = alpha + beta * angle;
            const auto unitVector = UnitVec::Get(angle);
            const auto location = radius * unitVector;
            const auto body = m_world.CreateBody(BodyConf{}
                                                 .UseType(BodyType::Dynamic)
                                                 .UseLocation(location + vertexOffset)
                                                 .UseLinearAcceleration(m_gravity));
            m_world.CreateFixture(body, diskShape);
            angle += angleIncrement;
            angleIncrement *= 0.999f;
        }
    }

private:
    BodyID m_ground;
};

}

#endif /* PLAYRHO_BAG_OF_DISKS_HPP */
