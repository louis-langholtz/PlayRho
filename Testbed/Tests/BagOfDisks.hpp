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

#ifndef BagOfDisks_hpp
#define BagOfDisks_hpp

#include "../Framework/Test.hpp"

namespace playrho {
    
    /// @brief Bag of disks test.
    class BagOfDisks: public Test
    {
    public:
        static constexpr auto Count = 180;

        BagOfDisks()
        {
            m_ground = m_world->CreateBody(BodyDef{}.UseType(BodyType::Kinematic));
            
            auto boundaryConf = ChainShape::Conf{}.UseFriction(Real(100));
            boundaryConf.UseVertexRadius(Real(0.04) * Meter);
            boundaryConf.vertices.push_back(Vec2(-12.0f, +20.0f) * Meter);
            boundaryConf.vertices.push_back(Vec2(-12.0f,  +0.0f) * Meter);
            boundaryConf.vertices.push_back(Vec2(+12.0f,  +0.0f) * Meter);
            boundaryConf.vertices.push_back(Vec2(+12.0f, +20.0f) * Meter);
            m_ground->CreateFixture(std::make_shared<ChainShape>(boundaryConf));
            
            const auto vertices = GetCircleVertices(Real(10.0f) * Meter, 90);
            const auto halfSegmentLength = GetLength(vertices[1] - vertices[0]) / Real(2);

            auto conf = EdgeShape::Conf{};
            conf.vertexRadius = Real(0.125) * Meter;
            conf.density = Real{10} * KilogramPerSquareMeter;
            conf.friction = 0.2f;
            conf.vertex1 = Length2D{-halfSegmentLength, Real(0) * Meter};
            conf.vertex2 = Length2D{+halfSegmentLength, Real(0) * Meter};
            const auto vertexOffset = Vec2(0, 14) * Meter;
            const auto shape = std::make_shared<EdgeShape>(conf);
            auto prevBody = static_cast<Body*>(nullptr);
            auto firstBody = static_cast<Body*>(nullptr);
            auto prevVertex = Optional<Length2D>{};
            for (const auto& vertex: vertices)
            {
                if (prevVertex.has_value())
                {
                    const auto midPoint = (vertex + *prevVertex) / Real(2);
                    const auto angle = GetAngle(vertex - *prevVertex);
                    const auto body = m_world->CreateBody(BodyDef{}
                                                          .UseType(BodyType::Dynamic)
                                                          .UseBullet(true)
                                                          .UseLocation(midPoint + vertexOffset)
                                                          .UseAngle(angle));
                    body->CreateFixture(shape);
                    if (prevBody)
                    {
	                    m_world->CreateJoint(RevoluteJointDef{body, prevBody, *prevVertex + vertexOffset});
                    }
                    else
                    {
                        firstBody = body;
                    }
                    prevBody = body;
                }
                prevVertex = vertex;
            }
            m_world->CreateJoint(RevoluteJointDef{prevBody, firstBody, vertices[0] + vertexOffset});

            const auto diskRadius = Real(0.15) * Meter;
            const auto diskShape = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                               .UseVertexRadius(diskRadius)
                                                               .UseDensity(Real(10) * KilogramPerSquareMeter)
                                                               .UseFriction(Real(0)));
            
            auto angleIncrement = Real(90) * Degree;
            auto angle = Angle(0);
            const auto alpha = diskRadius;
            const auto beta = Real(0.000125) * Meter / Degree;
            for (auto i = 0; i < 2000; ++i)
            {
                const auto radius = alpha + beta * angle;
                const auto unitVector = UnitVec2::Get(angle);
                const auto location = radius * unitVector;
                const auto body = m_world->CreateBody(BodyDef{}
                                                      .UseType(BodyType::Dynamic)
                                                      .UseLocation(location + vertexOffset));
                body->CreateFixture(diskShape);
                angle += angleIncrement;
                angleIncrement *= Real(0.999);
            }
        }
        
        void KeyboardDown(Key key) override
        {
            const auto angularVelocity = GetAngularVelocity(*m_ground);
            switch (key)
            {
                case Key_A:
                {
                    SetAngularVelocity(*m_ground, angularVelocity + Real(0.1) * RadianPerSecond);
                    break;
                }
                case Key_D:
                {
                    SetAngularVelocity(*m_ground, angularVelocity - Real(0.1) * RadianPerSecond);
                    break;
                }
                default:
                    break;
            }
        }
        
        void PostStep(const Settings&, Drawer& drawer) override
        {
            drawer.DrawString(5, m_textLine,
                              "Press 'A' or 'D' to increase angular velocity counter-clockwise or clockwise respectively.");
            m_textLine += DRAW_STRING_NEW_LINE;
        }

    private:
        Body* m_ground;
    };

}

#endif /* BagOfDisks_hpp */
