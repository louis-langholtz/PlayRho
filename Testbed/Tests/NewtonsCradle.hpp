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

#ifndef PLAYRHO_NEWTONS_CRADLE_HPP
#define PLAYRHO_NEWTONS_CRADLE_HPP

#include "../Framework/Test.hpp"

namespace playrho {
    
    /// Newton's Cradle test.
    /// @details
    /// Demonstrates the problems that are endemic to the handling multiple collisions.
    /// @sa http://www.myphysicslab.com/Collision-methods.html
    class NewtonsCradle : public Test
    {
    public:
        const Real scale = Real(1);
        const Length ball_radius = scale * Real(2) * Meter; // 2
        const Length frame_width_per_arm = ball_radius * Real{2};
        const Length frame_height = scale * Real(30) * Meter; // 30
        const Length arm_length = scale * Real(16) * Meter; // 16
        static const auto default_num_arms = 5;

        NewtonsCradle()
        {
            for (auto&& body: m_swings)
            {
                body = nullptr;
            }
            CreateCradle();
        }
        
        void CreateCradle()
        {
            if (m_frame)
            {
                return;
            }
            m_frame = [&]() {
                BodyDef bd;
                bd.type = BodyType::Static;
                bd.position = Length2D{0, frame_height};
                const auto body = m_world->CreateBody(bd);
                
                const auto frame_width = frame_width_per_arm * static_cast<Real>(m_num_arms);
                auto shape = PolygonShape((frame_width/Real{2}), (frame_width / Real{24}));
                shape.SetDensity(Real{20} * KilogramPerSquareMeter);
                body->CreateFixture(std::make_shared<PolygonShape>(shape));
                return body;
            }();
            
            for (auto i = decltype(m_num_arms){0}; i < m_num_arms; ++i)
            {
                const auto x = (((i + Real(0.5f)) - Real(m_num_arms) / Real(2)) * frame_width_per_arm);
                
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.bullet = m_bullet_mode;
                bd.position = Length2D{x, frame_height - (arm_length / Real{2})};
                
                m_swings[i] = m_world->CreateBody(bd);
                CreateArm(m_swings[i], arm_length);
                CreateBall(m_swings[i], Length2D{0, -arm_length / Real{2}}, ball_radius);
                
                m_world->CreateJoint(RevoluteJointDef(m_frame, m_swings[i], Length2D{x, frame_height}));
            }            
        }

        void DestroyCradle()
        {
            if (m_frame)
            {
                m_world->Destroy(m_frame);
                m_frame = nullptr;
            }
            for (auto&& body: m_swings)
            {
                if (body)
                {
                    m_world->Destroy(body);
                    body = nullptr;
                }
            }
            DestroyLeftSideWall();
            DestroyRightSideWall();
        }

        void CreateRightSideWall()
        {
            if (!m_right_side_wall) {
                const auto frame_width = static_cast<Real>(m_num_arms) * frame_width_per_arm;

                BodyDef def;
                def.type = BodyType::Static;
                def.position = Length2D{frame_width / Real{2} + frame_width / Real{24}, frame_height - (arm_length / Real{2})};
                const auto body = m_world->CreateBody(def);
                
                auto shape = PolygonShape((frame_width/Real{24}), (arm_length / Real{2} + frame_width / Real{24}));
                shape.SetDensity(Real{20} * KilogramPerSquareMeter);
                body->CreateFixture(std::make_shared<PolygonShape>(shape));
                
                m_right_side_wall = body;
            }
        }
        
        void CreateLeftSideWall()
        {
            if (!m_left_side_wall) {
                const auto frame_width = static_cast<Real>(m_num_arms) * frame_width_per_arm;

                BodyDef def;
                def.type = BodyType::Static;
                def.position = Length2D{
                    -(frame_width / Real{2} + frame_width / Real{24}),
                    frame_height - (arm_length / Real{2})
                };
                const auto body = m_world->CreateBody(def);
                
                auto shape = PolygonShape(frame_width/Real{24}, (arm_length / Real{2} + frame_width / Real{24}));
                shape.SetDensity(Real{20} * KilogramPerSquareMeter);
                body->CreateFixture(std::make_shared<PolygonShape>(shape));
                
                m_left_side_wall = body;
            }
        }

        void DestroyRightSideWall()
        {
            if (m_right_side_wall)
            {
                m_world->Destroy(m_right_side_wall);
                m_right_side_wall = nullptr;
            }
        }

        void DestroyLeftSideWall()
        {
            if (m_left_side_wall)
            {
                m_world->Destroy(m_left_side_wall);
                m_left_side_wall = nullptr;
            }
        }

        Fixture* CreateBall(Body* body, Length2D pos, Length radius)
        {
            auto conf = DiskShape::Conf{};
            conf.vertexRadius = radius;
            conf.location = pos;
            conf.density = Real{20} * KilogramPerSquareMeter;
            conf.restitution = 1;
            conf.friction = 0;
            return body->CreateFixture(std::make_shared<DiskShape>(conf));
        }

        Fixture* CreateArm(Body* body, Length length = Real(10) * Meter)
        {
            auto shape = PolygonShape(length / Real{2000}, length / Real{2});
            shape.SetDensity(Real{20} * KilogramPerSquareMeter);
            return body->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        void ToggleRightSideWall()
        {
            if (m_right_side_wall)
                DestroyRightSideWall();
            else
                CreateRightSideWall();
        }

        void ToggleLeftSideWall()
        {
            if (m_left_side_wall)
                DestroyLeftSideWall();
            else
                CreateLeftSideWall();
        }

        void ToggleBulletMode()
        {
            m_bullet_mode = !m_bullet_mode;
            for (auto&& body: m_world->GetBodies())
            {
                auto& b = GetRef(body);
                if (b.GetType() == BodyType::Dynamic)
                {
                    b.SetBullet(m_bullet_mode);
                }
            }
        }

        void KeyboardDown(Key key) override
        {
            switch (key)
            {
                case Key_Period:
                    ToggleBulletMode();
                    break;
                case Key_D:
                    ToggleRightSideWall();
                    break;
                case Key_A:
                    ToggleLeftSideWall();
                    break;
                case Key_1:
                    DestroyCradle();
                    m_num_arms = 1;
                    CreateCradle();
                    break;
                case Key_2:
                    DestroyCradle();
                    m_num_arms = 2;
                    CreateCradle();
                    break;
                case Key_3:
                    DestroyCradle();
                    m_num_arms = 3;
                    CreateCradle();
                    break;
                case Key_4:
                    DestroyCradle();
                    m_num_arms = 4;
                    CreateCradle();
                    break;
                case Key_5:
                    DestroyCradle();
                    m_num_arms = 5;
                    CreateCradle();
                    break;
                default:
                    break;
            }
        }
        
        void PostStep(const Settings&, Drawer& drawer) override
        {
            drawer.DrawString(5, m_textLine, Drawer::Left,
                              "Drag a circle with mouse, then let go to see how the physics is simulated");
            m_textLine += DRAW_STRING_NEW_LINE;
            drawer.DrawString(5, m_textLine, Drawer::Left,
                              "Press '.' to toggle bullet mode (currently %s).",
                              m_bullet_mode? "on": "off");
            m_textLine += DRAW_STRING_NEW_LINE;
            drawer.DrawString(5, m_textLine, Drawer::Left,
                              "Press 'A' to toggle left side wall (currently %s).",
                              m_left_side_wall? "on": "off");
            m_textLine += DRAW_STRING_NEW_LINE;
            drawer.DrawString(5, m_textLine, Drawer::Left,
                              "Press 'D' to toggle right side wall (currently %s).",
                              m_right_side_wall? "on": "off");
            m_textLine += DRAW_STRING_NEW_LINE;
            drawer.DrawString(5, m_textLine, Drawer::Left,
                              "Press '1-5' to set # of balls (currently %d).", m_num_arms);
            m_textLine += DRAW_STRING_NEW_LINE;
        }
    
        int m_num_arms = default_num_arms;
        bool m_bullet_mode = false;
        Body *m_frame = nullptr;
        Body *m_right_side_wall = nullptr;
        Body *m_left_side_wall = nullptr;
        Body *m_swings[5];
    };

} // namespace playrho
        
#endif /* PLAYRHO_NEWTONS_CRADLE_HPP */
