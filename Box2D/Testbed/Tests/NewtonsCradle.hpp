//
//  NewtonsCradle.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/9/16.
//
//

#ifndef NewtonsCradle_hpp
#define NewtonsCradle_hpp

namespace box2d {
	
	class NewtonsCradle : public Test
	{
	public:
		static constexpr auto scale = float_t(.01);
		static constexpr auto ball_radius = scale * float_t(2); // 2
		static constexpr auto frame_width_per_arm = ball_radius * 2;
		static constexpr auto frame_height = scale * float_t(30); // 30
		static constexpr auto arm_length = scale * float_t(16); // 16
		static constexpr auto default_num_arms = unsigned{5};

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
				bd.position = Vec2{0, frame_height};
				const auto body = m_world->CreateBody(bd);
				
				const auto frame_width = m_num_arms * frame_width_per_arm;
				const auto shape = PolygonShape(frame_width/2, frame_width / 24);
				body->CreateFixture(FixtureDef{&shape, 20});
				return body;
			}();
			
			for (auto i = decltype(m_num_arms){0}; i < m_num_arms; ++i)
			{
				const auto x = (((i + float_t(0.5)) - float_t(m_num_arms) / float_t(2)) * frame_width_per_arm);
				
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.bullet = m_bullet_mode;
				bd.position = Vec2{x, frame_height - (arm_length / 2)};
				
				m_swings[i] = m_world->CreateBody(bd);
				CreateArm(m_swings[i], arm_length);
				CreateBall(m_swings[i], Vec2{0, -arm_length / 2}, ball_radius);
				
				m_world->CreateJoint(RevoluteJointDef(m_frame, m_swings[i], Vec2{x, frame_height}));
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
				const auto frame_width = m_num_arms * frame_width_per_arm;

				BodyDef def;
				def.type = BodyType::Static;
				def.position = Vec2{frame_width / 2 + frame_width/24, frame_height - (arm_length / 2)};
				const auto body = m_world->CreateBody(def);
				
				const auto shape = PolygonShape(frame_width/24, arm_length / 2 + frame_width / 24);
				body->CreateFixture(FixtureDef{&shape, 20});
				
				m_right_side_wall = body;
			}
		}
		
		void CreateLeftSideWall()
		{
			if (!m_left_side_wall) {
				const auto frame_width = m_num_arms * frame_width_per_arm;

				BodyDef def;
				def.type = BodyType::Static;
				def.position = Vec2{-(frame_width / 2 + frame_width/24), frame_height - (arm_length / 2)};
				const auto body = m_world->CreateBody(def);
				
				const auto shape = PolygonShape(frame_width/24, arm_length / 2 + frame_width / 24);
				body->CreateFixture(FixtureDef{&shape, 20});
				
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

		Fixture* CreateBall(Body* body, Vec2 pos, float_t radius)
		{
			CircleShape shape{radius, pos};
			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20;
			fd.restitution = 1;
			fd.friction = 0;
			return body->CreateFixture(fd);
		}

		Fixture* CreateArm(Body* body, float_t length = float_t(10))
		{
			const auto shape = PolygonShape(length / 2000, length / 2);
			return body->CreateFixture(FixtureDef{&shape, 20});
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
			for (auto& b: m_world->GetBodies())
			{
				if (b.GetType() == BodyType::Dynamic)
				{
					b.SetBullet(m_bullet_mode);
				}
			}
		}

		void Keyboard(int key)
		{
			switch (key)
			{
				case GLFW_KEY_PERIOD:
					ToggleBulletMode();
					break;
				case GLFW_KEY_D:
					ToggleRightSideWall();
					break;
				case GLFW_KEY_A:
					ToggleLeftSideWall();
					break;
				case GLFW_KEY_1:
					DestroyCradle();
					m_num_arms = 1;
					CreateCradle();
					break;
				case GLFW_KEY_2:
					DestroyCradle();
					m_num_arms = 2;
					CreateCradle();
					break;
				case GLFW_KEY_3:
					DestroyCradle();
					m_num_arms = 3;
					CreateCradle();
					break;
				case GLFW_KEY_4:
					DestroyCradle();
					m_num_arms = 4;
					CreateCradle();
					break;
				case GLFW_KEY_5:
					DestroyCradle();
					m_num_arms = 5;
					CreateCradle();
					break;
			}
		}
		
		void Step(Settings* settings)
		{
			Test::Step(settings);
			g_debugDraw.DrawString(5, m_textLine, "Drag a circle with mouse, then let go to see how the physics is simulated");
			m_textLine += DRAW_STRING_NEW_LINE;
			g_debugDraw.DrawString(5, m_textLine, "Press '.' to toggle bullet mode (currently %s).", m_bullet_mode? "on": "off");
			m_textLine += DRAW_STRING_NEW_LINE;
			g_debugDraw.DrawString(5, m_textLine, "Press 'A' to toggle left side wall (currently %s).", m_left_side_wall? "on": "off");
			m_textLine += DRAW_STRING_NEW_LINE;
			g_debugDraw.DrawString(5, m_textLine, "Press 'D' to toggle right side wall (currently %s).", m_right_side_wall? "on": "off");
			m_textLine += DRAW_STRING_NEW_LINE;
			g_debugDraw.DrawString(5, m_textLine, "Press '1-5' to set # of balls (currently %d).", m_num_arms);
			m_textLine += DRAW_STRING_NEW_LINE;
		}
		
		static Test* Create()
		{
			return new NewtonsCradle;
		}
	
		unsigned m_num_arms = default_num_arms;
		bool m_bullet_mode = false;
		Body *m_frame = nullptr;
		Body *m_right_side_wall = nullptr;
		Body *m_left_side_wall = nullptr;
		Body *m_swings[5];
	};

} // namespace box2d
		
#endif /* NewtonsCradle_hpp */
