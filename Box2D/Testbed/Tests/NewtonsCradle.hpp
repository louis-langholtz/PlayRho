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
	
	// It is difficult to make a cantilever made of links completely rigid with weld joints.
	// You will have to use a high number of iterations to make them stiff.
	// So why not go ahead and use soft weld joints? They behave like a revolute
	// joint with a rotational spring.
	class NewtonsCradle : public Test
	{
	public:
		NewtonsCradle()
		{
			const auto frame_width = float_t{20};
			const auto frame_height = float_t(30);
			const auto arm_length = float_t(16);

			const auto frame = [&]() {
				BodyDef bd;
				bd.type = BodyType::Static;
				bd.position = Vec2{0, frame_height};
				const auto body = m_world->CreateBody(bd);

				PolygonShape shape;
				shape.SetAsBox(frame_width/2, frame_width / 24);
				body->CreateFixture(FixtureDef{&shape, 20});
				return body;
			}();

			const auto radius = float_t(2);
			const auto num_arms = unsigned{5};
			const auto frame_width_per_arm = radius * 2;
			for (auto i = decltype(num_arms){0}; i < num_arms; ++i)
			{
				const auto x = (((i + float_t(0.5)) - float_t(num_arms) / float_t(2)) * frame_width_per_arm);

				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.bullet = m_bullet_mode;
				bd.position = Vec2{x, frame_height - (arm_length / 2)};
				
				const auto body = m_world->CreateBody(bd);
				CreateArm(body, arm_length);
				CreateBall(body, Vec2{0, -arm_length / 2}, radius);
				
				RevoluteJointDef rjd;
				rjd.Initialize(frame, body, Vec2{x, frame_height});
				m_world->CreateJoint(rjd);
			}
		}
		
		Fixture* CreateBall(Body* body, Vec2 pos, float_t radius)
		{
			CircleShape shape{radius, pos};
			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 2000;
			fd.restitution = 1;
			return body->CreateFixture(fd);
		}

		Fixture* CreateArm(Body* body, float_t length = float_t(10), float_t width = float_t(0.01))
		{
			PolygonShape shape;
			shape.SetAsBox(width / 2, length / 2);
			return body->CreateFixture(FixtureDef{&shape, 20});
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
			}
		}
		
		void Step(Settings* settings)
		{
			Test::Step(settings);
			g_debugDraw.DrawString(5, m_textLine, "Drag a circle with mouse, then let go to see how the physics is simulated");
			m_textLine += DRAW_STRING_NEW_LINE;
			g_debugDraw.DrawString(5, m_textLine, "Press '.' to toggle bullet mode (currently %s).", m_bullet_mode? "on": "off");
			m_textLine += DRAW_STRING_NEW_LINE;
		}
		
		static Test* Create()
		{
			return new NewtonsCradle;
		}
	
		bool m_bullet_mode = false;
	};

} // namespace box2d
		
#endif /* NewtonsCradle_hpp */
