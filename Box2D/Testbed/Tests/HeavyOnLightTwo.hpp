/*
 * Original work Copyright (c) 2008-2014 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef HEAVY_ON_LIGHT_TWO_H
#define HEAVY_ON_LIGHT_TWO_H

namespace box2d {

class HeavyOnLightTwo : public Test
{
public:
    
	HeavyOnLightTwo()
	{
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);
            
			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, FixtureDef{});
		}
        
		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.position = Vec2(0.0f, 2.5f);
		Body* body = m_world->CreateBody(bd);
        
		CircleShape shape;
		shape.SetRadius(float_t(0.5));
		body->CreateFixture(&shape, FixtureDef{}.UseDensity(10));
        
        bd.position = Vec2(0.0f, 3.5f);
        body = m_world->CreateBody(bd);
		body->CreateFixture(&shape, FixtureDef{}.UseDensity(10));
        
        m_heavy = nullptr;
	}
    
    void ToggleHeavy()
    {
        if (m_heavy)
        {
            m_world->Destroy(m_heavy);
            m_heavy = nullptr;
        }
        else
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(0.0f, 9.0f);
            m_heavy = m_world->CreateBody(bd);
            
            CircleShape shape;
            shape.SetRadius(float_t(5));
			m_heavy->CreateFixture(&shape, FixtureDef{}.UseDensity(10));
        }
    }
    
	void Keyboard(Key key) override
	{
		switch (key)
		{
        case Key_H:
            ToggleHeavy();
            break;
		default:
			break;
		}
	}
    
	static Test* Create()
	{
		return new HeavyOnLightTwo;
	}
    
	Body* m_heavy;
};

} // namespace box2d

#endif
