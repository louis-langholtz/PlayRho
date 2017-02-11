/*
* Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef DYNAMIC_TREE_TEST_H
#define DYNAMIC_TREE_TEST_H

#include <Box2D/Collision/RayCastOutput.hpp>

namespace box2d {

class DynamicTreeTest : public Test
{
public:

	enum
	{
		e_actorCount = 128
	};

	DynamicTreeTest()
	{
		m_worldExtent = 15.0f;
		m_proxyExtent = 0.5f;

		srand(888);

		const auto aabbExtension = m_world->GetAabbExtension();
		const auto extension = Vec2{aabbExtension, aabbExtension};
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			Actor* actor = m_actors + i;
			actor->aabb = GetRandomAABB();
			actor->proxyId = m_tree.CreateProxy(actor->aabb + extension, actor);
		}

		m_stepCount = 0;

		const auto h = m_worldExtent;
		m_queryAABB = AABB{Vec2(-3.0f, -4.0f + h), Vec2(5.0f, 6.0f + h)};

		m_rayCastInput.p1 = Vec2(-5.0f, 5.0f + h);
		m_rayCastInput.p2 = Vec2(7.0f, -4.0f + h);
		//m_rayCastInput.p1 = Vec2(0.0f, 2.0f + h);
		//m_rayCastInput.p2 = Vec2(0.0f, -2.0f + h);
		m_rayCastInput.maxFraction = 1.0f;

		m_automated = false;
	}

	static Test* Create()
	{
		return new DynamicTreeTest;
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		BOX2D_NOT_USED(settings);

		m_rayActor = nullptr;
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			m_actors[i].fraction = 1.0f;
			m_actors[i].overlap = false;
		}

		if (m_automated)
		{
			const auto actionCount = Max(1, e_actorCount >> 2);
			for (auto i = decltype(actionCount){0}; i < actionCount; ++i)
			{
				Action();
			}
		}

		Query();
		RayCast();

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			const auto actor = m_actors + i;
			if (actor->proxyId == DynamicTree::NullNode)
				continue;

			Color c(0.9f, 0.9f, 0.9f);
			if (actor == m_rayActor && actor->overlap)
			{
				c = Color(0.9f, 0.6f, 0.6f);
			}
			else if (actor == m_rayActor)
			{
				c = Color(0.6f, 0.9f, 0.6f);
			}
			else if (actor->overlap)
			{
				c = Color(0.6f, 0.6f, 0.9f);
			}

			const auto p1 = actor->aabb.GetLowerBound();
			const auto p2 = Vec2(actor->aabb.GetUpperBound().x, actor->aabb.GetLowerBound().y);
			const auto p3 = actor->aabb.GetUpperBound();
			const auto p4 = Vec2(actor->aabb.GetLowerBound().x, actor->aabb.GetUpperBound().y);
			
			drawer.DrawSegment(p1, p2, c);
			drawer.DrawSegment(p2, p3, c);
			drawer.DrawSegment(p3, p4, c);
			drawer.DrawSegment(p4, p1, c);
		}

		Color c(0.7f, 0.7f, 0.7f);
		{
			// Draw the AABB.

			const auto p1 = m_queryAABB.GetLowerBound();
			const auto p2 = Vec2(m_queryAABB.GetUpperBound().x, m_queryAABB.GetLowerBound().y);
			const auto p3 = m_queryAABB.GetUpperBound();
			const auto p4 = Vec2(m_queryAABB.GetLowerBound().x, m_queryAABB.GetUpperBound().y);
			
			drawer.DrawSegment(p1, p2, c);
			drawer.DrawSegment(p2, p3, c);
			drawer.DrawSegment(p3, p4, c);
			drawer.DrawSegment(p4, p1, c);
		}

		drawer.DrawSegment(m_rayCastInput.p1, m_rayCastInput.p2, c);

		Color c1(0.2f, 0.9f, 0.2f);
		Color c2(0.9f, 0.2f, 0.2f);
		drawer.DrawPoint(m_rayCastInput.p1, 6.0f, c1);
		drawer.DrawPoint(m_rayCastInput.p2, 6.0f, c2);

		if (m_rayActor)
		{
			Color cr(0.2f, 0.2f, 0.9f);
			Vec2 p = m_rayCastInput.p1 + m_rayActor->fraction * (m_rayCastInput.p2 - m_rayCastInput.p1);
			drawer.DrawPoint(p, 6.0f, cr);
		}

		{
			DynamicTree::size_type height = m_tree.GetHeight();
			drawer.DrawString(5, m_textLine, "dynamic tree height = %d", height);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		++m_stepCount;
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_automated = !m_automated;
			break;

		case Key_C:
			CreateProxy();
			break;

		case Key_D:
			DestroyProxy();
			break;

		case Key_M:
			MoveProxy();
			break;

		default:
			break;
		}
	}

	bool QueryCallback(DynamicTree::size_type proxyId)
	{
		Actor* actor = (Actor*)m_tree.GetUserData(proxyId);
		actor->overlap = TestOverlap(m_queryAABB, actor->aabb);
		return true;
	}

	RealNum RayCastCallback(const RayCastInput& input, DynamicTree::size_type proxyId)
	{
		auto actor = static_cast<Actor*>(m_tree.GetUserData(proxyId));

		const auto output = box2d::RayCast(actor->aabb, input);

		if (output.hit)
		{
			m_rayCastOutput = output;
			m_rayActor = actor;
			m_rayActor->fraction = output.fraction;
			return output.fraction;
		}

		return input.maxFraction;
	}

private:

	struct Actor
	{
		AABB aabb;
		RealNum fraction;
		bool overlap;
		DynamicTree::size_type proxyId;
	};

	AABB GetRandomAABB()
	{
		const Vec2 w(m_proxyExtent * 2, m_proxyExtent * 2);
		//aabb->lowerBound.x = -m_proxyExtent;
		//aabb->lowerBound.y = -m_proxyExtent + m_worldExtent;
		const auto lowerBound = Vec2(RandomFloat(-m_worldExtent, m_worldExtent), RandomFloat(0.0f, 2.0f * m_worldExtent));
		const auto upperBound = lowerBound + w;
		return AABB(lowerBound, upperBound);
	}

	void MoveAABB(AABB* aabb)
	{
		const auto d = Vec2{RandomFloat(-0.5f, 0.5f), RandomFloat(-0.5f, 0.5f)};
		//d.x = 2.0f;
		//d.y = 0.0f;
		aabb->Move(d);

		const auto c0 = aabb->GetCenter();
		const auto min = Vec2(-m_worldExtent, RealNum(0));
		const auto max = Vec2(m_worldExtent, 2.0f * m_worldExtent);
		const auto c = Vec2{Clamp(c0.x, min.x, max.x), Clamp(c0.y, min.y, max.y)};

		aabb->Move(c - c0);
	}

	void CreateProxy()
	{
		const auto aabbExtension = m_world->GetAabbExtension();
		const auto extension = Vec2{aabbExtension, aabbExtension};
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = m_actors + j;
			if (actor->proxyId == DynamicTree::NullNode)
			{
				actor->aabb = GetRandomAABB();
				actor->proxyId = m_tree.CreateProxy(actor->aabb + extension, actor);
				return;
			}
		}
	}

	void DestroyProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = m_actors + j;
			if (actor->proxyId != DynamicTree::NullNode)
			{
				m_tree.DestroyProxy(actor->proxyId);
				actor->proxyId = DynamicTree::NullNode;
				return;
			}
		}
	}

	void MoveProxy()
	{
		const auto aabbExtension = m_world->GetAabbExtension();
		const auto extension = Vec2{aabbExtension, aabbExtension};
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = m_actors + j;
			if (actor->proxyId == DynamicTree::NullNode)
			{
				continue;
			}

			const auto aabb0 = actor->aabb;
			MoveAABB(&actor->aabb);
			const auto displacement = actor->aabb.GetCenter() - aabb0.GetCenter();
			m_tree.MoveProxy(actor->proxyId, actor->aabb + extension, displacement);
			return;
		}
	}

	void Action()
	{
		int32 choice = rand() % 20;

		switch (choice)
		{
		case 0:
			CreateProxy();
			break;

		case 1:
			DestroyProxy();
			break;

		default:
			MoveProxy();
		}
	}

	void Query()
	{
		m_tree.Query(this, m_queryAABB);

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			if (m_actors[i].proxyId == DynamicTree::NullNode)
			{
				continue;
			}

			const auto overlap = TestOverlap(m_queryAABB, m_actors[i].aabb);
			BOX2D_NOT_USED(overlap);
			assert(overlap == m_actors[i].overlap);
		}
	}

	void RayCast()
	{
		m_rayActor = nullptr;

		RayCastInput input = m_rayCastInput;

		// Ray cast against the dynamic tree.
		m_tree.RayCast(this, input);

		// Brute force ray cast.
		Actor* bruteActor = nullptr;
		RayCastOutput bruteOutput;
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			if (m_actors[i].proxyId == DynamicTree::NullNode)
			{
				continue;
			}

			const auto output = box2d::RayCast(m_actors[i].aabb, input);
			if (output.hit)
			{
				bruteActor = m_actors + i;
				bruteOutput = output;
				input.maxFraction = output.fraction;
			}
		}

		if (bruteActor)
		{
			assert(bruteOutput.fraction == m_rayCastOutput.fraction);
		}
	}

	RealNum m_worldExtent;
	RealNum m_proxyExtent;

	DynamicTree m_tree;
	AABB m_queryAABB;
	RayCastInput m_rayCastInput;
	RayCastOutput m_rayCastOutput;
	Actor* m_rayActor;
	Actor m_actors[e_actorCount];
	int32 m_stepCount;
	bool m_automated;
};

} // namespace box2d

#endif
