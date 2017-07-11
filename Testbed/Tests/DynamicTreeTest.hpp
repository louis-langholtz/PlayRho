/*
* Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef DYNAMIC_TREE_TEST_H
#define DYNAMIC_TREE_TEST_H

#include "../Framework/Test.hpp"

namespace box2d {

class DynamicTreeTest : public Test
{
public:

    static constexpr auto e_actorCount = 128;

    DynamicTreeTest()
    {
        m_worldExtent = 15.0f;
        m_proxyExtent = 0.5f;

        srand(888);

        const auto aabbExtension = StepConf{}.aabbExtension;
        for (auto i = 0; i < e_actorCount; ++i)
        {
            Actor* actor = m_actors + i;
            actor->aabb = GetRandomAABB();
            actor->proxyId = m_tree.CreateProxy(GetFattenedAABB(actor->aabb, aabbExtension), actor);
        }

        m_stepCount = 0;

        const auto h = m_worldExtent;
        m_queryAABB = AABB{Vec2(-3.0f, -4.0f + h) * Meter, Vec2(5.0f, 6.0f + h) * Meter};

        m_rayCastInput.p1 = Vec2(-5.0f, 5.0f + h) * Meter;
        m_rayCastInput.p2 = Vec2(7.0f, -4.0f + h) * Meter;
        //m_rayCastInput.p1 = Vec2(0.0f, 2.0f + h);
        //m_rayCastInput.p2 = Vec2(0.0f, -2.0f + h);
        m_rayCastInput.maxFraction = 1.0f;

        m_automated = false;
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        NOT_USED(settings);

        m_rayActor = nullptr;
        for (auto i = 0; i < e_actorCount; ++i)
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

        for (auto i = 0; i < e_actorCount; ++i)
        {
            const auto actor = m_actors + i;
            if (actor->proxyId == DynamicTree::InvalidIndex)
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
            const auto p2 = Length2D(actor->aabb.GetUpperBound().x, actor->aabb.GetLowerBound().y);
            const auto p3 = actor->aabb.GetUpperBound();
            const auto p4 = Length2D(actor->aabb.GetLowerBound().x, actor->aabb.GetUpperBound().y);
            
            drawer.DrawSegment(p1, p2, c);
            drawer.DrawSegment(p2, p3, c);
            drawer.DrawSegment(p3, p4, c);
            drawer.DrawSegment(p4, p1, c);
        }

        Color c(0.7f, 0.7f, 0.7f);
        {
            // Draw the AABB.

            const auto p1 = m_queryAABB.GetLowerBound();
            const auto p2 = Length2D(m_queryAABB.GetUpperBound().x, m_queryAABB.GetLowerBound().y);
            const auto p3 = m_queryAABB.GetUpperBound();
            const auto p4 = Length2D(m_queryAABB.GetLowerBound().x, m_queryAABB.GetUpperBound().y);
            
            drawer.DrawSegment(p1, p2, c);
            drawer.DrawSegment(p2, p3, c);
            drawer.DrawSegment(p3, p4, c);
            drawer.DrawSegment(p4, p1, c);
        }

        drawer.DrawSegment(m_rayCastInput.p1, m_rayCastInput.p2, c);

        Color c1(0.2f, 0.9f, 0.2f);
        Color c2(0.9f, 0.2f, 0.2f);
        drawer.DrawPoint(m_rayCastInput.p1, Real{6} * Meter, c1);
        drawer.DrawPoint(m_rayCastInput.p2, Real{6} * Meter, c2);

        if (m_rayActor)
        {
            Color cr(0.2f, 0.2f, 0.9f);
            const auto p = m_rayCastInput.p1 + m_rayActor->fraction * (m_rayCastInput.p2 - m_rayCastInput.p1);
            drawer.DrawPoint(p, Real{6} * Meter, cr);
        }

        {
            DynamicTree::size_type height = m_tree.GetHeight();
            drawer.DrawString(5, m_textLine, "dynamic tree height = %d", height);
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        ++m_stepCount;
    }

    void KeyboardDown(Key key) override
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

    Real RayCastCallback(const RayCastInput& input, DynamicTree::size_type proxyId)
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
        Real fraction;
        bool overlap;
        DynamicTree::size_type proxyId;
    };

    AABB GetRandomAABB()
    {
        const auto w = Vec2(m_proxyExtent * 2, m_proxyExtent * 2) * Meter;
        //aabb->lowerBound.x = -m_proxyExtent;
        //aabb->lowerBound.y = -m_proxyExtent + m_worldExtent;
        const auto lowerBound = Vec2(RandomFloat(-m_worldExtent, m_worldExtent), RandomFloat(0.0f, 2.0f * m_worldExtent)) * Meter;
        const auto upperBound = lowerBound + w;
        return AABB(lowerBound, upperBound);
    }

    void MoveAABB(AABB* aabb)
    {
        const auto d = Vec2{RandomFloat(-0.5f, 0.5f), RandomFloat(-0.5f, 0.5f)} * Meter;
        //d.x = 2.0f;
        //d.y = 0.0f;
        aabb->Move(d);

        const auto c0 = GetCenter(*aabb);
        const auto min = Vec2(-m_worldExtent, Real(0)) * Meter;
        const auto max = Vec2(m_worldExtent, 2.0f * m_worldExtent) * Meter;
        const auto c = Length2D{Clamp(c0.x, min.x, max.x), Clamp(c0.y, min.y, max.y)};

        aabb->Move(c - c0);
    }

    void CreateProxy()
    {
        const auto extension = StepConf{}.aabbExtension;
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            const auto j = rand() % e_actorCount;
            const auto actor = m_actors + j;
            if (actor->proxyId == DynamicTree::InvalidIndex)
            {
                actor->aabb = GetRandomAABB();
                actor->proxyId = m_tree.CreateProxy(GetFattenedAABB(actor->aabb, extension), actor);
                return;
            }
        }
    }

    void DestroyProxy()
    {
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            const auto j = rand() % e_actorCount;
            const auto actor = m_actors + j;
            if (actor->proxyId != DynamicTree::InvalidIndex)
            {
                m_tree.DestroyProxy(actor->proxyId);
                actor->proxyId = DynamicTree::InvalidIndex;
                return;
            }
        }
    }

    void MoveProxy()
    {
        const auto extension = StepConf{}.aabbExtension;
        const auto multiplier = StepConf{}.displaceMultiplier;
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            const auto j = rand() % e_actorCount;
            const auto actor = m_actors + j;
            if (actor->proxyId == DynamicTree::InvalidIndex)
            {
                continue;
            }

            const auto aabb0 = actor->aabb;
            MoveAABB(&actor->aabb);
            const auto displacement = GetCenter(actor->aabb) - GetCenter(aabb0);
            if (!m_tree.GetAABB(actor->proxyId).Contains(actor->aabb))
            {
                const auto newAabb = GetDisplacedAABB(GetFattenedAABB(actor->aabb, extension), multiplier * displacement);
                m_tree.UpdateProxy(actor->proxyId, newAabb);
            }
            return;
        }
    }

    void Action()
    {
        const auto choice = rand() % 20;

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
        m_tree.Query(m_queryAABB, [&](DynamicTree::size_type nodeId){ return QueryCallback(nodeId); });

        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            if (m_actors[i].proxyId == DynamicTree::InvalidIndex)
            {
                continue;
            }

            const auto overlap = TestOverlap(m_queryAABB, m_actors[i].aabb);
            NOT_USED(overlap);
            assert(overlap == m_actors[i].overlap);
        }
    }

    void RayCast()
    {
        m_rayActor = nullptr;

        auto input = m_rayCastInput;

        // Ray cast against the dynamic tree.
        m_tree.RayCast(input, [&](const RayCastInput& rci, DynamicTree::size_type proxyId) {
            return RayCastCallback(rci, proxyId);
        });

        // Brute force ray cast.
        Actor* bruteActor = nullptr;
        RayCastOutput bruteOutput;
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            if (m_actors[i].proxyId == DynamicTree::InvalidIndex)
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

    Real m_worldExtent;
    Real m_proxyExtent;

    DynamicTree m_tree;
    AABB m_queryAABB;
    RayCastInput m_rayCastInput;
    RayCastOutput m_rayCastOutput;
    Actor* m_rayActor;
    Actor m_actors[e_actorCount];
    int m_stepCount;
    bool m_automated;
};

} // namespace box2d

#endif
