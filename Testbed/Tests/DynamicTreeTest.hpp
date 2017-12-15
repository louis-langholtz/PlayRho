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

#ifndef PLAYRHO_DYNAMIC_TREE_TEST_HPP
#define  PLAYRHO_DYNAMIC_TREE_TEST_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class DynamicTreeTest : public Test
{
public:

    static PLAYRHO_CONSTEXPR const auto e_actorCount = 128;

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
            actor->treeId = m_tree.CreateLeaf(GetFattenedAABB(actor->aabb, aabbExtension), actor);
        }

        m_stepCount = 0;

        const auto h = m_worldExtent;
        m_queryAABB = AABB2D{Vec2(-3.0f, -4.0f + h) * 1_m, Vec2(5.0f, 6.0f + h) * 1_m};

        m_rayCastInput.p1 = Vec2(-5.0f, 5.0f + h) * 1_m;
        m_rayCastInput.p2 = Vec2(7.0f, -4.0f + h) * 1_m;
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
            const auto actionCount = std::max(1, e_actorCount >> 2);
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
            if (actor->treeId == DynamicTree::GetInvalidSize())
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

            const auto p1 = GetLowerBound(actor->aabb);
            const auto p2 = Length2{
                GetX(GetUpperBound(actor->aabb)), GetY(GetLowerBound(actor->aabb))
            };
            const auto p3 = GetUpperBound(actor->aabb);
            const auto p4 = Length2{
                GetX(GetLowerBound(actor->aabb)), GetY(GetUpperBound(actor->aabb))
            };
            
            drawer.DrawSegment(p1, p2, c);
            drawer.DrawSegment(p2, p3, c);
            drawer.DrawSegment(p3, p4, c);
            drawer.DrawSegment(p4, p1, c);
        }

        Color c(0.7f, 0.7f, 0.7f);
        {
            // Draw the AABB.

            const auto p1 = GetLowerBound(m_queryAABB);
            const auto p2 = Length2{
                GetX(GetUpperBound(m_queryAABB)), GetY(GetLowerBound(m_queryAABB))
            };
            const auto p3 = GetUpperBound(m_queryAABB);
            const auto p4 = Length2{
                GetX(GetLowerBound(m_queryAABB)), GetY(GetUpperBound(m_queryAABB))
            };
            
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
            const auto p = m_rayCastInput.p1 + m_rayActor->fraction * (m_rayCastInput.p2 - m_rayCastInput.p1);
            drawer.DrawPoint(p, 6.0f, cr);
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
            CreateLeaf();
            break;

        case Key_D:
            DestroyLeaf();
            break;

        case Key_M:
            MoveProxy();
            break;

        default:
            break;
        }
    }

    bool QueryCallback(DynamicTree::Size treeId)
    {
        Actor* actor = (Actor*)m_tree.GetLeafData(treeId);
        actor->overlap = TestOverlap(m_queryAABB, actor->aabb);
        return true;
    }

    Real RayCastCallback(const RayCastInput& input, DynamicTree::Size treeId)
    {
        auto actor = static_cast<Actor*>(m_tree.GetLeafData(treeId));

        const auto output = RayCast(actor->aabb, input);

        if (output.has_value())
        {
            m_rayCastOutput = output;
            m_rayActor = actor;
            m_rayActor->fraction = output->fraction;
            return output->fraction;
        }

        return input.maxFraction;
    }

private:

    struct Actor
    {
        AABB2D aabb;
        Real fraction;
        bool overlap;
        DynamicTree::Size treeId;
    };

    AABB2D GetRandomAABB()
    {
        const auto w = Vec2(m_proxyExtent * 2, m_proxyExtent * 2) * 1_m;
        //aabb->lowerBound.x = -m_proxyExtent;
        //aabb->lowerBound.y = -m_proxyExtent + m_worldExtent;
        const auto lowerBound = Vec2(RandomFloat(-m_worldExtent, m_worldExtent), RandomFloat(0.0f, 2.0f * m_worldExtent)) * 1_m;
        const auto upperBound = lowerBound + w;
        return AABB2D{lowerBound, upperBound};
    }

    void MoveAABB(AABB2D* aabb)
    {
        const auto d = Vec2{RandomFloat(-0.5f, 0.5f), RandomFloat(-0.5f, 0.5f)} * 1_m;
        //d.x = 2.0f;
        //d.y = 0.0f;
        Move(*aabb, d);

        const auto c0 = GetCenter(*aabb);
        const auto min = Vec2(-m_worldExtent, Real(0)) * 1_m;
        const auto max = Vec2(m_worldExtent, 2.0f * m_worldExtent) * 1_m;
        const auto c = Length2{
            Clamp(GetX(c0), GetX(min), GetX(max)), Clamp(GetY(c0), GetY(min), GetY(max))
        };

        Move(*aabb, c - c0);
    }

    void CreateLeaf()
    {
        const auto extension = StepConf{}.aabbExtension;
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            const auto j = rand() % e_actorCount;
            const auto actor = m_actors + j;
            if (actor->treeId == DynamicTree::GetInvalidSize())
            {
                actor->aabb = GetRandomAABB();
                actor->treeId = m_tree.CreateLeaf(GetFattenedAABB(actor->aabb, extension), actor);
                return;
            }
        }
    }

    void DestroyLeaf()
    {
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            const auto j = rand() % e_actorCount;
            const auto actor = m_actors + j;
            if (actor->treeId != DynamicTree::GetInvalidSize())
            {
                m_tree.DestroyLeaf(actor->treeId);
                actor->treeId = DynamicTree::GetInvalidSize();
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
            if (actor->treeId == DynamicTree::GetInvalidSize())
            {
                continue;
            }

            const auto aabb0 = actor->aabb;
            MoveAABB(&actor->aabb);
            const auto displacement = GetCenter(actor->aabb) - GetCenter(aabb0);
            if (!Contains(m_tree.GetAABB(actor->treeId), actor->aabb))
            {
                const auto newAabb = GetDisplacedAABB(GetFattenedAABB(actor->aabb, extension), multiplier * displacement);
                m_tree.UpdateLeaf(actor->treeId, newAabb);
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
            CreateLeaf();
            break;

        case 1:
            DestroyLeaf();
            break;

        default:
            MoveProxy();
        }
    }

    void Query()
    {
        Query(m_tree, m_queryAABB, [&](DynamicTree::Size nodeId){ return QueryCallback(nodeId); });

        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            if (m_actors[i].treeId == DynamicTree::GetInvalidSize())
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
        RayCast(m_tree, input, [&](const RayCastInput& rci, DynamicTree::Size treeId) {
            return RayCastCallback(rci, treeId);
        });

        // Brute force ray cast.
        Actor* bruteActor = nullptr;
        RayCastHit bruteOutput;
        for (auto i = decltype(e_actorCount){0}; i < e_actorCount; ++i)
        {
            if (m_actors[i].treeId == DynamicTree::GetInvalidSize())
            {
                continue;
            }

            const auto output = RayCast(m_actors[i].aabb, input);
            if (output.has_value())
            {
                bruteActor = m_actors + i;
                bruteOutput = *output;
                input.maxFraction = output->fraction;
            }
        }

        if (bruteActor)
        {
            assert(bruteOutput.fraction == m_rayCastOutput->fraction);
        }
    }

    Real m_worldExtent;
    Real m_proxyExtent;

    DynamicTree m_tree;
    AABB2D m_queryAABB;
    RayCastInput m_rayCastInput;
    RayCastOutput m_rayCastOutput;
    Actor* m_rayActor;
    Actor m_actors[e_actorCount];
    int m_stepCount;
    bool m_automated;
};

} // namespace testbed

#endif
