/*
 * Original work Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "Drawer.h"

using namespace box2d;

void Drawer::SetFlags(uint32 flags)
{
	m_drawFlags = flags;
}

uint32 Drawer::GetFlags() const
{
	return m_drawFlags;
}

void Drawer::AppendFlags(uint32 flags)
{
	m_drawFlags |= flags;
}

void Drawer::ClearFlags(uint32 flags)
{
	m_drawFlags &= ~flags;
}

#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/Joints/Joint.h>
#include <Box2D/Dynamics/Joints/PulleyJoint.h>
#include <Box2D/Rope/Rope.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

void box2d::Draw(Drawer& draw, const World& world)
{
	const auto flags = draw.GetFlags();
	
	if (flags & Drawer::e_shapeBit)
	{
		for (auto&& b: world.GetBodies())
		{
			const auto xf = b.GetTransformation();
			for (auto&& f: b.GetFixtures())
			{
				if (!b.IsActive())
				{
					Draw(draw, f, xf, Color(0.5f, 0.5f, 0.3f));
				}
				else if (b.GetType() == BodyType::Static)
				{
					Draw(draw, f, xf, Color(0.5f, 0.9f, 0.5f));
				}
				else if (b.GetType() == BodyType::Kinematic)
				{
					Draw(draw, f, xf, Color(0.5f, 0.5f, 0.9f));
				}
				else if (!b.IsAwake())
				{
					Draw(draw, f, xf, Color(0.6f, 0.6f, 0.6f));
				}
				else
				{
					Draw(draw, f, xf, Color(0.9f, 0.7f, 0.7f));
				}
			}
		}
	}
	
	if (flags & Drawer::e_jointBit)
	{
		for (auto&& j: world.GetJoints())
		{
			Draw(draw, j);
		}
	}
	
	if (flags & Drawer::e_pairBit)
	{
		//const Color color(0.3f, 0.9f, 0.9f);
		//for (auto&& c: m_contactMgr.GetContacts())
		//{
		//Fixture* fixtureA = c.GetFixtureA();
		//Fixture* fixtureB = c.GetFixtureB();
		
		//Vec2 cA = fixtureA->GetAABB().GetCenter();
		//Vec2 cB = fixtureB->GetAABB().GetCenter();
		
		//draw.DrawSegment(cA, cB, color);
		//}
	}
	
	if (flags & Drawer::e_aabbBit)
	{
		const Color color(0.9f, 0.3f, 0.9f);
		const auto bp = &world.GetContactManager().m_broadPhase;
		
		for (auto&& b: world.GetBodies())
		{
			if (!b.IsActive())
			{
				continue;
			}
			
			for (auto&& f: b.GetFixtures())
			{
				const auto proxy_count = f.GetProxyCount();
				for (auto i = decltype(proxy_count){0}; i < proxy_count; ++i)
				{
					const auto proxy = f.GetProxy(i);
					const auto aabb = bp->GetFatAABB(proxy->proxyId);
					Vec2 vs[4];
					vs[0] = Vec2{aabb.GetLowerBound().x, aabb.GetLowerBound().y};
					vs[1] = Vec2{aabb.GetUpperBound().x, aabb.GetLowerBound().y};
					vs[2] = Vec2{aabb.GetUpperBound().x, aabb.GetUpperBound().y};
					vs[3] = Vec2{aabb.GetLowerBound().x, aabb.GetUpperBound().y};
					
					draw.DrawPolygon(vs, 4, color);
				}
			}
		}
	}
	
	if (flags & Drawer::e_centerOfMassBit)
	{
		for (auto&& b: world.GetBodies())
		{
			auto xf = b.GetTransformation();
			xf.p = b.GetWorldCenter();
			draw.DrawTransform(xf);
		}
	}
}

void box2d::Draw(Drawer& draw, const Fixture& fixture, const Transformation& xf, const Color& color)
{
	switch (GetType(fixture))
	{
		case Shape::e_circle:
		{
			const auto circle = static_cast<const CircleShape*>(fixture.GetShape());
			const auto center = Transform(circle->GetPosition(), xf);
			const auto radius = circle->GetRadius();
			const auto axis = Rotate(Vec2{float_t{1}, float_t{0}}, xf.q);
			draw.DrawSolidCircle(center, radius, axis, color);
		}
			break;
			
		case Shape::e_edge:
		{
			const auto edge = static_cast<const EdgeShape*>(fixture.GetShape());
			const auto v1 = Transform(edge->GetVertex1(), xf);
			const auto v2 = Transform(edge->GetVertex2(), xf);
			draw.DrawSegment(v1, v2, color);
		}
			break;
			
		case Shape::e_chain:
		{
			const auto chain = static_cast<const ChainShape*>(fixture.GetShape());
			const auto count = chain->GetVertexCount();
			auto v1 = Transform(chain->GetVertex(0), xf);
			for (auto i = decltype(count){1}; i < count; ++i)
			{
				const auto v2 = Transform(chain->GetVertex(i), xf);
				draw.DrawSegment(v1, v2, color);
				draw.DrawCircle(v1, float_t(0.05), color);
				v1 = v2;
			}
		}
			break;
			
		case Shape::e_polygon:
		{
			const auto poly = static_cast<const PolygonShape*>(fixture.GetShape());
			const auto vertexCount = poly->GetVertexCount();
			assert(vertexCount <= MaxPolygonVertices);
			Vec2 vertices[MaxPolygonVertices];
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Transform(poly->GetVertex(i), xf);
			}
			draw.DrawSolidPolygon(vertices, vertexCount, color);
		}
			break;
			
		default:
			break;
	}
}

void box2d::Draw(Drawer& draw, const Joint& joint)
{
	const auto bodyA = joint.GetBodyA();
	const auto bodyB = joint.GetBodyB();
	const auto xf1 = bodyA->GetTransformation();
	const auto xf2 = bodyB->GetTransformation();
	const auto x1 = xf1.p;
	const auto x2 = xf2.p;
	const auto p1 = joint.GetAnchorA();
	const auto p2 = joint.GetAnchorB();
	
	const Color color(float_t(0.5), float_t(0.8), float_t(0.8));
	
	switch (joint.GetType())
	{
		case JointType::Distance:
			draw.DrawSegment(p1, p2, color);
			break;
			
		case JointType::Pulley:
		{
			const auto pulley = static_cast<const PulleyJoint&>(joint);
			const auto s1 = pulley.GetGroundAnchorA();
			const auto s2 = pulley.GetGroundAnchorB();
			draw.DrawSegment(s1, p1, color);
			draw.DrawSegment(s2, p2, color);
			draw.DrawSegment(s1, s2, color);
		}
			break;
			
		case JointType::Mouse:
			// don't draw this
			break;
			
		default:
			draw.DrawSegment(x1, p1, color);
			draw.DrawSegment(p1, p2, color);
			draw.DrawSegment(x2, p2, color);
	}
}

void box2d::Draw(Drawer& drawer, const Rope& rope)
{
	const auto c = Color(float_t(0.4), float_t(0.5), float_t(0.7));
	
	const auto count = rope.GetVertexCount();
	for (auto i = decltype(count - 1){0}; i < count - 1; ++i)
	{
		drawer.DrawSegment(rope.GetVertex(i), rope.GetVertex(i + 1), c);
	}
}
