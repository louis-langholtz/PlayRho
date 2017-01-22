/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include "Test.hpp"
#include "Drawer.hpp"
#include <stdio.h>
#include <vector>

#include <Box2D/Rope/Rope.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>

using namespace box2d;

static void Draw(Drawer& drawer, const CircleShape& shape, const Transformation& xf, const Color& color)
{
	const auto center = Transform(shape.GetLocation(), xf);
	const auto radius = shape.GetRadius();
	const auto fillColor = Color{0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};
	drawer.DrawSolidCircle(center, radius, fillColor);
	drawer.DrawCircle(center, radius, color);

	// Draw a line fixed in the circle to animate rotation.
	const auto axis = Rotate(Vec2{RealNum{1}, RealNum{0}}, xf.q);
	drawer.DrawSegment(center, center + radius * axis, color);
}

static void DrawCorner(Drawer& drawer, Vec2 p, RealNum r, Angle a0, Angle a1, Color color)
{
	const auto angleDiff = GetRevRotationalAngle(a0, a1);
	auto lastAngle = 0_deg;
	for (auto angle = 5_deg; angle < angleDiff; angle += 5_deg)
	{
		const auto c0 = p + r * UnitVec2(a0 + lastAngle);
		const auto c1 = p + r * UnitVec2(a0 + angle);
		drawer.DrawSegment(c0, c1, color);
		lastAngle = angle;
	}
	{
		const auto c0 = p + r * UnitVec2(a0 + lastAngle);
		const auto c1 = p + r * UnitVec2(a1);
		drawer.DrawSegment(c0, c1, color);
	}
}

static void Draw(Drawer& drawer, const EdgeShape& shape, const Transformation& xf, const Color& color, bool skins)
{
	const auto v1 = Transform(shape.GetVertex1(), xf);
	const auto v2 = Transform(shape.GetVertex2(), xf);
	drawer.DrawSegment(v1, v2, color);
	
	if (skins)
	{
		const auto r = shape.GetVertexRadius();
		if (r > 0)
		{
			const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};
			const auto worldNormal0 = GetFwdPerpendicular(GetUnitVector(v2 - v1));
			const auto offset = worldNormal0 * r;
			drawer.DrawSegment(v1 + offset, v2 + offset, skinColor);
			drawer.DrawSegment(v1 - offset, v2 - offset, skinColor);
			
			const auto angle0 = GetAngle(worldNormal0);
			const auto angle1 = GetAngle(-worldNormal0);
			DrawCorner(drawer, v2, r, angle0, angle1, skinColor);
			DrawCorner(drawer, v1, r, angle1, angle0, skinColor);
		}
	}
}

static void Draw(Drawer& drawer, const ChainShape& shape, const Transformation& xf, const Color& color)
{
	const auto count = shape.GetVertexCount();
	auto v1 = Transform(shape.GetVertex(0), xf);
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		const auto v2 = Transform(shape.GetVertex(i), xf);
		drawer.DrawSegment(v1, v2, color);
		drawer.DrawCircle(v1, RealNum(0.05), color);
		v1 = v2;
	}
}

static void Draw(Drawer& drawer, const PolygonShape& shape, const Transformation& xf, const Color& color, bool skins)
{
	const auto vertexCount = shape.GetVertexCount();
	auto vertices = std::vector<Vec2>(vertexCount);
	for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
	{
		vertices[i] = Transform(shape.GetVertex(i), xf);
	}
	const auto fillColor = Color{0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};
	drawer.DrawSolidPolygon(&vertices[0], vertexCount, fillColor);
	drawer.DrawPolygon(&vertices[0], vertexCount, color);
	
	if (!skins)
	{
		return;
	}

	const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};
	const auto r = shape.GetVertexRadius();
	for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
	{
		if (i > 0)
		{
			const auto worldNormal0 = Rotate(shape.GetNormal(i - 1), xf.q);
			const auto p0 = vertices[i-1] + worldNormal0 * r;
			const auto p1 = vertices[i] + worldNormal0 * r;
			drawer.DrawSegment(p0, p1, skinColor);
			const auto normal1 = shape.GetNormal(i);
			const auto worldNormal1 = Rotate(normal1, xf.q);
			const auto angle0 = GetAngle(worldNormal0);
			const auto angle1 = GetAngle(worldNormal1);
			DrawCorner(drawer, vertices[i], r, angle0, angle1, skinColor);
		}
	}
	if (vertexCount > 1)
	{
		const auto worldNormal0 = Rotate(shape.GetNormal(vertexCount - 1), xf.q);
		drawer.DrawSegment(vertices[vertexCount - 1] + worldNormal0 * r, vertices[0] + worldNormal0 * r, skinColor);
		const auto worldNormal1 = Rotate(shape.GetNormal(0), xf.q);
		const auto angle0 = GetAngle(worldNormal0);
		const auto angle1 = GetAngle(worldNormal1);
		DrawCorner(drawer, vertices[0], r, angle0, angle1, skinColor);
	}
	else if (vertexCount == 1)
	{
		DrawCorner(drawer, vertices[0], r, 0_deg, 360_deg, skinColor);
	}
}

static void Draw(Drawer& drawer, const Fixture& fixture, const Transformation& xf, const Color& color, bool skins)
{
	switch (GetType(fixture))
	{
		case Shape::e_circle:
			Draw(drawer, *static_cast<const CircleShape*>(fixture.GetShape()), xf, color);
			break;
			
		case Shape::e_edge:
			Draw(drawer, *static_cast<const EdgeShape*>(fixture.GetShape()), xf, color, skins);
			break;
			
		case Shape::e_chain:
			Draw(drawer, *static_cast<const ChainShape*>(fixture.GetShape()), xf, color);
			break;
			
		case Shape::e_polygon:
			Draw(drawer, *static_cast<const PolygonShape*>(fixture.GetShape()), xf, color, skins);
			break;
			
		default:
			break;
	}
}

static Color GetColor(const Body& body)
{
	if (!body.IsActive())
	{
		return Color{0.5f, 0.5f, 0.3f};
	}
	if (body.GetType() == BodyType::Static)
	{
		return Color{0.5f, 0.9f, 0.5f};
	}
	if (body.GetType() == BodyType::Kinematic)
	{
		return Color{0.5f, 0.5f, 0.9f};
	}
	if (!body.IsAwake())
	{
		return Color{0.6f, 0.6f, 0.6f};
	}
	return Color{0.9f, 0.7f, 0.7f};
}

static void Draw(Drawer& drawer, const Body& body, bool skins)
{
	const auto xf = body.GetTransformation();
	const auto color = GetColor(body);
	for (auto&& f: body.GetFixtures())
	{
		Draw(drawer, f, xf, color, skins);
	}
}

static void Draw(Drawer& drawer, const Joint& joint)
{
	const auto bodyA = joint.GetBodyA();
	const auto bodyB = joint.GetBodyB();
	const auto xf1 = bodyA->GetTransformation();
	const auto xf2 = bodyB->GetTransformation();
	const auto x1 = xf1.p;
	const auto x2 = xf2.p;
	const auto p1 = joint.GetAnchorA();
	const auto p2 = joint.GetAnchorB();
	
	const Color color{0.5f, 0.8f, 0.8f};
	
	switch (joint.GetType())
	{
		case JointType::Distance:
			drawer.DrawSegment(p1, p2, color);
			break;
			
		case JointType::Pulley:
		{
			const auto pulley = static_cast<const PulleyJoint&>(joint);
			const auto s1 = pulley.GetGroundAnchorA();
			const auto s2 = pulley.GetGroundAnchorB();
			drawer.DrawSegment(s1, p1, color);
			drawer.DrawSegment(s2, p2, color);
			drawer.DrawSegment(s1, s2, color);
		}
			break;
			
		case JointType::Mouse:
			// don't draw this
			break;
			
		default:
			drawer.DrawSegment(x1, p1, color);
			drawer.DrawSegment(p1, p2, color);
			drawer.DrawSegment(x2, p2, color);
	}
}

static void Draw(Drawer& drawer, const World& world, const Settings& settings)
{
	if (settings.drawShapes)
	{
		for (auto&& b: world.GetBodies())
		{
			Draw(drawer, b, settings.drawSkins);
		}
	}
	
	if (settings.drawJoints)
	{
		for (auto&& j: world.GetJoints())
		{
			Draw(drawer, j);
		}
	}
	
	if (settings.drawAABBs)
	{
		const auto color = Color{0.9f, 0.3f, 0.9f};
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
					
					drawer.DrawPolygon(vs, 4, color);
				}
			}
		}
	}
	
	if (settings.drawCOMs)
	{
		const auto k_axisScale = RealNum(0.4);
		const auto red = Color{1.0f, 0.0f, 0.0f};
		const auto green = Color{0.0f, 1.0f, 0.0f};
		for (auto&& b: world.GetBodies())
		{
			auto xf = b.GetTransformation();
			xf.p = b.GetWorldCenter();
			const auto p1 = xf.p;
			drawer.DrawSegment(p1, p1 + k_axisScale * GetXAxis(xf.q), red);
			drawer.DrawSegment(p1, p1 + k_axisScale * GetYAxis(xf.q), green);			
		}
	}
}

void Test::DestructionListenerImpl::SayGoodbye(Joint& joint)
{
	if (test->m_mouseJoint == &joint)
	{
		test->m_mouseJoint = nullptr;
	}
	else
	{
		test->JointDestroyed(&joint);
	}
}

Test::Test(const World::Def& conf):
	m_world{new World(conf)}
{
	m_destructionListener.test = this;
	m_world->SetDestructionListener(&m_destructionListener);
	m_world->SetContactListener(this);
	
	m_groundBody = m_world->CreateBody();

	memset(&m_maxProfile, 0, sizeof(Profile));
	memset(&m_totalProfile, 0, sizeof(Profile));
}

Test::~Test()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
}

void Test::PreSolve(Contact& contact, const Manifold& oldManifold)
{
	const auto& manifold = contact.GetManifold();

	const auto manifoldPointCount = manifold.GetPointCount();
	if (manifoldPointCount == 0)
	{
		return;
	}

	auto fixtureA = contact.GetFixtureA();
	auto fixtureB = contact.GetFixtureB();

	PointStateArray state1;
	PointStateArray state2;
	GetPointStates(state1, state2, oldManifold, manifold);

	const auto worldManifold = GetWorldManifold(contact);

	for (auto i = decltype(manifoldPointCount){0}; (i < manifoldPointCount) && (m_pointCount < k_maxContactPoints); ++i)
	{
		auto& cp = m_points[m_pointCount];
		cp.fixtureA = fixtureA;
		cp.fixtureB = fixtureB;
		cp.position = worldManifold.GetPoint(i);
		cp.normal = worldManifold.GetNormal();
		cp.state = state2[i];
		const auto ci = manifold.GetContactImpulses(i);
		cp.normalImpulse = ci.m_normal;
		cp.tangentImpulse = ci.m_tangent;
		cp.separation = worldManifold.GetSeparation(i);
		++m_pointCount;
	}
}

void Test::DrawTitle(Drawer& drawer, const char *string)
{
    drawer.DrawString(5, DRAW_STRING_NEW_LINE, string);
    m_textLine = 3 * DRAW_STRING_NEW_LINE;
}

class QueryCallback : public QueryFixtureReporter
{
public:
	QueryCallback(const Vec2& point)
	{
		m_point = point;
		m_fixture = nullptr;
	}

	bool ReportFixture(Fixture* fixture)
	{
		const auto body = fixture->GetBody();
		if (body->GetType() == BodyType::Dynamic)
		{
			if (TestPoint(*fixture, m_point))
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	Vec2 m_point;
	Fixture* m_fixture;
};

void Test::MouseDown(const Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint)
	{
		return;
	}

	// Make a small box.
	const auto aabb = AABB{p, p} + Vec2(0.001f, 0.001f);
	
	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	m_world->QueryAABB(&callback, aabb);

	SetSelectedFixture(callback.m_fixture);

	if (callback.m_fixture)
	{
		const auto body = callback.m_fixture->GetBody();
		MouseJointDef md;
		md.bodyA = m_groundBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0f * GetMass(*body);
		m_mouseJoint = static_cast<MouseJoint*>(m_world->CreateJoint(md));
		body->SetAwake();
	}
}

void Test::SpawnBomb(const Vec2& worldPt)
{
	m_bombSpawnPoint = worldPt;
	m_bombSpawning = true;
}
    
void Test::CompleteBombSpawn(const Vec2& p)
{
	if (!m_bombSpawning)
	{
		return;
	}

	const auto vel = (m_bombSpawnPoint - p) * 30.0f;
	LaunchBomb(m_bombSpawnPoint,vel);
	m_bombSpawning = false;
}

void Test::ShiftMouseDown(const Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint)
	{
		return;
	}

	SpawnBomb(p);
}

void Test::MouseUp(const Vec2& p)
{
	if (m_mouseJoint)
	{
		m_world->Destroy(m_mouseJoint);
		m_mouseJoint = nullptr;
	}
	
	if (m_bombSpawning)
	{
		CompleteBombSpawn(p);
	}
}

void Test::MouseMove(const Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint)
	{
		m_mouseJoint->SetTarget(p);
	}
}

void Test::LaunchBomb()
{
	const auto p = Vec2(RandomFloat(-15.0f, 15.0f), 30.0f);
	const auto v = -5.0f * p;
	LaunchBomb(p, v);
}

void Test::LaunchBomb(const Vec2& position, const Vec2& linearVelocity)
{
	if (m_bomb)
	{
		m_world->Destroy(m_bomb);
		m_bomb = nullptr;
	}

	m_bomb = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(position).UseBullet(true));
	m_bomb->SetVelocity(Velocity{linearVelocity, 0_rad});
	
	const auto circle = std::make_shared<CircleShape>(0.3f);

	FixtureDef fd{};
	fd.density = 20.0f;
	fd.restitution = 0.0f;
	m_bomb->CreateFixture(circle, fd);
}

void Test::Step(const Settings& settings, Drawer& drawer)
{
	PreStep(settings, drawer);

	if (settings.pause)
	{
		drawer.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += DRAW_STRING_NEW_LINE;
		
		if ((settings.dt == 0) && m_mouseJoint)
		{
			const auto bodyB = m_mouseJoint->GetBodyB();
			const auto anchorB = m_mouseJoint->GetAnchorB();
			const auto centerB = bodyB->GetLocation();
			const auto destB = m_mouseJoint->GetTarget();
			bodyB->SetTransform(destB - (anchorB - centerB), bodyB->GetAngle());
		}
	}

	m_world->SetSubStepping(settings.enableSubStepping);

	m_pointCount = 0;

	StepConf stepConf;
	stepConf.set_dt(settings.dt);
	stepConf.regVelocityIterations = static_cast<StepConf::iteration_type>(settings.regVelocityIterations);
	stepConf.regPositionIterations = static_cast<StepConf::iteration_type>(settings.regPositionIterations);
	stepConf.toiVelocityIterations = static_cast<StepConf::iteration_type>(settings.toiVelocityIterations);
	stepConf.toiPositionIterations = static_cast<StepConf::iteration_type>(settings.toiPositionIterations);
	stepConf.maxTranslation = static_cast<decltype(stepConf.maxTranslation)>(settings.maxTranslation);
	stepConf.maxRotation = settings.maxRotation * 1_deg;
	stepConf.maxLinearCorrection = settings.maxLinearCorrection;
	stepConf.maxAngularCorrection = (settings.maxAngularCorrection * 1_deg) / 1_rad;
	stepConf.regResolutionRate = settings.regPosResRate / 100.0f;
	stepConf.toiResolutionRate = settings.toiPosResRate / 100.0f;
	if (!settings.enableSleep)
	{
		stepConf.minStillTimeToSleep = GetInvalid<RealNum>();
		Awaken(*m_world);
	}
	stepConf.doToi = settings.enableContinuous;
	stepConf.doWarmStart = settings.enableWarmStarting;

	const auto stepStats = m_world->Step(stepConf);

	Draw(drawer, *m_world, settings);

	drawer.Flush();

	if (settings.dt > 0)
	{
		++m_stepCount;
		m_stepStats = stepStats;
	}

	if (settings.drawStats)
	{
		drawer.DrawString(5, m_textLine, "step#=%d:", m_stepCount);
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "  pre-info: contacts-added=%d contacts-ignored=%d contacts-destroyed=%d contacts-updated=%d",
						  m_stepStats.pre.added, m_stepStats.pre.ignored, m_stepStats.pre.destroyed, m_stepStats.pre.updated);
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "  reg-info: contacts-added=%d islands-found=%d islands-solved=%d bodies-slept=%d",
						  m_stepStats.reg.contactsAdded, m_stepStats.reg.islandsFound, m_stepStats.reg.islandsSolved, m_stepStats.reg.bodiesSlept);
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "  toi-info: contacts-added=%d islands-found=%d contacts-checked=%d",
						  m_stepStats.toi.contactsAdded, m_stepStats.toi.islandsFound, m_stepStats.toi.contactsChecked);
		m_textLine += DRAW_STRING_NEW_LINE;

		const auto sleepCount = [&](){
			auto count = unsigned(0);
			for (auto&& body: m_world->GetBodies())
			{
				if (!body.IsAwake())
				{
					++count;
				}
			}
			return count;
		}();
		const auto bodyCount = GetBodyCount(*m_world);
		const auto contactCount = GetContactCount(*m_world);
		const auto jointCount = GetJointCount(*m_world);
		const auto fixtureCount = GetFixtureCount(*m_world);
		const auto shapeCount = GetShapeCount(*m_world);
		drawer.DrawString(5, m_textLine, "  sleep=%d, bodies=%d, fixtures=%d, shapes=%d, contacts=%d, joints=%d",
						  sleepCount, bodyCount, fixtureCount, shapeCount, contactCount, jointCount);
		m_textLine += DRAW_STRING_NEW_LINE;

		const auto proxyCount = m_world->GetProxyCount();
		const auto height = m_world->GetTreeHeight();
		const auto balance = m_world->GetTreeBalance();
		const auto quality = m_world->GetTreeQuality();
		drawer.DrawString(5, m_textLine, "  proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	// Track maximum profile times
	{
		const auto p = Profile{};
		m_maxProfile.step = Max(m_maxProfile.step, p.step);
		m_maxProfile.collide = Max(m_maxProfile.collide, p.collide);
		m_maxProfile.solve = Max(m_maxProfile.solve, p.solve);
		m_maxProfile.solveInit = Max(m_maxProfile.solveInit, p.solveInit);
		m_maxProfile.solveVelocity = Max(m_maxProfile.solveVelocity, p.solveVelocity);
		m_maxProfile.solvePosition = Max(m_maxProfile.solvePosition, p.solvePosition);
		m_maxProfile.solveTOI = Max(m_maxProfile.solveTOI, p.solveTOI);
		m_maxProfile.broadphase = Max(m_maxProfile.broadphase, p.broadphase);

		m_totalProfile.step += p.step;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.solveInit += p.solveInit;
		m_totalProfile.solveVelocity += p.solveVelocity;
		m_totalProfile.solvePosition += p.solvePosition;
		m_totalProfile.solveTOI += p.solveTOI;
		m_totalProfile.broadphase += p.broadphase;
	}

	if (settings.drawProfile)
	{
		const auto p = Profile{};
		
		Profile aveProfile;
		memset(&aveProfile, 0, sizeof(Profile));
		if (m_stepCount > 0)
		{
			const auto scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.solveInit = scale * m_totalProfile.solveInit;
			aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
			aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
			aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
			aveProfile.broadphase = scale * m_totalProfile.broadphase;
		}

		drawer.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	if (m_mouseJoint)
	{
		const auto p1 = m_mouseJoint->GetAnchorB();
		const auto p2 = m_mouseJoint->GetTarget();

		drawer.DrawPoint(p1, 4.0f, Color{0.0f, 1.0f, 0.0f});
		drawer.DrawPoint(p2, 4.0f, Color{0.0f, 1.0f, 0.0f});

		drawer.DrawSegment(p1, p2, Color{0.8f, 0.8f, 0.8f});
	}
	
	if (m_bombSpawning)
	{
		drawer.DrawPoint(m_bombSpawnPoint, 4.0f, Color{0.0f, 0.0f, 1.0f});
		drawer.DrawSegment(m_mouseWorld, m_bombSpawnPoint, Color{0.8f, 0.8f, 0.8f});
	}

	if (settings.drawContactPoints)
	{
		const auto k_impulseScale = RealNum(0.1);
		const auto k_axisScale = RealNum(0.3);

		for (auto i = decltype(m_pointCount){0}; i < m_pointCount; ++i)
		{
			const auto point = m_points + i;

			if (point->state == PointState::AddState)
			{
				// Add
				drawer.DrawPoint(point->position, 10.0f, Color{0.3f, 0.95f, 0.3f});
			}
			else if (point->state == PointState::PersistState)
			{
				// Persist
				drawer.DrawPoint(point->position, 5.0f, Color{0.3f, 0.3f, 0.95f});
			}

			if (settings.drawContactNormals)
			{
				const auto p1 = point->position;
				const auto p2 = p1 + k_axisScale * point->normal;
				drawer.DrawSegment(p1, p2, Color{0.9f, 0.9f, 0.9f});
			}
			else if (settings.drawContactImpulse)
			{
				const auto p1 = point->position;
				const auto p2 = p1 + k_impulseScale * point->normalImpulse * point->normal;
				drawer.DrawSegment(p1, p2, Color{0.9f, 0.9f, 0.3f});
			}

			if (settings.drawFrictionImpulse)
			{
				const auto tangent = GetFwdPerpendicular(point->normal);
				const auto p1 = point->position;
				const auto p2 = p1 + k_impulseScale * point->tangentImpulse * tangent;
				drawer.DrawSegment(p1, p2, Color{0.9f, 0.9f, 0.3f});
			}
		}
	}
	
	PostStep(settings, drawer);
}

void Test::ShiftOrigin(const Vec2& newOrigin)
{
	m_world->ShiftOrigin(newOrigin);
}

constexpr auto RAND_LIMIT = 32767;

RealNum box2d::RandomFloat()
{
	auto r = static_cast<RealNum>(std::rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

RealNum box2d::RandomFloat(RealNum lo, RealNum hi)
{
	auto r = static_cast<RealNum>(std::rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

static const char* GetName(ContactFeature::Type type)
{
	switch (type)
	{
		case ContactFeature::e_face: return "face";
		case ContactFeature::e_vertex: return "vertex";
	}
	return "unknown";
}

::std::ostream& box2d::operator<<(::std::ostream& os, const ContactFeature& value)
{
	os << "{";
	os << ::GetName(value.typeA);
	os << ",";
	os << unsigned(value.indexA);
	os << ",";
	os << ::GetName(value.typeB);
	os << ",";
	os << unsigned(value.indexB);
	os << "}";
	return os;
}
