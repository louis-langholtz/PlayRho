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

#ifndef TEST_H
#define TEST_H

#include <Box2D/Box2D.h>
#include "DebugDraw.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#endif
#include <glfw/glfw3.h>

#include <cstdlib>

namespace box2d {

class Test;
struct Settings;

typedef Test* TestCreateFcn();

#define	RAND_LIMIT	32767
#define DRAW_STRING_NEW_LINE 16

/// Random number in range [-1,1]
inline float_t RandomFloat()
{
	auto r = static_cast<float_t>(std::rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

/// Random floating point number in range [lo, hi]
inline float_t RandomFloat(float_t lo, float_t hi)
{
	auto r = static_cast<float_t>(std::rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
	Settings()
	{
		hz = 60.0f;
		velocityIterations = 8;
		positionIterations = 3;
		drawShapes = true;
		drawJoints = true;
		drawAABBs = false;
		drawContactPoints = false;
		drawContactNormals = false;
		drawContactImpulse = false;
		drawFrictionImpulse = false;
		drawCOMs = false;
		drawStats = false;
		drawProfile = false;
		enableWarmStarting = true;
		enableContinuous = true;
		enableSubStepping = false;
		enableSleep = true;
		pause = false;
		singleStep = false;
	}

	float_t hz;
	int32 velocityIterations;
	int32 positionIterations;
	bool drawShapes;
	bool drawJoints;
	bool drawAABBs;
	bool drawContactPoints;
	bool drawContactNormals;
	bool drawContactImpulse;
	bool drawFrictionImpulse;
	bool drawCOMs;
	bool drawStats;
	bool drawProfile;
	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSubStepping;
	bool enableSleep;
	bool pause;
	bool singleStep;
};

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern TestEntry g_testEntries[];
// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class TestDestructionListener : public DestructionListener
{
public:
	void SayGoodbye(Fixture& fixture) { BOX2D_NOT_USED(fixture); }
	void SayGoodbye(Joint& joint);

	Test* test;
};

const int32 k_maxContactPoints = 2048;

struct ContactPoint
{
	Fixture* fixtureA;
	Fixture* fixtureB;
	Vec2 normal;
	Vec2 position;
	PointState state;
	float_t normalImpulse;
	float_t tangentImpulse;
	float_t separation;
};

class Test : public ContactListener
{
public:

	Test();
	virtual ~Test();

	void DrawTitle(const char *string);
	virtual void Step(Settings* settings);
	virtual void Keyboard(int key) { BOX2D_NOT_USED(key); }
	virtual void KeyboardUp(int key) { BOX2D_NOT_USED(key); }
	void ShiftMouseDown(const Vec2& p);
	virtual void MouseDown(const Vec2& p);
	virtual void MouseUp(const Vec2& p);
	void MouseMove(const Vec2& p);
	void LaunchBomb();
	void LaunchBomb(const Vec2& position, const Vec2& velocity);
	
	void SpawnBomb(const Vec2& worldPt);
	void CompleteBombSpawn(const Vec2& p);

	// Let derived tests know that a joint was destroyed.
	virtual void JointDestroyed(Joint* joint) { BOX2D_NOT_USED(joint); }

	// Callbacks for derived classes.
	virtual void BeginContact(Contact* contact) { BOX2D_NOT_USED(contact); }
	virtual void EndContact(Contact* contact) { BOX2D_NOT_USED(contact); }
	virtual void PreSolve(Contact* contact, const Manifold* oldManifold);
	virtual void PostSolve(Contact* contact, const ContactImpulse* impulse)
	{
		BOX2D_NOT_USED(contact);
		BOX2D_NOT_USED(impulse);
	}

	void ShiftOrigin(const Vec2& newOrigin);

protected:
	friend class TestDestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	Body* m_groundBody;
	AABB m_worldAABB;
	ContactPoint m_points[k_maxContactPoints];
	int32 m_pointCount;
	TestDestructionListener m_destructionListener;
	int32 m_textLine;
	World* m_world;
	Body* m_bomb;
	MouseJoint* m_mouseJoint;
	Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	Vec2 m_mouseWorld;
	int32 m_stepCount;

	Profile m_maxProfile;
	Profile m_totalProfile;
};

} // namespace box2d

#endif
