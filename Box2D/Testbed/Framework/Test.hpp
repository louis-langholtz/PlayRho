/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Box2D.hpp>
#include <cstdlib>
#include "Drawer.hpp"
#include <Box2D/Collision/CollideShapes.hpp>
#include <Box2D/Dynamics/Profile.hpp>

namespace box2d {

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
	float maxTranslation = 4;
	float maxRotation = 90; // in degrees
	float hz = 60;
	float dt = 1 / hz;
	float maxLinearCorrection = StripUnit(DefaultMaxLinearCorrection); // in meters
	float maxAngularCorrection = static_cast<float>(DefaultMaxAngularCorrection / Degree); // in degrees
	float linearSlop = StripUnit(DefaultLinearSlop);
	float angularSlop = static_cast<float>(DefaultAngularSlop / Radian);
	float regMinSeparation = StripUnit(DefaultLinearSlop) * -3;
	float toiMinSeparation = StripUnit(DefaultLinearSlop) * -1.5f;
	int regPosResRate = 20; // in percent
	int toiPosResRate = 75; // in percent
	int regVelocityIterations = 8;
	int regPositionIterations = 3;
	int toiVelocityIterations = 8;
	int toiPositionIterations = 20;
	int maxSubSteps = DefaultMaxSubSteps;
	bool drawShapes = true;
	bool drawSkins = false;
	bool drawJoints = true;
	bool drawAABBs = false;
	bool drawContactPoints = false;
	bool drawContactNormals = false;
	bool drawContactImpulse = false;
	bool drawFrictionImpulse = false;
	bool drawCOMs = false;
	bool drawStats = false;
	bool drawProfile = false;
	bool enableWarmStarting = true;
	bool enableContinuous = true;
	bool enableSubStepping = false;
	bool enableSleep = true;
	bool pause = false;
	bool singleStep = false;
};

class Test : public ContactListener
{
public:
	enum Key {
		Key_Comma, Key_Minus, Key_Period, Key_Equal,
		Key_0, Key_1, Key_2, Key_3, Key_4, Key_5, Key_6, Key_7, Key_8, Key_9,
		Key_A, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G, Key_H, Key_I, Key_J, Key_K, Key_L, Key_M,
		Key_N, Key_O, Key_P, Key_Q, Key_R, Key_S, Key_T, Key_U, Key_V, Key_W, Key_X, Key_Y, Key_Z,
		Key_Subtract, Key_Add,
		Key_Unknown
	};
	
	Test(const World::Def& config = World::Def{}.UseGravity(Vec2(0.0f, -10.0f) * MeterPerSquareSecond));
	virtual ~Test();

	void DrawTitle(Drawer& drawer, const char *string);
	void Step(const Settings& settings, Drawer& drawer);
	void DrawStats(Drawer& drawer, const StepConf& stepConf);
	void DrawContactPoints(const Settings& settings, Drawer& drawer);
	void ShiftMouseDown(const Length2D& p);
	void MouseMove(const Length2D& p);
	void LaunchBomb();
	void LaunchBomb(const Length2D& position, const LinearVelocity2D velocity);
	void SpawnBomb(const Length2D& worldPt);
	void CompleteBombSpawn(const Length2D& p);
	void ShiftOrigin(const Length2D& newOrigin);
	
	virtual void KeyboardDown(Key key) { NOT_USED(key); }
	virtual void KeyboardUp(Key key) { NOT_USED(key); }
	virtual void MouseDown(const Length2D& p);
	virtual void MouseUp(const Length2D& p);
	
	// Let derived tests know that a joint was destroyed.
	virtual void JointDestroyed(Joint* joint) { NOT_USED(joint); }

	// Callbacks for derived classes.
	virtual void BeginContact(Contact& contact) override { NOT_USED(contact); }
	virtual void EndContact(Contact& contact) override { NOT_USED(contact); }
	virtual void PreSolve(Contact& contact, const Manifold& oldManifold) override;
	virtual void PostSolve(Contact& contact, const ContactImpulsesList& impulse, ContactListener::iteration_type solved) override
	{
		NOT_USED(contact);
		NOT_USED(impulse);
		NOT_USED(solved);
	}

	Fixture* GetSelectedFixture() const noexcept { return m_selectedFixture; }

	void SetSelectedFixture(Fixture* value) noexcept
	{
		m_selectedFixture = value;
	}

protected:

	struct ContactPoint
	{
		Fixture* fixtureA;
		Fixture* fixtureB;
		UnitVec2 normal;
		Length2D position;
		PointState state;
		Momentum normalImpulse;
		Momentum tangentImpulse;
		Length separation;
	};
	
	// This is called when a joint in the world is implicitly destroyed
	// because an attached body is destroyed. This gives us a chance to
	// nullify the mouse joint.
	class DestructionListenerImpl : public DestructionListener
	{
	public:
		void SayGoodbye(Fixture& fixture) { NOT_USED(fixture); }
		void SayGoodbye(Joint& joint);
		
		Test* test;
	};
	
	using PointCount = int;
	using TextLinePos = int;
	static constexpr auto k_maxContactPoints = PointCount{2048};
	static constexpr auto DRAW_STRING_NEW_LINE = TextLinePos{16};

	virtual void PreStep(const Settings& settings, Drawer& drawer)
	{
		NOT_USED(settings);
		NOT_USED(drawer);
	}

	virtual void PostStep(const Settings& settings, Drawer& drawer)
	{
		NOT_USED(settings);
		NOT_USED(drawer);		
	}

	int GetStepCount() const noexcept { return m_stepCount; }
	PointCount GetPointCount() const noexcept { return m_pointCount; }
	const ContactPoint* GetPoints() const noexcept { return m_points; }
	const Body* GetBomb() const noexcept { return m_bomb; }
	
	World* const m_world;
	TextLinePos m_textLine = TextLinePos{30};
	
private:
	Body* m_groundBody;
	Fixture* m_selectedFixture = nullptr;
	AABB m_worldAABB;
	ContactPoint m_points[k_maxContactPoints];
	PointCount m_pointCount = 0;
	DestructionListenerImpl m_destructionListener;
	Body* m_bomb = nullptr;
	MouseJoint* m_mouseJoint = nullptr;
	Length2D m_bombSpawnPoint;
	bool m_bombSpawning = false;
	Length2D m_mouseWorld;
	double m_sumDeltaTime = 0.0;
	int m_stepCount = 0;
	StepStats m_stepStats;
	size_t m_numContacts = 0;
	size_t m_maxContacts = 0;
	std::uint64_t m_sumContactsUpdatedToi = 0;
	std::uint64_t m_sumContactsAtMaxSubSteps = 0;
	std::uint64_t m_sumRegIslandsFound = 0;
	std::uint64_t m_sumRegIslandsSolved = 0;
	std::uint64_t m_sumToiIslandsFound = 0;
	std::uint64_t m_sumToiIslandsSolved = 0;
	std::uint64_t m_sumRegPosIters = 0;
	std::uint64_t m_sumRegVelIters = 0;
	std::uint64_t m_sumToiPosIters = 0;
	std::uint64_t m_sumToiVelIters = 0;
	std::uint64_t m_sumRegProxiesMoved = 0;
	std::uint64_t m_sumToiProxiesMoved = 0;
	Length m_minRegSep = std::numeric_limits<RealNum>::infinity() * Meter;
	Length m_maxRegSep = -std::numeric_limits<RealNum>::infinity() * Meter;
	Length m_minToiSep = 0;

	using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;
	using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
	using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
	
	dist_iter_type m_maxDistIters = 0;
	toi_iter_type m_maxToiIters = 0;
	root_iter_type m_maxRootIters = 0;

	Profile m_maxProfile;
	Profile m_totalProfile;
};

typedef Test* TestCreateFcn();

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern const TestEntry g_testEntries[];

/// Random number in range [-1,1]
RealNum RandomFloat();

/// Random floating point number in range [lo, hi]
RealNum RandomFloat(RealNum lo, RealNum hi);

::std::ostream& operator<<(::std::ostream& os, const ContactFeature& value);

} // namespace box2d

#endif
