/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_TEST_HPP
#define PLAYRHO_TEST_HPP

#include <PlayRho/PlayRho.hpp>
#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Collision/ShapeSeparation.hpp>
#include <PlayRho/Collision/Shapes/ShapeVisitor.hpp>
#include <PlayRho/Dynamics/Contacts/PositionSolverManifold.hpp>
#include <PlayRho/Common/Range.hpp>
#include "Drawer.hpp"
#include <chrono>
#include <vector>
#include <iterator>

namespace playrho {

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
    float maxTranslation = static_cast<float>(Real{DefaultMaxTranslation / Meter});
    float maxRotation = 90; // in degrees
    float hz = 60;
    float dt = 1 / hz;
    float maxLinearCorrection = static_cast<float>(Real{DefaultMaxLinearCorrection / Meter}); // in meters
    float maxAngularCorrection = static_cast<float>(Real{DefaultMaxAngularCorrection / Degree}); // in degrees

    /// @brief Linear slop.
    /// @note Explicily coded to default to the same value as used in Erin's Box2D 2.3.2
    float linearSlop = static_cast<float>(Real{DefaultLinearSlop / Meter});
    
    /// @brief Angular slop.
    /// @note Explicily coded to default to the same value as used in Erin's Box2D 2.3.2
    float angularSlop = static_cast<float>(Real{DefaultAngularSlop / Radian});
    
    float regMinSeparation = static_cast<float>(Real{DefaultLinearSlop / Meter}) * -3.0f;
    float toiMinSeparation = static_cast<float>(Real{DefaultLinearSlop / Meter}) * -1.5f;
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
        Key_Space, Key_Comma, Key_Minus, Key_Period, Key_Equal,
        Key_0, Key_1, Key_2, Key_3, Key_4, Key_5, Key_6, Key_7, Key_8, Key_9,
        Key_A, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G, Key_H, Key_I, Key_J,
        Key_K, Key_L, Key_M, Key_N, Key_O, Key_P, Key_Q, Key_R, Key_S, Key_T,
        Key_U, Key_V, Key_W, Key_X, Key_Y, Key_Z,
        Key_Backspace, Key_Subtract, Key_Add,
        Key_Unknown
    };
    
    using Fixtures = std::vector<Fixture*>;
    
    Test(const WorldDef& config = WorldDef{}.UseGravity(LinearAcceleration2D{
        Real(0.0f) * MeterPerSquareSecond, -Real(10.0f) * MeterPerSquareSecond
    }).UseMinVertexRadius(Real(0.0001f) * Real{2} * Meter));
    virtual ~Test();

    void DrawTitle(Drawer& drawer, const char *string);
    void Step(const Settings& settings, Drawer& drawer);
    void DrawStats(Drawer& drawer, const StepConf& stepConf);
    void DrawStats(Drawer& drawer, const Fixture& fixture);
    void DrawContactInfo(const Settings& settings, Drawer& drawer);
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
    virtual void BeginContact(Contact&) override { }
    virtual void EndContact(Contact&) override { }
    virtual void PreSolve(Contact& contact, const Manifold& oldManifold) override;
    virtual void PostSolve(Contact&, const ContactImpulsesList&,
                           ContactListener::iteration_type) override { }

    static bool Contains(const Fixtures& fixtures, const Fixture* f) noexcept;

    Fixtures GetSelectedFixtures() const noexcept { return m_selectedFixtures; }

    void SetSelectedFixtures(Fixtures value) noexcept
    {
        m_selectedFixtures = value;
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
    
    static inline bool HasFixture(const ContactPoint& cp, const Fixtures& fixtures) noexcept
    {
        for (auto fixture: fixtures)
        {
            if (fixture == cp.fixtureA || fixture == cp.fixtureB)
            {
                return true;
            }
        }
        return false;
    }

    using ContactPoints = std::vector<ContactPoint>;

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

    void ResetWorld(const World& saved);

    int GetStepCount() const noexcept { return m_stepCount; }

    SizedRange<ContactPoints::const_iterator> GetPoints() const noexcept
    {
        return SizedRange<ContactPoints::const_iterator>(std::cbegin(m_points),
                                                         std::cend(m_points),
                                                         m_points.size());
    }

    const Body* GetBomb() const noexcept { return m_bomb; }
    void SetBomb(Body* body) noexcept { m_bomb = body; }
    void SetGroundBody(Body* body) noexcept { m_groundBody = body; }
    
    World* const m_world;
    TextLinePos m_textLine = TextLinePos{30};
    
private:
    Body* m_groundBody;
    Fixtures m_selectedFixtures;
    AABB m_worldAABB;
    ContactPoints m_points;
    DestructionListenerImpl m_destructionListener;
    Body* m_bomb = nullptr;
    MouseJoint* m_mouseJoint = nullptr;
    Length2D m_bombSpawnPoint;
    bool m_bombSpawning = false;
    Length2D m_mouseWorld;
    double m_sumDeltaTime = 0.0;
    int m_stepCount = 0;
    StepStats m_stepStats;
    std::size_t m_numContacts = 0;
    std::size_t m_maxContacts = 0;
    std::uint64_t m_sumContactsUpdatedPre = 0;
    std::uint64_t m_sumContactsSkippedPre = 0;
    std::uint64_t m_sumContactsIgnoredPre = 0;
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
    std::uint64_t m_sumToiContactsUpdatedTouching = 0;
    std::uint64_t m_sumToiContactsSkippedTouching = 0;
    std::uint64_t m_sumRegProxiesMoved = 0;
    std::uint64_t m_sumToiProxiesMoved = 0;
    Length m_minRegSep = std::numeric_limits<Real>::infinity() * Meter;
    Length m_maxRegSep = -std::numeric_limits<Real>::infinity() * Meter;
    Length m_minToiSep = 0;

    using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;
    using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
    using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
    
    dist_iter_type m_maxDistIters = 0;
    toi_iter_type m_maxToiIters = 0;
    root_iter_type m_maxRootIters = 0;

    std::chrono::duration<double> m_curStepDuration{0};
    std::chrono::duration<double> m_maxStepDuration{0};
    std::chrono::duration<double> m_sumStepDuration{0};
};

/// Random number in range [-1,1]
Real RandomFloat();

/// Random floating point number in range [lo, hi]
Real RandomFloat(Real lo, Real hi);

::std::ostream& operator<<(::std::ostream& os, const ContactFeature& value);

} // namespace playrho

#endif
