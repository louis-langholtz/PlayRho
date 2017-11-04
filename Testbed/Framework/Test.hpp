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
#include "UiState.hpp"
#include <chrono>
#include <vector>
#include <iterator>
#include <functional>
#include <GLFW/glfw3.h>
#include <deque>
#include <algorithm>
#include <limits>
#include <set>

namespace playrho {

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
    float maxTranslation = static_cast<float>(Real{DefaultMaxTranslation / Meter});
    float maxRotation = 90; // in degrees

    float dt = 1.0f / 60; // in seconds.
    float minDt = 1.0f / 120;
    float maxDt = 1.0f / 5;

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

    float cameraZoom = 1.0f;

    int regPosResRate = 20; // in percent
    int toiPosResRate = 75; // in percent
    int regVelocityIterations = 8;
    int regPositionIterations = 3;
    int toiVelocityIterations = 8;
    int toiPositionIterations = 20;
    int maxSubSteps = DefaultMaxSubSteps;
    bool drawShapes = true;
    bool drawSkins = false;
    bool drawLabels = false;
    bool drawJoints = true;
    bool drawAABBs = false;
    bool drawContactPoints = false;
    bool drawContactNormals = false;
    bool drawContactImpulse = false;
    bool drawFrictionImpulse = false;
    bool drawCOMs = false;
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
    
    using KeyHandlerID = std::size_t;
    
    using KeyID = int;
    
    using KeyMods = int;
    
    using KeyAction = int;
    
    struct KeyActionMods
    {
        KeyID key;
        KeyAction action;
        KeyMods mods;
    };

    using KeyHandler = std::function<void(KeyActionMods keyActMods)>;

    using NeededSettings = std::uint32_t;
    enum NeededSettingsField: std::uint8_t {
        NeedDrawSkinsField,
        NeedDrawLabelsField,
        NeedLinearSlopField,
        NeedCameraZoom,
        NeedMaxTranslation,
        NeedDeltaTime,
    };
    
    using KeyHandlers = std::vector<std::pair<std::string, KeyHandler>>;
    using HandledKeys = std::vector<std::pair<KeyActionMods, KeyHandlerID>>;

    using FixtureSet = std::set<Fixture*>;
    using BodySet = std::set<Body*>;

    virtual ~Test();

    /// @brief Steps this test's world forward and visualizes what's going on.
    /// @note This method calls the PreStep and PostStep methods which give
    ///   sub-classes a way to augment some of what this operation does.
    /// @note This uses Non-Virtual Interface idiom/pattern related to the
    ///   Template Method pattern.
    /// @sa https://en.wikipedia.org/wiki/Non-virtual_interface_pattern
    /// @sa https://en.wikipedia.org/wiki/Template_method_pattern
    void Step(const Settings& settings, Drawer& drawer, UiState& ui);
    
    void ShiftMouseDown(const Length2& p);
    void MouseMove(const Length2& p);
    void LaunchBomb();
    void LaunchBomb(const Length2& position, const LinearVelocity2 velocity);
    void SpawnBomb(const Length2& worldPt);
    void CompleteBombSpawn(const Length2& p);
    void ShiftOrigin(const Length2& newOrigin);
    
    void KeyboardHandler(KeyID key, KeyAction action, KeyMods mods);
    
    const std::string& GetKeyHandlerInfo(KeyHandlerID id) const
    {
        return m_keyHandlers[id].first;
    }
    
    SizedRange<HandledKeys::const_iterator> GetHandledKeys() const
    {
        return SizedRange<HandledKeys::const_iterator>(std::cbegin(m_handledKeys),
                                                       std::cend(m_handledKeys),
                                                       m_handledKeys.size());
    }

    void MouseDown(const Length2& p);
    void MouseUp(const Length2& p);
    
    // Let derived tests know that a joint was destroyed.
    virtual void JointDestroyed(Joint* joint) { NOT_USED(joint); }

    // Callbacks for derived classes.
    virtual void BeginContact(Contact&) override { }
    virtual void EndContact(Contact&) override { }
    virtual void PreSolve(Contact& contact, const Manifold& oldManifold) override;
    virtual void PostSolve(Contact&, const ContactImpulsesList&,
                           ContactListener::iteration_type) override { }

    static bool Contains(const FixtureSet& fixtures, const Fixture* f) noexcept;
    
    const std::string& GetDescription() const noexcept { return m_description; }
    NeededSettings GetNeededSettings() const noexcept { return m_neededSettings; }
    const Settings& GetSettings() const noexcept { return m_settings; }
    const std::string& GetCredits() const noexcept { return m_credits; }
    const std::string& GetSeeAlso() const noexcept { return m_seeAlso; }
    const std::string& GetStatus() const noexcept { return m_status; }

    FixtureSet GetSelectedFixtures() const noexcept { return m_selectedFixtures; }
    BodySet GetSelectedBodies() const noexcept { return m_selectedBodies; }

    World m_world;

protected:
    
    EdgeShape::Conf GetGroundEdgeConf() const noexcept
    {
        return EdgeShape::Conf{}.UseVertex1(Vec2(-40, 0) * 1_m).UseVertex2(Vec2(40, 0) * 1_m);
    }

    struct Conf
    {
        /// @brief World definition/configuration data.
        /// @note Explicitly uses -10 for gravity here to behave more like
        ///   Erin Catto's Box2D Testbed (which uses -10 for Earthly gravity).
        WorldDef worldDef = WorldDef{}.UseGravity(LinearAcceleration2{
            Real(0.0f) * MeterPerSquareSecond, -Real(10.0f) * MeterPerSquareSecond
        }).UseMinVertexRadius(0.0002_m);

        Settings settings;
        
        NeededSettings neededSettings = 0u;
        
        std::string description;
        std::string seeAlso;
        std::string credits;
    };

    static Conf GetDefaultConf()
    {
        return Conf{};
    }

    Test(Conf config = GetDefaultConf());

    struct ContactPoint
    {
        Fixture* fixtureA;
        Fixture* fixtureB;
        UnitVec2 normal;
        Length2 position;
        PointState state;
        Momentum normalImpulse;
        Momentum tangentImpulse;
        Length separation;
    };
    
    static inline bool HasFixture(const ContactPoint& cp, const FixtureSet& fixtures) noexcept
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
    
    void SetSelectedFixtures(FixtureSet value) noexcept;
    
    void ClearSelectedFixtures()
    {
        m_selectedFixtures.clear();
        m_selectedBodies.clear();
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
    
    Length2 GetMouseWorld() const noexcept { return m_mouseWorld; }

    void SetMouseWorld(Length2 value) noexcept { m_mouseWorld = value; }
    
    KeyHandlerID RegisterKeyHandler(const std::string& info, KeyHandler handler)
    {
        const auto index = m_keyHandlers.size();
        m_keyHandlers.push_back(std::make_pair(info, handler));
        return index;
    }

    void RegisterForKey(KeyID key, KeyAction action, KeyMods mods, KeyHandlerID id);

    void RegisterForKey(KeyID key, KeyAction action, KeyMods mods,
                        const std::string& info, KeyHandler handler)
    {
        RegisterForKey(key, action, mods, RegisterKeyHandler(info, handler));
    }

    std::string m_status;
    TextLinePos m_textLine = TextLinePos{30};

private:
    void DrawStats(const StepConf& stepConf, UiState& ui);
    void DrawContactInfo(const Settings& settings, Drawer& drawer);
    bool DrawWorld(Drawer& drawer, const World& world, const Settings& settings,
                   const FixtureSet& selected);

    const Settings m_settings;
    const NeededSettings m_neededSettings;
    const std::string m_description;
    const std::string m_credits;
    const std::string m_seeAlso; ///< Reference - like a URL - which user may copy into copy/paste buffer.

    FixtureSet m_selectedFixtures;
    BodySet m_selectedBodies;
    AABB m_maxAABB;
    ContactPoints m_points;
    DestructionListenerImpl m_destructionListener;
    Body* m_bomb = nullptr;
    MouseJoint* m_mouseJoint = nullptr;
    Length2 m_bombSpawnPoint;
    bool m_bombSpawning = false;
    Length2 m_mouseWorld = Length2{};
    double m_sumDeltaTime = 0.0;
    int m_stepCount = 0;
    StepStats m_stepStats;
    ContactCounter m_numContacts = 0;
    ContactCounter m_maxContacts = 0;
    ContactCounter m_maxTouching = 0;
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
    Length m_minRegSep = std::numeric_limits<Length>::infinity();
    Length m_maxRegSep = -std::numeric_limits<Length>::infinity();
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
    
    KeyHandlers m_keyHandlers;
    HandledKeys m_handledKeys;
    
    std::size_t m_maxHistory = std::size_t{600u};
    std::deque<std::size_t> m_numContactsPerStep;
    std::deque<std::size_t> m_numTouchingPerStep;
};

// Free functions...

/// Random number in range [-1,1]
Real RandomFloat();

/// Random floating point number in range [lo, hi]
Real RandomFloat(Real lo, Real hi);

template <class Container, class T>
inline bool IsWithin(const Container& container, const T& element) noexcept
{
    const auto first = std::cbegin(container);
    const auto last = std::cend(container);
    const auto it = std::find(first, last, element);
    return it != last;
}

::std::ostream& operator<<(::std::ostream& os, const ContactFeature& value);

} // namespace playrho

#endif
