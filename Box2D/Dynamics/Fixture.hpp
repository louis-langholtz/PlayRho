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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

/// @file
/// Declarations of the Filter, FixtureDef, and Fixture classes, and free functions
///   associated with them.

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Span.hpp>
#include <Box2D/Dynamics/Filter.hpp>
#include <Box2D/Dynamics/FixtureDef.hpp>
#include <limits>
#include <memory>

namespace box2d {

class Body;
struct FixtureProxy;
class Shape;

/// @brief Fixture.
///
/// @details
/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
///
/// @warning you cannot reuse fixtures.
/// @note Fixtures are created via Body::CreateFixture.
/// @note This structure is 56-bytes large (using a 4-byte RealNum on at least one 64-bit
///   architecture/build).
///
class Fixture
{
public:
    Fixture() = delete; // explicitly deleted

    /// Gets the parent body of this fixture.
    /// @details This is nullptr if the fixture is not attached.
    /// @return the parent body.
    Body* GetBody() noexcept;
    
    /// Gets the parent body of this fixture.
    /// @details This is nullptr if the fixture is not attached.
    /// @return the parent body.
    const Body* GetBody() const noexcept;

    /// Gets the child shape.
    /// @details The shape is not modifiable. Use a new fixture instead.
    const Shape* GetShape() const noexcept;
    
    /// Set if this fixture is a sensor.
    void SetSensor(bool sensor) noexcept;

    /// Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    bool IsSensor() const noexcept;

    /// Sets the contact filtering data.
    /// @note This won't update contacts until the next time step when either parent body
    ///    is speedable and awake.
    /// @note This automatically calls Refilter.
    void SetFilterData(const Filter filter);

    /// Gets the contact filtering data.
    Filter GetFilterData() const noexcept;

    /// Refilter the fixture.
    /// @note Call this if you want to establish collision that was previously disabled by
    ///   ContactFilter::ShouldCollide.
    void Refilter();

    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    void* GetUserData() const noexcept;

    /// Set the user data. Use this to store your application specific data.
    void SetUserData(void* data) noexcept;

    /// Sets the density of this fixture.
    /// @note This will _not_ automatically adjust the mass of the body.
    ///   You must call Body::ResetMassData to update the body's mass.
    /// @warning Behavior is undefined if given a negative value.
    /// @param density Non-negative density in kg/m^2.
    void SetDensity(RealNum density) noexcept;

    /// Gets the density of this fixture.
    /// @return Non-negative density in kg/m^2.
    Density GetDensity() const noexcept;

    /// Gets the coefficient of friction.
    RealNum GetFriction() const noexcept;

    /// Sets the coefficient of friction. This will _not_ change the friction of
    /// existing contacts.
    void SetFriction(RealNum friction) noexcept;

    /// Gets the coefficient of restitution.
    RealNum GetRestitution() const noexcept;

    /// Sets the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    void SetRestitution(RealNum restitution) noexcept;

    child_count_t GetProxyCount() const noexcept;

    const FixtureProxy* GetProxy(child_count_t index) const noexcept;

    /// Destructor.
    /// @pre Proxy count is zero.
    /// @warning Behavior is undefined if proxy count is greater than zero.
    ~Fixture()
    {
        // No access to BroadPhase now so can't call DestroyProxies here.
        assert(!m_proxies);
        assert(m_proxyCount == 0);
    }

private:

    friend class FixtureAtty;
    
    using FixtureProxies = FixtureProxy*;

    /// Initializing constructor.
    ///
    /// @warning Behavior is undefined if a <code>nullptr</code> initial body setting is used.
    /// @warning Behavior is undefined if a <code>nullptr</code> initial shape setting is used.
    /// @warning Behavior is undefined if a negative initial density setting is used.
    /// @warning Behavior is undefined if a negative initial friction setting is used.
    /// @warning Behavior is undefined if the restitution value is not less than infinity.
    /// @warning Behavior is undefined if the restitution value is not greater than -infinity.
    ///
    /// @param body Body the new fixture is to be associated with.
    /// @param def Initial fixture settings.
    ///    Friction must be greater-than-or-equal-to zero.
    ///    Density must be greater-than-or-equal-to zero.
    /// @param shape Sharable shape to associate fixture with. Must be non-null.
    ///
    Fixture(Body* body, const FixtureDef& def, std::shared_ptr<const Shape> shape):
        m_body{body},
        m_shape{shape},
        m_filter{def.filter},
        m_isSensor{def.isSensor},
        m_userData{def.userData}
    {
        assert(body);
        assert(shape);
    }
    
    Span<FixtureProxy> GetProxies() const noexcept;
    void SetProxies(Span<FixtureProxy> value) noexcept;

    // Data ordered here for memory compaction.
    
    Body* const m_body = nullptr; ///< Parent body. Set on construction. 8-bytes.

    /// Shape (of fixture).
    /// @note Set on construction.
    /// @note Either null or pointer to a heap-memory private copy of the assigned shape.
    /// @note 16-bytes.
    std::shared_ptr<const Shape> m_shape;
    
    FixtureProxies m_proxies = nullptr; ///< Array of fixture proxies for the assigned shape. 8-bytes.
    
    void* m_userData = nullptr; ///< User data. 8-bytes.

    /// Proxy count.
    /// @details This is the fixture shape's child count after proxy creation. 4-bytes.
    child_count_t m_proxyCount = 0;

    Filter m_filter; ///< Filter object. 6-bytes.
    
    bool m_isSensor = false; ///< Is/is-not sensor. 1-bytes.
};

inline const Shape* Fixture::GetShape() const noexcept
{
    return m_shape.get();
}

inline bool Fixture::IsSensor() const noexcept
{
    return m_isSensor;
}

inline Filter Fixture::GetFilterData() const noexcept
{
    return m_filter;
}

inline void* Fixture::GetUserData() const noexcept
{
    return m_userData;
}

inline void Fixture::SetUserData(void* data) noexcept
{
    m_userData = data;
}

inline Body* Fixture::GetBody() noexcept
{
    return m_body;
}

inline const Body* Fixture::GetBody() const noexcept
{
    return m_body;
}

inline child_count_t Fixture::GetProxyCount() const noexcept
{
    return m_proxyCount;
}

inline void Fixture::SetFilterData(const Filter filter)
{
    m_filter = filter;
    Refilter();
}

inline Span<FixtureProxy> Fixture::GetProxies() const noexcept
{
    return Span<FixtureProxy>(m_proxies, m_proxyCount);
}

inline void Fixture::SetProxies(Span<FixtureProxy> value) noexcept
{
    assert(value.size() < std::numeric_limits<child_count_t>::max());
    m_proxies = value.begin();
    m_proxyCount = static_cast<decltype(m_proxyCount)>(value.size());
}

// Free functions...

/// Test a point for containment in a fixture.
/// @param f Fixture to use for test.
/// @param p Point in world coordinates.
bool TestPoint(const Fixture& f, const Length2D p) noexcept;

/// Sets the associated body's sleep status to awake.
/// @note This is a convenience function that simply looks up the fixture's body and
///   calls that body' SetAwake method.
/// @param f Fixture whose body should be awoken.
void SetAwake(Fixture& f) noexcept;

/// Gets the transformation associated with the given fixture.
/// @warning Behavior is undefined if the fixture doesn't have an associated body - i.e.
///   behavior is undefined if the fixture has <code>nullptr</code> as its associated body.
Transformation GetTransformation(const Fixture& f) noexcept;
    
} // namespace box2d

#endif
