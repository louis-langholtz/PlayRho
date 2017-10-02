/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_DYNAMICS_FIXTURE_HPP
#define PLAYRHO_DYNAMICS_FIXTURE_HPP

/// @file
/// Declarations of the Fixture class, and free functions associated with it.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Span.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Dynamics/Filter.hpp>
#include <PlayRho/Dynamics/FixtureDef.hpp>
#include <limits>
#include <memory>

namespace playrho {

class Body;
struct FixtureProxy;
class Shape;

/// @brief An association between a body and a shape.
///
/// @details A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as collision filters, etc.
///
/// @warning you cannot reuse fixtures.
/// @note Fixtures are created via Body::CreateFixture.
/// @note This structure is 56-bytes large (using a 4-byte Real on at least one 64-bit
///   architecture/build).
///
/// @sa Body, Shape
///
class Fixture
{
public:
    Fixture() = delete; // explicitly deleted
    
    /// @brief Initializing constructor.
    ///
    /// @warning Behavior is undefined if shape is <code>nullptr</code>.
    ///
    /// @note This is not meant to be called by normal user code. Use the Body::CreateFixture
    ///    method instead.
    ///
    /// @param body Body the new fixture is to be associated with.
    /// @param def Initial fixture settings.
    ///    Friction must be greater-than-or-equal-to zero.
    ///    Density must be greater-than-or-equal-to zero.
    /// @param shape Sharable shape to associate fixture with. Must be non-null.
    ///
    Fixture(NonNull<Body*> body, const FixtureDef& def, const std::shared_ptr<const Shape>& shape):
        m_body{body},
        m_shape{shape},
        m_filter{def.filter},
        m_isSensor{def.isSensor},
        m_userData{def.userData}
    {
        assert(shape);
    }
    
    /// @brief Destructor.
    /// @pre Proxy count is zero.
    /// @warning Behavior is undefined if proxy count is greater than zero.
    ~Fixture()
    {
        assert(!m_proxies);
        assert(m_proxyCount == 0);
    }

    /// @brief Gets the parent body of this fixture.
    /// @return Non-null pointer to the parent body.
    NonNull<Body*> GetBody() const noexcept;
    
    /// @brief Gets the child shape.
    /// @details The shape is not modifiable. Use a new fixture instead.
    std::shared_ptr<const Shape> GetShape() const noexcept;
    
    /// @brief Set if this fixture is a sensor.
    void SetSensor(bool sensor) noexcept;

    /// @brief Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    bool IsSensor() const noexcept;

    /// @brief Sets the contact filtering data.
    /// @note This won't update contacts until the next time step when either parent body
    ///    is speedable and awake.
    /// @note This automatically calls Refilter.
    void SetFilterData(Filter filter);

    /// @brief Gets the contact filtering data.
    Filter GetFilterData() const noexcept;

    /// @brief Refilter the fixture.
    /// @note Call this if you want to establish collision that was previously disabled by
    ///   ContactFilter::ShouldCollide.
    void Refilter();

    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    void* GetUserData() const noexcept;

    /// @brief Sets the user data.
    /// @note Use this to store your application specific data.
    void SetUserData(void* data) noexcept;

    /// @brief Gets the density of this fixture.
    /// @return Non-negative density (in mass per area).
    Density GetDensity() const noexcept;

    /// @brief Gets the coefficient of friction.
    /// @return Value of 0 or higher.
    Real GetFriction() const noexcept;

    /// @brief Gets the coefficient of restitution.
    Real GetRestitution() const noexcept;

    /// @brief Sets the coefficient of restitution.
    /// @note This will _not_ change the restitution of existing contacts.
    void SetRestitution(Real restitution) noexcept;

    /// @brief Gets the proxy count.
    /// @note This will be zero until a world step has been run since this fixture's
    ///   creation.
    ChildCounter GetProxyCount() const noexcept;

    /// @brief Gets the proxy for the given index.
    /// @return Pointer to fixture proxy or <code>nullptr</code> if not given a valid index.
    const FixtureProxy* GetProxy(ChildCounter index) const noexcept;

private:

    friend class FixtureAtty;
    
    using FixtureProxies = FixtureProxy*;
    
    Span<FixtureProxy> GetProxies() const noexcept;
    void SetProxies(Span<FixtureProxy> value) noexcept;

    // Data ordered here for memory compaction.
    
    NonNull<Body*> const m_body; ///< Parent body. Set on construction. 8-bytes.

    /// Shape (of fixture).
    /// @note Set on construction.
    /// @note Either null or pointer to a heap-memory private copy of the assigned shape.
    /// @note 16-bytes.
    std::shared_ptr<const Shape> m_shape;
    
    FixtureProxies m_proxies = nullptr; ///< Array of fixture proxies for the assigned shape. 8-bytes.
    
    void* m_userData = nullptr; ///< User data. 8-bytes.

    /// Proxy count.
    /// @details This is the fixture shape's child count after proxy creation. 4-bytes.
    ChildCounter m_proxyCount = 0;

    Filter m_filter; ///< Filter object. 6-bytes.
    
    bool m_isSensor = false; ///< Is/is-not sensor. 1-bytes.
};

inline std::shared_ptr<const Shape> Fixture::GetShape() const noexcept
{
    return m_shape;
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

inline NonNull<Body*> Fixture::GetBody() const noexcept
{
    return m_body;
}

inline ChildCounter Fixture::GetProxyCount() const noexcept
{
    return m_proxyCount;
}

inline void Fixture::SetFilterData(Filter filter)
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
    assert(value.size() < std::numeric_limits<ChildCounter>::max());
    m_proxies = value.begin();
    m_proxyCount = static_cast<decltype(m_proxyCount)>(value.size());
}

// Free functions...

/// @brief Tests a point for containment in a fixture.
/// @param f Fixture to use for test.
/// @param p Point in world coordinates.
/// @relatedalso Fixture
/// @ingroup TestPointGroup
bool TestPoint(const Fixture& f, Length2D p) noexcept;

/// @brief Sets the associated body's sleep status to awake.
/// @note This is a convenience function that simply looks up the fixture's body and
///   calls that body' SetAwake method.
/// @param f Fixture whose body should be awoken.
/// @relatedalso Fixture
void SetAwake(const Fixture& f) noexcept;

/// @brief Gets the transformation associated with the given fixture.
/// @warning Behavior is undefined if the fixture doesn't have an associated body - i.e.
///   behavior is undefined if the fixture has <code>nullptr</code> as its associated body.
/// @relatedalso Fixture
Transformation GetTransformation(const Fixture& f) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_FIXTURE_HPP
