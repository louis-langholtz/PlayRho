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

#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/NonZero.hpp>
#include <PlayRho/Common/Span.hpp>
#include <PlayRho/Dynamics/Filter.hpp>
#include <PlayRho/Dynamics/FixtureConf.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>

#include <array>
#include <limits>
#include <memory>
#include <vector>

namespace playrho
{
    namespace d2
    {

        /// @brief An association between a body and a shape.
        ///
        /// @details A fixture is used to attach a shape to a body for collision detection. A fixture
        /// inherits its transform from its parent. Fixtures hold additional non-geometric data
        /// such as collision filters, etc.
        ///
        /// @warning you cannot reuse fixtures.
        /// @note Fixtures should be created using the <code>Body::CreateFixture</code> method.
        /// @note Destroy these using the <code>Body::Destroy(Fixture*, bool)</code> method.
        /// @note This structure is 56-bytes large (using a 4-byte Real on at least one 64-bit
        ///   architecture/build).
        ///
        /// @ingroup PhysicalEntities
        ///
        /// @see Body, Shape
        ///
        class Fixture
        {
         public:
            /// @brief Fixture proxies container.
            using Proxies = std::vector<FixtureProxy>;

            Fixture() = default;

            /// @brief Initializing constructor.
            ///
            /// @note This is not meant to be called by normal user code. Use the
            ///   <code>Body::CreateFixture</code> method instead.
            ///
            /// @param body Body the new fixture is to be associated with.
            /// @param shape Shareable shape to associate fixture with. Must be non-null.
            /// @param def Initial optional fixture settings.
            ///    Friction must be greater-than-or-equal-to zero.
            ///    <code>AreaDensity</code> must be greater-than-or-equal-to zero.
            ///
            Fixture(BodyID body, const Shape& shape, const FixtureConf& def = GetDefaultFixtureConf())
                : m_shape{shape},
                  m_body{body},
                  m_filter{def.filter},
                  m_isSensor{def.isSensor}
            {
                // Intentionally empty.
            }

            /// @brief Copy constructor (explicitly deleted).
            Fixture(const Fixture& other) = default;

            /// @brief Gets the parent body of this fixture.
            /// @return Non-null pointer to the parent body.
            BodyID GetBody() const noexcept;

            /// @brief Gets the child shape.
            /// @details The shape is not modifiable. Use a new fixture instead.
            Shape GetShape() const noexcept;

            /// @brief Set if this fixture is a sensor.
            void SetSensor(bool sensor) noexcept;

            /// @brief Is this fixture a sensor (non-solid)?
            /// @return the true if the shape is a sensor.
            bool IsSensor() const noexcept;

            /// @brief Gets the contact filtering data.
            Filter GetFilterData() const noexcept;

            /// @brief Sets the contact filtering data.
            void SetFilterData(Filter filter) noexcept;

            /// @brief Gets the density of this fixture.
            /// @return Non-negative density (in mass per area).
            AreaDensity GetDensity() const noexcept;

            /// @brief Gets the coefficient of friction.
            /// @return Value of 0 or higher.
            Real GetFriction() const noexcept;

            /// @brief Gets the coefficient of restitution.
            Real GetRestitution() const noexcept;

            /// @brief Gets the proxies associated with this fixture.
            const Proxies& GetProxies() const noexcept;

            /// @brief Sets the proxies associated with this fixture.
            void SetProxies(Proxies value) noexcept;

         private:
            // Data ordered here for memory compaction.

            /// Shape (of fixture).
            /// @note Set on construction.
            /// @note 16-bytes.
            Shape m_shape;

            Proxies m_proxies;///< Cache of fixture proxies for the assigned shape. 16+ bytes.

            BodyID m_body = InvalidBodyID;///< Parent body. 2-bytes.

            Filter m_filter;///< Filter object. 6-bytes.

            bool m_isSensor = false;///< Is/is-not sensor. 1-bytes.
        };

        inline Shape Fixture::GetShape() const noexcept
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

        inline void Fixture::SetFilterData(Filter filter) noexcept
        {
            m_filter = filter;
        }

        inline BodyID Fixture::GetBody() const noexcept
        {
            return m_body;
        }

        inline const Fixture::Proxies& Fixture::GetProxies() const noexcept
        {
            return m_proxies;
        }

        inline void Fixture::SetProxies(Proxies value) noexcept
        {
            m_proxies = std::move(value);
        }

        inline Real Fixture::GetFriction() const noexcept
        {
            return playrho::d2::GetFriction(m_shape);
        }

        inline Real Fixture::GetRestitution() const noexcept
        {
            return playrho::d2::GetRestitution(m_shape);
        }

        inline AreaDensity Fixture::GetDensity() const noexcept
        {
            return playrho::d2::GetDensity(m_shape);
        }

        inline void Fixture::SetSensor(bool sensor) noexcept
        {
            m_isSensor = sensor;
        }

        // Free functions...

        /// @brief Whether contact calculations should be performed between the two fixtures.
        /// @return <code>true</code> if contact calculations should be performed between these
        ///   two fixtures; <code>false</code> otherwise.
        /// @relatedalso Fixture
        inline bool ShouldCollide(const Fixture& fixtureA, const Fixture& fixtureB) noexcept
        {
            return ShouldCollide(fixtureA.GetFilterData(), fixtureB.GetFilterData());
        }

        /// @brief Gets the default friction amount for the given fixtures.
        Real GetDefaultFriction(const Fixture& fixtureA, const Fixture& fixtureB);

        /// @brief Gets the default restitution amount for the given fixtures.
        Real GetDefaultRestitution(const Fixture& fixtureA, const Fixture& fixtureB);

    }// namespace d2
}// namespace playrho

#endif// PLAYRHO_DYNAMICS_FIXTURE_HPP
