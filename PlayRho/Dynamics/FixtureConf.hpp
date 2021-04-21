/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DYNAMICS_FIXTURECONF_HPP
#define PLAYRHO_DYNAMICS_FIXTURECONF_HPP

/// @file
/// Declarations of the FixtureConf struct and any free functions associated with it.

#include <PlayRho/Dynamics/Filter.hpp>
#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Collision/Shapes/ShapeID.hpp>

#include <utility> // for std::move, std::forward
#include <type_traits> // for std::decay_t, std::void_t

namespace playrho {
namespace d2 {

class Fixture;

/// @brief Fixture definition.
/// @details A fixture is used to attach a shape to a body for collision detection. A fixture
///   inherits its transform from its body. Fixtures hold additional non-geometric data
///   such as collision filters, etc.
/// @ingroup PhysicalEntities
/// @see World::CreateFixture, World::GetFixture, World::SetFixture, World::Destroy.
struct FixtureConf {
    /// @brief Uses the given value for the shape member variable.
    FixtureConf& UseShape(ShapeID value) noexcept
    {
        shape = std::move(value);
        return *this;
    }

    /// @brief Uses the given value for the body member variable.
    FixtureConf& UseBody(BodyID value) noexcept
    {
        body = value;
        return *this;
    }

    /// @brief Identifier of shape to associate the fixture with.
    ShapeID shape = InvalidShapeID;

    /// @brief Identifier of body to associate the fixture with.
    BodyID body = InvalidBodyID;
};

/// @brief Operator equals.
/// @relatedalso FixtureConf
inline bool operator==(const FixtureConf& lhs, const FixtureConf& rhs)
{
    return lhs.shape == rhs.shape && //
           lhs.body == rhs.body;
}

/// @brief Operator not-equals.
/// @relatedalso FixtureConf
inline bool operator!=(const FixtureConf& lhs, const FixtureConf& rhs)
{
    return !(lhs == rhs);
}

/// @brief Gets the body of the given configuration.
/// @relatedalso FixtureConf
inline BodyID GetBody(const FixtureConf& conf) noexcept
{
    return conf.body;
}

/// @brief Gets the shape of the given configuration.
/// @relatedalso FixtureConf
inline ShapeID GetShape(const FixtureConf& conf) noexcept
{
    return conf.shape;
}

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_FIXTURECONF_HPP
