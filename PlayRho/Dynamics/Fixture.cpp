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

#include <PlayRho/Dynamics/Fixture.hpp>

#include <PlayRho/Dynamics/Contacts/Contact.hpp> // for MixFriction, MixRestitution

#include <type_traits>

namespace playrho
{
namespace d2
{

static_assert(std::is_default_constructible<Fixture>::value, "Fixture must be default constructible!");
static_assert(std::is_copy_constructible<Fixture>::value, "Fixture must be copy constructible!");
static_assert(std::is_move_constructible<Fixture>::value, "Fixture must be move constructible!");
static_assert(std::is_copy_assignable<Fixture>::value, "Fixture must be copy assignable!");
static_assert(std::is_move_assignable<Fixture>::value, "Fixture must be move assignable!");

Real GetDefaultFriction(const Fixture &fixtureA, const Fixture &fixtureB)
{
    return MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
}

Real GetDefaultRestitution(const Fixture &fixtureA, const Fixture &fixtureB)
{
    return MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
}

} // namespace d2
} // namespace playrho
