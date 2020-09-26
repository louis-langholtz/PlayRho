/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DYNAMICS_BODYATTY_HPP
#define PLAYRHO_DYNAMICS_BODYATTY_HPP

/// @file
/// Declaration of the BodyAtty class.

#include <PlayRho/Dynamics/Body.hpp>

#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/Contacts/ContactKey.hpp>

#include <algorithm>
#include <utility>

namespace playrho {
namespace d2 {

/// @brief Body attorney.
///
/// @details This is the "body attorney" which provides limited privileged access to the
///   Body class for the World class.
///
/// @note This class uses the "attorney-client" idiom to control the granularity of
///   friend-based access to the Body class. This is meant to help preserve and enforce
///   the invariants of the Body class.
///
/// @see https://en.wikibooks.org/wiki/More_C++_Idioms/Friendship_and_the_Attorney-Client
///
class BodyAtty
{
private:

    friend class WorldImpl;
};

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_BODYATTY_HPP
