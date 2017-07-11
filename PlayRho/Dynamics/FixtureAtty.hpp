/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef FixtureAtty_hpp
#define FixtureAtty_hpp

/// @file
/// Declaration of the FixtureAtty class.

#include <PlayRho/Common/Span.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>

namespace box2d
{
    /// @brief Fixture attorney.
    ///
    /// @details This class uses the "attorney-client" idiom to control the granularity of
    ///   friend-based access to the Fixture class. This is meant to help preserve and enforce
    ///   the invariants of the Fixture class.
    ///
    /// @sa https://en.wikibooks.org/wiki/More_C++_Idioms/Friendship_and_the_Attorney-Client
    ///
    class FixtureAtty
    {
    private:
        static Span<FixtureProxy> GetProxies(const Fixture& fixture)
        {
            return fixture.GetProxies();
        }
        
        static void SetProxies(Fixture& fixture, Span<FixtureProxy> value)
        {
            fixture.SetProxies(value);
        }
        
        static Fixture* Create(Body* body, const FixtureDef& def, std::shared_ptr<const Shape> shape)
        {
            return new Fixture{body, def, shape};
        }
        
        friend class World;
    };

} // namespace box2d

#endif /* FixtureAtty_hpp */
