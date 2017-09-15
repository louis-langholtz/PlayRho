/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_DYNAMICS_CONTACTATTY_HPP
#define PLAYRHO_DYNAMICS_CONTACTATTY_HPP

/// @file
/// Declaration of the ContactAtty class.

#include <PlayRho/Dynamics/Contacts/Contact.hpp>

namespace playrho
{

    /// @brief Contact attorney.
    ///
    /// @details This class uses the "attorney-client" idiom to control the granularity of
    ///   friend-based access to the Contact class. This is meant to help preserve and enforce
    ///   the invariants of the Contact class.
    ///
    /// @sa https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Friendship_and_the_Attorney-Client
    ///
    class ContactAtty
    {
    private:

        static Manifold& GetMutableManifold(Contact& c) noexcept
        {
            return c.GetMutableManifold();
        }
        
        static void CopyFlags(Contact& to, const Contact& from) noexcept
        {
            to.m_flags = from.m_flags;
        }

        static void SetToi(Contact& c, Real value) noexcept
        {
            c.SetToi(value);
        }
        
        static void UnsetToi(Contact& c) noexcept
        {
            c.UnsetToi();
        }
        
        static void IncrementToiCount(Contact& c) noexcept
        {
            ++c.m_toiCount;
        }
        
        static void SetToiCount(Contact& c, Contact::substep_type value) noexcept
        {
            c.SetToiCount(value);
        }

        static void ResetToiCount(Contact& c) noexcept
        {
            c.SetToiCount(0);
        }
        
        static void UnflagForFiltering(Contact& c) noexcept
        {
            c.UnflagForFiltering();
        }
        
        static void Update(Contact& c, const Contact::UpdateConf& conf, ContactListener* listener)
        {
            c.Update(conf, listener);
        }
        
        static bool IsIslanded(const Contact& c) noexcept
        {
            return c.IsIslanded();
        }
        
        static void SetIslanded(Contact& c) noexcept
        {
            c.SetIslanded();
        }
        
        static void UnsetIslanded(Contact& c) noexcept
        {
            c.UnsetIslanded();
        }
        
        friend class World;
    };

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_CONTACTATTY_HPP
