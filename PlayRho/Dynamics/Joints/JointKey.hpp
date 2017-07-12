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

#ifndef JointKey_hpp
#define JointKey_hpp

/// @file
/// Definition of the JointKey class and any associated free functions.

#include <PlayRho/Common/Settings.hpp>
#include <utility>
#include <functional>

namespace playrho
{
    class Joint;
    class Body;

    class JointKey
    {
    public:
        static constexpr JointKey Get(const Body* bodyA, const Body* bodyB) noexcept
        {
            return (bodyA < bodyB)? JointKey{bodyA, bodyB}: JointKey{bodyB, bodyA};
        }
        
        constexpr const Body* GetBody1() const noexcept
        {
            return m_body1;
        }
        
        constexpr const Body* GetBody2() const
        {
            return m_body2;
        }

    private:
        constexpr JointKey(const Body* body1, const Body* body2):
        	m_body1(body1), m_body2(body2)
        {
            // Intentionally empty.
        }

        const Body* m_body1;
        const Body* m_body2;
    };
    
    JointKey GetJointKey(const Joint& joint) noexcept;

    constexpr int Compare(const JointKey& lhs, const JointKey& rhs) noexcept
    {
        if (lhs.GetBody1() < rhs.GetBody1())
        {
            return -1;
        }
        if (lhs.GetBody1() > rhs.GetBody1())
        {
            return +1;
        }
        if (lhs.GetBody2() < rhs.GetBody2())
        {
            return -1;
        }
        if (lhs.GetBody2() > rhs.GetBody2())
        {
            return +1;
        }
        return 0;
    }
    
    constexpr bool IsFor(const JointKey key, const Body* body) noexcept
    {
        return body == key.GetBody1() || body == key.GetBody2();
    }
    
}

namespace std
{
    template <>
    struct less<playrho::JointKey>
    {
        constexpr bool operator()(const playrho::JointKey& lhs,
                                  const playrho::JointKey& rhs) const
        {
            return playrho::Compare(lhs, rhs) < 0;
        }
    };
    
    template <>
    struct equal_to<playrho::JointKey>
    {
        constexpr bool operator()( const playrho::JointKey& lhs, const playrho::JointKey& rhs ) const
        {
            return playrho::Compare(lhs, rhs) == 0;
        }
    };

}

namespace playrho
{
    inline Joint* GetJointPtr(std::pair<JointKey, Joint*> value)
    {
        return value.second;
    }
}

#endif /* JointKey_hpp */
