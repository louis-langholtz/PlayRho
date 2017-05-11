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

#ifndef JointAtty_hpp
#define JointAtty_hpp

/// @file
/// Declaration of the JointAtty class.

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d
{
    
    /// @brief Joint attorney.
    ///
    /// @details This class uses the "attorney-client" idiom to control the granularity of
    ///   friend-based access to the Joint class. This is meant to help preserve and enforce
    ///   the invariants of the Joint class.
    ///
    /// @sa https://en.wikibooks.org/wiki/More_C++_Idioms/Friendship_and_the_Attorney-Client
    ///
    class JointAtty
    {
    private:
        static Joint* Create(const box2d::JointDef &def)
        {
            return Joint::Create(def);
        }
        
        static void Destroy(Joint* j)
        {
            Joint::Destroy(j);
        }
        
        static void InitVelocityConstraints(Joint& j, BodyConstraints &bodies,
                                            const box2d::StepConf &step, const ConstraintSolverConf &conf)
        {
            j.InitVelocityConstraints(bodies, step, conf);
        }
        
        static bool SolveVelocityConstraints(Joint& j, BodyConstraints &bodies, const box2d::StepConf &conf)
        {
            return j.SolveVelocityConstraints(bodies, conf);
        }
        
        static bool SolvePositionConstraints(Joint& j, BodyConstraints &bodies, const ConstraintSolverConf &conf)
        {
            return j.SolvePositionConstraints(bodies, conf);
        }
        
        friend class World;
    };

}

#endif /* JointAtty_hpp */
