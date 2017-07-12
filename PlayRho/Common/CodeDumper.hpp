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

#ifndef CodeDumper_hpp
#define CodeDumper_hpp

#include <PlayRho/Common/Settings.hpp>

namespace playrho
{
    class World;
    class Body;
    class Joint;
    class Fixture;
    class DistanceJoint;
    class FrictionJoint;
    class GearJoint;
    class MotorJoint;
    class MouseJoint;
    class PrismaticJoint;
    class PulleyJoint;
    class RevoluteJoint;
    class RopeJoint;
    class WeldJoint;
    class WheelJoint;

    /// Dump the world into the log file.
    /// @warning this should be called outside of a time step.
    void Dump(const World& world);
    
    /// Dump body to a log file
    void Dump(const Body& body, std::size_t bodyIndex);
    
    /// Dump joint to the log file.
    void Dump(const Joint& joint, std::size_t index);

    /// Dump fixture to log file.
    void Dump(const Fixture& fixture, std::size_t bodyIndex);

    /// Dump joint to dmLog
    void Dump(const DistanceJoint& joint, std::size_t index);

    /// Dump joint to the log file.
    void Dump(const FrictionJoint& joint, std::size_t index);

    void Dump(const GearJoint& joint, std::size_t index);
    
    void Dump(const MotorJoint& joint, std::size_t index);

    void Dump(const MouseJoint& joint, std::size_t index);

    void Dump(const PrismaticJoint& joint, std::size_t index);

    /// Dump joint to dmLog
    void Dump(const PulleyJoint& joint, std::size_t index);

    void Dump(const RevoluteJoint& joint, std::size_t index);

    void Dump(const RopeJoint& joint, std::size_t index);

    void Dump(const WeldJoint& joint, std::size_t index);

    void Dump(const WheelJoint& joint, std::size_t index);
    
} // namespace playrho

#endif /* CodeDumper_hpp */
