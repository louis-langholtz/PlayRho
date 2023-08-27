/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/WorldImplJoint.hpp>

#include <PlayRho/Dynamics/WorldImpl.hpp>

#include <PlayRho/d2/Joint.hpp>
#include <PlayRho/d2/RevoluteJointConf.hpp>
#include <PlayRho/d2/PrismaticJointConf.hpp>
#include <PlayRho/d2/DistanceJointConf.hpp>
#include <PlayRho/d2/PulleyJointConf.hpp>
#include <PlayRho/d2/TargetJointConf.hpp>
#include <PlayRho/d2/GearJointConf.hpp>
#include <PlayRho/d2/WheelJointConf.hpp>
#include <PlayRho/d2/WeldJointConf.hpp>
#include <PlayRho/d2/FrictionJointConf.hpp>
#include <PlayRho/d2/RopeJointConf.hpp>
#include <PlayRho/d2/MotorJointConf.hpp>

namespace playrho {
namespace d2 {

JointCounter GetJointRange(const WorldImpl& world) noexcept
{
    return world.GetJointRange();
}

JointID CreateJoint(WorldImpl& world, const Joint& def)
{
    return world.CreateJoint(def);
}

void Destroy(WorldImpl& world, JointID id)
{
    world.Destroy(id);
}

const Joint& GetJoint(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id);
}

void SetJoint(WorldImpl& world, JointID id, const Joint& def)
{
    world.SetJoint(id, def);
}

} // namespace d2
} // namespace playrho
