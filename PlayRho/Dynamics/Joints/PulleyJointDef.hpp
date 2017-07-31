/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PulleyJointDef_hpp
#define PulleyJointDef_hpp

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class PulleyJoint;

/// @brief Pulley joint definition.
/// @details This requires two ground anchors, two dynamic body anchor points, and a pulley ratio.
struct PulleyJointDef : public JointBuilder<PulleyJointDef>
{
    using super = JointBuilder<PulleyJointDef>;
    
    PulleyJointDef() noexcept: super{JointType::Pulley}
    {
        collideConnected = true;
    }
    
    /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
    PulleyJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB,
                   const Length2D groundAnchorA, const Length2D groundAnchorB,
                   const Length2D anchorA, const Length2D anchorB);
    
    PulleyJointDef& UseRatio(Real v) noexcept;
    
    /// The first ground anchor in world coordinates. This point never moves.
    Length2D groundAnchorA = Length2D{Real(-1) * Meter, Real(1) * Meter};
    
    /// The second ground anchor in world coordinates. This point never moves.
    Length2D groundAnchorB = Length2D{Real(1) * Meter, Real(1) * Meter};
    
    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{Real(-1) * Meter, Real(0) * Meter};
    
    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{Real(1) * Meter, Real(0) * Meter};
    
    /// The a reference length for the segment attached to bodyA.
    Length lengthA = Length{0};
    
    /// The a reference length for the segment attached to bodyB.
    Length lengthB = Length{0};
    
    /// The pulley ratio, used to simulate a block-and-tackle.
    Real ratio = 1;
};

inline PulleyJointDef& PulleyJointDef::UseRatio(Real v) noexcept
{
    ratio = v;
    return *this;
}

PulleyJointDef GetPulleyJointDef(const PulleyJoint& joint) noexcept;

} // namespace playrho

#endif /* PulleyJointDef_hpp */
