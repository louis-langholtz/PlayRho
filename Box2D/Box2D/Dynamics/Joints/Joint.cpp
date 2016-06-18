/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/Joint.h>
#include <Box2D/Dynamics/Joints/DistanceJoint.h>
#include <Box2D/Dynamics/Joints/WheelJoint.h>
#include <Box2D/Dynamics/Joints/MouseJoint.h>
#include <Box2D/Dynamics/Joints/RevoluteJoint.h>
#include <Box2D/Dynamics/Joints/PrismaticJoint.h>
#include <Box2D/Dynamics/Joints/PulleyJoint.h>
#include <Box2D/Dynamics/Joints/GearJoint.h>
#include <Box2D/Dynamics/Joints/WeldJoint.h>
#include <Box2D/Dynamics/Joints/FrictionJoint.h>
#include <Box2D/Dynamics/Joints/RopeJoint.h>
#include <Box2D/Dynamics/Joints/MotorJoint.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Common/BlockAllocator.h>

#include <new>

namespace box2d
{

Joint* Joint::Create(const JointDef& def, BlockAllocator* allocator)
{
	auto joint = static_cast<Joint*>(nullptr);

	switch (def.type)
	{
	case JointType::e_distanceJoint:
		{
			void* mem = allocator->Allocate(sizeof(DistanceJoint));
			joint = new (mem) DistanceJoint(static_cast<const DistanceJointDef&>(def));
		}
		break;

	case JointType::e_mouseJoint:
		{
			void* mem = allocator->Allocate(sizeof(MouseJoint));
			joint = new (mem) MouseJoint(static_cast<const MouseJointDef&>(def));
		}
		break;

	case JointType::e_prismaticJoint:
		{
			void* mem = allocator->Allocate(sizeof(PrismaticJoint));
			joint = new (mem) PrismaticJoint(static_cast<const PrismaticJointDef&>(def));
		}
		break;

	case JointType::e_revoluteJoint:
		{
			void* mem = allocator->Allocate(sizeof(RevoluteJoint));
			joint = new (mem) RevoluteJoint(static_cast<const RevoluteJointDef&>(def));
		}
		break;

	case JointType::e_pulleyJoint:
		{
			void* mem = allocator->Allocate(sizeof(PulleyJoint));
			joint = new (mem) PulleyJoint(static_cast<const PulleyJointDef&>(def));
		}
		break;

	case JointType::e_gearJoint:
		{
			void* mem = allocator->Allocate(sizeof(GearJoint));
			joint = new (mem) GearJoint(static_cast<const GearJointDef&>(def));
		}
		break;

	case JointType::e_wheelJoint:
		{
			void* mem = allocator->Allocate(sizeof(WheelJoint));
			joint = new (mem) WheelJoint(static_cast<const WheelJointDef&>(def));
		}
		break;

	case JointType::e_weldJoint:
		{
			void* mem = allocator->Allocate(sizeof(WeldJoint));
			joint = new (mem) WeldJoint(static_cast<const WeldJointDef&>(def));
		}
		break;
        
	case JointType::e_frictionJoint:
		{
			void* mem = allocator->Allocate(sizeof(FrictionJoint));
			joint = new (mem) FrictionJoint(static_cast<const FrictionJointDef&>(def));
		}
		break;

	case JointType::e_ropeJoint:
		{
			void* mem = allocator->Allocate(sizeof(RopeJoint));
			joint = new (mem) RopeJoint(static_cast<const RopeJointDef&>(def));
		}
		break;

	case JointType::e_motorJoint:
		{
			void* mem = allocator->Allocate(sizeof(MotorJoint));
			joint = new (mem) MotorJoint(static_cast<const MotorJointDef&>(def));
		}
		break;

	default:
		assert(false);
		break;
	}

	return joint;
}

void Joint::Destroy(Joint* joint, BlockAllocator* allocator)
{
	joint->~Joint();
	switch (joint->m_type)
	{
	case JointType::e_distanceJoint:
		allocator->Free(joint, sizeof(DistanceJoint));
		break;

	case JointType::e_mouseJoint:
		allocator->Free(joint, sizeof(MouseJoint));
		break;

	case JointType::e_prismaticJoint:
		allocator->Free(joint, sizeof(PrismaticJoint));
		break;

	case JointType::e_revoluteJoint:
		allocator->Free(joint, sizeof(RevoluteJoint));
		break;

	case JointType::e_pulleyJoint:
		allocator->Free(joint, sizeof(PulleyJoint));
		break;

	case JointType::e_gearJoint:
		allocator->Free(joint, sizeof(GearJoint));
		break;

	case JointType::e_wheelJoint:
		allocator->Free(joint, sizeof(WheelJoint));
		break;
    
	case JointType::e_weldJoint:
		allocator->Free(joint, sizeof(WeldJoint));
		break;

	case JointType::e_frictionJoint:
		allocator->Free(joint, sizeof(FrictionJoint));
		break;

	case JointType::e_ropeJoint:
		allocator->Free(joint, sizeof(RopeJoint));
		break;

	case JointType::e_motorJoint:
		allocator->Free(joint, sizeof(MotorJoint));
		break;

	default:
		assert(false);
		break;
	}
}

Joint::Joint(const JointDef& def):
	m_type(def.type), m_bodyA(def.bodyA), m_bodyB(def.bodyB),
	m_collideConnected(def.collideConnected), m_userData(def.userData)
{
	assert(def.bodyA != def.bodyB);
}

bool Joint::IsActive() const noexcept
{
	return m_bodyA->IsActive() && m_bodyB->IsActive();
}

void SetAwake(Joint& j) noexcept
{
	j.GetBodyA()->SetAwake();
	j.GetBodyB()->SetAwake();
}

} // namespace box2d