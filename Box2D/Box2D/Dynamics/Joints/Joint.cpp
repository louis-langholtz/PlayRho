/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/Joint.hpp>
#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Dynamics/Joints/WheelJoint.hpp>
#include <Box2D/Dynamics/Joints/MouseJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <Box2D/Dynamics/Joints/GearJoint.hpp>
#include <Box2D/Dynamics/Joints/WeldJoint.hpp>
#include <Box2D/Dynamics/Joints/FrictionJoint.hpp>
#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Dynamics/Joints/MotorJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Common/BlockAllocator.hpp>

#include <new>

namespace box2d
{

namespace
{
	inline DistanceJoint* Create(const DistanceJointDef& def, BlockAllocator& allocator)
	{
		if (DistanceJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(DistanceJoint));
			return new (mem) DistanceJoint(def);
		}
		return nullptr;
	}
	
	inline MouseJoint* Create(const MouseJointDef& def, BlockAllocator& allocator)
	{
		if (MouseJoint::IsOkay(static_cast<const MouseJointDef&>(def)))
		{
			void* mem = allocator.Allocate(sizeof(MouseJoint));
			return new (mem) MouseJoint(def);
		}
		return nullptr;
	}
	
	inline PrismaticJoint* Create(const PrismaticJointDef& def, BlockAllocator& allocator)
	{
		if (PrismaticJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(PrismaticJoint));
			return new (mem) PrismaticJoint(def);
		}
		return nullptr;
	}
	
	inline RevoluteJoint* Create(const RevoluteJointDef& def, BlockAllocator& allocator)
	{
		if (RevoluteJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(RevoluteJoint));
			return new (mem) RevoluteJoint(def);
		}
		return nullptr;
	}
	
	inline PulleyJoint* Create(const PulleyJointDef& def, BlockAllocator& allocator)
	{
		if (PulleyJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(PulleyJoint));
			return new (mem) PulleyJoint(def);
		}
		return nullptr;
	}
	
	inline GearJoint* Create(const GearJointDef& def, BlockAllocator& allocator)
	{
		if (GearJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(GearJoint));
			return new (mem) GearJoint(def);
		}
		return nullptr;
	}
	
	inline WheelJoint* Create(const WheelJointDef& def, BlockAllocator& allocator)
	{
		if (WheelJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(WheelJoint));
			return new (mem) WheelJoint(def);
		}
		return nullptr;
	}
	
	inline WeldJoint* Create(const WeldJointDef& def, BlockAllocator& allocator)
	{
		if (WeldJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(WeldJoint));
			return new (mem) WeldJoint(def);
		}
		return nullptr;
	}
	
	inline FrictionJoint* Create(const FrictionJointDef& def, BlockAllocator& allocator)
	{
		if (FrictionJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(FrictionJoint));
			return new (mem) FrictionJoint(def);
		}
		return nullptr;
	}
	
	inline RopeJoint* Create(const RopeJointDef& def, BlockAllocator& allocator)
	{
		if (RopeJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(RopeJoint));
			return new (mem) RopeJoint(def);
		}
		return nullptr;
	}
	
	inline MotorJoint* Create(const MotorJointDef& def, BlockAllocator& allocator)
	{
		if (MotorJoint::IsOkay(def))
		{
			void* mem = allocator.Allocate(sizeof(MotorJoint));
			return new (mem) MotorJoint(def);
		}
		return nullptr;
	}
	
} // anonymous namespace
	
Joint* Joint::Create(const JointDef& def, BlockAllocator& allocator)
{
	switch (def.type)
	{
	case JointType::Distance:
		return box2d::Create(static_cast<const DistanceJointDef&>(def), allocator);
	case JointType::Mouse:
		return box2d::Create(static_cast<const MouseJointDef&>(def), allocator);
	case JointType::Prismatic:
		return box2d::Create(static_cast<const PrismaticJointDef&>(def), allocator);
	case JointType::Revolute:
		return box2d::Create(static_cast<const RevoluteJointDef&>(def), allocator);
	case JointType::Pulley:
		return box2d::Create(static_cast<const PulleyJointDef&>(def), allocator);
	case JointType::Gear:
		return box2d::Create(static_cast<const GearJointDef&>(def), allocator);
	case JointType::Wheel:
		return box2d::Create(static_cast<const WheelJointDef&>(def), allocator);
	case JointType::Weld:
		return box2d::Create(static_cast<const WeldJointDef&>(def), allocator);
	case JointType::Friction:
		return box2d::Create(static_cast<const FrictionJointDef&>(def), allocator);
	case JointType::Rope:
		return box2d::Create(static_cast<const RopeJointDef&>(def), allocator);
	case JointType::Motor:
		return box2d::Create(static_cast<const MotorJointDef&>(def), allocator);
	case JointType::Unknown:
		assert(false);
		break;
	default:
		break;
	}
	return nullptr;
}

void Joint::Destroy(Joint* joint, BlockAllocator& allocator)
{
	joint->~Joint();
	switch (joint->m_type)
	{
	case JointType::Distance:
		allocator.Free(joint, sizeof(DistanceJoint));
		break;

	case JointType::Mouse:
		allocator.Free(joint, sizeof(MouseJoint));
		break;

	case JointType::Prismatic:
		allocator.Free(joint, sizeof(PrismaticJoint));
		break;

	case JointType::Revolute:
		allocator.Free(joint, sizeof(RevoluteJoint));
		break;

	case JointType::Pulley:
		allocator.Free(joint, sizeof(PulleyJoint));
		break;

	case JointType::Gear:
		allocator.Free(joint, sizeof(GearJoint));
		break;

	case JointType::Wheel:
		allocator.Free(joint, sizeof(WheelJoint));
		break;
    
	case JointType::Weld:
		allocator.Free(joint, sizeof(WeldJoint));
		break;

	case JointType::Friction:
		allocator.Free(joint, sizeof(FrictionJoint));
		break;

	case JointType::Rope:
		allocator.Free(joint, sizeof(RopeJoint));
		break;

	case JointType::Motor:
		allocator.Free(joint, sizeof(MotorJoint));
		break;

	case JointType::Unknown:
		assert(false);
		break;
	}
}

bool Joint::IsOkay(const JointDef& def) noexcept
{
	if (def.bodyA == def.bodyB)
	{
		return false;
	}
	return true;
}

Joint::Joint(const JointDef& def):
	m_type{def.type}, m_bodyA{def.bodyA}, m_bodyB{def.bodyB},
	m_collideConnected{def.collideConnected}, m_userData{def.userData}
{
	// Intentionally empty.
}

bool IsEnabled(const Joint& j) noexcept
{
	return j.GetBodyA()->IsEnabled() && j.GetBodyB()->IsEnabled();
}

void SetAwake(Joint& j) noexcept
{
	j.GetBodyA()->SetAwake();
	j.GetBodyB()->SetAwake();
}

size_t GetWorldIndex(const Joint* joint)
{
	if (joint)
	{
		const auto bA = joint->GetBodyA();
		const auto bB = joint->GetBodyB();
		const auto world = bA? bA->GetWorld(): bB? bB->GetWorld(): static_cast<const World*>(nullptr);
		if (world)
		{
			auto i = size_t{0};
			for (auto&& j: world->GetJoints())
			{
				if (j == joint)
				{
					return i;
				}
				++i;
			}
		}
	}
	return size_t(-1);
}

} // namespace box2d
