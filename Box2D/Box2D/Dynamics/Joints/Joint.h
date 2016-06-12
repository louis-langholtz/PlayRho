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

#ifndef B2_JOINT_H
#define B2_JOINT_H

#include <Box2D/Common/Math.h>

namespace box2d {

class Body;
class Joint;
struct SolverData;
class BlockAllocator;

enum JointType
{
	e_unknownJoint,
	e_revoluteJoint,
	e_prismaticJoint,
	e_distanceJoint,
	e_pulleyJoint,
	e_mouseJoint,
	e_gearJoint,
	e_wheelJoint,
    e_weldJoint,
	e_frictionJoint,
	e_ropeJoint,
	e_motorJoint
};

enum LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

struct Jacobian
{
	Vec2 linear;
	float_t angularA;
	float_t angularB;
};

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
struct JointEdge
{
	Body* other;			///< provides quick access to the other body attached.
	Joint* joint;			///< the joint
	JointEdge* prev;		///< the previous joint edge in the body's joint list
	JointEdge* next;		///< the next joint edge in the body's joint list
};

/// Joint definitions are used to construct joints.
struct JointDef
{
	JointDef() = delete;

	constexpr JointDef(JointType t) noexcept: type(t) {}
	constexpr JointDef(JointType t, Body* bA, Body* bB) noexcept:
		type(t), bodyA(bA), bodyB(bB) {}

	/// The joint type is set automatically for concrete joint types.
	const JointType type;

	/// Use this to attach application specific data to your joints.
	void* userData = nullptr;

	/// The first attached body.
	Body* bodyA = nullptr;

	/// The second attached body.
	Body* bodyB = nullptr;

	/// Set this flag to true if the attached bodies should collide.
	bool collideConnected = false;
};

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
class Joint
{
public:
	using index_t = size_t;

	/// Get the type of the concrete joint.
	JointType GetType() const noexcept;

	/// Get the first body attached to this joint.
	Body* GetBodyA() noexcept;

	/// Get the second body attached to this joint.
	Body* GetBodyB() noexcept;

	/// Get the anchor point on bodyA in world coordinates.
	virtual Vec2 GetAnchorA() const = 0;

	/// Get the anchor point on bodyB in world coordinates.
	virtual Vec2 GetAnchorB() const = 0;

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	virtual Vec2 GetReactionForce(float_t inv_dt) const = 0;

	/// Get the reaction torque on bodyB in N*m.
	virtual float_t GetReactionTorque(float_t inv_dt) const = 0;

	/// Get the next joint the world joint list.
	Joint* GetNext() noexcept;
	const Joint* GetNext() const noexcept;

	/// Get the user data pointer.
	void* GetUserData() const noexcept;

	/// Set the user data pointer.
	void SetUserData(void* data) noexcept;

	/// Short-cut function to determine if either body is inactive.
	bool IsActive() const noexcept;

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	bool GetCollideConnected() const noexcept;

	/// Dump this joint to the log file.
	virtual void Dump() { log("// Dump is not supported for this joint type.\n"); }

	/// Shift the origin for any points stored in world coordinates.
	virtual void ShiftOrigin(const Vec2& newOrigin) { BOX2D_NOT_USED(newOrigin);  }

protected:
	friend class World;
	friend class Body;
	friend class Island;
	friend class GearJoint;

	static Joint* Create(const JointDef& def, BlockAllocator* allocator);
	static void Destroy(Joint* joint, BlockAllocator* allocator);

	Joint(const JointDef& def);
	virtual ~Joint() {}

	virtual void InitVelocityConstraints(const SolverData& data) = 0;
	virtual void SolveVelocityConstraints(const SolverData& data) = 0;

	// This returns true if the position errors are within tolerance.
	virtual bool SolvePositionConstraints(const SolverData& data) = 0;

	bool IsInIsland() const noexcept;
	void SetInIsland(bool value) noexcept;

	const JointType m_type;
	Joint* m_prev = nullptr;
	Joint* m_next = nullptr;
	JointEdge m_edgeA = {nullptr, nullptr, nullptr, nullptr};
	JointEdge m_edgeB = {nullptr, nullptr, nullptr, nullptr};
	Body* m_bodyA;
	Body* m_bodyB;

	index_t m_index = 0;

	bool m_islandFlag = false;
	bool m_collideConnected;

	void* m_userData;
};

inline JointType Joint::GetType() const noexcept
{
	return m_type;
}

inline Body* Joint::GetBodyA() noexcept
{
	return m_bodyA;
}

inline Body* Joint::GetBodyB() noexcept
{
	return m_bodyB;
}

inline Joint* Joint::GetNext() noexcept
{
	return m_next;
}

inline const Joint* Joint::GetNext() const noexcept
{
	return m_next;
}

inline void* Joint::GetUserData() const noexcept
{
	return m_userData;
}

inline void Joint::SetUserData(void* data) noexcept
{
	m_userData = data;
}

inline bool Joint::GetCollideConnected() const noexcept
{
	return m_collideConnected;
}

inline bool Joint::IsInIsland() const noexcept
{
	return m_islandFlag;
}

inline void Joint::SetInIsland(bool value) noexcept
{
	m_islandFlag = value;
}

// Wakes up the joined bodies.
void SetAwake(Joint& j) noexcept;

} // namespace box2d

#endif
