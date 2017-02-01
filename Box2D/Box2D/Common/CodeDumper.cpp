/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "CodeDumper.hpp"

#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>
#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Dynamics/Joints/FrictionJoint.hpp>
#include <Box2D/Dynamics/Joints/MotorJoint.hpp>
#include <Box2D/Dynamics/Joints/WeldJoint.hpp>
#include <Box2D/Dynamics/Joints/MouseJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Joints/GearJoint.hpp>
#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Dynamics/Joints/WheelJoint.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>

using namespace box2d;

namespace
{
	// You can modify this to use your logging facility.
	void log(const char* string, ...)
	{
		va_list args;
		va_start(args, string);
		std::vprintf(string, args);
		va_end(args);
	}
}

void box2d::Dump(const World& world)
{
	const auto gravity = world.GetGravity();
	log("Vec2 g(%.15lef, %.15lef);\n", gravity.x, gravity.y);
	log("m_world->SetGravity(g);\n");
	
	const auto& bodies = world.GetBodies();
	log("Body** bodies = (Body**)alloc(%d * sizeof(Body*));\n", bodies.size());
	auto i = size_t{0};
	for (auto&& b: bodies)
	{
		Dump(b, i);
		++i;
	}
	
	const auto& joints = world.GetJoints();
	log("Joint** joints = (Joint**)alloc(%d * sizeof(Joint*));\n", joints.size());
	i = 0;
	for (auto&& j: joints)
	{
		log("{\n");
		Dump(j, i);
		log("}\n");
		++i;
	}
	
	log("free(joints);\n");
	log("free(bodies);\n");
	log("joints = nullptr;\n");
	log("bodies = nullptr;\n");
}

void box2d::Dump(const Body& body, size_t bodyIndex)
{
	log("{\n");
	log("  BodyDef bd;\n");
	log("  bd.type = BodyType(%d);\n", body.GetType());
	log("  bd.position = Vec2(%.15lef, %.15lef);\n", body.GetLocation().x, body.GetLocation().y);
	log("  bd.angle = %.15lef;\n", body.GetAngle());
	log("  bd.linearVelocity = Vec2(%.15lef, %.15lef);\n", body.GetVelocity().linear.x, body.GetVelocity().linear.y);
	log("  bd.angularVelocity = %.15lef;\n", body.GetVelocity().angular);
	log("  bd.linearDamping = %.15lef;\n", body.GetLinearDamping());
	log("  bd.angularDamping = %.15lef;\n", body.GetAngularDamping());
	log("  bd.allowSleep = bool(%d);\n", body.IsSleepingAllowed());
	log("  bd.awake = bool(%d);\n", body.IsAwake());
	log("  bd.fixedRotation = bool(%d);\n", body.IsFixedRotation());
	log("  bd.bullet = bool(%d);\n", body.IsImpenetrable());
	log("  bd.active = bool(%d);\n", body.IsActive());
	log("  bodies[%d] = m_world->CreateBody(bd);\n", bodyIndex);
	log("\n");
	for (auto&& fixture: body.GetFixtures())
	{
		log("  {\n");
		Dump(fixture, bodyIndex);
		log("  }\n");
	}
	log("}\n");
}

void box2d::Dump(const Joint& joint, size_t index)
{
	switch (joint.GetType())
	{
		case JointType::Pulley:
			Dump(static_cast<const PulleyJoint&>(joint), index);
			break;
		case JointType::Distance:
			Dump(static_cast<const DistanceJoint&>(joint), index);
			break;
		case JointType::Friction:
			Dump(static_cast<const FrictionJoint&>(joint), index);
			break;
		case JointType::Motor:
			Dump(static_cast<const MotorJoint&>(joint), index);
			break;
		case JointType::Weld:
			Dump(static_cast<const WeldJoint&>(joint), index);
			break;
		case JointType::Mouse:
			Dump(static_cast<const MouseJoint&>(joint), index);
			break;
		case JointType::Revolute:
			Dump(static_cast<const RevoluteJoint&>(joint), index);
			break;
		case JointType::Prismatic:
			Dump(static_cast<const PrismaticJoint&>(joint), index);
			break;
		case JointType::Gear:
			Dump(static_cast<const GearJoint&>(joint), index);
			break;
		case JointType::Rope:
			Dump(static_cast<const RopeJoint&>(joint), index);
			break;
		case JointType::Wheel:
			Dump(static_cast<const WheelJoint&>(joint), index);
			break;
		case JointType::Unknown:
			assert(false);
			break;
	}
}

void box2d::Dump(const Fixture& fixture, size_t bodyIndex)
{
	log("    FixtureDef fd;\n");
	log("    fd.friction = %.15lef;\n", fixture.GetFriction());
	log("    fd.restitution = %.15lef;\n", fixture.GetRestitution());
	log("    fd.density = %.15lef;\n", fixture.GetDensity());
	log("    fd.isSensor = bool(%d);\n", fixture.IsSensor());
	log("    fd.filter.categoryBits = uint16(%d);\n", fixture.GetFilterData().categoryBits);
	log("    fd.filter.maskBits = uint16(%d);\n", fixture.GetFilterData().maskBits);
	log("    fd.filter.groupIndex = int16(%d);\n", fixture.GetFilterData().groupIndex);
	
	switch (fixture.GetShape()->GetType())
	{
		case Shape::e_circle:
		{
			auto s = static_cast<const CircleShape*>(fixture.GetShape());
			log("    CircleShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", s->GetRadius());
			log("    shape.m_p = Vec2(%.15lef, %.15lef);\n", s->GetLocation().x, s->GetLocation().y);
		}
			break;
			
		case Shape::e_edge:
		{
			auto s = static_cast<const EdgeShape*>(fixture.GetShape());
			log("    EdgeShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", GetVertexRadius(*s));
			log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s->GetVertex0().x, s->GetVertex0().y);
			log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s->GetVertex1().x, s->GetVertex1().y);
			log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s->GetVertex2().x, s->GetVertex2().y);
			log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s->GetVertex3().x, s->GetVertex3().y);
			log("    shape.m_hasVertex0 = bool(%d);\n", s->HasVertex0());
			log("    shape.m_hasVertex3 = bool(%d);\n", s->HasVertex3());
		}
			break;
			
		case Shape::e_polygon:
		{
			const auto s = static_cast<const PolygonShape*>(fixture.GetShape());
			const auto vertexCount = s->GetVertexCount();
			log("    PolygonShape shape;\n");
			log("    Vec2 vs[%d];\n", vertexCount);
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->GetVertex(i).x, s->GetVertex(i).y);
			}
			log("    shape.Set(vs, %d);\n", vertexCount);
		}
			break;
			
		case Shape::e_chain:
		{
			auto s = static_cast<const ChainShape*>(fixture.GetShape());
			log("    ChainShape shape;\n");
			log("    Vec2 vs[%d];\n", s->GetVertexCount());
			for (auto i = decltype(s->GetVertexCount()){0}; i < s->GetVertexCount(); ++i)
			{
				log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->GetVertex(i).x, s->GetVertex(i).y);
			}
			log("    shape.CreateChain(vs, %d);\n", s->GetVertexCount());
		}
			break;
			
		default:
			return;
	}
	
	log("\n");
	log("    fd.shape = &shape;\n");
	log("\n");
	log("    bodies[%d]->CreateFixture(fd);\n", bodyIndex);
}

void box2d::Dump(const DistanceJoint& joint, size_t index)
{
	log("  DistanceJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.length = %.15lef;\n", joint.GetLength());
	log("  jd.frequencyHz = %.15lef;\n", joint.GetFrequency());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const FrictionJoint& joint, size_t index)
{
	log("  FrictionJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.maxForce = %.15lef;\n", joint.GetMaxForce());
	log("  jd.maxTorque = %.15lef;\n", joint.GetMaxTorque());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const GearJoint& joint, size_t index)
{
	log("  GearJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.joint1 = joints[%d];\n", GetWorldIndex(joint.GetJoint1()));
	log("  jd.joint2 = joints[%d];\n", GetWorldIndex(joint.GetJoint2()));
	log("  jd.ratio = %.15lef;\n", joint.GetRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const MotorJoint& joint, size_t index)
{
	log("  MotorJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.linearOffset = Vec2(%.15lef, %.15lef);\n", joint.GetLinearOffset().x, joint.GetLinearOffset().y);
	log("  jd.angularOffset = %.15lef;\n", joint.GetAngularOffset());
	log("  jd.maxForce = %.15lef;\n", joint.GetMaxForce());
	log("  jd.maxTorque = %.15lef;\n", joint.GetMaxTorque());
	log("  jd.correctionFactor = %.15lef;\n", joint.GetCorrectionFactor());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const MouseJoint& joint, size_t index)
{
	log("  MouseJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.frequencyHz = %.15lef;\n", joint.GetFrequency());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
	log("  jd.maxForce = %.15lef;\n", joint.GetMaxForce());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const PrismaticJoint& joint, size_t index)
{
	log("  PrismaticJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.localAxisA = Vec2(%.15lef, %.15lef);\n", GetX(joint.GetLocalAxisA()), GetY(joint.GetLocalAxisA()));
	log("  jd.referenceAngle = %.15lef;\n", joint.GetReferenceAngle());
	log("  jd.enableLimit = bool(%d);\n", joint.IsLimitEnabled());
	log("  jd.lowerTranslation = %.15lef;\n", joint.GetLowerLimit());
	log("  jd.upperTranslation = %.15lef;\n", joint.GetUpperLimit());
	log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
	log("  jd.motorSpeed = %.15lef;\n", joint.GetMotorSpeed());
	log("  jd.maxMotorForce = %.15lef;\n", joint.GetMaxMotorForce());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const PulleyJoint& joint, size_t index)
{
	log("  PulleyJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.groundAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetGroundAnchorA().x, joint.GetGroundAnchorA().y);
	log("  jd.groundAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetGroundAnchorB().x, joint.GetGroundAnchorB().y);
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x,  joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",  joint.GetLocalAnchorB().x,  joint.GetLocalAnchorB().y);
	log("  jd.lengthA = %.15lef;\n", joint.GetLengthA());
	log("  jd.lengthB = %.15lef;\n", joint.GetLengthB());
	log("  jd.ratio = %.15lef;\n", joint.GetRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const RevoluteJoint& joint, size_t index)
{
	log("  RevoluteJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.referenceAngle = %.15lef;\n", joint.GetReferenceAngle());
	log("  jd.enableLimit = bool(%d);\n", joint.IsLimitEnabled());
	log("  jd.lowerAngle = %.15lef;\n", joint.GetLowerLimit());
	log("  jd.upperAngle = %.15lef;\n", joint.GetUpperLimit());
	log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
	log("  jd.motorSpeed = %.15lef;\n", joint.GetMotorSpeed());
	log("  jd.maxMotorTorque = %.15lef;\n", joint.GetMaxMotorTorque());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const RopeJoint& joint, size_t index)
{
	log("  RopeJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.maxLength = %.15lef;\n", joint.GetMaxLength());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const WeldJoint& joint, size_t index)
{
	log("  WeldJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.referenceAngle = %.15lef;\n", joint.GetReferenceAngle());
	log("  jd.frequencyHz = %.15lef;\n", joint.GetFrequency());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const WheelJoint& joint, size_t index)
{
	log("  WheelJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.localAxisA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAxisA().x, joint.GetLocalAxisA().y);
	log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
	log("  jd.motorSpeed = %.15lef;\n", joint.GetMotorSpeed());
	log("  jd.maxMotorTorque = %.15lef;\n", joint.GetMaxMotorTorque());
	log("  jd.frequencyHz = %.15lef;\n", joint.GetSpringFrequencyHz());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetSpringDampingRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
