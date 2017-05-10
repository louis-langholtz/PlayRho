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

#include <Box2D/Common/CodeDumper.hpp>

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

#include <cstdarg>

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
    
    struct ShapeDumper: public Shape::Visitor
    {
    public:
        void Visit(const CircleShape& shape) override;
        void Visit(const EdgeShape& shape) override;
        void Visit(const PolygonShape& shape) override;
        void Visit(const ChainShape& shape) override;
        void Visit(const MultiShape& shape) override;
    };
    
    void ShapeDumper::Visit(const box2d::CircleShape& s)
    {
        log("    CircleShape shape;\n");
        log("    shape.m_radius = %.15lef;\n", static_cast<double>(StripUnit(s.GetRadius())));
        log("    shape.m_p = Vec2(%.15lef, %.15lef);\n",
            static_cast<double>(StripUnit(s.GetLocation().x)),
            static_cast<double>(StripUnit(s.GetLocation().y)));
    }
    
    void ShapeDumper::Visit(const box2d::EdgeShape& s)
    {
        log("    EdgeShape shape;\n");
        log("    shape.m_radius = %.15lef;\n", static_cast<double>(StripUnit(GetVertexRadius(s))));
        log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n",
            static_cast<double>(StripUnit(s.GetVertex1().x)),
            static_cast<double>(StripUnit(s.GetVertex1().y)));
        log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n",
            static_cast<double>(StripUnit(s.GetVertex2().x)),
            static_cast<double>(StripUnit(s.GetVertex2().y)));
    }
    
    void ShapeDumper::Visit(const box2d::PolygonShape& s)
    {
        const auto vertexCount = s.GetVertexCount();
        log("    PolygonShape shape;\n");
        log("    Vec2 vs[%d];\n", vertexCount);
        for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
        {
            log("    vs[%d].Set(%.15lef, %.15lef);\n", i,
                static_cast<double>(StripUnit(s.GetVertex(i).x)), static_cast<double>(StripUnit(s.GetVertex(i).y)));
        }
        log("    shape.Set(vs, %d);\n", vertexCount);
    }
    
    void ShapeDumper::Visit(const box2d::ChainShape& s)
    {
        log("    ChainShape shape;\n");
        log("    Vec2 vs[%d];\n", s.GetVertexCount());
        for (auto i = decltype(s.GetVertexCount()){0}; i < s.GetVertexCount(); ++i)
        {
            log("    vs[%d].Set(%.15lef, %.15lef);\n", i,
                static_cast<double>(StripUnit(s.GetVertex(i).x)),
                static_cast<double>(StripUnit(s.GetVertex(i).y)));
        }
        log("    shape.CreateChain(vs, %d);\n", s.GetVertexCount());
    }

    void ShapeDumper::Visit(const box2d::MultiShape&)
    {
        // TODO
    }
}

void box2d::Dump(const World& world)
{
    const auto gravity = world.GetGravity();
    log("Vec2 g(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{gravity.x / MeterPerSquareSecond}),
        static_cast<double>(RealNum{gravity.y / MeterPerSquareSecond}));
    log("m_world->SetGravity(g);\n");
    
    const auto& bodies = world.GetBodies();
    log("Body** bodies = (Body**)alloc(%d * sizeof(Body*));\n", bodies.size());
    auto i = size_t{0};
    for (auto&& b: bodies)
    {
        Dump(*b, i);
        ++i;
    }
    
    const auto& joints = world.GetJoints();
    log("Joint** joints = (Joint**)alloc(%d * sizeof(Joint*));\n", joints.size());
    i = 0;
    for (auto&& j: joints)
    {
        log("{\n");
        Dump(*j, i);
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
    log("  bd.position = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(StripUnits(body.GetLocation()).x),
        static_cast<double>(StripUnits(body.GetLocation()).y));
    log("  bd.angle = %.15lef;\n", static_cast<double>(RealNum{body.GetAngle() / Radian}));
    log("  bd.linearVelocity = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{body.GetVelocity().linear.x / MeterPerSecond}),
        static_cast<double>(RealNum{body.GetVelocity().linear.y / MeterPerSecond}));
    log("  bd.angularVelocity = %.15lef;\n",
        static_cast<double>(RealNum{body.GetVelocity().angular / RadianPerSecond}));
    log("  bd.linearDamping = %.15lef;\n", static_cast<double>(body.GetLinearDamping()));
    log("  bd.angularDamping = %.15lef;\n", static_cast<double>(body.GetAngularDamping()));
    log("  bd.allowSleep = bool(%d);\n", body.IsSleepingAllowed());
    log("  bd.awake = bool(%d);\n", body.IsAwake());
    log("  bd.fixedRotation = bool(%d);\n", body.IsFixedRotation());
    log("  bd.bullet = bool(%d);\n", body.IsImpenetrable());
    log("  bd.enabled = bool(%d);\n", body.IsEnabled());
    log("  bodies[%d] = m_world->CreateBody(bd);\n", bodyIndex);
    log("\n");
    for (auto&& fixture: body.GetFixtures())
    {
        log("  {\n");
        Dump(*fixture, bodyIndex);
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
    log("    fd.friction = %.15lef;\n", static_cast<double>(fixture.GetFriction()));
    log("    fd.restitution = %.15lef;\n", static_cast<double>(fixture.GetRestitution()));
    log("    fd.density = %.15lef;\n",
        static_cast<double>(RealNum{fixture.GetDensity() * SquareMeter / Kilogram}));
    log("    fd.isSensor = bool(%d);\n", fixture.IsSensor());
    log("    fd.filter.categoryBits = Filter::bits_type(%u);\n",
        fixture.GetFilterData().categoryBits);
    log("    fd.filter.maskBits = Filter::bits_type(%u);\n",
        fixture.GetFilterData().maskBits);
    log("    fd.filter.groupIndex = Filter::index_type(%d);\n",
        fixture.GetFilterData().groupIndex);
    
    const auto shape = fixture.GetShape();
    ShapeDumper shapeDumper;
    shape->Accept(shapeDumper);
    
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
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.length = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetLength() / Meter}));
    log("  jd.frequencyHz = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetFrequency() / Hertz}));
    log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const FrictionJoint& joint, size_t index)
{
    log("  FrictionJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.maxForce = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxForce() / Newton}));
    log("  jd.maxTorque = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxTorque() / NewtonMeter}));
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
    log("  jd.linearOffset = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLinearOffset().x / Meter}),
        static_cast<double>(RealNum{joint.GetLinearOffset().y / Meter}));
    log("  jd.angularOffset = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetAngularOffset() / Radian}));
    log("  jd.maxForce = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxForce() / Newton}));
    log("  jd.maxTorque = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxTorque() / NewtonMeter}));
    log("  jd.correctionFactor = %.15lef;\n", joint.GetCorrectionFactor());
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const MouseJoint& joint, size_t index)
{
    log("  MouseJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.frequencyHz = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetFrequency() / Hertz}));
    log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
    log("  jd.maxForce = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxForce() / Newton}));
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const PrismaticJoint& joint, size_t index)
{
    log("  PrismaticJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.localAxisA = Vec2(%.15lef, %.15lef);\n",
        GetX(joint.GetLocalAxisA()), GetY(joint.GetLocalAxisA()));
    log("  jd.referenceAngle = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetReferenceAngle() / Radian}));
    log("  jd.enableLimit = bool(%d);\n", joint.IsLimitEnabled());
    log("  jd.lowerTranslation = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetLowerLimit() / Meter}));
    log("  jd.upperTranslation = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetUpperLimit() / Meter}));
    log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
    log("  jd.motorSpeed = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMotorSpeed() / RadianPerSecond}));
    log("  jd.maxMotorForce = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxMotorForce() / Newton}));
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const PulleyJoint& joint, size_t index)
{
    log("  PulleyJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.groundAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetGroundAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetGroundAnchorA().y / Meter}));
    log("  jd.groundAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetGroundAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetGroundAnchorB().y / Meter}));
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.lengthA = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetLengthA() / Meter}));
    log("  jd.lengthB = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetLengthB() / Meter}));
    log("  jd.ratio = %.15lef;\n", joint.GetRatio());
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const RevoluteJoint& joint, size_t index)
{
    log("  RevoluteJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.referenceAngle = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetReferenceAngle() / Radian}));
    log("  jd.enableLimit = bool(%d);\n", joint.IsLimitEnabled());
    log("  jd.lowerAngle = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetLowerLimit() / Radian}));
    log("  jd.upperAngle = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetUpperLimit() / Radian}));
    log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
    log("  jd.motorSpeed = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMotorSpeed() / RadianPerSecond}));
    log("  jd.maxMotorTorque = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxMotorTorque() / NewtonMeter}));
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const RopeJoint& joint, size_t index)
{
    log("  RopeJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.maxLength = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxLength() / Meter}));
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const WeldJoint& joint, size_t index)
{
    log("  WeldJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.referenceAngle = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetReferenceAngle() / Radian}));
    log("  jd.frequencyHz = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetFrequency() / Hertz}));
    log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

void box2d::Dump(const WheelJoint& joint, size_t index)
{
    log("  WheelJointDef jd;\n");
    log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
    log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
    log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
    log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorA().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorA().y / Meter}));
    log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(RealNum{joint.GetLocalAnchorB().x / Meter}),
        static_cast<double>(RealNum{joint.GetLocalAnchorB().y / Meter}));
    log("  jd.localAxisA = Vec2(%.15lef, %.15lef);\n",
        static_cast<double>(GetX(joint.GetLocalAxisA())),
        static_cast<double>(GetY(joint.GetLocalAxisA())));
    log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
    log("  jd.motorSpeed = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMotorSpeed() / RadianPerSecond}));
    log("  jd.maxMotorTorque = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetMaxMotorTorque() / NewtonMeter}));
    log("  jd.frequencyHz = %.15lef;\n",
        static_cast<double>(RealNum{joint.GetSpringFrequencyHz() / Hertz}));
    log("  jd.dampingRatio = %.15lef;\n", joint.GetSpringDampingRatio());
    log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
