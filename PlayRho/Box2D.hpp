/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef BOX2D_H
#define BOX2D_H

/**
\mainpage Box2D API Documentation

\section intro_sec Getting Started

For user documentation, please see http://box2d.org/documentation.html

For discussion, please visit http://box2d.org/forum
*/

// These include files constitute the main Box2D API

#include <Box2D/Common/Settings.hpp>

#include <Box2D/Collision/Shapes/DiskShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/MultiShape.hpp>

#include <Box2D/Collision/Collision.hpp>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>

#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/WorldCallbacks.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/World.hpp>

#include <Box2D/Dynamics/Contacts/Contact.hpp>

#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Dynamics/Joints/FrictionJoint.hpp>
#include <Box2D/Dynamics/Joints/GearJoint.hpp>
#include <Box2D/Dynamics/Joints/MotorJoint.hpp>
#include <Box2D/Dynamics/Joints/MouseJoint.hpp>
#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Dynamics/Joints/WeldJoint.hpp>
#include <Box2D/Dynamics/Joints/WheelJoint.hpp>

#endif
