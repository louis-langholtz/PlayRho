/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_PLAYRHO_HPP
#define PLAYRHO_PLAYRHO_HPP

/**
@mainpage PlayRho API Documentation

@section intro_sec Getting Started

For source code, please see: https://github.com/louis-langholtz/PlayRho .

For issues, visit: https://github.com/louis-langholtz/PlayRho/issues .

For coding, simply begin by including the <code>PlayRho/PlayRho.hpp</code> file and
 make an instance of the
 <a href="classplayrho_1_1World.html"><code>playrho::d2::World</code></a> class.
For a more elaborate starting point, see the
 <a href="HelloWorld_8cpp-example.html"><code>HelloWorld.cpp</code></a> example application.

For mathemtical insight, see:
<a href="http://box2d.org/files/GDC2009/GDC2009_Catto_Erin_Solver.ppt">Erin Catto's
2009 Modeling and Solving Constraints slides</a>.

@sa PhysicalEntities
*/

// These include files constitute the main PlayRho API

/// @defgroup TestPointGroup Point Containment Test Functions
/// @brief Collection of functions testing for a point's containment within various objects.

/// @defgroup ExceptionsGroup Library Defined Exceptions
/// @brief Exceptions defined and used by the PlayRho library.
/// @details The PlayRho library defines its own exception classes to recognize errors.
///    These classes are all sub-classed from sub-classes of the C++ Standard Library
///    std::exception class.

/// @namespace std
/// Name space for specializations of the standard library.

/// @namespace playrho
/// Name space for all PlayRho related names.

/// @namespace playrho::d2
/// Name space for 2-dimensionally related PlayRho names.

/// @namespace playrho::detail
/// Name space for internal/detail related PlayRho names.

#include <PlayRho/Common/Settings.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Shapes/ChainShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/MultiShapeConf.hpp>

#include <PlayRho/Collision/Collision.hpp>
#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>

#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/WorldCallbacks.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/World.hpp>

#include <PlayRho/Dynamics/Contacts/Contact.hpp>

#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/Joints/MotorJoint.hpp>
#include <PlayRho/Dynamics/Joints/MouseJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/RopeJoint.hpp>
#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
#include <PlayRho/Dynamics/Joints/WheelJoint.hpp>

#endif // PLAYRHO_PLAYRHO_HPP
