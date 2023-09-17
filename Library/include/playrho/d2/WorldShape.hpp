/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_D2_WORLDSHAPE_HPP
#define PLAYRHO_D2_WORLDSHAPE_HPP

/// @file
/// Declarations of free functions of World for shapes identified by <code>ShapeID</code>.
/// @details This is a collection of non-member non-friend functions - also called "free"
///   functions - that are related to shapes within an instance of a <code>World</code>.
///   Many are just "wrappers" to similarly named member functions but some are additional
///   functionality built on those member functions. A benefit to using free functions that
///   are now just wrappers, is that of helping to isolate your code from future changes that
///   might occur to the underlying <code>World</code> member functions. Free functions in
///   this sense are "cheap" abstractions. While using these incurs extra run-time overhead
///   when compiled without any compiler optimizations enabled, enabling optimizations
///   should entirely eliminate that overhead.
/// @note The four basic categories of these functions are "CRUD": create, read, update,
///   and delete.
/// @see World, ShapeID.
/// @see https://en.wikipedia.org/wiki/Create,_read,_update_and_delete.

#include <iterator>
#include <vector>

#include <playrho/ShapeID.hpp>
#include <playrho/Span.hpp>

#include <playrho/d2/MassData.hpp>
#include <playrho/d2/Math.hpp>
#include <playrho/d2/Shape.hpp>

namespace playrho {

struct Filter;

namespace d2 {

class World;

/// @example WorldShape.cpp
/// This is the <code>googletest</code> based unit testing file for the free function
///   interfaces to <code>playrho::d2::World</code> shape member functions and additional
///   functionality.

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_D2_WORLDSHAPE_HPP
