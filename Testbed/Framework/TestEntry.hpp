/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_TEST_ENTRY_HPP
#define PLAYRHO_TEST_ENTRY_HPP

#include <PlayRho/Span.hpp>

#include <memory> // for std::unique_ptr
#include <vector>

namespace testbed {

class Test;

/// @brief A name and function pointer dataset for a test entry.
///
struct TestEntry
{
    typedef std::unique_ptr<Test> CreateFcn();

    const char *name;
    CreateFcn *createFcn;
};

/// @brief Gets the test entries array.
///
/// @note This serves as a wrapper to avoid any possible startup-time dependencies issues that
///   might be caused by having global data defined in a different file than it's used in.
///
std::vector<TestEntry> GetTestEntries();

} // namespace testbed

#endif /* PLAYRHO_TEST_ENTRY_HPP */
