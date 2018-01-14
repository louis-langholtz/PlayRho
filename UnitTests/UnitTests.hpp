/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef UnitTests_hpp
#define UnitTests_hpp

#include "gtest/gtest.h"

#include <PlayRho/Common/Templates.hpp>

struct UnitTestsVisitorData
{
    int visitedShape = 0;
    int visitedDisk = 0;
    int visitedEdge = 0;
    int visitedPolygon = 0;
    int visitedChain = 0;
    int visitedMulti = 0;
};

namespace playrho {
namespace d2 {

struct DiskShapeConf;
class EdgeShapeConf;
class PolygonShapeConf;
class ChainShapeConf;
class MultiShapeConf;

} // namespace d2

// Specialize the template Visit function.
// Note: These should be included by all code (within the UnitTests application)
//   that uses the Shape class in order to avoid U.B. from O.D.R. violations.

template <>
inline bool Visit(const d2::DiskShapeConf&, void* userData)
{
    const auto data = static_cast<UnitTestsVisitorData*>(userData);
    ++(data->visitedDisk);
    return true;
}

template <>
inline bool Visit(const d2::EdgeShapeConf&, void* userData)
{
    const auto data = static_cast<UnitTestsVisitorData*>(userData);
    ++(data->visitedEdge);
    return true;
}

template <>
inline bool Visit(const d2::PolygonShapeConf&, void* userData)
{
    const auto data = static_cast<UnitTestsVisitorData*>(userData);
    ++(data->visitedPolygon);
    return true;
}

template <>
inline bool Visit(const d2::ChainShapeConf&, void* userData)
{
    const auto data = static_cast<UnitTestsVisitorData*>(userData);
    ++(data->visitedChain);
    return true;
}

template <>
inline bool Visit(const d2::MultiShapeConf&, void* userData)
{
    const auto data = static_cast<UnitTestsVisitorData*>(userData);
    ++(data->visitedMulti);
    return true;
}

} // namespace playrho

#endif /* UnitTests_hpp */
