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

#include "UnitTests.hpp"

#include <PlayRho/d2/ShapeConf.hpp>

#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(BaseShapeConf, Traits)
{
    EXPECT_TRUE(std::is_default_constructible_v<BaseShapeConf>);
    EXPECT_TRUE(std::is_copy_constructible_v<BaseShapeConf>);
#ifndef PLAYRHO_USE_BOOST_UNITS
    EXPECT_TRUE(std::is_nothrow_default_constructible_v<BaseShapeConf>);
    EXPECT_TRUE(std::is_nothrow_copy_constructible_v<BaseShapeConf>);
#endif
}

TEST(BaseShapeConf, DefaultConstruction)
{
    EXPECT_EQ(BaseShapeConf().friction, BaseShapeConf::DefaultFriction);
    EXPECT_EQ(BaseShapeConf().restitution, BaseShapeConf::DefaultRestitution);
    EXPECT_EQ(BaseShapeConf().density, BaseShapeConf::DefaultDensity);
    EXPECT_EQ(BaseShapeConf().filter, BaseShapeConf::DefaultFilter);
    EXPECT_EQ(BaseShapeConf().isSensor, BaseShapeConf::DefaultIsSensor);
}
