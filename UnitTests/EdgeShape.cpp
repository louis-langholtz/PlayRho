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

#include "gtest/gtest.h"
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

using namespace playrho;

TEST(EdgeShapeConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(52));
#else
            EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(48));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(96)); break;
        case 16: EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(192)); break;
        default: FAIL(); break;
    }
}

TEST(EdgeShapeConf, GetInvalidChildThrows)
{
    const auto foo = EdgeShapeConf{};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_NO_THROW(GetChild(foo, 0));
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(EdgeShapeConf, Accept)
{
    auto visited = false;
    auto shapeVisited = false;
    const auto foo = EdgeShapeConf{};
    ASSERT_FALSE(visited);
    ASSERT_FALSE(shapeVisited);
    Accept(Shape(foo), [&](const std::type_info& ti, const void*) {
        visited = true;
        if (ti == typeid(EdgeShapeConf))
        {
            shapeVisited = true;
        }
    });
    EXPECT_TRUE(visited);
    EXPECT_TRUE(shapeVisited);
}

#if 0
TEST(EdgeShapeConf, BaseVisitorForDiskShape)
{
    const auto shape = EdgeShapeConf{};
    auto visitor = IsVisitedShapeVisitor{};
    ASSERT_FALSE(visitor.IsVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsVisited());
}
#endif
