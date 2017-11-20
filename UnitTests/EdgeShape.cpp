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
#include <PlayRho/Collision/Shapes/EdgeShape.hpp>
#include <PlayRho/Collision/Shapes/ShapeVisitor.hpp>

using namespace playrho;

TEST(EdgeShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(EdgeShape), std::size_t(52));
#else
            EXPECT_EQ(sizeof(EdgeShape), std::size_t(56));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(EdgeShape), std::size_t(104)); break;
        case 16: EXPECT_EQ(sizeof(EdgeShape), std::size_t(208)); break;
        default: FAIL(); break;
    }
}

TEST(EdgeShape, GetInvalidChildThrows)
{
    EdgeShape foo{};
    
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_NO_THROW(foo.GetChild(0));
    EXPECT_THROW(foo.GetChild(1), InvalidArgument);
}

TEST(EdgeShape, Accept)
{
    class Visitor: public IsVisitedShapeVisitor
    {
    public:
        void Visit(const EdgeShape&) override
        {
            visited = true;
        }
        bool visited = false;
    };

    EdgeShape foo{};
    Visitor v;
    ASSERT_FALSE(v.visited);
    ASSERT_FALSE(v.IsVisited());
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
    EXPECT_FALSE(v.IsVisited());
}

TEST(EdgeShape, BaseVisitorForDiskShape)
{
    const auto shape = EdgeShape{};
    auto visitor = IsVisitedShapeVisitor{};
    ASSERT_FALSE(visitor.IsVisited());
    shape.Accept(visitor);
    EXPECT_TRUE(visitor.IsVisited());
}
