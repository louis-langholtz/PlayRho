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
#include <PlayRho/Collision/Shapes/MultiShape.hpp>
#include <array>

using namespace playrho;

TEST(MultiShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(MultiShape), std::size_t(56)); break;
        case  8: EXPECT_EQ(sizeof(MultiShape), std::size_t(80)); break;
        case 16: EXPECT_EQ(sizeof(MultiShape), std::size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(MultiShape, DefaultConstruction)
{
    MultiShape foo{};
    const auto defaultMassData = MassData{};
    const auto defaultConf = MultiShape::Conf{};
    
    EXPECT_EQ(typeid(foo), typeid(MultiShape));
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{0});
    EXPECT_EQ(foo.GetMassData(), defaultMassData);
    
    EXPECT_EQ(foo.GetVertexRadius(), MultiShape::GetDefaultVertexRadius());
    EXPECT_EQ(foo.GetDensity(), defaultConf.density);
    EXPECT_EQ(foo.GetFriction(), defaultConf.friction);
    EXPECT_EQ(foo.GetRestitution(), defaultConf.restitution);
}

TEST(MultiShape, GetInvalidChildThrows)
{
    MultiShape foo{};
    
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{0});
    EXPECT_THROW(foo.GetChild(0), InvalidArgument);
    EXPECT_THROW(foo.GetChild(1), InvalidArgument);
}

TEST(MultiShape, Accept)
{
    class Visitor: public Shape::Visitor
    {
    public:
        void Visit(const MultiShape&) override
        {
            visited = true;
        }
        bool visited = false;
    };
    
    MultiShape foo{};
    Visitor v;
    ASSERT_FALSE(v.visited);
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
}
