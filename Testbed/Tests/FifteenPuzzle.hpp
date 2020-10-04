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

#ifndef FifteenPuzzle_hpp
#define FifteenPuzzle_hpp

#include "../Framework/Test.hpp"

namespace testbed {
    
    /// @brief 15 Puzzle.
    /// @see https://en.wikipedia.org/wiki/15_puzzle
    class FifteenPuzzle: public Test
    {
    public:
        static Test::Conf GetTestConf()
        {
            auto conf = Test::Conf{};
            conf.settings.drawSkins = true;
            conf.settings.drawLabels = true;
            conf.neededSettings = (0x1u << NeedDrawLabelsField)|(0x1u << NeedDrawSkinsField);
            conf.description = "Slide square tiles around using the mouse. See if you can re-order them. Good luck!";
            return conf;
        }
        
        FifteenPuzzle(): Test(GetTestConf())
        {
            m_gravity = LinearAcceleration2{};
            auto conf = GetChainShapeConf(16_m + 2 * GetVertexRadius());
            conf.UseVertexRadius(GetVertexRadius());
            const auto enclosure = m_world.CreateBody();
            CreateFixture(m_world, enclosure, Shape{conf});
            SetLocation(m_world, enclosure, GetCenter());
            for (auto i = 0; i < 15; ++i)
            {
                const auto row = 3 - i / 4;
                const auto col = i % 4;
                CreateSquareTile(col, row);
            }
        }
        
        Length GetVertexRadius() const
        {
            return DefaultLinearSlop * Real{100};
        }

        Length2 GetCenter() const
        {
            return Vec2{Real{0}, Real{20}} * 1_m;
        }

        BodyID CreateSquareTile(int col, int row)
        {
            const auto sideLength = 4_m;
            const auto skinWidth = GetVertexRadius();
            const auto halfSide = sideLength / Real{2} - skinWidth;
            const auto relPos = Length2{(col - 2) * sideLength, (row - 2) * sideLength};
            
            auto conf = PolygonShapeConf{};
            conf.density = 1_kgpm2;
            conf.vertexRadius = skinWidth;
            conf.SetAsBox(halfSide, halfSide);
            
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.bullet = true;
            bd.location = GetCenter() + relPos + Length2{sideLength / 2, sideLength / 2};
            bd.linearDamping = 20_Hz;
            const auto body = m_world.CreateBody(bd);
            m_world.CreateFixture(body, Shape{conf});
            
            return body;
        }
    };
    
} // namespace testbed

#endif /* FifteenPuzzle_hpp */
