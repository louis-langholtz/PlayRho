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

namespace playrho {
    
    /// @brief 15 Puzzle.
    /// @sa https://en.wikipedia.org/wiki/15_puzzle
    class FifteenPuzzle: public Test
    {
    public:
        
        FifteenPuzzle()
        {
            m_world->SetGravity(LinearAcceleration2D{});
            const auto enclosure = CreateSquareEnclosingBody(*m_world,
                16 * Meter + 2 * GetVertexRadius(), ShapeConf{}
                .UseVertexRadius(GetVertexRadius()));
            SetLocation(*enclosure, GetCenter());
            
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

        Length2D GetCenter() const
        {
            return Vec2{Real{0}, Real{20}} * Meter;
        }

        Body* CreateSquareTile(int col, int row)
        {
            const auto sideLength = Real{4} * Meter;
            const auto skinWidth = GetVertexRadius();
            const auto halfSide = sideLength / Real{2} - skinWidth;
            const auto relPos = Length2D{(col - 2) * sideLength, (row - 2) * sideLength};
            
            auto conf = PolygonShape::Conf{};
            conf.density = Real{1} * KilogramPerSquareMeter;
            conf.vertexRadius = skinWidth;
            
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.bullet = true;
            bd.location = GetCenter() + relPos + Length2D{sideLength / 2, sideLength / 2};
            bd.linearDamping = 20.0f * Hertz;
            const auto body = m_world->CreateBody(bd);
            body->CreateFixture(std::make_shared<PolygonShape>(halfSide, halfSide, conf));
            
            return body;
        }
        
        void PostStep(const Settings& settings, Drawer& drawer) override
        {
            drawer.DrawString(5, m_textLine, Drawer::Left,
                "Slide square tiles around using the mouse. See if you can re-order them. Good luck!");
            m_textLine += DRAW_STRING_NEW_LINE;
            if (!settings.drawSkins)
            {
                drawer.DrawString(5, m_textLine, Drawer::Left,
                                  "Enable \"Skins\" to better see what things look like!");
                m_textLine += DRAW_STRING_NEW_LINE;
            }
            if (!settings.drawLabels)
            {
                drawer.DrawString(5, m_textLine, Drawer::Left,
                                  "Enable \"Labels\" to get a better idea of what's what!");
                m_textLine += DRAW_STRING_NEW_LINE;
            }
        }
        
    };
    
} // namespace playrho

#endif /* FifteenPuzzle_hpp */
