//
//  FifteenPuzzle.hpp
//  PlayRho
//
//  Created by Louis D. Langholtz on 9/30/17.
//
//

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
            const auto enclosure = CreateSquareEnclosure(*m_world,
                16 * Meter + 2 * GetVertexRadius(), Shape::Conf{}
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
            bd.position = GetCenter() + relPos + Length2D{sideLength / 2, sideLength / 2};
            bd.linearDamping = 20.0f;
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
