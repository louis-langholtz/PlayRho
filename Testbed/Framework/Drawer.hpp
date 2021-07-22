/*
 * Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DRAWER_HPP
#define PLAYRHO_DRAWER_HPP

#include <PlayRho/Common/Math.hpp>
#include <algorithm>

namespace testbed {

/// RGBA Color.
/// @details Color for drawing. Each value has the range [0,1].
struct Color
{
    Color() = default;
    
    constexpr Color(float ri, float gi, float bi, float ai = 1):
        r(std::clamp(ri, 0.0f, 1.0f)),
        g(std::clamp(gi, 0.0f, 1.0f)),
        b(std::clamp(bi, 0.0f, 1.0f)),
        a(std::clamp(ai, 0.0f, 1.0f))
    {
        // Intentionally empty.
    }

    constexpr Color(Color copy, float new_a):
        Color{copy.r, copy.g, copy.b, new_a}
    {
        // Intentionally empty.
    }

    float r;
    float g;
    float b;
    float a = 1; ///< Alpha value. @details 0 for fully transparent to 1 for fully opaque.
};

Color Brighten(Color color, float factor);
    
class Drawer
{
public:
    /// @brief Size type.
    using size_type = std::size_t;

    enum TextAlign
    {
        Left,
        Center,
        Right,
        AboveCenter,
    };

    Drawer() = default;

    virtual ~Drawer() noexcept = 0;

    /// Draw a closed polygon provided in CCW order.
    virtual void DrawPolygon(const playrho::Length2* vertices, size_type vertexCount, const Color& color) = 0;

    /// Draw a solid closed polygon provided in CCW order.
    virtual void DrawSolidPolygon(const playrho::Length2* vertices, size_type vertexCount, const Color& color) = 0;

    /// Draw a circle.
    virtual void DrawCircle(const playrho::Length2& center, playrho::Length radius, const Color& color) = 0;
    
    /// Draw a solid circle.
    virtual void DrawSolidCircle(const playrho::Length2& center, playrho::Length radius, const Color& color) = 0;
 
    /// Draws a line segment from point 1 to point 2.
    virtual void DrawSegment(const playrho::Length2& p1, const playrho::Length2& p2, const Color& c) = 0;

    /// Draws a line segment from point 1 to point 2 in color 1 to color 2.
    virtual void DrawSegment(const playrho::Length2& p1, const Color& c1,
                             const playrho::Length2& p2, const Color& c2) = 0;

    virtual void DrawPoint(const playrho::Length2& p, float size, const Color& color) = 0;
    
    /// Draws a string at the given world coordinates.
    virtual void DrawString(const playrho::Length2& p, TextAlign align, const char* string, ...) = 0;
        
    virtual void Flush() = 0;
    
    virtual void SetTranslation(playrho::Length2 value) = 0;

    virtual playrho::Length2 GetTranslation() const = 0;
};

/// @brief Draws segment with a single constant color.
inline void DrawSegment(Drawer& drawer, const playrho::Length2& p1, const playrho::Length2& p2,
                        const Color& color)
{
    drawer.DrawSegment(p1, p2, color);
}

} // namespace testbed

#endif
