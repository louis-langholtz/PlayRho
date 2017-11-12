/*
 * Original work Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_DEBUGDRAW_HPP
#define  PLAYRHO_DEBUGDRAW_HPP

#include <PlayRho/PlayRho.hpp>
#include "Drawer.hpp"

namespace playrho {

struct GLRenderPoints;
struct GLRenderLines;
struct GLRenderTriangles;

struct ProjectionMatrix
{
    float m[16];
};

struct Coord2D
{
    float x;
    float y;
};

/// @brief Multiplication operator.
inline Coord2D operator* (Coord2D coord, float scalar)
{
    return Coord2D{coord.x * scalar, coord.y * scalar};
}

/// @brief Multiplication operator.
inline Coord2D operator* (float scalar, Coord2D coord)
{
    return Coord2D{coord.x * scalar, coord.y * scalar};
}

/// @brief Division operator.
inline Coord2D operator/ (Coord2D coord, float scalar)
{
    return Coord2D{coord.x / scalar, coord.y / scalar};
}

/// @brief Addition operator.
inline Coord2D operator+ (Coord2D a, Coord2D b)
{
    return Coord2D{a.x + b.x, a.y + b.y};
}

/// @brief Subtraction operator.
inline Coord2D operator- (Coord2D a, Coord2D b)
{
    return Coord2D{a.x - b.x, a.y - b.y};
}

struct Camera
{
    Coord2D m_center = Coord2D{0.0f, 20.0f};
    float m_extent = 25.0f;
    float m_zoom = 1.0f;
    int m_width = 1280;
    int m_height = 800;
};

extern Camera g_camera;
    
Length2 ConvertScreenToWorld(const Coord2D screenPoint, const Camera& camera = g_camera);
AABB2D ConvertScreenToWorld(const Camera& camera = g_camera);
Coord2D ConvertWorldToScreen(const Length2 worldPoint, const Camera& camera = g_camera);
ProjectionMatrix GetProjectionMatrix(float zBias, const Camera& camera = g_camera);

class DebugDraw : public Drawer
{
public:
    DebugDraw(Camera& camera);

    virtual ~DebugDraw() noexcept;
    
    void DrawPolygon(const Length2* vertices, size_type vertexCount, const Color& color) override;

    void DrawSolidPolygon(const Length2* vertices, size_type vertexCount, const Color& color) override;

    void DrawCircle(const Length2& center, Length radius, const Color& color) override;

    void DrawSolidCircle(const Length2& center, Length radius, const Color& color) override;

    void DrawSegment(const Length2& p1, const Length2& p2, const Color& color) override;

    void DrawSegment(const Length2& p1, const Color& c1,
                     const Length2& p2, const Color& c2) override;

    void DrawPoint(const Length2& p, float size, const Color& color) override;

    void DrawString(const Length2& p, TextAlign align, const char* string, ...) override;

    void Flush() override;
    
    Length2 GetTranslation() const override;

    void SetTranslation(Length2 value) override;

private:
    void DrawTriangle(const Length2& p1, const Length2& p2, const Length2& p3, const Color& color);

    Camera& m_camera;
    GLRenderPoints* m_points;
    GLRenderLines* m_lines;
    GLRenderTriangles* m_triangles;
    int m_circleParts = 16;
    Real m_cosInc;
    Real m_sinInc;
};

} // namespace playrho

#endif
