/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <PlayRho/Collision/Simplex.hpp>

namespace playrho {

Simplex Simplex::Get(const SimplexEdge& s0) noexcept
{
    return Simplex{{s0}, {1}};
}

Simplex Simplex::Get(const SimplexEdge& s0, const SimplexEdge& s1) noexcept
{
    assert(s0.GetIndexPair() != s1.GetIndexPair() || s0 == s1);

    // Solves the given line segment simplex using barycentric coordinates.
    //
    // p = a1 * w1 + a2 * w2
    // a1 + a2 = 1
    //
    // The vector from the origin to the closest point on the line is
    // perpendicular to the line.
    // e12 = w2 - w1
    // dot(p, e) = 0
    // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
    //
    // 2-by-2 linear system
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    //
    // Define
    // d12_1 =  dot(w2, e12)
    // d12_2 = -dot(w1, e12)
    // d12_sum = d12_1 + d12_2
    //
    // Solution
    // a1 = d12_1 / d12_sum
    // a2 = d12_2 / d12_sum

    const auto w1 = GetPointDelta(s0);
    const auto w2 = GetPointDelta(s1);
    const auto e12 = w2 - w1;
    
    // w1 region
    const auto d12_2 = -Dot(w1, e12);
    if (d12_2 <= 0_m2)
    {
        // a2 <= 0, so we clamp it to 0
        return Simplex{{s0}, {1}};
    }
    
    // w2 region
    const auto d12_1 = Dot(w2, e12);
    if (d12_1 <= 0_m2)
    {
        // a1 <= 0, so we clamp it to 0
        return Simplex{{s1}, {1}};
    }
    
    // Must be in e12 region.
    const auto d12_sum = d12_1 + d12_2;
    return Simplex{{s0, s1}, {d12_1 / d12_sum, d12_2 / d12_sum}};
}

Simplex Simplex::Get(const SimplexEdge& s0, const SimplexEdge& s1, const SimplexEdge& s2) noexcept
{
    // Solves the given 3-edge simplex.
    //
    // Possible regions:
    // - points[2]
    // - edge points[0]-points[2]
    // - edge points[1]-points[2]
    // - inside the triangle

    const auto w1 = GetPointDelta(s0);
    const auto w2 = GetPointDelta(s1);
    const auto w3 = GetPointDelta(s2);
    
    // Edge12
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    const auto e12 = w2 - w1;
    const auto d12_1 = Dot(w2, e12);
    const auto d12_2 = -Dot(w1, e12);
    
    // Edge13
    // [1      1     ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    const auto e13 = w3 - w1;
    const auto d13_1 = Dot(w3, e13);
    const auto d13_2 = -Dot(w1, e13);
    
    // Edge23
    // [1      1     ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    const auto e23 = w3 - w2;
    const auto d23_1 = Dot(w3, e23);
    const auto d23_2 = -Dot(w2, e23);
    
    // w1 region
    if ((d12_2 <= 0_m2) && (d13_2 <= 0_m2))
    {
        return Simplex{{s0}, {1}};
    }
    
    // w2 region
    if ((d12_1 <= 0_m2) && (d23_2 <= 0_m2))
    {
        return Simplex{{s1}, {1}};
    }
    
    // w3 region
    if ((d13_1 <= 0_m2) && (d23_1 <= 0_m2))
    {
        return Simplex{{s2}, {1}};
    }

    // Triangle123
    const auto n123 = Cross(e12, e13);

    // e12
    const auto cp_w1_w2 = Cross(w1, w2);
    const auto d123_3 = n123 * cp_w1_w2;
    if ((d12_1 > 0_m2) && (d12_2 > 0_m2) && (d123_3 <= 0 * SquareMeter * SquareMeter))
    {
        const auto d12_sum = d12_1 + d12_2;
        return Simplex{{s0, s1}, {d12_1 / d12_sum, d12_2 / d12_sum}};
    }
    
    // e13
    const auto cp_w3_w1 = Cross(w3, w1);
    const auto d123_2 = n123 * cp_w3_w1;
    if ((d13_1 > 0_m2) && (d13_2 > 0_m2) && (d123_2 <= 0 * SquareMeter * SquareMeter))
    {
        const auto d13_sum = d13_1 + d13_2;
        return Simplex{{s0, s2}, {d13_1 / d13_sum, d13_2 / d13_sum}};
    }
    
    // e23
    const auto cp_w2_w3 = Cross(w2, w3);
    const auto d123_1 = n123 * cp_w2_w3;
    if ((d23_1 > 0_m2) && (d23_2 > 0_m2) && (d123_1 <= 0 * SquareMeter * SquareMeter))
    {
        const auto d23_sum = d23_1 + d23_2;
        return Simplex{{s2, s1}, {d23_2 / d23_sum, d23_1 / d23_sum}};
    }
    
    // Must be in triangle123
    const auto d123_sum = d123_1 + d123_2 + d123_3;
    return Simplex{{s0, s1, s2}, {d123_1 / d123_sum, d123_2 / d123_sum, d123_3 / d123_sum}};
}

Simplex Simplex::Get(const SimplexEdges& edges) noexcept
{
    const auto count = edges.size();
    assert(count < 4);
    switch (count)
    {
        case 0: return Simplex{};
        case 1: return Get(edges[0]);
        case 2: return Get(edges[0], edges[1]);
        case 3: return Get(edges[0], edges[1], edges[2]);
        default: break;
    }
    return Simplex{};
}

} // namespace playrho
