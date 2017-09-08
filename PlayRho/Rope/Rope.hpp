/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef B2_ROPE_H
#define B2_ROPE_H

#include <PlayRho/Common/Math.hpp>

namespace playrho {

/// @brief Rope definition.
struct RopeDef
{

    /// @brief Size type.
    using size_type = std::size_t;

    constexpr RopeDef() = default;

    Vec2* vertices = nullptr; ///< Vertices.
    size_type count = 0; ///< Count.
    Real* masses = nullptr; ///< Masses.
    Vec2 gravity = Vec2_zero; ///< Gravity.
    Real damping = Real{1} / Real(10); ///< Damping.
    Real k2 = Real(9) / Real(10); ///< Stretching stiffness

    /// @brief Bending stiffness. Values above 0.5 can make the simulation blow up.
    Real k3 = Real{1} / Real(10);
};

/// @brief Rope.
class Rope
{
public:

    /// @brief Size type.
    using size_type = std::size_t;

    constexpr Rope() = default;
    ~Rope();

    /// @brief Initializes this object.
    void Initialize(const RopeDef* def);

    /// @brief Steps this object.
    void Step(Real timeStep, int iterations);

    /// @brief Gets the vertex count for this object.
    size_type GetVertexCount() const noexcept
    {
        return m_count;
    }

    /// @brief Gets the vertices for this object.
    const Vec2* GetVertices() const noexcept
    {
        return m_ps;
    }

    /// @brief Gets the vertex for this object that's at the given index.
    Vec2 GetVertex(size_type index) const noexcept
    {
        assert(index < m_count);
        return m_ps[index];
    }

    /// @brief Sets the angle.
    void SetAngle(Angle angle);

private:

    void SolveC2();
    void SolveC3();

    size_type m_count = 0;
    Vec2* m_ps = nullptr;
    Vec2* m_p0s = nullptr;
    Vec2* m_vs = nullptr;

    Real* m_ims = nullptr;

    Real* m_Ls = nullptr;
    Angle* m_as = nullptr;

    Vec2 m_gravity = Vec2_zero;
    Real m_damping = Real{0};

    Real m_k2 = Real{1};
    Real m_k3 = Real(0.1);
};

} // namespace playrho

#endif
