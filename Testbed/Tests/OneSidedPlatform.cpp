/*
 * Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "../Framework/Test.hpp"

namespace testbed {

class OneSidedPlatform : public Test
{
public:
    static inline const auto registered =
        RegisterTest("One-Sided Platform", MakeUniqueTest<OneSidedPlatform>);
    enum State { e_unknown, e_above, e_below };

    OneSidedPlatform()
    {
        SetPreSolveContactListener(GetWorld(), [this](ContactID id, const Manifold& manifold) {
            Test::PreSolve(id, manifold);
            OneSidedPlatform::PreSolve(id, manifold);
        });

        // Ground
        Attach(GetWorld(), CreateBody(GetWorld()),
               CreateShape(GetWorld(),
                           EdgeShapeConf{Vec2(-20.0f, 0.0f) * 1_m, Vec2(20.0f, 0.0f) * 1_m}));

        // Platform
        {
            BodyConf bd;
            bd.location = Vec2(0.0f, 10.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            m_platform = CreateShape(GetWorld(), PolygonShapeConf{}.SetAsBox(3_m, 0.5_m));
            Attach(GetWorld(), body, m_platform);
            m_bottom = Real(10.0f - 0.5f) * 1_m;
            m_top = Real(10.0f + 0.5f) * 1_m;
        }

        // Actor
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.location = Vec2(0.0f, 12.0f) * 1_m;
            m_characterBodyId = CreateBody(GetWorld(), bd);
            auto conf = DiskShapeConf{};
            conf.vertexRadius = m_radius;
            conf.density = 20_kgpm2;
            m_character = CreateShape(GetWorld(), Shape(conf));
            Attach(GetWorld(), m_characterBodyId, m_character);
            SetVelocity(GetWorld(), m_characterBodyId, Velocity{Vec2(0.0f, -50.0f) * 1_mps, 0_rpm});
        }
    }

    void PreSolve(ContactID contact, const Manifold&)
    {
        const auto shapeA = GetShapeA(GetWorld(), contact);
        const auto shapeB = GetShapeB(GetWorld(), contact);
        if (shapeA != m_platform && shapeA != m_character) {
            return;
        }
        if (shapeB != m_platform && shapeB != m_character) {
            return;
        }
        const auto position = GetLocation(GetWorld(), m_characterBodyId);
        if (GetY(position) <
            m_top + m_radius - GetVertexRadius(GetShape(GetWorld(), m_platform), 0)) {
            UnsetEnabled(GetWorld(), contact);
        }
    }

    void PostStep(const Settings&, Drawer&) override
    {
        const auto v = GetLinearVelocity(GetWorld(), m_characterBodyId);
        std::stringstream stream;
        stream << "Character linear velocity: ";
        stream << static_cast<double>(Real{GetY(v) / 1_mps});
        stream << " m/s.";
        SetStatus(stream.str());
    }

    Length m_radius = 0.5_m;
    Length m_top;
    Length m_bottom;
    State m_state = e_unknown;
    BodyID m_characterBodyId = InvalidBodyID;
    ShapeID m_platform = InvalidShapeID;
    ShapeID m_character = InvalidShapeID;
};

} // namespace testbed
