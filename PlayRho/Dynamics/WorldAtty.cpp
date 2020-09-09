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

#include <PlayRho/Dynamics/WorldAtty.hpp>
#include <PlayRho/Dynamics/WorldImpl.hpp>

namespace playrho {
namespace d2 {

void WorldAtty::TouchProxies(World& world, Fixture& fixture) noexcept
{
    assert(fixture.GetBody()->GetWorld() == &world);
    world.m_impl->TouchProxies(fixture);
}

void WorldAtty::SetType(World& world, Body& body, playrho::BodyType type)
{
    assert(body.GetWorld() == &world);
    world.m_impl->SetType(body, type);
}

Fixture* WorldAtty::CreateFixture(World& world, Body& body, const Shape& shape,
                                  const FixtureConf& def, bool resetMassData)
{
    assert(body.GetWorld() == &world);
    return world.m_impl->CreateFixture(body, shape, def, resetMassData);
}

bool WorldAtty::Destroy(World& world, Fixture& fixture, bool resetMassData)
{
    assert(fixture.GetBody()->GetWorld() == &world);
    return world.m_impl->Destroy(fixture, resetMassData);
}

void WorldAtty::RegisterForProxies(World& world, Body& body)
{
    assert(body.GetWorld() == &world);
    world.m_impl->RegisterForProxies(body);
}

void WorldAtty::RegisterForProxies(World& world, Fixture& fixture)
{
    assert(fixture.GetBody()->GetWorld() == &world);
    world.m_impl->RegisterForProxies(fixture);
}

} // namespace d2
} // namespace playrho
