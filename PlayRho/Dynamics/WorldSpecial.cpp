/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/World.hpp>

#include <PlayRho/Dynamics/Body.hpp> // for completing WorldImpl type
#include <PlayRho/Dynamics/Contacts/Contact.hpp> // for completing WorldImpl type
#include <PlayRho/Dynamics/Joints/Joint.hpp> // for completing WorldImpl type
#include <PlayRho/Dynamics/WorldImpl.hpp> // for completing WorldImpl type
#include <PlayRho/Dynamics/WorldImplMisc.hpp>

#include <PlayRho/Collision/Manifold.hpp> // for completing WorldImpl type

namespace playrho {
namespace d2 {

pmr::PoolMemoryResource::Options World::GetContactsOptions()
{
    return WorldImpl::GetContactsOptions();
}

void World::SetContactsOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetContactsOptions(options);
}

pmr::PoolMemoryResource::Stats World::GetContactsStats()
{
    return WorldImpl::GetContactsStats();
}

pmr::PoolMemoryResource::Options World::GetContactKeysOptions()
{
    return WorldImpl::GetContactKeysOptions();
}

void World::SetContactKeysOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetContactKeysOptions(options);
}

pmr::PoolMemoryResource::Stats World::GetContactKeysStats()
{
    return WorldImpl::GetContactKeysStats();
}

pmr::PoolMemoryResource::Options World::GetBodyConstraintsOptions()
{
    return WorldImpl::GetBodyConstraintsOptions();
}

void World::SetBodyConstraintsOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetBodyConstraintsOptions(options);
}

pmr::PoolMemoryResource::Stats World::GetBodyConstraintsStats()
{
    return WorldImpl::GetBodyConstraintsStats();
}

pmr::PoolMemoryResource::Options World::GetPositionConstraintsOptions()
{
    return WorldImpl::GetPositionConstraintsOptions();
}

void World::SetPositionConstraintsOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetPositionConstraintsOptions(options);
}

pmr::PoolMemoryResource::Stats World::GetPositionConstraintsStats()
{
    return WorldImpl::GetPositionConstraintsStats();
}

pmr::PoolMemoryResource::Options World::GetVelocityConstraintsOptions()
{
    return WorldImpl::GetVelocityConstraintsOptions();
}

void World::SetVelocityConstraintsOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetVelocityConstraintsOptions(options);
}

pmr::PoolMemoryResource::Stats World::GetVelocityConstraintsStats()
{
    return WorldImpl::GetVelocityConstraintsStats();
}

pmr::PoolMemoryResource::Options World::GetBodyStackOptions()
{
    return WorldImpl::GetBodyStackOptions();
}

void World::SetBodyStackOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetBodyStackOptions(options);
}

pmr::PoolMemoryResource::Stats World::GetBodyStackStats()
{
    return WorldImpl::GetBodyStackStats();
}

pmr::PoolMemoryResource::Options World::GetIslandBodiesOptions()
{
    return WorldImpl::GetIslandBodiesOptions();
}

void World::SetIslandBodiesOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetIslandBodiesOptions(options);
}

pmr::PoolMemoryResource::Options World::GetIslandContactsOptions()
{
    return WorldImpl::GetIslandContactsOptions();
}

void World::SetIslandContactsOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetIslandContactsOptions(options);
}

pmr::PoolMemoryResource::Options World::GetIslandJointsOptions()
{
    return WorldImpl::GetIslandJointsOptions();
}

void World::SetIslandJointsOptions(const pmr::PoolMemoryResource::Options& options)
{
    WorldImpl::SetIslandJointsOptions(options);
}

World::World(const WorldConf& def): m_impl{CreateWorldImpl(def)}
{
}

World::World(const World& other): m_impl{CreateWorldImpl(*other.m_impl)}
{
}

World::World(World&& other) noexcept = default;

World& World::operator=(World&& other) noexcept = default;

World::~World() noexcept
{
    if (m_impl) {
        // Call implementation's clear while World still valid to give destruction
        // listening callbacks chance to run while world data is still valid.
        m_impl->Clear();
    }
}

World& World::operator= (const World& other)
{
    m_impl = CreateWorldImpl(*other.m_impl);
    return *this;
}

} // namespace d2
} // namespace playrho
