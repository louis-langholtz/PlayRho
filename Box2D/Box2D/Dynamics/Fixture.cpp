/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>

using namespace box2d;

const FixtureProxy* Fixture::GetProxy(child_count_t index) const noexcept
{
	assert(index < m_proxyCount);
	return (index < m_proxyCount)? m_proxies + index: nullptr;
}

void Fixture::Refilter()
{
	const auto body = GetBody();
	const auto world = body->GetWorld();
	world->Refilter(*this);
}

void Fixture::SetSensor(bool sensor) noexcept
{
	if (sensor != m_isSensor)
	{
		m_isSensor = sensor;
		const auto body = GetBody();
		if (body)
		{
			body->SetAwake();
		}
	}
}

AABB box2d::GetAABB(const Fixture& fixture, child_count_t childIndex) noexcept
{
	return fixture.GetProxy(childIndex)->aabb;
}

bool box2d::TestPoint(const Fixture& f, const Vec2 p)
{
	return TestPoint(*f.GetShape(), f.GetBody()->GetTransformation(), p);
}

void box2d::SetAwake(Fixture& f) noexcept
{
	const auto b = f.GetBody();
	if (b)
	{
		b->SetAwake();
	}
}
