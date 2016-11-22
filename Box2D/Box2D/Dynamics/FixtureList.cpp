//
//  b2FixtureList.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#include <Box2D/Dynamics/FixtureList.hpp>
#include <Box2D/Dynamics/Fixture.hpp>

using namespace box2d;

FixtureList::pointer FixtureList::q = nullptr;

void FixtureList::push_front(pointer value) noexcept
{
	value->m_next = p;
	p = value;
}

void FixtureList::pop_front() noexcept
{
	p = p->m_next;
}

FixtureList::iterator FixtureList::erase(FixtureList::iterator pos)
{
	*(pos.p) = (*pos.p)->m_next;
	return pos;
}
