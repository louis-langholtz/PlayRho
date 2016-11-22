//
//  BodyList.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#include <Box2D/Dynamics/BodyList.hpp>
#include <Box2D/Dynamics/Body.hpp>

using namespace box2d;

void BodyList::push_front(pointer value) noexcept
{
	assert(n < max_size());
	if (n < max_size())
	{
		value->m_prev = nullptr;
		value->m_next = p;
		if (!empty())
		{
			p->m_prev = value;
		}
		p = value;
		++n;
	}
}

void BodyList::pop_front() noexcept
{
	assert(n > 0);
	if (p->m_prev)
	{
		p->m_prev->m_next = p->m_next;
	}
	if (p->m_next)
	{
		p->m_next->m_prev = p->m_prev;
	}
	p = p->m_next;
	--n;
}

BodyList::iterator BodyList::erase(BodyList::iterator pos)
{
	if (n > 0)
	{
		if (pos.p->m_prev)
		{
			pos.p->m_prev->m_next = pos.p->m_next;
		}
		if (pos.p->m_next)
		{
			pos.p->m_next->m_prev = pos.p->m_prev;
		}
		if (p == pos.p)
		{
			p = pos.p->m_next;
		}
		--n;
	}
	return pos;
}
