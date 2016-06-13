//
//  JointList.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#include <Box2D/Dynamics/JointList.hpp>
#include <Box2D/Dynamics/Joints/Joint.h>

using namespace box2d;

void JointList::push_front(pointer value) noexcept
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

JointList::iterator JointList::erase(JointList::iterator pos)
{
	assert(n > 0);
	if (n > 0)
	{
		const auto next = pos.p->m_next;
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
		return iterator{next};
	}
	return pos;
}