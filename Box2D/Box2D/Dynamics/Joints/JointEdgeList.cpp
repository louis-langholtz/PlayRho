//
//  JointEdgeList.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/18/16.
//
//

#include <Box2D/Dynamics/Joints/JointEdgeList.hpp>
#include <Box2D/Dynamics/Joints/Joint.h>

using namespace box2d;

void JointEdgeList::push_front(pointer value) noexcept
{
	value->prev = nullptr;
	value->next = p;
	if (!empty())
	{
		p->prev = value;
	}
	p = value;
}

void JointEdgeList::pop_front() noexcept
{
	if (p->prev)
	{
		p->prev->next = p->next;
	}
	if (p->next)
	{
		p->next->prev = p->prev;
	}
	p = p->next;
}

JointEdgeList::iterator JointEdgeList::erase(JointEdgeList::iterator pos)
{
	if (pos.p->prev)
	{
		pos.p->prev->next = pos.p->next;
	}
	if (pos.p->next)
	{
		pos.p->next->prev = pos.p->prev;
	}
	if (p == pos.p)
	{
		p = pos.p->next;
	}
	return pos;
}