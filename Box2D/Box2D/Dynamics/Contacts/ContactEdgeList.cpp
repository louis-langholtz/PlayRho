//
//  ContactEdgeList.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/17/16.
//
//

#include <Box2D/Dynamics/Contacts/ContactEdgeList.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>

using namespace box2d;

void ContactEdgeList::push_front(pointer value) noexcept
{
	value->prev = nullptr;
	value->next = p;
	if (!empty())
	{
		p->prev = value;
	}
	p = value;
}

void ContactEdgeList::pop_front() noexcept
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

ContactEdgeList::iterator ContactEdgeList::erase(ContactEdgeList::iterator pos)
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
