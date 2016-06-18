//
//  ContactEdgeIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/17/16.
//
//

#include <Box2D/Dynamics/Contacts/ContactEdgeIterator.hpp>
#include <Box2D/Dynamics/Contacts/Contact.h>

using namespace box2d;

ContactEdgeIterator::pointer ContactEdgeIterator::next(pointer q) const noexcept
{
	return p->next;
}