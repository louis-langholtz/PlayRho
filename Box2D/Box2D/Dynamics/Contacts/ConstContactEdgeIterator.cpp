//
//  ConstContactEdgeIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/17/16.
//
//

#include <Box2D/Dynamics/Contacts/ConstContactEdgeIterator.hpp>
#include <Box2D/Dynamics/Contacts/Contact.h>

using namespace box2d;

ConstContactEdgeIterator::pointer ConstContactEdgeIterator::next(pointer q) const noexcept
{
	return p->next;
}