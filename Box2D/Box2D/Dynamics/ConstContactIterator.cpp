//
//  ConstContactIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/14/16.
//
//

#include <Box2D/Dynamics/ConstContactIterator.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>

using namespace box2d;

ConstContactIterator::pointer ConstContactIterator::next(pointer q) const noexcept
{
	return q->m_next;
}
