//
//  ContactIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/14/16.
//
//

#include <Box2D/Dynamics/ContactIterator.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>

using namespace box2d;

ContactIterator::pointer ContactIterator::next(pointer q) const noexcept
{
	return q->m_next;
}
