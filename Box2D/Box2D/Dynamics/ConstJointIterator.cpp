//
//  ConstJointIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#include <Box2D/Dynamics/ConstJointIterator.hpp>
#include <Box2D/Dynamics/Joints/Joint.h>

using namespace box2d;

ConstJointIterator::pointer ConstJointIterator::next(pointer q) const noexcept
{
	return p->m_next;
}