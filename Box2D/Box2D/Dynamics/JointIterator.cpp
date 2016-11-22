//
//  JointIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#include <Box2D/Dynamics/JointIterator.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>

using namespace box2d;

JointIterator::pointer JointIterator::next(pointer q) const noexcept
{
	return p->m_next;
}
