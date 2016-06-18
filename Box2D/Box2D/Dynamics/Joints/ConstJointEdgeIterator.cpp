//
//  ConstJointEdgeIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/18/16.
//
//

#include <Box2D/Dynamics/Joints/ConstJointEdgeIterator.hpp>
#include <Box2D/Dynamics/Joints/Joint.h>

using namespace box2d;

ConstJointEdgeIterator::pointer ConstJointEdgeIterator::next(pointer q) const noexcept
{
	return p->next;
}