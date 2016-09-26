//
//  Contact.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/26/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Contacts/Contact.h>

using namespace box2d;

TEST(ContactEdge, ByteSizeIs32)
{
	EXPECT_EQ(sizeof(ContactEdge), size_t(32));
}

TEST(Contact, ByteSizeIs216)
{
	EXPECT_EQ(sizeof(Contact), size_t(216));
}
