//
//  ContactFeature.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/8/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/ContactFeature.hpp>

using namespace box2d;

TEST(ContactFeature, Init)
{
	const auto typeA = ContactFeature::e_vertex;
	const auto typeB = ContactFeature::e_face;
	const auto indexA = ContactFeature::index_t{1};
	const auto indexB = ContactFeature::index_t{2};
	ContactFeature foo{typeA, indexA, typeB, indexB};
	
	EXPECT_EQ(foo.typeA, typeA);
	EXPECT_EQ(foo.typeB, typeB);
	EXPECT_EQ(foo.indexA, indexA);
	EXPECT_EQ(foo.indexB, indexB);
}

TEST(ContactFeature, DefaultContactFeature)
{
	EXPECT_EQ(DefaultContactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(DefaultContactFeature.typeB, ContactFeature::e_vertex);
	EXPECT_EQ(DefaultContactFeature.indexA, ContactFeature::index_t{0});
	EXPECT_EQ(DefaultContactFeature.indexB, ContactFeature::index_t{0});
}

TEST(ContactFeature, Flip)
{
	const auto typeA = ContactFeature::e_vertex;
	const auto typeB = ContactFeature::e_face;
	const auto indexA = ContactFeature::index_t{1};
	const auto indexB = ContactFeature::index_t{2};
	const auto foo = ContactFeature{typeA, indexA, typeB, indexB};
	const auto bar = Flip(foo);
	EXPECT_EQ(bar.typeA, typeB);
	EXPECT_EQ(bar.typeB, typeA);
	EXPECT_EQ(bar.indexA, indexB);
	EXPECT_EQ(bar.indexB, indexA);	
}

TEST(ContactFeature, Equals)
{
	const auto typeA = ContactFeature::e_vertex;
	const auto typeB = ContactFeature::e_face;
	const auto indexA = ContactFeature::index_t{1};
	const auto indexB = ContactFeature::index_t{2};
	const auto foo = ContactFeature{typeA, indexA, typeB, indexB};
	EXPECT_EQ(foo, foo);
}

TEST(ContactFeature, NotEquals)
{
	{
		const auto cf1 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 1};
		const auto cf2 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 0};
		EXPECT_NE(cf1, cf2);
	}
	{
		const auto cf1 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 1};
		const auto cf2 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 1};
		EXPECT_NE(cf1, cf2);
	}
	{
		const auto cf1 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 0};
		const auto cf2 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 0};
		EXPECT_NE(cf1, cf2);
	}
	{
		const auto cf1 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 0};
		const auto cf2 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 1};
		EXPECT_NE(cf1, cf2);
	}
	{
		const auto cf1 = ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_face, 1};
		const auto cf2 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 1};
		EXPECT_NE(cf1, cf2);
	}
	{
		const auto cf1 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 1};
		const auto cf2 = ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_face, 1};
		EXPECT_NE(cf1, cf2);
	}
}
