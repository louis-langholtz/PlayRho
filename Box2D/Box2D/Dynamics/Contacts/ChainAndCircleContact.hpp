/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_CHAIN_AND_CIRCLE_CONTACT_H
#define B2_CHAIN_AND_CIRCLE_CONTACT_H

#include <Box2D/Dynamics/Contacts/Contact.hpp>

namespace box2d {

class BlockAllocator;

class ChainAndCircleContact : public Contact
{
public:
	static Contact* Create(Fixture* fixtureA, child_count_t indexA,
							 Fixture* fixtureB, child_count_t indexB,
							 BlockAllocator& allocator);
	static void Destroy(Contact* contact, BlockAllocator& allocator);

	ChainAndCircleContact(Fixture* fixtureA, child_count_t indexA,
						  Fixture* fixtureB, child_count_t indexB);
	~ChainAndCircleContact() {}

	Manifold Evaluate() const override;
};

} // namespace box2d

#endif
