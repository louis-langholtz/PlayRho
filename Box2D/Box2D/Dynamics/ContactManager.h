/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include <Box2D/Collision/BroadPhase.h>

namespace box2d {

class Contact;
class ContactFilter;
class ContactListener;
class BlockAllocator;

// Delegate of World.
class ContactManager
{
public:
	using size_type = size_t;

	ContactManager(BlockAllocator* allocator, ContactFilter* filter, ContactListener* listener);

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB);

	void FindNewContacts();

	void Destroy(Contact* c);

	void Collide();
	
	inline size_type GetContactCount() const noexcept { return m_contactCount; }
	const Contact* GetContactList() const noexcept { return m_contactList; }
	Contact* GetContactList() noexcept { return m_contactList; }

	BroadPhase m_broadPhase;
	ContactFilter* m_contactFilter;
	ContactListener* m_contactListener;

private:
	size_type m_contactCount = 0;
	Contact* m_contactList = nullptr;
	BlockAllocator* const m_allocator;
};

} // namespace box2d

#endif
