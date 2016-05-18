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

#include <Box2D/Collision/b2BroadPhase.h>

namespace box2d {

class b2Contact;
class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;

// Delegate of b2World.
class b2ContactManager
{
public:
	using size_type = b2_size_t;

	b2ContactManager(b2BlockAllocator* allocator, b2ContactFilter* filter, b2ContactListener* listener);

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB);

	void FindNewContacts();

	void Destroy(b2Contact* c);

	void Collide();
	
	inline size_type GetContactCount() const noexcept { return m_contactCount; }
	const b2Contact* GetContactList() const noexcept { return m_contactList; }
	b2Contact* GetContactList() noexcept { return m_contactList; }

	b2BroadPhase m_broadPhase;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;

private:
	size_type m_contactCount = 0;
	b2Contact* m_contactList = nullptr;
	b2BlockAllocator* const m_allocator;
};

} // namespace box2d

#endif
