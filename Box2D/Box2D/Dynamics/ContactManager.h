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
struct FixtureProxy;

/// Contact Manager.
/// This is a delegate of World (every World instance has one of these).
/// Objects of this class manage the contacts for the world they are in.
class ContactManager
{
public:
	using size_type = size_t;

	ContactManager(BlockAllocator* allocator, ContactFilter* filter, ContactListener* listener):
		m_allocator(allocator), m_contactFilter(filter), m_contactListener(listener) {}
	
	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB)
	{
		Add(static_cast<FixtureProxy*>(proxyUserDataA), static_cast<FixtureProxy*>(proxyUserDataB));
	}

	void FindNewContacts();

	/// Destroys the given contact and removes it from its list.
	/// @detail This updates the contact list, returns the memory to the allocator,
	///   and decrements the contact manager's contact count.
	/// @param c Contact to destroy.
	void Destroy(Contact* c);

	/// Processes the narrow phase collision for the contact list.
	/// @detail
	/// This finds and destroys the contacts that need filtering and no longer should collide or
	/// that no longer have AABB-based overlapping fixtures. Those contacts that persist and
	/// have active bodies (either or both) get their Update methods called with the current
	/// contact listener as its argument.
	/// Essentially this really just purges contacts that are no longer relevant.
	void Collide();
	
	/// Gets the contact count.
	/// @return Number of contacts referenced by the contact list (0 if empty).
	inline size_type GetContactCount() const noexcept { return m_contactCount; }

	/// Gets the contact list.
	/// @return Contact list or <code>nullptr</code> if empty.
	const Contact* GetContactList() const noexcept { return m_contactList; }
	
	/// Gets the contact list.
	/// @return Contact list or <code>nullptr</code> if empty.
	Contact* GetContactList() noexcept { return m_contactList; }

	BroadPhase m_broadPhase;
	ContactFilter* m_contactFilter;
	ContactListener* m_contactListener;

private:
	void Add(FixtureProxy* proxyA, FixtureProxy* proxyB);
	void Add(Contact* contact);
	void Remove(Contact* contact);
	
	size_type m_contactCount = 0;
	Contact* m_contactList = nullptr;
	BlockAllocator* const m_allocator;
};

} // namespace box2d

#endif
