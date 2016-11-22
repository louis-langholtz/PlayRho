/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_ISLAND_H
#define B2_ISLAND_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/AllocatedArray.hpp>

namespace box2d {

class Body;
class Contact;
class Joint;
class StackAllocator;
class ContactListener;

/// Island.
/// @detail A container of bodies contacts and joints relavent to handling world dynamics.
/// @note This is an internal class.
class Island
{
public:
	using BodyContainer = AllocatedArray<Body*, StackAllocator&>;
	using ContactContainer = AllocatedArray<Contact*, StackAllocator&>;
	using JointContainer = AllocatedArray<Joint*, StackAllocator&>;
	
	Island(body_count_t bodyCapacity, contact_count_t contactCapacity, island_count_t jointCapacity,
		   StackAllocator& allocator);

	Island(const Island& copy) = delete;

	Island(Island&& other) noexcept:
		m_bodies{std::move(other.m_bodies)},
		m_contacts{std::move(other.m_contacts)},
		m_joints{std::move(other.m_joints)}
	{}

	/// Destructor.
	~Island() = default;

	Island& operator= (Island&& other) = delete;

	BodyContainer m_bodies;
	ContactContainer m_contacts;
	JointContainer m_joints;
};

inline bool IsFullOfBodies(const Island& island)
{
	return island.m_bodies.size() == island.m_bodies.max_size();
}

inline bool IsFullOfContacts(const Island& island)
{
	return island.m_contacts.size() == island.m_contacts.max_size();
}
	
} // namespace box2d

#endif
