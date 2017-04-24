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

#ifndef B2_ISLAND_H
#define B2_ISLAND_H

#include <Box2D/Common/Math.hpp>
#include <vector>

namespace box2d {

class Body;
class Contact;
class Joint;

/// Island.
/// @details A container of bodies contacts and joints relavent to handling world dynamics.
/// @note This is an internal class.
/// @note This data structure is 72-bytes large (on at least one 64-bit platform).
class Island
{
public:
	using Bodies = std::vector<Body*>;
	using Contacts = std::vector<Contact*>;
	using Joints = std::vector<Joint*>;
	
	Island(Bodies::size_type bodyCapacity, Contacts::size_type contactCapacity, Joints::size_type jointCapacity);

	Island(const Island& copy) noexcept:
		m_bodies(copy.m_bodies),
		m_contacts(copy.m_contacts),
		m_joints(copy.m_joints)
	{}

	Island(Island&& other) noexcept:
		m_bodies{std::move(other.m_bodies)},
		m_contacts{std::move(other.m_contacts)},
		m_joints{std::move(other.m_joints)}
	{}

	/// Destructor.
	~Island() = default;

	Island& operator= (Island&& other) noexcept
	{
		m_bodies = std::move(other.m_bodies);
		m_contacts = std::move(other.m_contacts);
		m_joints = std::move(other.m_joints);
		return *this;
	}

	Bodies m_bodies;
	Contacts m_contacts;
	Joints m_joints;
};

inline bool IsFullOfBodies(const Island& island)
{
	return island.m_bodies.size() == island.m_bodies.max_size();
}

inline bool IsFullOfContacts(const Island& island)
{
	return island.m_contacts.size() == island.m_contacts.max_size();
}

std::size_t Count(const Island& island, const Body* entry);
std::size_t Count(const Island& island, const Contact* entry);
std::size_t Count(const Island& island, const Joint* entry);

} // namespace box2d

#endif
