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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

namespace box2d {

class b2BlockAllocator;
class b2Body;
class b2BroadPhase;
class b2Fixture;

/// This holds contact filtering data.
struct b2Filter
{
	constexpr b2Filter() = default;

	/// The collision category bits. Normally you would just set one bit.
	uint16 categoryBits = 0x0001;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint16 maskBits = 0xFFFF;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	int16 groupIndex = 0;
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct b2FixtureDef
{
	/// The constructor sets the default fixture definition values.
	constexpr b2FixtureDef() = default;

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	const b2Shape* shape = nullptr;

	/// Use this to store application specific fixture data.
	void* userData = nullptr;

	/// The friction coefficient, usually in the range [0,1].
	b2Float friction = b2Float(0.2);

	/// The restitution (elasticity) usually in the range [0,1].
	b2Float restitution = b2Float{0};

	/// The density, usually in kg/m^2.
	b2Float density = b2Float{0};

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor = false;

	/// Contact filtering data.
	b2Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct b2FixtureProxy
{
	using size_type = b2_size_t;

	b2AABB aabb;
	b2Fixture* fixture;
	child_count_t childIndex;
	size_type proxyId;
};

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
class b2Fixture
{
public:
	b2Fixture() = delete;

	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	b2Shape::Type GetType() const noexcept;

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	b2Shape* GetShape() noexcept;
	const b2Shape* GetShape() const noexcept;

	/// Set if this fixture is a sensor.
	void SetSensor(bool sensor);

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	bool IsSensor() const noexcept;

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls Refilter.
	void SetFilterData(const b2Filter& filter);

	/// Get the contact filtering data.
	const b2Filter& GetFilterData() const noexcept;

	/// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
	void Refilter();

	/// Get the parent body of this fixture. This is nullptr if the fixture is not attached.
	/// @return the parent body.
	b2Body* GetBody() noexcept;
	const b2Body* GetBody() const noexcept;

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	b2Fixture* GetNext();
	const b2Fixture* GetNext() const;

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	bool TestPoint(const b2Vec2& p) const;

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input, child_count_t childIndex) const;

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	b2MassData GetMassData() const;

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call b2Body::ResetMassData to update the body's mass.
	void SetDensity(b2Float density);

	/// Get the density of this fixture.
	b2Float GetDensity() const;

	/// Get the coefficient of friction.
	b2Float GetFriction() const;

	/// Set the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	void SetFriction(b2Float friction);

	/// Get the coefficient of restitution.
	b2Float GetRestitution() const;

	/// Set the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	void SetRestitution(b2Float restitution);

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	const b2AABB& GetAABB(child_count_t childIndex) const;

	/// Dump this fixture to the log file.
	void Dump(island_count_t bodyIndex);

protected:

	friend class b2Body;
	friend class b2World;
	friend class b2ContactManager;

	b2Fixture(b2Body* body) noexcept: m_body(body) {}

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	void Create(b2BlockAllocator* allocator, const b2FixtureDef* def);
	void Destroy(b2BlockAllocator* allocator);

	// These support body activation/deactivation.
	void CreateProxies(b2BroadPhase& broadPhase, const b2Transform& xf);
	void DestroyProxies(b2BroadPhase& broadPhase);

	void Synchronize(b2BroadPhase& broadPhase, const b2Transform& xf1, const b2Transform& xf2);

	b2Body* const m_body;
	b2Float m_density = b2Float{0};
	b2Fixture* m_next = nullptr;
	b2Shape* m_shape = nullptr;
	b2Float m_friction;
	b2Float m_restitution;
	b2FixtureProxy* m_proxies = nullptr;
	child_count_t m_proxyCount = 0;
	b2Filter m_filter;
	bool m_isSensor;
	void* m_userData = nullptr;
};

inline b2Shape::Type b2Fixture::GetType() const noexcept
{
	return m_shape->GetType();
}

inline b2Shape* b2Fixture::GetShape() noexcept
{
	return m_shape;
}

inline const b2Shape* b2Fixture::GetShape() const noexcept
{
	return m_shape;
}

inline bool b2Fixture::IsSensor() const noexcept
{
	return m_isSensor;
}

inline const b2Filter& b2Fixture::GetFilterData() const noexcept
{
	return m_filter;
}

inline void* b2Fixture::GetUserData() const noexcept
{
	return m_userData;
}

inline void b2Fixture::SetUserData(void* data)
{
	m_userData = data;
}

inline b2Body* b2Fixture::GetBody() noexcept
{
	return m_body;
}

inline const b2Body* b2Fixture::GetBody() const noexcept
{
	return m_body;
}

inline b2Fixture* b2Fixture::GetNext()
{
	return m_next;
}

inline const b2Fixture* b2Fixture::GetNext() const
{
	return m_next;
}

inline void b2Fixture::SetDensity(b2Float density)
{
	b2Assert(b2IsValid(density) && density >= b2Float{0});
	m_density = density;
}

inline b2Float b2Fixture::GetDensity() const
{
	return m_density;
}

inline b2Float b2Fixture::GetFriction() const
{
	return m_friction;
}

inline void b2Fixture::SetFriction(b2Float friction)
{
	m_friction = friction;
}

inline b2Float b2Fixture::GetRestitution() const
{
	return m_restitution;
}

inline void b2Fixture::SetRestitution(b2Float restitution)
{
	m_restitution = restitution;
}

inline bool b2Fixture::TestPoint(const b2Vec2& p) const
{
	return m_shape->TestPoint(m_body->GetTransform(), p);
}

inline bool b2Fixture::RayCast(b2RayCastOutput* output, const b2RayCastInput& input, child_count_t childIndex) const
{
	return m_shape->RayCast(output, input, m_body->GetTransform(), childIndex);
}

inline b2MassData b2Fixture::GetMassData() const
{
	return m_shape->ComputeMass(m_density);
}

inline const b2AABB& b2Fixture::GetAABB(child_count_t childIndex) const
{
	b2Assert(childIndex >= 0);
	b2Assert(childIndex < m_proxyCount);
	return m_proxies[childIndex].aabb;
}

} // namespace box2d

#endif
