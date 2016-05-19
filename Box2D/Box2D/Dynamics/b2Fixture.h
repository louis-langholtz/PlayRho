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

class BlockAllocator;
class Body;
class BroadPhase;
class Fixture;

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
struct FixtureDef
{
	/// The constructor sets the default fixture definition values.
	constexpr FixtureDef() = default;

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	const Shape* shape = nullptr;

	/// Use this to store application specific fixture data.
	void* userData = nullptr;

	/// The friction coefficient, usually in the range [0,1].
	float_t friction = float_t(0.2);

	/// The restitution (elasticity) usually in the range [0,1].
	float_t restitution = float_t{0};

	/// The density, usually in kg/m^2.
	float_t density = float_t{0};

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor = false;

	/// Contact filtering data.
	b2Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct FixtureProxy
{
	using size_type = size_t;

	AABB aabb;
	Fixture* fixture;
	child_count_t childIndex;
	size_type proxyId;
};

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via Body::CreateFixture.
/// @warning you cannot reuse fixtures.
class Fixture
{
public:
	Fixture() = delete;

	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	Shape::Type GetType() const noexcept;

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	Shape* GetShape() noexcept;
	const Shape* GetShape() const noexcept;

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

	/// Call this if you want to establish collision that was previously disabled by ContactFilter::ShouldCollide.
	void Refilter();

	/// Get the parent body of this fixture. This is nullptr if the fixture is not attached.
	/// @return the parent body.
	Body* GetBody() noexcept;
	const Body* GetBody() const noexcept;

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	Fixture* GetNext();
	const Fixture* GetNext() const;

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	bool TestPoint(const Vec2& p) const;

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input, child_count_t childIndex) const;

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	b2MassData GetMassData() const;

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call Body::ResetMassData to update the body's mass.
	void SetDensity(float_t density);

	/// Get the density of this fixture.
	float_t GetDensity() const;

	/// Get the coefficient of friction.
	float_t GetFriction() const;

	/// Set the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	void SetFriction(float_t friction);

	/// Get the coefficient of restitution.
	float_t GetRestitution() const;

	/// Set the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	void SetRestitution(float_t restitution);

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	const AABB& GetAABB(child_count_t childIndex) const;

	/// Dump this fixture to the log file.
	void Dump(island_count_t bodyIndex);

protected:

	friend class Body;
	friend class World;
	friend class ContactManager;

	Fixture(Body* body) noexcept: m_body(body) {}

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	void Create(BlockAllocator* allocator, const FixtureDef* def);
	void Destroy(BlockAllocator* allocator);

	// These support body activation/deactivation.
	void CreateProxies(BroadPhase& broadPhase, const Transform& xf);
	void DestroyProxies(BroadPhase& broadPhase);

	void Synchronize(BroadPhase& broadPhase, const Transform& xf1, const Transform& xf2);

	Body* const m_body;
	float_t m_density = float_t{0};
	Fixture* m_next = nullptr;
	Shape* m_shape = nullptr;
	float_t m_friction;
	float_t m_restitution;
	FixtureProxy* m_proxies = nullptr;
	child_count_t m_proxyCount = 0;
	b2Filter m_filter;
	bool m_isSensor;
	void* m_userData = nullptr;
};

inline Shape::Type Fixture::GetType() const noexcept
{
	return m_shape->GetType();
}

inline Shape* Fixture::GetShape() noexcept
{
	return m_shape;
}

inline const Shape* Fixture::GetShape() const noexcept
{
	return m_shape;
}

inline bool Fixture::IsSensor() const noexcept
{
	return m_isSensor;
}

inline const b2Filter& Fixture::GetFilterData() const noexcept
{
	return m_filter;
}

inline void* Fixture::GetUserData() const noexcept
{
	return m_userData;
}

inline void Fixture::SetUserData(void* data)
{
	m_userData = data;
}

inline Body* Fixture::GetBody() noexcept
{
	return m_body;
}

inline const Body* Fixture::GetBody() const noexcept
{
	return m_body;
}

inline Fixture* Fixture::GetNext()
{
	return m_next;
}

inline const Fixture* Fixture::GetNext() const
{
	return m_next;
}

inline void Fixture::SetDensity(float_t density)
{
	assert(IsValid(density) && density >= float_t{0});
	m_density = density;
}

inline float_t Fixture::GetDensity() const
{
	return m_density;
}

inline float_t Fixture::GetFriction() const
{
	return m_friction;
}

inline void Fixture::SetFriction(float_t friction)
{
	m_friction = friction;
}

inline float_t Fixture::GetRestitution() const
{
	return m_restitution;
}

inline void Fixture::SetRestitution(float_t restitution)
{
	m_restitution = restitution;
}

inline bool Fixture::TestPoint(const Vec2& p) const
{
	return m_shape->TestPoint(m_body->GetTransform(), p);
}

inline bool Fixture::RayCast(b2RayCastOutput* output, const b2RayCastInput& input, child_count_t childIndex) const
{
	return m_shape->RayCast(output, input, m_body->GetTransform(), childIndex);
}

inline b2MassData Fixture::GetMassData() const
{
	return m_shape->ComputeMass(m_density);
}

inline const AABB& Fixture::GetAABB(child_count_t childIndex) const
{
	assert(childIndex >= 0);
	assert(childIndex < m_proxyCount);
	return m_proxies[childIndex].aabb;
}

} // namespace box2d

#endif
