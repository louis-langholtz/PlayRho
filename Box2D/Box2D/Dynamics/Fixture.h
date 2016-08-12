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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include <Box2D/Dynamics/Body.h>
#include <Box2D/Collision/Shapes/Shape.h>

namespace box2d {

class BlockAllocator;
class Body;
class BroadPhase;
class Fixture;

/// This holds contact filtering data.
struct Filter
{
	constexpr Filter() = default;

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
	constexpr FixtureDef() noexcept = default;

	/// Initializing constructor.
	/// @param s Shape.
	/// @param d Density.
	constexpr FixtureDef(const Shape* s, float_t d) noexcept: shape{s}, density{d} {}

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	const Shape* shape = nullptr;

	/// Use this to store application specific fixture data.
	void* userData = nullptr;

	/// The friction coefficient, usually in the range [0,1].
	float_t friction = float_t{2} / float_t{10};

	/// The restitution (elasticity) usually in the range [0,1].
	float_t restitution = float_t{0};

	/// The density, usually in kg/m^2.
	float_t density = float_t{0};

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor = false;

	/// Contact filtering data.
	Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct FixtureProxy
{
	using size_type = std::remove_const<decltype(MaxContacts)>::type;

	FixtureProxy() = default;
	
	FixtureProxy(const FixtureProxy& copy) = default;
	
	FixtureProxy(const AABB& bb, size_type pid, Fixture* f, child_count_t ci):
		aabb{bb}, fixture{f}, proxyId{pid}, childIndex{ci} {}

	AABB aabb; ///< Axis Aligned Bounding Box.
	Fixture* fixture; ///< Fixture.
	size_type proxyId; ///< Proxy ID.
	child_count_t childIndex; ///< Child index.
};

/// Fixture.
/// @detail
/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via Body::CreateFixture.
/// @warning you cannot reuse fixtures.
/// @note This structure is at least 68-bytes large.
class Fixture
{
public:
	Fixture() = delete;

	/// Gets the child shape.
	/// @detail You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	Shape* GetShape() noexcept;

	/// Gets the child shape.
	/// @detail You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	const Shape* GetShape() const noexcept;

	/// Set if this fixture is a sensor.
	void SetSensor(bool sensor);

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	bool IsSensor() const noexcept;

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls Refilter.
	void SetFilterData(const Filter& filter);

	/// Get the contact filtering data.
	const Filter& GetFilterData() const noexcept;

	/// Call this if you want to establish collision that was previously disabled by ContactFilter::ShouldCollide.
	void Refilter();

	/// Gets the parent body of this fixture.
	/// @detail This is nullptr if the fixture is not attached.
	/// @return the parent body.
	Body* GetBody() noexcept;

	/// Gets the parent body of this fixture.
	/// @detail This is nullptr if the fixture is not attached.
	/// @return the parent body.
	const Body* GetBody() const noexcept;

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) noexcept;

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	bool TestPoint(const Vec2& p) const;

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	bool RayCast(RayCastOutput* output, const RayCastInput& input, child_count_t childIndex) const;

	/// Computes the mass data for this fixture.
	/// @detail
	/// The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	MassData ComputeMassData() const;

	/// Sets the density of this fixture.
	/// @note This will _not_ automatically adjust the mass of the body.
	///   You must call Body::ResetMassData to update the body's mass.
	/// @warning Behavior is undefined if given a negative value.
	/// @param density Non-negative density in kg/m^2.
	void SetDensity(float_t density) noexcept;

	/// Gets the density of this fixture.
	/// @return Non-negative density in kg/m^2.
	float_t GetDensity() const noexcept;

	/// Gets the coefficient of friction.
	float_t GetFriction() const noexcept;

	/// Sets the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	void SetFriction(float_t friction) noexcept;

	/// Gets the coefficient of restitution.
	float_t GetRestitution() const noexcept;

	/// Sets the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	void SetRestitution(float_t restitution) noexcept;

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
	friend class FixtureList;
	friend class FixtureIterator;
	friend class ConstFixtureIterator;

	Fixture(Body* body, const FixtureDef& def, Shape* shape):
		m_body{body},
		m_density{Max(def.density, float_t{0})}, 
		m_friction{def.friction},
		m_restitution{def.restitution},
		m_filter{def.filter},
		m_isSensor{def.isSensor},
		m_shape{shape},
		m_userData{def.userData}
	{
		assert(body != nullptr);
		assert(shape != nullptr);
		assert(def.density >= 0);
	}

	/// Creates proxies for every child of this fixture's shape.
	/// This sets the proxy count to the child count of the shape.
	void CreateProxies(BlockAllocator& allocator, BroadPhase& broadPhase, const Transformation& xf);

	/// Destroys this fixture's proxies.
	/// This resets the proxy count to 0.
	void DestroyProxies(BlockAllocator& allocator, BroadPhase& broadPhase);

	void Synchronize(BroadPhase& broadPhase, const Transformation& xf1, const Transformation& xf2);

	Body* m_body = nullptr; ///< Parent body. 8-bytes.
	float_t m_density = float_t{0}; ///< Density. 4-bytes.
	Fixture* m_next = nullptr; ///< Next fixture in parent body's fixture list. 8-bytes.
	Shape* m_shape = nullptr; ///< Pointer to shape. Either null or pointer to a heap-memory private copy of the assigned shape. 8-bytes.
	float_t m_friction = float_t{2} / float_t{10}; ///< Friction as a coefficient. 4-bytes.
	float_t m_restitution = float_t{0}; ///< Restitution as a coefficient. 4-bytes.
	FixtureProxy* m_proxies = nullptr; ///< Array of fixture proxies for the assigned shape. 8-bytes.
	child_count_t m_proxyCount = 0; ///< Proxy count. @detail This is the fixture shape's child count after proxy creation. 4-bytes.
	Filter m_filter; ///< Filter object. 8-bytes.
	bool m_isSensor = false; ///< Is/is-not sensor. 4-bytes.
	void* m_userData = nullptr; ///< User data. 8-bytes.
};

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

inline const Filter& Fixture::GetFilterData() const noexcept
{
	return m_filter;
}

inline void* Fixture::GetUserData() const noexcept
{
	return m_userData;
}

inline void Fixture::SetUserData(void* data) noexcept
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

inline void Fixture::SetDensity(float_t density) noexcept
{
	assert(IsValid(density) && density >= float_t{0});
	m_density = density;
}

inline float_t Fixture::GetDensity() const noexcept
{
	return m_density;
}

inline float_t Fixture::GetFriction() const noexcept
{
	return m_friction;
}

inline void Fixture::SetFriction(float_t friction) noexcept
{
	m_friction = friction;
}

inline float_t Fixture::GetRestitution() const noexcept
{
	return m_restitution;
}

inline void Fixture::SetRestitution(float_t restitution) noexcept
{
	m_restitution = restitution;
}

inline bool Fixture::TestPoint(const Vec2& p) const
{
	return GetShape()->TestPoint(m_body->GetTransformation(), p);
}

inline bool Fixture::RayCast(RayCastOutput* output, const RayCastInput& input, child_count_t childIndex) const
{
	return GetShape()->RayCast(output, input, m_body->GetTransformation(), childIndex);
}

inline MassData Fixture::ComputeMassData() const
{
	return GetShape()->ComputeMass(GetDensity());
}

inline const AABB& Fixture::GetAABB(child_count_t childIndex) const
{
	assert(childIndex >= 0);
	assert(childIndex < m_proxyCount);
	return m_proxies[childIndex].aabb;
}

inline void SetAwake(Fixture& f) noexcept
{
	const auto b = f.GetBody();
	if (b)
	{
		b->SetAwake();
	}
}

inline Shape::Type GetType(const Fixture& fixture) noexcept
{
	return fixture.GetShape()->GetType();
}

} // namespace box2d

#endif
