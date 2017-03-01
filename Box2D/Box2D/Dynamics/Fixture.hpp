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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include <memory>
#include <Box2D/Common/Math.hpp>

namespace box2d {

class BlockAllocator;
class Body;
class BroadPhase;
class Fixture;
struct FixtureProxy;
class AABB;
class Shape;

/// This holds contact filtering data.
struct Filter
{
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

/// Fixture definition.
/// @detail
/// A fixture definition is used to create a fixture.
/// @sa Body::CreateFixture.
struct FixtureDef
{
	constexpr FixtureDef& UseUserData(void* value) noexcept;
	constexpr FixtureDef& UseFriction(RealNum value) noexcept;
	constexpr FixtureDef& UseRestitution(RealNum value) noexcept;
	constexpr FixtureDef& UseDensity(RealNum value) noexcept;
	constexpr FixtureDef& UseIsSensor(bool value) noexcept;
	constexpr FixtureDef& UseFilter(Filter value) noexcept;

	/// Use this to store application specific fixture data.
	void* userData = nullptr;

	/// Friction coefficient.
	///
	/// @note This must be a value between 0 and +infinity.
	/// @note This is usually in the range [0,1].
	/// @note The square-root of the product of this value multiplied by a touching fixture's
	/// friction becomes the friction coefficient for the contact.
	///
	RealNum friction = RealNum{2} / RealNum{10};

	/// Restitution (elasticity) of the associated shape.
	///
	/// @note This should be a valid finite value.
 	/// @note This is usually in the range [0,1].
	///
	RealNum restitution = RealNum{0};

	/// Density of the associated shape.
	///
	/// @note This is usually in kg/m^2.
	/// @note This must be a non-negative value.
	/// @note Use 0 to indicate that the shape's associated mass should be 0.
	///
	RealNum density = RealNum{0};

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor = false;

	/// Contact filtering data.
	Filter filter;
};

constexpr inline FixtureDef& FixtureDef::UseUserData(void* value) noexcept
{
	userData = value;
	return *this;
}
	
constexpr inline FixtureDef& FixtureDef::UseFriction(RealNum value) noexcept
{
	friction = value;
	return *this;
}
	
constexpr inline FixtureDef& FixtureDef::UseRestitution(RealNum value) noexcept
{
	restitution = value;
	return *this;
}
	
constexpr inline FixtureDef& FixtureDef::UseDensity(RealNum value) noexcept
{
	density = value;
	return *this;
}
	
constexpr inline FixtureDef& FixtureDef::UseIsSensor(bool value) noexcept
{
	isSensor = value;
	return *this;
}
	
constexpr inline FixtureDef& FixtureDef::UseFilter(Filter value) noexcept
{
	filter = value;
	return *this;
}

/// Fixture.
///
/// @detail
/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
///
/// @warning you cannot reuse fixtures.
/// @note Fixtures are created via Body::CreateFixture.
/// @note This structure is 72-bytes large (using a 4-byte RealNum on at least one 64-bit architecture/build).
///
class Fixture
{
public:
	Fixture() = delete; // explicitly deleted

	/// Gets the parent body of this fixture.
	/// @detail This is nullptr if the fixture is not attached.
	/// @return the parent body.
	Body* GetBody() noexcept;
	
	/// Gets the parent body of this fixture.
	/// @detail This is nullptr if the fixture is not attached.
	/// @return the parent body.
	const Body* GetBody() const noexcept;

	/// Gets the child shape.
	/// @detail The shape is not modifiable. Use a new fixture instead.
	const Shape* GetShape() noexcept;

	/// Gets the child shape.
	/// @detail The shape is not modifiable. Use a new fixture instead.
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

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) noexcept;

	/// Sets the density of this fixture.
	/// @note This will _not_ automatically adjust the mass of the body.
	///   You must call Body::ResetMassData to update the body's mass.
	/// @warning Behavior is undefined if given a negative value.
	/// @param density Non-negative density in kg/m^2.
	void SetDensity(RealNum density) noexcept;

	/// Gets the density of this fixture.
	/// @return Non-negative density in kg/m^2.
	RealNum GetDensity() const noexcept;

	/// Gets the coefficient of friction.
	RealNum GetFriction() const noexcept;

	/// Sets the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	void SetFriction(RealNum friction) noexcept;

	/// Gets the coefficient of restitution.
	RealNum GetRestitution() const noexcept;

	/// Sets the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	void SetRestitution(RealNum restitution) noexcept;

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	const AABB& GetAABB(child_count_t childIndex) const;

	child_count_t GetProxyCount() const;
	const FixtureProxy* GetProxy(child_count_t index) const;

private:

	friend class Body;
	friend class World;
	friend class ContactManager;
	friend class FixtureList;
	friend class FixtureIterator;
	friend class ConstFixtureIterator;

	/// Initializing constructor.
	///
	/// @warning Behavior is undefined if a <code>nullptr</code> initial body setting is used.
	/// @warning Behavior is undefined if a <code>nullptr</code> initial shape setting is used.
	/// @warning Behavior is undefined if a negative initial density setting is used.
	/// @warning Behavior is undefined if a negative initial friction setting is used.
	/// @warning Behavior is undefined if the restitution value is not less than infinity.
	/// @warning Behavior is undefined if the restitution value is not greater than -infinity.
	///
	/// @param body Body the new fixture is to be associated with.
	/// @param def Initial fixture settings.
	///    Friction must be greater-than-or-equal-to zero.
	///    Density must be greater-than-or-equal-to zero.
	/// @param shape Sharable shape to associate fixture with. Must be non-null.
	///
	Fixture(Body* body, const FixtureDef& def, std::shared_ptr<const Shape> shape):
		m_body{body},
		m_shape{shape},
		m_density{Max(def.density, RealNum{0})},
		m_friction{def.friction},
		m_restitution{def.restitution},
		m_filter{def.filter},
		m_isSensor{def.isSensor},
		m_userData{def.userData}
	{
		assert(body);
		assert(shape);
		assert(def.density >= 0);
		assert(def.friction >= 0);
		assert(def.restitution < std::numeric_limits<decltype(def.restitution)>::infinity());
		assert(def.restitution > -std::numeric_limits<decltype(def.restitution)>::infinity());
	}

	/// Creates proxies for every child of this fixture's shape.
	/// This sets the proxy count to the child count of the shape.
	void CreateProxies(BlockAllocator& allocator, BroadPhase& broadPhase, const Transformation& xf);

	/// Destroys this fixture's proxies.
	/// This resets the proxy count to 0.
	void DestroyProxies(BlockAllocator& allocator, BroadPhase& broadPhase);

	/// Touches each proxy so that new pairs may be created.
	void TouchProxies(BroadPhase& broadPhase);

	void Synchronize(BroadPhase& broadPhase, const Transformation& xf1, const Transformation& xf2);

	// Data ordered here for memory compaction.
	
	// 0-bytes of memory (at first).
	Body* const m_body = nullptr; ///< Parent body. Set on construction. 8-bytes.

	/// Shape (of fixture).
	/// @note Set on construction.
	/// @note Either null or pointer to a heap-memory private copy of the assigned shape.
	/// @note 16-bytes.
	std::shared_ptr<const Shape> m_shape;

	Fixture* m_next = nullptr; ///< Next fixture in parent body's fixture list. 8-bytes.
	FixtureProxy* m_proxies = nullptr; ///< Array of fixture proxies for the assigned shape. 8-bytes.
	void* m_userData = nullptr; ///< User data. 8-bytes.
	// 48-bytes so far.
	RealNum m_density = 0; ///< Density. 4-bytes.
	RealNum m_friction = RealNum{2} / RealNum{10}; ///< Friction as a coefficient. 4-bytes.
	RealNum m_restitution = 0; ///< Restitution as a coefficient. 4-bytes.
	child_count_t m_proxyCount = 0; ///< Proxy count. @detail This is the fixture shape's child count after proxy creation. 4-bytes.
	// 48 + 16 = 64-bytes now.
	Filter m_filter; ///< Filter object. 6-bytes.
	bool m_isSensor = false; ///< Is/is-not sensor. 1-bytes.

	// 71-bytes data + 1-byte alignment padding is 72-bytes.
};

inline const Shape* Fixture::GetShape() noexcept
{
	return m_shape.get();
}

inline const Shape* Fixture::GetShape() const noexcept
{
	return m_shape.get();
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

inline void Fixture::SetDensity(RealNum density) noexcept
{
	assert(IsValid(density) && density >= RealNum{0});
	m_density = density;
}

inline RealNum Fixture::GetDensity() const noexcept
{
	return m_density;
}

inline RealNum Fixture::GetFriction() const noexcept
{
	return m_friction;
}

inline void Fixture::SetFriction(RealNum friction) noexcept
{
	m_friction = friction;
}

inline RealNum Fixture::GetRestitution() const noexcept
{
	return m_restitution;
}

inline void Fixture::SetRestitution(RealNum restitution) noexcept
{
	m_restitution = restitution;
}

inline child_count_t Fixture::GetProxyCount() const
{
	return m_proxyCount;
}

/// Test a point for containment in a fixture.
/// @param f Fixture to use for test.
/// @param p Point in world coordinates.
bool TestPoint(const Fixture& f, const Vec2 p);

void SetAwake(Fixture& f) noexcept;

} // namespace box2d

#endif
