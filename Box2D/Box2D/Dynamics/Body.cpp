/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>

#include <iterator>

using namespace box2d;

bool Body::IsValid(const Shape& shape)
{
	if (!(shape.GetDensity() >= Density{0}))
	{
		return false;
	}
	if (!(shape.GetFriction() >= 0))
	{
		return false;
	}
	if (!(shape.GetRestitution() < std::numeric_limits<decltype(shape.GetRestitution())>::infinity()))
	{
		return false;
	}
	if (!(shape.GetRestitution() > -std::numeric_limits<decltype(shape.GetRestitution())>::infinity()))
	{
		return false;
	}
	return true;
}

const FixtureDef& box2d::GetDefaultFixtureDef() noexcept
{
	static const auto def = FixtureDef{};
	return def;
}

Body::FlagsType Body::GetFlags(const BodyDef& bd) noexcept
{
	// @invariant Only bodies that allow sleeping, can be put to sleep.
	// @invariant Only "speedable" bodies can be awake.
	// @invariant Only "speedable" bodies can have non-zero velocities.
	// @invariant Only "accelerable" bodies can have non-zero accelerations.
	// @invariant Only "accelerable" bodies can have non-zero "under-active" times.
	
	auto flags = GetFlags(bd.type);
	if (bd.bullet)
	{
		flags |= e_impenetrableFlag;
	}
	if (bd.fixedRotation)
	{
		flags |= e_fixedRotationFlag;
	}
	if (bd.allowSleep)
	{
		flags |= e_autoSleepFlag;
	}
	if (bd.awake)
	{
		if (flags & e_velocityFlag)
		{
			flags |= e_awakeFlag;
		}
	}
	else
	{
		if (!bd.allowSleep && (flags & e_velocityFlag))
		{
			flags |= e_awakeFlag;
		}
	}
	if (bd.enabled)
	{
		flags |= e_enabledFlag;
	}
	return flags;
}

Body::Body(const BodyDef& bd, World* world):
	m_flags{GetFlags(bd)},
	m_xf{bd.position, UnitVec2{bd.angle}},
	m_world{world},
	m_sweep{Position{bd.position, bd.angle}},
	m_invMass{(bd.type == BodyType::Dynamic)? InverseMass{RealNum{1} / Kilogram}: InverseMass{0}},
	m_linearDamping{bd.linearDamping},
	m_angularDamping{bd.angularDamping},
	m_userData{bd.userData}
{
	assert(::box2d::IsValid(bd.position));
	assert(::box2d::IsValid(bd.linearVelocity));
	assert(::box2d::IsValid(bd.angle));
	assert(::box2d::IsValid(bd.angularVelocity));
	assert(::box2d::IsValid(bd.angularDamping) && (bd.angularDamping >= RealNum{0}));
	assert(::box2d::IsValid(bd.linearDamping) && (bd.linearDamping >= RealNum{0}));
	
	SetVelocity(Velocity{bd.linearVelocity, bd.angularVelocity});
	SetAcceleration(bd.linearAcceleration, bd.angularAcceleration);
	SetUnderActiveTime(bd.underActiveTime);
}

Body::~Body()
{
	assert(m_joints.empty());
	assert(m_contacts.empty());
	assert(m_fixtures.empty());
}

void Body::SetType(BodyType type)
{
	m_world->SetType(*this, type);
}

Fixture* Body::CreateFixture(std::shared_ptr<const Shape> shape, const FixtureDef& def, bool resetMassData)
{
	return m_world->CreateFixture(*this, shape, def, resetMassData);
}

bool Body::DestroyFixture(Fixture* fixture, bool resetMassData)
{
	if (fixture->GetBody() != this)
	{
		return false;
	}
	return m_world->DestroyFixture(fixture, resetMassData);
}

void Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.

	// Non-dynamic bodies (Static and kinematic ones) have zero mass.
	if (!IsAccelerable())
	{
		m_invMass = 0;
		m_invRotI = 0;
		m_sweep = Sweep{Position{GetLocation(), GetAngle()}};
		UnsetMassDataDirty();
		return;
	}

	const auto massData = ComputeMassData(*this);

	// Force all dynamic bodies to have a positive mass.
	const auto mass = (massData.mass > Mass{0})? massData.mass: Kilogram;
	m_invMass = RealNum{1} / mass;
	
	// Compute center of mass.
	const auto localCenter = massData.center * RealNum{m_invMass * Kilogram};
	
	if ((massData.I > MomentOfInertia{0}) && (!IsFixedRotation()))
	{
		// Center the inertia about the center of mass.
		const auto lengthSquared = GetLengthSquared(localCenter) * SquareMeter;
		const auto I = massData.I - MomentOfInertia{(mass * lengthSquared / SquareRadian)};
		//assert((massData.I - mass * lengthSquared) > 0);
		m_invRotI = RealNum{(SquareMeter * Kilogram / SquareRadian) / I};
	}
	else
	{
		m_invRotI = 0;
	}

	// Move center of mass.
	const auto oldCenter = GetWorldCenter();
	m_sweep = Sweep{Position{Transform(localCenter, GetTransformation()), GetAngle()}, localCenter};

	// Update center of mass velocity.
	m_velocity.linear += GetRevPerpendicular(GetWorldCenter() - oldCenter) * RealNum{m_velocity.angular / Radian};
	
	UnsetMassDataDirty();
}

void Body::SetMassData(const MassData& massData)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	if (!IsAccelerable())
	{
		return;
	}

	const auto mass = (massData.mass > Mass{0})? massData.mass: Kilogram;
	m_invMass = RealNum{1} / mass;

	if ((massData.I > MomentOfInertia{0}) && (!IsFixedRotation()))
	{
		const auto lengthSquared = GetLengthSquared(massData.center) * SquareMeter;
		const auto I = massData.I - MomentOfInertia{mass * lengthSquared / SquareRadian};
		assert(I > MomentOfInertia{0});
		m_invRotI = RealNum{(SquareMeter * Kilogram / SquareRadian) / I};
	}
	else
	{
		m_invRotI = 0;
	}

	// Move center of mass.
	const auto oldCenter = GetWorldCenter();

	m_sweep = Sweep{Position{Transform(massData.center, GetTransformation()), GetAngle()}, massData.center};

	// Update center of mass velocity.
	m_velocity.linear += GetRevPerpendicular(GetWorldCenter() - oldCenter) * RealNum{m_velocity.angular / Radian};
	
	UnsetMassDataDirty();
}

void Body::SetVelocity(const Velocity& velocity) noexcept
{
	if ((velocity.linear != Vec2_zero) || (velocity.angular != Angle{0}))
	{
		if (!IsSpeedable())
		{
			return;
		}
		SetAwakeFlag();
	}
	m_velocity = velocity;
}

void Body::SetAcceleration(const Vec2 linear, const Angle angular) noexcept
{
	assert(::box2d::IsValid(linear));
	assert(::box2d::IsValid(angular));

	if ((linear != Vec2_zero) || (angular != Angle{0}))
	{
		if (!IsAccelerable())
		{
			return;
		}
		SetAwakeFlag();
	}
	m_linearAcceleration = linear;
	m_angularAcceleration = angular;
}

void Body::SetTransform(const Vec2 position, Angle angle)
{
	assert(::box2d::IsValid(position));
	assert(::box2d::IsValid(angle));
	assert(!GetWorld()->IsLocked());
	
	if (GetWorld()->IsLocked())
	{
		return;
	}
	
	const auto xfm = Transformation{position, UnitVec2{angle}};
	SetTransformation(xfm);
	
	const auto sweep = Sweep{Position{Transform(GetLocalCenter(), xfm), angle}, GetLocalCenter()};
	m_sweep = sweep;
	
	GetWorld()->RegisterForProxies(this);
}

void Body::SetEnabled(bool flag)
{
	if (IsEnabled() == flag)
	{
		return;
	}
	
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}
	
	if (flag)
	{
		SetEnabledFlag();
	}
	else
	{
		UnsetEnabledFlag();
	}
	
	// Register for proxies so contacts created or destroyed the next time step.
	for (auto&& fixture: GetFixtures())
	{
		m_world->RegisterForProxies(fixture);
	}
}

void Body::SetFixedRotation(bool flag)
{
	const auto status = IsFixedRotation();
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_fixedRotationFlag;
	}
	else
	{
		m_flags &= ~e_fixedRotationFlag;
	}

	m_velocity.angular = Angle{0};

	ResetMassData();
}

// Free functions...

bool box2d::ShouldCollide(const Body& lhs, const Body& rhs) noexcept
{
	// At least one body should be accelerable/dynamic.
	if (!lhs.IsAccelerable() && !rhs.IsAccelerable())
	{
		return false;
	}
	
	// Does a joint prevent collision?
	for (auto&& joint: lhs.GetJoints())
	{
		if (joint->GetBodyA() == &rhs || joint->GetBodyB() == &rhs)
		{
			if (!(joint->GetCollideConnected()))
			{
				return false;
			}
		}
	}
	
	return true;
}

void box2d::DestroyFixtures(Body& body)
{
	while (!body.GetFixtures().empty())
	{
		const auto fixture = body.GetFixtures().front();
		body.DestroyFixture(fixture);
	}
}

box2d::size_t box2d::GetWorldIndex(const Body* body)
{
	if (body)
	{
		const auto world = body->GetWorld();
		
		auto i = size_t{0};
		for (auto&& b: world->GetBodies())
		{
			if (b == body)
			{
				return i;
			}
			++i;
		}
	}
	return size_t(-1);
}

Velocity box2d::GetVelocity(const Body& body, Time h) noexcept
{
	const auto timeInSecs = RealNum{h / Second};
	assert(IsValid(timeInSecs));

	// Integrate velocity and apply damping.
	auto velocity = body.GetVelocity();
	if (body.IsAccelerable())
	{
		// Integrate velocities.
		velocity.linear += timeInSecs * body.GetLinearAcceleration();
		velocity.angular += timeInSecs * body.GetAngularAcceleration();
		
		// Apply damping.
		// Ordinary differential equation: dv/dt + c * v = 0
		//                       Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation (see https://en.wikipedia.org/wiki/Pad%C3%A9_approximant ):
		// v2 = v1 * 1 / (1 + c * dt)
		velocity.linear  /= (1 + timeInSecs * body.GetLinearDamping());
		velocity.angular /= (1 + timeInSecs * body.GetAngularDamping());
	}
	return velocity;
}

size_t box2d::GetFixtureCount(const Body& body)
{
	const auto& fixtures = body.GetFixtures();
	return static_cast<size_t>(std::distance(std::begin(fixtures), std::end(fixtures)));
}

MassData box2d::ComputeMassData(const Body& body) noexcept
{
	auto mass = Mass{0};
	auto I = MomentOfInertia{0};
	auto center = Vec2_zero;
	for (auto&& fixture: body.GetFixtures())
	{
		if (fixture->GetDensity() > Density{0})
		{
			const auto massData = GetMassData(*fixture);
			mass += massData.mass;
			center += RealNum{massData.mass / Kilogram} * massData.center;
			I += massData.I;
		}
	}
	return MassData{mass, center, I};
}

void box2d::RotateAboutWorldPoint(Body& body, Angle amount, Vec2 worldPoint)
{
	const auto xfm = body.GetTransformation();
	const auto p = xfm.p - worldPoint;
	const auto c = std::cos(amount / Radian);
	const auto s = std::sin(amount / Radian);
	const auto x = p.x * c - p.y * s;
	const auto y = p.x * s + p.y * c;
	const auto pos = Vec2{x, y} + worldPoint;
	const auto angle = GetAngle(xfm.q) + amount;
	body.SetTransform(pos, angle);
}

void box2d::RotateAboutLocalPoint(Body& body, Angle amount, Vec2 localPoint)
{
	RotateAboutWorldPoint(body, amount, GetWorldPoint(body, localPoint));
}

Vec2 box2d::GetCentripetalForce(const Body& body, const Vec2 axis)
{
	// For background on centripetal force, see:
	//   https://en.wikipedia.org/wiki/Centripetal_force

	const auto velocity = GetLength(GetLinearVelocity(body));
	const auto location = body.GetLocation();
	const auto mass = GetMass(body);
	const auto delta = axis - location;
	const auto radius = GetLength(delta);
	const auto dir = delta / radius;
	return dir * (RealNum{mass / Kilogram} * Square(velocity) / radius);
}
