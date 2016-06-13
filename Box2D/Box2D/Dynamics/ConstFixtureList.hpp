//
//  ConstFixtureList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef ConstFixtureList_hpp
#define ConstFixtureList_hpp

#include <Box2D/Common/ConstFixtureIterator.hpp>
#include <Box2D/Dynamics/FixtureList.hpp>

namespace box2d {

class Fixture;

class ConstFixtureList
{
public:
	using const_iterator = ConstFixtureIterator;
	using pointer = const Fixture*;
	using reference = const Fixture&;

	ConstFixtureList() = default;
	constexpr ConstFixtureList(const ConstFixtureList& copy) noexcept: p{copy.p} {}
	constexpr ConstFixtureList(pointer f) noexcept: p{f} {}
	constexpr ConstFixtureList(const FixtureList& f) noexcept: p{f.p} {}

	ConstFixtureList& operator= (const ConstFixtureList& rhs) noexcept { p = rhs.p; return *this; }
		
	const_iterator begin() noexcept { return const_iterator{&p}; }
	const_iterator end() noexcept { return const_iterator{&q}; }
	
	constexpr bool operator== (const ConstFixtureList& rhs) const noexcept { return p == rhs.p; }
	constexpr bool operator!= (const ConstFixtureList& rhs) const noexcept { return p != rhs.p; }

	reference front() const noexcept { return *p; }

private:
	pointer p = nullptr;
	pointer q = nullptr;
};

} // namespace box2d

#endif /* ConstFixtureList_hpp */
