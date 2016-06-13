//
//  b2FixtureList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef b2FixtureList_hpp
#define b2FixtureList_hpp

#include <Box2D/Common/FixtureIterator.hpp>
#include <Box2D/Common/ConstFixtureIterator.hpp>

namespace box2d {

class Fixture;

class FixtureList
{
public:
	using iterator = FixtureIterator;
	using const_iterator = ConstFixtureIterator;
	using pointer = Fixture*;
	using reference = Fixture&;
	using const_reference = const Fixture&;
	
	FixtureList() = default;
	FixtureList(const FixtureList& copy) = delete;
	
	FixtureList& operator= (const FixtureList& rhs) = delete;
	
	iterator begin() noexcept { return iterator{&p}; }
	iterator end() noexcept { return iterator{&q}; }
	
	const_iterator begin() const noexcept { return const_iterator{&p}; }
	const_iterator end() const noexcept { return const_iterator{&q}; }
	
	constexpr bool empty() const noexcept { return p == nullptr; }

	constexpr bool operator== (const FixtureList& rhs) const noexcept { return p == rhs.p; }
	constexpr bool operator!= (const FixtureList& rhs) const noexcept { return p != rhs.p; }

	void push_front(pointer value) noexcept;
	void pop_front() noexcept;
	iterator erase(iterator pos);

	reference front() noexcept { return *p; }
	const_reference front() const noexcept { return *p; }

private:
	pointer p = nullptr;
	pointer q = nullptr;
};

} // namespace box2d

#endif /* b2FixtureList_hpp */
