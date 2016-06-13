//
//  ConstBodyList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef ConstBodyList_hpp
#define ConstBodyList_hpp

#include <Box2D/Common/ConstBodyIterator.hpp>
#include <Box2D/Dynamics/BodyList.hpp>

namespace box2d {

class Body;

class ConstBodyList
{
public:
	using const_iterator = ConstBodyIterator;
	using const_pointer = const Body*;

	ConstBodyList() = default;
	constexpr ConstBodyList(const ConstBodyList& copy) noexcept: p{copy.p} {}
	constexpr ConstBodyList(const_pointer b) noexcept: p{b} {}
	constexpr ConstBodyList(const BodyList& b) noexcept: p{b.get()} {}

	ConstBodyList& operator= (const ConstBodyList& rhs) noexcept { p = rhs.p; return *this; }

	const_iterator begin() noexcept { return const_iterator(p); }
	const_iterator end() noexcept { return const_iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
	constexpr bool empty() const noexcept { return p == nullptr; }
	constexpr explicit operator bool() const noexcept { return p != nullptr; }
	constexpr bool operator! () const noexcept { return p == nullptr; }
	constexpr bool operator== (const ConstBodyList& rhs) const noexcept { return p == rhs.p; }
	constexpr bool operator!= (const ConstBodyList& rhs) const noexcept { return p != rhs.p; }

	constexpr const_pointer get() const noexcept { return p; }

private:
	const_pointer p = nullptr;
};

} // namespace box2d

#endif /* ConstBodyList_hpp */
