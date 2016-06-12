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

namespace box2d {

class Body;

class ConstBodyList
{
public:
	using const_iterator = ConstBodyIterator;
	using const_pointer = const Body*;

	ConstBodyList() = default;
	constexpr ConstBodyList(const_pointer b) noexcept: p{b} {}
	
	const_iterator begin() noexcept { return const_iterator(p); }
	const_iterator end() noexcept { return const_iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
	constexpr explicit operator bool() const noexcept { return p != nullptr; }
	constexpr bool operator! () const noexcept { return p == nullptr; }

private:
	const_pointer p = nullptr;
};

} // namespace box2d

#endif /* ConstBodyList_hpp */
