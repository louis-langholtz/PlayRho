//
//  BodyList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef BodyList_hpp
#define BodyList_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Common/BodyIterator.hpp>
#include <Box2D/Common/ConstBodyIterator.hpp>

namespace box2d {

class Body;

class BodyList
{
public:
	using iterator = BodyIterator;
	using const_iterator = ConstBodyIterator;
	using pointer = Body*;
	using size_type = body_count_t;

	BodyList() = default;
	constexpr BodyList(const BodyList& copy) noexcept: p{copy.p} {}
	constexpr BodyList(pointer b) noexcept: p{b} {}
	
	BodyList& operator= (const BodyList& rhs) noexcept { p = rhs.p; return *this; }

	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
	constexpr bool empty() const noexcept { return p == nullptr; }
	size_type size() const noexcept { return n; }
	constexpr size_type max_size() const noexcept { return MaxBodies; }

	constexpr explicit operator bool() const noexcept { return p != nullptr; }
	constexpr bool operator! () const noexcept { return p == nullptr; }
	constexpr bool operator== (const BodyList& rhs) const noexcept { return p == rhs.p; }
	constexpr bool operator!= (const BodyList& rhs) const noexcept { return p != rhs.p; }

	constexpr pointer get() const noexcept { return p; }
	pointer operator-> () const { return p; }
	typename std::add_lvalue_reference<Body>::type operator*() const { return *p; }

	void push_front(pointer value) noexcept;
	iterator erase(iterator pos);

private:
	pointer p = nullptr;
	size_type n = 0;
};

} // namespace box2d

#endif /* BodyList_hpp */
