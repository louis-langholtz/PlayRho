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
	using size_type = body_count_t;

	ConstBodyList() = default;
	constexpr ConstBodyList(const ConstBodyList& copy) noexcept: m_bodies{copy.m_bodies} {}
	constexpr ConstBodyList(const BodyList& b) noexcept: m_bodies{&b} {}

	ConstBodyList& operator= (const ConstBodyList& rhs) noexcept { m_bodies = rhs.m_bodies; return *this; }

	const_iterator begin() noexcept { return const_iterator(m_bodies->p); }
	const_iterator end() noexcept { return const_iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(m_bodies->p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
	constexpr bool empty() const noexcept { return m_bodies == nullptr || m_bodies->p == nullptr; }
	size_type size() const noexcept { return (m_bodies == nullptr)? 0: m_bodies->n; }
	constexpr size_type max_size() const noexcept { return MaxBodies; }

	constexpr bool operator== (const ConstBodyList& rhs) const noexcept { return m_bodies == rhs.m_bodies; }
	constexpr bool operator!= (const ConstBodyList& rhs) const noexcept { return m_bodies != rhs.m_bodies; }

private:
	const BodyList* m_bodies = nullptr;
};

} // namespace box2d

#endif /* ConstBodyList_hpp */
