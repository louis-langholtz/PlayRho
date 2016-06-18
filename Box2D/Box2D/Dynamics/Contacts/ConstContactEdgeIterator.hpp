//
//  ConstContactEdgeIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/17/16.
//
//

#ifndef ConstContactEdgeIterator_hpp
#define ConstContactEdgeIterator_hpp

#include <iterator>

namespace box2d {
	
	class ContactEdge;
	
	class ConstContactEdgeIterator: public std::iterator<std::forward_iterator_tag, const ContactEdge>
	{
	public:
		constexpr explicit ConstContactEdgeIterator(pointer b) noexcept: p{b} {}
		constexpr ConstContactEdgeIterator(const ConstContactEdgeIterator& it) noexcept: p{it.p} {}
		
		ConstContactEdgeIterator& operator++() noexcept { p = next(p); return *this; }
		ConstContactEdgeIterator operator++(int) { ConstContactEdgeIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const ConstContactEdgeIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const ConstContactEdgeIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		
	private:
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* ConstContactEdgeIterator_hpp */
