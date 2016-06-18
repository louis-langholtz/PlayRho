//
//  ContactEdgeIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/17/16.
//
//

#ifndef ContactEdgeIterator_hpp
#define ContactEdgeIterator_hpp

#include <iterator>

namespace box2d {
	
	class ContactEdge;
	
	class ContactEdgeIterator: public std::iterator<std::forward_iterator_tag, ContactEdge>
	{
	public:
		constexpr explicit ContactEdgeIterator(pointer b) noexcept: p{b} {}
		constexpr ContactEdgeIterator(const ContactEdgeIterator& it) noexcept: p{it.p} {}
		
		ContactEdgeIterator& operator++() noexcept { p = next(p); return *this; }
		ContactEdgeIterator operator++(int) { ContactEdgeIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const ContactEdgeIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const ContactEdgeIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		
	private:
		friend class ContactEdgeList;
		
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* ContactEdgeIterator_hpp */
