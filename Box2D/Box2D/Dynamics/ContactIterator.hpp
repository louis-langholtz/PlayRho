//
//  ContactIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/14/16.
//
//

#ifndef ContactIterator_hpp
#define ContactIterator_hpp

#include <iterator>

namespace box2d {
	
	class Contact;
	
	class ContactIterator: public std::iterator<std::forward_iterator_tag, Contact>
	{
	public:
		constexpr explicit ContactIterator(pointer b) noexcept: p{b} {}
		constexpr ContactIterator(const ContactIterator& it) noexcept: p{it.p} {}
		
		ContactIterator& operator++() noexcept { p = next(p); return *this; }
		ContactIterator operator++(int) { ContactIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const ContactIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const ContactIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		pointer operator->() const noexcept { return p; }
		
	private:
		friend class ContactList;
		
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* ContactIterator_hpp */
