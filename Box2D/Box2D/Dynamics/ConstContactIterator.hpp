//
//  ConstContactIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/14/16.
//
//

#ifndef ConstContactIterator_hpp
#define ConstContactIterator_hpp

#include <iterator>

namespace box2d {
	
	class Contact;
	
	class ConstContactIterator: public std::iterator<std::forward_iterator_tag, const Contact>
	{
	public:
		constexpr explicit ConstContactIterator(pointer b) noexcept: p{b} {}
		constexpr ConstContactIterator(const ConstContactIterator& it) noexcept: p{it.p} {}
		
		ConstContactIterator& operator++() noexcept { p = next(p); return *this; }
		ConstContactIterator operator++(int) { ConstContactIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const ConstContactIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const ConstContactIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		
	private:
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* ConstContactIterator_hpp */
