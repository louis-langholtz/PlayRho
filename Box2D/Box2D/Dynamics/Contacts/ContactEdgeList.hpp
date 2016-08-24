//
//  ContactEdgeList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/17/16.
//
//

#ifndef ContactEdgeList_hpp
#define ContactEdgeList_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Dynamics/Contacts/ContactEdgeIterator.hpp>
#include <Box2D/Dynamics/Contacts/ConstContactEdgeIterator.hpp>

namespace box2d {
	
	class ContactEdge;
	
	class ContactEdgeList
	{
	public:
		using iterator = ContactEdgeIterator;
		using const_iterator = ConstContactEdgeIterator;
		using pointer = ContactEdge*;
		using reference = ContactEdge&;
		
		ContactEdgeList() = default;
		constexpr ContactEdgeList(const ContactEdgeList& copy) = delete;
		
		ContactEdgeList& operator= (const ContactEdgeList& rhs) = delete;
		
		iterator begin() noexcept { return iterator(p); }
		iterator end() noexcept { return iterator(nullptr); }
		
		const_iterator begin() const noexcept { return const_iterator{p}; }
		const_iterator end() const noexcept { return const_iterator{nullptr}; }
		
		constexpr bool empty() const noexcept { return p == nullptr; }
		
		constexpr bool operator== (const ContactEdgeList& rhs) const noexcept { return p == rhs.p; }
		constexpr bool operator!= (const ContactEdgeList& rhs) const noexcept { return p != rhs.p; }

		reference front() noexcept { return *p; }

	private:
		friend class ContactManager;
		friend class Body;

		void push_front(pointer value) noexcept;
		void pop_front() noexcept;
		iterator erase(iterator pos);
		
		pointer p = nullptr;
	};
	
}; // namespace box2d

#endif /* ContactEdgeList_hpp */
