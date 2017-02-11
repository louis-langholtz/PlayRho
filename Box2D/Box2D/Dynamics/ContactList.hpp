//
//  ContactList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/14/16.
//
//

#ifndef ContactList_hpp
#define ContactList_hpp

#include <Box2D/Common/Settings.hpp>
#include <Box2D/Dynamics/ContactIterator.hpp>
#include <Box2D/Dynamics/ConstContactIterator.hpp>

namespace box2d {
	
	class Contact;
	
	class ContactList
	{
	public:
		using iterator = ContactIterator;
		using const_iterator = ConstContactIterator;
		
		using pointer = Contact*;
		using reference = Contact&;
		using size_type = contact_count_t;
		
		ContactList() = default;

		constexpr ContactList(const ContactList& copy) = delete;
		
		ContactList& operator= (const ContactList& rhs) = delete;
		
		iterator begin() noexcept { return iterator(p); }
		iterator end() noexcept { return iterator(nullptr); }
		
		const_iterator begin() const noexcept { return const_iterator{p}; }
		const_iterator end() const noexcept { return const_iterator{nullptr}; }
		
		constexpr bool empty() const noexcept { return p == nullptr; }
		size_type size() const noexcept { return n; }
		constexpr size_type max_size() const noexcept { return MaxContacts; }
		
		constexpr bool operator== (const ContactList& rhs) const noexcept { return p == rhs.p; }
		constexpr bool operator!= (const ContactList& rhs) const noexcept { return p != rhs.p; }
		
		reference front() noexcept { return *p; }

	private:
		friend class ContactManager;

		void push_front(pointer value) noexcept;
		iterator erase(iterator pos);
		
		pointer p = nullptr;
		contact_count_t n = 0;
	};
	
}; // namespace box2d

#endif /* ContactList_hpp */
