/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include <cps/Config.h>
#include <cps/Attribute.h>

namespace CPS {
	/// Base class of objects having attributes to access member variables.
	class AttributeList {
	protected:

		/// Map of all attributes
		AttributeBase::Map mAttributes;

		// template<typename T, typename... Args>
		// typename Attribute<T>::Ptr addAttribute(const String &name, bool dynamic, Args&&... args) {
		// 	typename Attribute<T>::Ptr newAttr;
		// 	if (dynamic) {
		// 		newAttr = AttributeDynamic<T>::make(std::forward<Args>(args)...);
		// 	} else {
		// 		newAttr = AttributeStatic<T>::make(std::forward<Args>(args)...);
		// 	}
			 
		// 	mAttributes[name] = newAttr;
		// 	return newAttr;
		// }

	public:
		typedef std::shared_ptr<AttributeList> Ptr;

		AttributeList() { };

		const AttributeBase::Map & attributes() { return mAttributes; };

		/// Return pointer to an attribute.
		AttributeBase::Ptr attribute(const String &name) {
			auto it = mAttributes.find(name);
			if (it == mAttributes.end())
				throw InvalidAttributeException();

			return it->second;
		}

		/// Return pointer to an attribute.
		template<typename T>
		typename Attribute<T>::Ptr attribute(const String &name) {
			auto attr = attribute(name);
			auto attrPtr = std::dynamic_pointer_cast<Attribute<T>>(attr.getPtr());

			if (attrPtr == nullptr)
				throw InvalidAttributeException();

			return typename Attribute<T>::Ptr(attrPtr);
		}

		// void reset() {
		// 	for (auto a : mAttributes) {
		// 		a.second->reset();
		// 	}
		// }
	};
}
