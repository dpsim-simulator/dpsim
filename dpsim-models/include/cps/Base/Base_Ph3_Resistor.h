/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>
#include <cps/AttributeList.h>
namespace CPS {
namespace Base {
namespace Ph3 {
	class Resistor {
	protected:
		///Conductance [S]
		///CHECK: Does this have to be its own member variable?
		Matrix mConductance;
	public:
		///Resistance [ohm]
		CPS::Attribute<Matrix>::Ptr mResistance;
		/// Sets model specific parameters
		void setParameters(Matrix resistance) {
			**mResistance = resistance;
		}
	};
}
}
}