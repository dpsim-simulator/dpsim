/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief resistor inductor series element
	class ResIndSeries :
		public SimPowerComp<Complex>,
		public SharedFactory<ResIndSeries> {
	protected:
		/// Inductance [H]
		Real mInductance;
		///Resistance [ohm]
		Real mResistance;
		///Conductance [S]
		Real mConductance;
		/// Impedance
		Real mImpedance;
		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value for harmonics
		MatrixComp mPrevCurrFac;
	public:
		/// Defines UID, name and log level
		ResIndSeries(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		ResIndSeries(String name, Logger::Level logLevel = Logger::Level::off)
			: ResIndSeries(name, name, logLevel) { }

		// #### General ####
		/// Sets model specific parameters
		void setParameters(Real resistance, Real inductance) {
			mResistance = resistance;
			mInductance = inductance;
		}
		/// Return new instance with the same parameters
		SimPowerComp<Complex>::Ptr clone(String name);
		/// Initializes state variables considering the number of frequencies
		void initialize(Matrix frequencies);
		/// Initializes states from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system results
		void mnaUpdateCurrent();

		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(ResIndSeries& resIndSeries) :
				Task(resIndSeries.mName + ".MnaPreStep"), mResIndSeries(resIndSeries) {
				mResIndSeries.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mResIndSeries.mnaPreStep(time, timeStepCount); };
		private:
			ResIndSeries& mResIndSeries;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(ResIndSeries& resIndSeries, Attribute<Matrix>::Ptr leftVector) :
				Task(resIndSeries.mName + ".MnaPostStep"), mResIndSeries(resIndSeries), mLeftVector(leftVector) {
				mResIndSeries.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mResIndSeries.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			ResIndSeries& mResIndSeries;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
