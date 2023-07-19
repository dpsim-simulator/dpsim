/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>
#include <dpsim-models/EMT/EMT_Ph3_Transformer.h>
#include <dpsim-models/Base/Base_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/Signal/VCO.h>
#include <dpsim-models/Signal/VoltageControllerVSI.h>
#include <dpsim-models/Signal/PowerControllerVSI.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	class VSIVoltageControlVCO :
		public Base::AvVoltageSourceInverterDQ,
		public CompositePowerComp<Real>,
		public SharedFactory<VSIVoltageControlVCO> {
	protected:

		// ### General Parameters ###
		/// Nominal system angle
		Real mThetaN = 0;
		/// Nominal voltage
		Real mVnom;
		/// Simulation step
		Real mTimeStep;
		/// Active power reference

		// ### Control Subcomponents ###
		/// VCO
		std::shared_ptr<Signal::VCO> mVCO;
		/// Power Controller
		std::shared_ptr<Signal::VoltageControllerVSI> mVoltageControllerVSI;

		// ### Electrical Subcomponents ###	
		/// Controlled voltage source
		std::shared_ptr<EMT::Ph3::VoltageSource> mSubCtrledVoltageSource;
		/// Resistor Rf as part of LCL filter
		std::shared_ptr<EMT::Ph3::Resistor> mSubResistorF;
		/// Capacitor Cf as part of LCL filter
		std::shared_ptr<EMT::Ph3::Capacitor> mSubCapacitorF;
		/// Inductor Lf as part of LCL filter
		std::shared_ptr<EMT::Ph3::Inductor> mSubInductorF;
		/// Resistor Rc as part of LCL filter
		std::shared_ptr<EMT::Ph3::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<EMT::Ph3::Transformer> mConnectionTransformer;


		/// Flag for connection transformer usage
		Bool mWithConnectionTransformer=false;
		/// Flag for controller usage
		Bool mWithControl=true;

		// #### solver ####
		///
		std::vector<const Matrix*> mRightVectorStamps;

	public:
		// ### General Parameters ###

		/// Nominal frequency
		const Attribute<Real>::Ptr mOmegaN;
		/// Voltage d reference
		const Attribute<Real>::Ptr mVdRef;
		/// Voltage q reference
		const Attribute<Real>::Ptr mVqRef;

		// ### Inverter Interfacing Variables ###
		// Control inputs
		/// Measured voltage d-axis in local reference frame
		/// Input for VCO
		const Attribute<Real>::Ptr mVcd;
		/// Measured voltage q-axis in local reference frame
		/// Input for VCO
		const Attribute<Real>::Ptr mVcq;
		/// Measured current d-axis in local reference frame
		const Attribute<Real>::Ptr mIrcd;
		/// Measured current q-axis in local reference frame
		const Attribute<Real>::Ptr mIrcq;

		const Attribute<Real>::Ptr mElecActivePower;
		const Attribute<Real>::Ptr mElecPassivePower;

		// Control outputs
		/// Voltage as control output after transformation interface
		const Attribute<Matrix>::Ptr mVsref;

		// Sub voltage source
		const Attribute<Matrix>::Ptr mVs;

		// VCO
		const Attribute<Real>::Ptr mVCOOutput;

		// input, state and output vector for logging
		const Attribute<Matrix>::Ptr mVoltagectrlInputs;
		const Attribute<Matrix>::Ptr mVoltagectrlStates;
		const Attribute<Matrix>::Ptr mVoltagectrlOutputs;

		/// Defines name amd logging level
		VSIVoltageControlVCO(String name, Logger::Level logLevel = Logger::Level::off)
			: VSIVoltageControlVCO(name, name, logLevel) {}
		/// Defines UID, name, logging level and connection trafo existence
		VSIVoltageControlVCO(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Setter for gengit eral parameters of inverter
		void setParameters(Real sysOmega, Real VdRef, Real VqRef);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_nominal);
		/// Setter for parameters of transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower,
			Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega);
		/// Setter for parameters of filter
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		/// Setter for initial values applied in controllers
		void setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);

		void withControl(Bool controlOn) { mWithControl = controlOn; };

		// #### Mathematical Matrix Transforms ####
		///
		Matrix getParkTransformMatrixPowerInvariant(Real theta);
		///
		Matrix parkTransformPowerInvariant(Real theta, const Matrix &fabc);
		///
		Matrix getInverseParkTransformMatrixPowerInvariant(Real theta);
		///
		Matrix inverseParkTransformPowerInvariant(Real theta, const Matrix &fdq);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Updates current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates voltage across component
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// MNA pre step operations
		void mnaParentPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		// #### Control section ####
		/// Control pre step operations
		void controlPreStep(Real time, Int timeStepCount);
		/// Perform step of controller
		void controlStep(Real time, Int timeStepCount);
		/// Add control step dependencies
		void addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add control step dependencies
		void addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		class ControlPreStep : public CPS::Task {
		public:
			ControlPreStep(VSIVoltageControlVCO& VSIVoltageControlVCO) :
				Task(**VSIVoltageControlVCO.mName + ".ControlPreStep"), mVSIVoltageControlVCO(VSIVoltageControlVCO) {
					mVSIVoltageControlVCO.addControlPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVSIVoltageControlVCO.controlPreStep(time, timeStepCount); };

		private:
			VSIVoltageControlVCO& mVSIVoltageControlVCO;
		};

		class ControlStep : public CPS::Task {
		public:
			ControlStep(VSIVoltageControlVCO& VSIVoltageControlVCO) :
				Task(**VSIVoltageControlVCO.mName + ".ControlStep"), mVSIVoltageControlVCO(VSIVoltageControlVCO) {
					mVSIVoltageControlVCO.addControlStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVSIVoltageControlVCO.controlStep(time, timeStepCount); };

		private:
			VSIVoltageControlVCO& mVSIVoltageControlVCO;
		};
	};
}
}
}