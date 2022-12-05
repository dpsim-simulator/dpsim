// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
	enum class MNA_SUBCOMP_TASK_ORDER {
		NO_TASK,
		TASK_BEFORE_PARENT,
		TASK_AFTER_PARENT
	};

	/// Base class for composite power components
	template <typename VarType>
	class CompositePowerComp : public MNASimPowerComp<VarType> {

	private:
		MNAInterface::List mSubcomponentsMNA;
		MNAInterface::List mSubcomponentsBeforePreStep;
		MNAInterface::List mSubcomponentsAfterPreStep;
		MNAInterface::List mSubcomponentsBeforePostStep;
		MNAInterface::List mSubcomponentsAfterPostStep;

		std::vector<CPS::Attribute<Matrix>::Ptr> mRightVectorStamps;

		Bool mHasPreStep;
		Bool mHasPostStep;

	public:
		typedef VarType Type;
		typedef std::shared_ptr<CompositePowerComp<VarType>> Ptr;
		typedef std::vector<Ptr> List;

		/// Basic constructor that takes UID, name and log level
		CompositePowerComp(String uid, String name, Bool hasPreStep, Bool hasPostStep, Logger::Level logLevel)
			: MNASimPowerComp<VarType>(uid, name, logLevel) {
				mHasPreStep = hasPreStep;
				mHasPostStep = hasPostStep;
			}

		/// Basic constructor that takes name and log level and sets the UID to name as well
		CompositePowerComp(String name, Bool hasPreStep = true, Bool hasPostStep = true, Logger::Level logLevel = Logger::Level::off)
			: CompositePowerComp<VarType>(name, name, hasPreStep, hasPostStep, logLevel) { }

		/// Destructor - does not do anything
		virtual ~CompositePowerComp() = default;

		/// @brief Add a new subcomponent implementing MNA methods
		/// @param subc The new subcomponent
		/// @param preStepOrder When to execute the subcomponent's pre-step in relation to the parent
		/// @param postStepOrder When to execute the subcomponent's post-step in relation to the parent
		void addMNASubComponent(typename SimPowerComp<VarType>::Ptr subc, MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder, Bool contributeToRightVector);

		// #### MNA Interface Functions ####
		/// Initializes variables of components
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps right side (source) vector
		virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// MNA pre step operations
		virtual void mnaPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		virtual void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		virtual void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		virtual void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		// #### MNA Parent Functions ####
		virtual void mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) { };
		virtual void mnaParentApplySystemMatrixStamp(Matrix& systemMatrix) { };
		virtual void mnaParentApplyRightSideVectorStamp(Matrix& rightVector) { };
		virtual void mnaParentPreStep(Real time, Int timeStepCount) { };
		virtual void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) { };
		virtual void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) { };
		virtual void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) { };
	};
}