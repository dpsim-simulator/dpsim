/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_RxLine.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::RxLine::RxLine(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(2);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Matrix>("R", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Matrix>("L", &mSeriesInd, Flags::read | Flags::write);
	mSLog->flush();
}

SimPowerComp<Real>::Ptr EMT::Ph3::RxLine::clone(String name) {
	auto copy = RxLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd);
	return copy;
}

void EMT::Ph3::RxLine::initializeFromNodesAndTerminals(Real frequency) {

	// Static calculation
	Real omega = 2. * PI * frequency;
	MatrixComp impedance = MatrixComp::Zero(3, 3);
	impedance <<
		Complex(mSeriesRes(0, 0), omega * mSeriesInd(0, 0)), Complex(mSeriesRes(0, 1), omega * mSeriesInd(0, 1)), Complex(mSeriesRes(0, 2), omega * mSeriesInd(0, 2)),
		Complex(mSeriesRes(1, 0), omega * mSeriesInd(1, 0)), Complex(mSeriesRes(1, 1), omega * mSeriesInd(1, 1)), Complex(mSeriesRes(1, 2), omega * mSeriesInd(1, 2)),
		Complex(mSeriesRes(2, 0), omega * mSeriesInd(2, 0)), Complex(mSeriesRes(2, 1), omega * mSeriesInd(2, 1)), Complex(mSeriesRes(2, 2), omega * mSeriesInd(2, 2));

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

	mIntfCurrent = (impedance.inverse() * vInitABC).real();
	mIntfVoltage = vInitABC.real();
	// Initialization of virtual node
	// Initial voltage of phase B,C is set after A
	MatrixComp vInitTerm0 = MatrixComp::Zero(3, 1);
	vInitTerm0(0, 0) = initialSingleVoltage(0);
	vInitTerm0(1, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_B;
	vInitTerm0(2, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_C;

	mVirtualNodes[0]->setInitialVoltage(vInitTerm0 + mSeriesRes * mIntfCurrent);

	// Default model with virtual node in between
	mSubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_res", mLogLevel);
	mSubResistor->setParameters(mSeriesRes);
	mSubResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubResistor->initializeFromNodesAndTerminals(frequency);

	mSubInductor = std::make_shared<EMT::Ph3::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mSeriesInd);
	mSubInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubInductor->initializeFromNodesAndTerminals(frequency);

	mInitialResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_snubber_res", mLogLevel);
	Matrix defaultSnubRes = Matrix::Zero(3, 1);
	defaultSnubRes <<
		1e6, 0, 0,
		0, 1e6, 0,
		0, 0, 1e6;
	mInitialResistor->setParameters(defaultSnubRes);
	mInitialResistor->connect({ SimNode::GND, mTerminals[1]->node() });
	mInitialResistor->initializeFromNodesAndTerminals(frequency);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void EMT::Ph3::RxLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mSubInductor->mnaInitialize(omega, timeStep, leftVector);
	mSubResistor->mnaInitialize(omega, timeStep, leftVector);
	mInitialResistor->mnaInitialize(omega, timeStep, leftVector);
	for (auto task : mSubInductor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	for (auto task : mSubResistor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::RxLine::mnaApplyInitialSystemMatrixStamp(Matrix& systemMatrix) {
	mInitialResistor->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::RxLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mInitialResistor->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::RxLine::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void EMT::Ph3::RxLine::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLine.mnaApplyRightSideVectorStamp(mLine.mRightVector);
}

void EMT::Ph3::RxLine::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLine.mnaUpdateVoltage(*mLeftVector);
	mLine.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::RxLine::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	}
}

void EMT::Ph3::RxLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSubInductor->intfCurrent();
}
