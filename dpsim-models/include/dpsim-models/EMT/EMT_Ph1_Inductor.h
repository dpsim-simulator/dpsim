/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Inductor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/EigenvalueDynamicCompInterface.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// \brief Inductor
///
/// The inductor is represented by a DC equivalent circuit which corresponds to
/// one iteration of the trapezoidal integration method.
/// The equivalent DC circuit is a resistance in paralel with a current source.
/// The resistance is constant for a defined time step and system
/// frequency and the current source changes for each iteration.
class Inductor : public MNASimPowerComp<Real>,
                 public Base::Ph1::Inductor,
                 public SharedFactory<Inductor>,
                 public EigenvalueDynamicCompInterface<Real> {
protected:
  /// DC equivalent current source [A]
  Real mEquivCurrent;
  /// Equivalent conductance [S]
  Real mEquivCond;

public:
  /// Defines UID, name, component parameters and logging level
  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### Implementation of eigenvalue dynamic component interface ####
  void stampSignMatrix(MatrixVar<Real> &signMatrix, Complex coeffDP) final;
  void stampDiscretizationMatrix(MatrixVar<Real> &discretizationMatrix,
                                 Complex coeffDP) final;
  void stampBranchNodeIncidenceMatrix(Matrix &branchNodeIncidenceMatrix) final;
  void setBranchIdx(UInt i) final;

private:
  /// Branch index
  UInt mBranchIdx;
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
