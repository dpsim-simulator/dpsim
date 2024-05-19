#include <dpsim/MNAEigenvalueExtractor.h>

namespace DPsim {
template <typename VarType>
MNAEigenvalueExtractor<VarType>::MNAEigenvalueExtractor(
    CPS::Logger::Level logLevel)
    : mLogger("MNAEigenvalueExtractor", logLevel) {
  mEigenvalues = CPS::AttributeStatic<MatrixComp>::make();
  mDiscreteEigenvalues = CPS::AttributeStatic<MatrixComp>::make();
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::initialize(
    const CPS::SystemTopology &topology, UInt numMatrixNodeIndices,
    Real timeStep) {
  setParameters(topology, timeStep);
  identifyEigenvalueComponents(topology.mComponents);
  setBranchIndices();
  createEmptyEigenvalueMatrices(numMatrixNodeIndices);
  stampEigenvalueMatrices();
  mLogger.setLogAttributes(mEigenvalues, mDiscreteEigenvalues);
  mLogger.logInitialization(mSignMatrix, mDiscretizationMatrix,
                            mBranchNodeIncidenceMatrix,
                            mNodeBranchIncidenceMatrix);
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::setParameters(
    const CPS::SystemTopology &topology, Real timeStep) {
  mTimeStep = timeStep;
  // Relevant only for DP
  mSystemOmega = topology.mSystemOmega;

  Real denominator = mTimeStep * mTimeStep * mSystemOmega * mSystemOmega + 4.0;
  Real realPart =
      (4.0 - mTimeStep * mTimeStep * mSystemOmega * mSystemOmega) / denominator;
  Real imagPart = (-4.0 * mTimeStep * mSystemOmega) / denominator;
  mCoeffDP = Complex(realPart, imagPart);
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::identifyEigenvalueComponents(
    const CPS::IdentifiedObject::List &components) {
  // TODO: throw exception if topology contains components that do not implement EigenvalueCompInterface
  for (auto comp : components) {
    auto eigenvalueComponent =
        std::dynamic_pointer_cast<CPS::EigenvalueCompInterface>(comp);
    if (eigenvalueComponent) {
      mEigenvalueComponents.push_back(eigenvalueComponent);
      auto eigenvalueDynamicComponent = std::dynamic_pointer_cast<
          CPS::EigenvalueDynamicCompInterface<VarType>>(comp);
      if (eigenvalueDynamicComponent) {
        mEigenvalueDynamicComponents.push_back(eigenvalueDynamicComponent);
      }
    }
  }
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::setBranchIndices() {
  int size = mEigenvalueComponents.size();
  for (int i = 0; i < size; i++) {
    mEigenvalueComponents[i]->setBranchIdx(i);
  }
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::createEmptyEigenvalueMatrices(
    UInt numMatrixNodeIndices) {
  int nBranches = mEigenvalueComponents.size();
  mSignMatrix = MatrixVar<VarType>::Zero(nBranches, nBranches);
  mDiscretizationMatrix = MatrixVar<VarType>::Zero(nBranches, nBranches);
  mBranchNodeIncidenceMatrix = Matrix::Zero(nBranches, numMatrixNodeIndices);
  **mEigenvalues = MatrixComp::Zero(mEigenvalueDynamicComponents.size(), 1);
  **mDiscreteEigenvalues =
      MatrixComp::Zero(mEigenvalueDynamicComponents.size(), 1);
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::stampEigenvalueMatrices() {
  for (auto comp : mEigenvalueComponents) {
    comp->stampBranchNodeIncidenceMatrix(mBranchNodeIncidenceMatrix);
  }
  mNodeBranchIncidenceMatrix = mBranchNodeIncidenceMatrix.transpose();
  for (auto dynamicComp : mEigenvalueDynamicComponents) {
    dynamicComp->stampSignMatrix(mSignMatrix, mCoeffDP);
    dynamicComp->stampDiscretizationMatrix(mDiscretizationMatrix, mCoeffDP);
  }
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::extractEigenvalues(
    const Matrix &powerSystemMatrix, Real time, Int timeStepCount) {
  calculateStateMatrix(powerSystemMatrix);
  computeDiscreteEigenvalues();
  recoverEigenvalues();
  mLogger.logExtraction(time, timeStepCount, mStateMatrix);
}

template <>
void MNAEigenvalueExtractor<Real>::calculateStateMatrix(
    const Matrix &powerSystemMatrix) {
  // TODO: use right hand side solving of factorized power system matrix instead of inversion (performance).
  Matrix intermediateResult =
      powerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
  mStateMatrix = mSignMatrix - mDiscretizationMatrix *
                                   mBranchNodeIncidenceMatrix *
                                   intermediateResult;
}

template <>
void MNAEigenvalueExtractor<Complex>::calculateStateMatrix(
    const Matrix &powerSystemMatrix) {
  // TODO: use right hand side solving of factorized power system matrix instead of inversion (performance).
  MatrixComp compPowerSystemMatrix =
      CPS::Math::convertRealEquivalentToComplexMatrix(powerSystemMatrix);
  MatrixComp intermediateResult =
      compPowerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
  mStateMatrix = mSignMatrix - mDiscretizationMatrix *
                                   mBranchNodeIncidenceMatrix *
                                   intermediateResult;
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::computeDiscreteEigenvalues() {
  auto discreteEigenvaluesIncludingZeros = mStateMatrix.eigenvalues();
  **mDiscreteEigenvalues =
      CPS::Math::returnNonZeroElements(discreteEigenvaluesIncludingZeros);
  // TODO: filter out eigenvalues = -1 + 0i to avoid division by zero in recoverEigenvalues().
}

template <> void MNAEigenvalueExtractor<Real>::recoverEigenvalues() {
  **mEigenvalues = 2.0 / mTimeStep * ((**mDiscreteEigenvalues).array() - 1.0) /
                   ((**mDiscreteEigenvalues).array() + 1.0);
}

template <> void MNAEigenvalueExtractor<Complex>::recoverEigenvalues() {
  **mEigenvalues = 2.0 / mTimeStep * ((**mDiscreteEigenvalues).array() - 1.0) /
                       ((**mDiscreteEigenvalues).array() + 1.0) +
                   Complex(0.0, 1.0) * mSystemOmega;
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::closeLogger() {
  mLogger.close();
}

template class MNAEigenvalueExtractor<Real>;
template class MNAEigenvalueExtractor<Complex>;
} // namespace DPsim