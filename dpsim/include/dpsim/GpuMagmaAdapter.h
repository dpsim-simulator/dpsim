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
#include <list>
#include <unordered_map>
#include <bitset>
#include <memory>

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolver.h>

#include <magma_v2.h>
#include <magmasparse.h>
#include <cusolverSp.h>


namespace DPsim
{
	class GpuMagmaAdapter : public DirectLinearSolver
    {
		protected:
		
		std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>> mTransp;
		// #### Attributes required for GPU ####
		/// Solver-Handle
		magma_dopts mMagmaOpts;
		magma_queue_t mMagmaQueue;

		/// Systemmatrix
		magma_d_matrix mHostSysMat;
		magma_d_matrix mDevSysMat;

		/// RHS-Vector
		magma_d_matrix mHostRhsVec;
		magma_d_matrix mDevRhsVec;
		/// LHS-Vector
		magma_d_matrix mHostLhsVec;
		magma_d_matrix mDevLhsVec;

		using Solver::mSLog;

		void iluPreconditioner();

        public:
		
		/// Destructor
		virtual ~GpuMagmaAdapter();

		/// initialization function for linear solver
		virtual void initialize() override;

		/// preprocessing function pre-ordering and scaling the matrix
		virtual void preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) override;

		/// factorization function with partial pivoting
		virtual void factorize(SparseMatrix& mVariableSystemMatrix) override;

		/// refactorization without partial pivoting
		virtual void refactorize(SparseMatrix& mVariableSystemMatrix) override;

		/// partial refactorization withouth partial pivoting
		virtual void partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) override;

		/// solution function for a right hand side
		virtual Matrix solve(Matrix& mRightSideVector) override;
    };
}