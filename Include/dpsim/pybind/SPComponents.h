/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addSPComponents(py::module_ mSP);
void addSPPh1Components(py::module_ mSPPh1);
void addSPPh3Components(py::module_ mSPPh3);
