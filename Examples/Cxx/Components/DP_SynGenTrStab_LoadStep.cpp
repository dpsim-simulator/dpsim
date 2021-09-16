/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Define simulation parameters
	Real timeStep = 0.0005;
	Real finalTime = 0.1;
	String simName = "DP_SynGen_TrStab_LoadStep";
	Logger::setLogDir("logs/"+simName);

	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real H = 3.7;
	Real Ll = 0.15;
	Real Lmd = 1.6599;
	Real Llfd = 0.1648;
	// Initialization parameters
	Complex initElecPower = Complex(300e6, 0);
	Real initTerminalVolt = 24000;
	Real initVoltAngle = 0;
	Complex initVoltage = Complex(initTerminalVolt * cos(initVoltAngle), initTerminalVolt * sin(initVoltAngle));
	Real mechPower = 300e6;
	// Define grid parameters
	Real Rload = 1.92;
	Real RloadStep = 0.7;

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::Single, std::vector<Complex>{ initVoltage });

	// Components
	auto gen = Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);
	gen->connect({n1});
	gen->setInitialValues(initElecPower, mechPower);

	auto load = Ph1::Switch::make("StepLoad", Logger::Level::debug);
	load->setParameters(Rload, RloadStep);
	load->connect({SimNode::GND, n1});
	load->open();

	// System
	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attributeMatrixComp("v"));
	logger->addAttribute("i_gen", gen->attributeMatrixComp("i_intf"));
	logger->addAttribute("i_load", load->attributeMatrixComp("i_intf"));
	logger->addAttribute("wr_gen", gen->attribute("w_r"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);

	sim.addLogger(logger);
	// Events
	auto sw1 = SwitchEvent::make(0.05, load, true);

	sim.addEvent(sw1);

	sim.run();

	return 0;
}
