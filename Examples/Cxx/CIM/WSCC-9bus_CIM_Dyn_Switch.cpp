/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *argv[]) {

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = Utils::findFiles({
			"WSCC-09_RX_DI.xml",
			"WSCC-09_RX_EQ.xml",
			"WSCC-09_RX_SV.xml",
			"WSCC-09_RX_TP.xml"
		}, "build/_deps/cim-data-src/WSCC-09/WSCC-09_RX_Dyn", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "WSCC-9bus_dyn_switch";
	Logger::setLogDir("logs/"+simName);

	CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::info);
	SystemTopology sys = reader.loadCIM(60, filenames, Domain::DP, PhaseType::Single, CPS::GeneratorType::TransientStability);

	// Extend topology with switch
	auto sw = Ph1::Switch::make("StepLoad", Logger::Level::info);
	sw->setParameters(1e9, 0.1);
	sw->connect({ SimNode::GND, sys.node<SimNode>("BUS6") });
	sw->open();
	sys.addComponent(sw);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", sys.node<SimNode>("BUS1")->attribute("v"));
	logger->addAttribute("v2", sys.node<SimNode>("BUS2")->attribute("v"));
	logger->addAttribute("v3", sys.node<SimNode>("BUS3")->attribute("v"));
	logger->addAttribute("v4", sys.node<SimNode>("BUS4")->attribute("v"));
	logger->addAttribute("v5", sys.node<SimNode>("BUS5")->attribute("v"));
	logger->addAttribute("v6", sys.node<SimNode>("BUS6")->attribute("v"));
	logger->addAttribute("v7", sys.node<SimNode>("BUS7")->attribute("v"));
	logger->addAttribute("v8", sys.node<SimNode>("BUS8")->attribute("v"));
	logger->addAttribute("v9", sys.node<SimNode>("BUS9")->attribute("v"));
	logger->addAttribute("wr_1", sys.component<Ph1::SynchronGeneratorTrStab>("GEN1")->attribute("w_r"));
	logger->addAttribute("wr_2", sys.component<Ph1::SynchronGeneratorTrStab>("GEN2")->attribute("w_r"));
	logger->addAttribute("wr_3", sys.component<Ph1::SynchronGeneratorTrStab>("GEN3")->attribute("w_r"));


	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(0.0001);
	sim.setFinalTime(2);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);
	sim.doInitFromNodesAndTerminals(true);

	auto swEvent1 = SwitchEvent::make(0.2, sw, true);
	//auto swEvent2 = SwitchEvent::make(0.07, sw, false);

	sim.addEvent(swEvent1);
	//sim.addEvent(swEvent2);
	sim.addLogger(logger);
	sim.run();

	return 0;
}
