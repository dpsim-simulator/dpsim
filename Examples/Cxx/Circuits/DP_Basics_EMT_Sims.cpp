/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

void EMT_VS_RL_f60_largeTs();
void EMT_VS_RL_f60();
void EMT_VS_RL_f500();

int main(int argc, char* argv[]) {
	EMT_VS_RL_f60_largeTs();
	EMT_VS_RL_f60();
	EMT_VS_RL_f500();
}

void EMT_VS_RL_f60_largeTs() {
	Real timeStep = 0.01;
	Real finalTime = 0.2;
	String simName = "EMT_VS_RL_f60_largeTs";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 60);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attributeMatrixReal("v"));
	logger->addAttribute("v2", n2->attributeMatrixReal("v"));
	logger->addAttribute("v3", n3->attributeMatrixReal("v"));
	logger->addAttribute("i_line", rline->attributeMatrixReal("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_RL_f60() {
	Real timeStep = 0.0001;
	Real finalTime = 0.2;
	String simName = "EMT_VS_RL_f60";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 60);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attributeMatrixReal("v"));
	logger->addAttribute("v2", n2->attributeMatrixReal("v"));
	logger->addAttribute("v3", n3->attributeMatrixReal("v"));
	logger->addAttribute("i_line", rline->attributeMatrixReal("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_RL_f500() {
	Real timeStep = 0.00001;
	Real finalTime = 0.2;
	String simName = "EMT_VS_RL_f500";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 500);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attributeMatrixReal("v"));
	logger->addAttribute("v2", n2->attributeMatrixReal("v"));
	logger->addAttribute("v3", n3->attributeMatrixReal("v"));
	logger->addAttribute("i_line", rline->attributeMatrixReal("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}


