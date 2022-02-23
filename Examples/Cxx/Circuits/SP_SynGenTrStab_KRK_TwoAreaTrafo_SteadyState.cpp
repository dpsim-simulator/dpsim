/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::KRK_TwoArea;
using namespace CIM::Examples;

ScenarioConfig KRK_TwoArea;

void SP_SynGenTrStab_KRK_TwoAreaTrafo_SteadyState(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdInertia_G3, Real cmdInertia_G4, Real cmdDamping_G1, Real cmdDamping_G2, Real cmdDamping_G3, Real cmdDamping_G4) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+ timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);
    auto n4PF = SimNode<Complex>::make("n4", PhaseType::Single);
	auto n5PF = SimNode<Complex>::make("n5", PhaseType::Single);
	auto n6PF = SimNode<Complex>::make("n6", PhaseType::Single);
    auto n7PF = SimNode<Complex>::make("n7", PhaseType::Single);
	auto n8PF = SimNode<Complex>::make("n8", PhaseType::Single);
	auto n9PF = SimNode<Complex>::make("n9", PhaseType::Single);
    auto n10PF = SimNode<Complex>::make("n10", PhaseType::Single);
	auto n11PF = SimNode<Complex>::make("n11", PhaseType::Single);

	//Synchronous generator 1
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator1", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.initActivePower_G1, KRK_TwoArea.setPointVoltage_G1, PowerflowBusType::PV, KRK_TwoArea.initReactivePower_G1);
	gen1PF->setBaseVoltage(KRK_TwoArea.nomPhPhVoltRMS_G1);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator2", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.initActivePower_G2, KRK_TwoArea.setPointVoltage_G2, PowerflowBusType::PV, KRK_TwoArea.initReactivePower_G2);
	gen2PF->setBaseVoltage(KRK_TwoArea.nomPhPhVoltRMS_G2);

    //Synchronous generator 3
	auto gen3PF = SP::Ph1::SynchronGenerator::make("Generator3", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen3PF->setParameters(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.initActivePower_G3, KRK_TwoArea.setPointVoltage_G3, PowerflowBusType::VD, KRK_TwoArea.initReactivePower_G3);
	gen3PF->setBaseVoltage(KRK_TwoArea.nomPhPhVoltRMS_G3);

    //Synchronous generator 4
	auto gen4PF = SP::Ph1::SynchronGenerator::make("Generator4", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen4PF->setParameters(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.initActivePower_G4, KRK_TwoArea.setPointVoltage_G4, PowerflowBusType::PV, KRK_TwoArea.initReactivePower_G4);
	gen4PF->setBaseVoltage(KRK_TwoArea.nomPhPhVoltRMS_G4);

	auto trafo15PF= SP::Ph1::Transformer::make("Transformer15", Logger::Level::debug);
	auto trafo26PF= SP::Ph1::Transformer::make("Transformer26", Logger::Level::debug);
	auto trafo311PF= SP::Ph1::Transformer::make("Transformer311", Logger::Level::debug);
	auto trafo410PF= SP::Ph1::Transformer::make("Transformer410", Logger::Level::debug);

	Real voltageMVSide = KRK_TwoArea.nomPhPhVoltRMS_G1;
	Real voltageHVSide = KRK_TwoArea.Vnom;
	Real ratio = voltageMVSide / voltageHVSide;
	Real trafoResistance = 0;
	Real trafoInductance = 23.4e-3; //set base to V_end2
	Real trafoPower = 900e6;
	// Note: to be consistent impedance values must be referred to high voltage side (and base voltage set to higher voltage)
	trafo15PF->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);
	trafo15PF->setBaseVoltage(KRK_TwoArea.Vnom);

	trafo26PF->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);
	trafo26PF->setBaseVoltage(KRK_TwoArea.Vnom);

	trafo311PF->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);
	trafo311PF->setBaseVoltage(KRK_TwoArea.Vnom);

	trafo410PF->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);
	trafo410PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //use Shunt as Load for powerflow
	auto load7PF = SP::Ph1::Load::make("Load7", Logger::Level::debug);
	load7PF->setParameters(KRK_TwoArea.activePower_L7, KRK_TwoArea.reactivePower_L7_inductive-KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7PF_c = SP::Ph1::Load::make("Load7_c", Logger::Level::debug);
	// load7PF_c->setParameters(0, -KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7PF_c = SP::Ph1::Shunt::make("Load7_c", Logger::Level::debug);
	// load7PF_c->setParameters(0, KRK_TwoArea.reactivePower_L7_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load7PF_c->setBaseVoltage(KRK_TwoArea.Vnom);

	auto load9PF = SP::Ph1::Load::make("Load9", Logger::Level::debug);
	load9PF->setParameters(KRK_TwoArea.activePower_L9, KRK_TwoArea.reactivePower_L9_inductive-KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9PF_c = SP::Ph1::Load::make("Load9_c", Logger::Level::debug);
	// load9PF_c->setParameters(0, -KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9PF_c = SP::Ph1::Shunt::make("Load9_c", Logger::Level::debug);
	// load9PF_c->setParameters(0, KRK_TwoArea.reactivePower_L9_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load9PF_c->setBaseVoltage(KRK_TwoArea.Vnom);

	//Line56
	auto line56PF = SP::Ph1::PiLine::make("PiLine56", Logger::Level::debug);
	line56PF->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	line56PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line67
	auto line67PF = SP::Ph1::PiLine::make("PiLine67", Logger::Level::debug);
	line67PF->setParameters(KRK_TwoArea.lineResistance67, KRK_TwoArea.lineInductance67, KRK_TwoArea.lineCapacitance67, KRK_TwoArea.lineConductance67);
	line67PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line78_1
	auto line78_1PF = SP::Ph1::PiLine::make("Piline78_1", Logger::Level::debug);
	line78_1PF->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	line78_1PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line78_2
	auto line78_2PF = SP::Ph1::PiLine::make("Piline78_2", Logger::Level::debug);
	line78_2PF->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	line78_2PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line89_1
	auto line89_1PF = SP::Ph1::PiLine::make("Piline89_1", Logger::Level::debug);
	line89_1PF->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	line89_1PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line89_2
	auto line89_2PF = SP::Ph1::PiLine::make("Piline89_2", Logger::Level::debug);
	line89_2PF->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	line89_2PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line910
	auto line910PF = SP::Ph1::PiLine::make("PiLine910", Logger::Level::debug);
	line910PF->setParameters(KRK_TwoArea.lineResistance910, KRK_TwoArea.lineInductance910, KRK_TwoArea.lineCapacitance910, KRK_TwoArea.lineConductance910);
	line910PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line1011
	auto line1011PF = SP::Ph1::PiLine::make("PiLine1011", Logger::Level::debug);
	line1011PF->setParameters(KRK_TwoArea.lineResistance1011, KRK_TwoArea.lineInductance1011, KRK_TwoArea.lineCapacitance1011, KRK_TwoArea.lineConductance1011);
	line1011PF->setBaseVoltage(KRK_TwoArea.Vnom);

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
    gen3PF->connect({ n3PF });
    gen4PF->connect({ n4PF });

	trafo15PF->connect({ n1PF, n5PF});
	trafo26PF->connect({ n2PF, n6PF});
	trafo311PF->connect({ n3PF, n11PF});
	trafo410PF->connect({ n4PF, n10PF});

	load7PF->connect({ n7PF });
    load9PF->connect({ n9PF });
	// load7PF_c->connect({ n7PF });
    // load9PF_c->connect({ n9PF });

	line56PF->connect({ n5PF, n6PF });
	line67PF->connect({ n6PF, n7PF });
	line78_1PF->connect({ n7PF, n8PF });
	line78_2PF->connect({ n7PF, n8PF });
    line89_1PF->connect({ n8PF, n9PF });
	line89_2PF->connect({ n8PF, n9PF });
    line910PF->connect({ n9PF, n10PF });
    line1011PF->connect({ n10PF, n11PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{ n1PF, n2PF, n3PF, n4PF, n5PF, n6PF, n7PF, n8PF, n9PF, n10PF, n11PF},
			SystemComponentList{gen1PF, gen2PF, gen3PF, gen4PF, trafo15PF, trafo26PF, trafo311PF, trafo410PF, load7PF, load9PF, line56PF, line67PF, line78_1PF, line78_2PF, line89_1PF, line89_2PF, line910PF, line1011PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));
    loggerPF->addAttribute("v_bus4", n4PF->attribute("v"));
	loggerPF->addAttribute("v_bus5", n5PF->attribute("v"));
	loggerPF->addAttribute("v_bus6", n6PF->attribute("v"));
    loggerPF->addAttribute("v_bus7", n7PF->attribute("v"));
	loggerPF->addAttribute("v_bus8", n8PF->attribute("v"));
	loggerPF->addAttribute("v_bus9", n9PF->attribute("v"));
    loggerPF->addAttribute("v_bus10", n10PF->attribute("v"));
	loggerPF->addAttribute("v_bus11", n11PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simNameSP = simName + "_SP";
	Logger::setLogDir("logs/"+simNameSP);

	// Nodes
	auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3SP = SimNode<Complex>::make("n3", PhaseType::Single);
	auto n4SP = SimNode<Complex>::make("n4", PhaseType::Single);
	auto n5SP = SimNode<Complex>::make("n5", PhaseType::Single);
	auto n6SP = SimNode<Complex>::make("n6", PhaseType::Single);
	auto n7SP = SimNode<Complex>::make("n7", PhaseType::Single);
	auto n8SP = SimNode<Complex>::make("n8", PhaseType::Single);
	auto n9SP = SimNode<Complex>::make("n9", PhaseType::Single);
	auto n10SP = SimNode<Complex>::make("n10", PhaseType::Single);
	auto n11SP = SimNode<Complex>::make("n11", PhaseType::Single);

	// Components
	//Synchronous generator 1
	auto gen1SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen1", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen1SP->setStandardParametersPU(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.nomFreq_G1, KRK_TwoArea.Xpd, KRK_TwoArea.H_G1, KRK_TwoArea.Rs, 10.0);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1 = gen1PF->getApparentPower();
	Real initMechPower_G1 = initApparentPower_G1.real();
	gen1SP->setInitialValues(initApparentPower_G1, initMechPower_G1);

	// Directly with the Kundur's book data
	// Complex initApparentPower_G1 = {700e6, 185e6};
	// gen1SP->setInitialValues(initApparentPower_G1, KRK_TwoArea.initMechPower_G1);

	//Synchronous generator 2
	auto gen2SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen2", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2SP->setStandardParametersPU(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.nomFreq_G2, KRK_TwoArea.Xpd, KRK_TwoArea.H_G2, KRK_TwoArea.Rs, 10.0);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G2 = gen2PF->getApparentPower();
	Real initMechPower_G2 = initApparentPower_G2.real();
	gen2SP->setInitialValues(initApparentPower_G2, initMechPower_G2);

	// Directly with the Kundur's book data
	// Complex initApparentPower_G2 = {700e6, 235e6};
	// gen2SP->setInitialValues(initApparentPower_G2, KRK_TwoArea.initMechPower_G2);

	//Synchronous generator 3
	auto gen3SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen3", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen3SP->setStandardParametersPU(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.nomFreq_G3, KRK_TwoArea.Xpd, KRK_TwoArea.H_G3, KRK_TwoArea.Rs, 10.0);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G3 = gen3PF->getApparentPower();
	Real initMechPower_G3 = initApparentPower_G3.real();
	gen3SP->setInitialValues(initApparentPower_G3, initMechPower_G3);

	// Directly with the Kundur's book data
	// Complex initApparentPower_G3 = {719e6, 176e6};
	// gen3SP->setInitialValues(initApparentPower_G3, KRK_TwoArea.initMechPower_G3);

	//Synchronous generator 4
	auto gen4SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen4", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen4SP->setStandardParametersPU(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.nomFreq_G4, KRK_TwoArea.Xpd, KRK_TwoArea.H_G4, KRK_TwoArea.Rs, 10.0);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G4 = gen4PF->getApparentPower();
	Real initMechPower_G4 = initApparentPower_G4.real();
	gen4SP->setInitialValues(initApparentPower_G4, initMechPower_G4);

	// Directly with the Kundur's book data
	// Complex initApparentPower_G4 = {700e6, 202e6};
	// gen4SP->setInitialValues(initApparentPower_G4, KRK_TwoArea.initMechPower_G4);

	gen1SP->setModelFlags(true, true);
	gen1SP->setReferenceOmega(gen3SP->attribute<Real>("w_r"), gen3SP->attribute<Real>("delta_r"));

	gen2SP->setModelFlags(true, true);
	gen2SP->setReferenceOmega(gen3SP->attribute<Real>("w_r"), gen3SP->attribute<Real>("delta_r"));

	gen4SP->setModelFlags(true, true);
	gen4SP->setReferenceOmega(gen3SP->attribute<Real>("w_r"), gen3SP->attribute<Real>("delta_r"));

	auto trafo15SP = SP::Ph1::Transformer::make("trafo15", Logger::Level::debug);
	trafo15SP->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);

	auto trafo26SP = SP::Ph1::Transformer::make("trafo26", Logger::Level::debug);
	trafo26SP->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);

	auto trafo311SP = SP::Ph1::Transformer::make("trafo311", Logger::Level::debug);
	trafo311SP->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);

	auto trafo410SP = SP::Ph1::Transformer::make("trafo410", Logger::Level::debug);
	trafo410SP->setParameters(voltageMVSide, voltageHVSide, trafoPower, std::abs(ratio), std::arg(ratio), trafoResistance, trafoInductance);

	///Loads
	auto load7SP = SP::Ph1::Load::make("Load7", Logger::Level::debug);
	load7SP->setParameters(KRK_TwoArea.activePower_L7, KRK_TwoArea.reactivePower_L7_inductive-KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7SP_c = SP::Ph1::Load::make("Load7_c", Logger::Level::debug);
	// load7SP_c->setParameters(0, -KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7SP_c = SP::Ph1::Shunt::make("Load7_c", Logger::Level::debug);
	// load7SP_c->setParameters(0, KRK_TwoArea.reactivePower_L7_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load7SP_c->setBaseVoltage(KRK_TwoArea.Vnom);

	auto load9SP = SP::Ph1::Load::make("Load9", Logger::Level::debug);
	load9SP->setParameters(KRK_TwoArea.activePower_L9, KRK_TwoArea.reactivePower_L9_inductive-KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9SP_c = SP::Ph1::Load::make("Load9_c", Logger::Level::debug);
	// load9SP_c->setParameters(0, -KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9SP_c = SP::Ph1::Shunt::make("Load9_c", Logger::Level::debug);
	// load9SP_c->setParameters(0, KRK_TwoArea.reactivePower_L9_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load9SP_c->setBaseVoltage(KRK_TwoArea.Vnom);

	//Line45
	auto line45SP = SP::Ph1::PiLine::make("PiLine45", Logger::Level::debug);
	line45SP->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	//Line56
	auto line56SP = SP::Ph1::PiLine::make("PiLine56", Logger::Level::debug);
	line56SP->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	//Line67
	auto line67SP = SP::Ph1::PiLine::make("PiLine67", Logger::Level::debug);
	line67SP->setParameters(KRK_TwoArea.lineResistance67, KRK_TwoArea.lineInductance67, KRK_TwoArea.lineCapacitance67, KRK_TwoArea.lineConductance67);
	//Line78_1
	auto line78_1SP = SP::Ph1::PiLine::make("PiLine78_1", Logger::Level::debug);
	line78_1SP->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	//Line78_2
	auto line78_2SP = SP::Ph1::PiLine::make("PiLine78_2", Logger::Level::debug);
	line78_2SP->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	//Line89_1
	auto line89_1SP = SP::Ph1::PiLine::make("PiLine89_1", Logger::Level::debug);
	line89_1SP->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	//Line89_2
	auto line89_2SP = SP::Ph1::PiLine::make("PiLine89_2", Logger::Level::debug);
	line89_2SP->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	//Line910
	auto line910SP = SP::Ph1::PiLine::make("PiLine910", Logger::Level::debug);
	line910SP->setParameters(KRK_TwoArea.lineResistance910, KRK_TwoArea.lineInductance910, KRK_TwoArea.lineCapacitance910, KRK_TwoArea.lineConductance910);
	//Line1011
	auto line1011SP = SP::Ph1::PiLine::make("PiLine1011", Logger::Level::debug);
	line1011SP->setParameters(KRK_TwoArea.lineResistance1011, KRK_TwoArea.lineInductance1011, KRK_TwoArea.lineCapacitance1011, KRK_TwoArea.lineConductance1011);

	// Topology
	gen1SP->connect({ n1SP });
	gen2SP->connect({ n2SP });
	gen3SP->connect({ n3SP });
	gen4SP->connect({ n4SP });

	trafo15SP->connect({n1SP, n5SP});
	trafo26SP->connect({n2SP, n6SP});
	trafo311SP->connect({n3SP, n11SP});
	trafo410SP->connect({n4SP, n10SP});

	load7SP->connect({ n7SP });
	load9SP->connect({ n9SP });
	// load7SP_c->connect({ n7SP });
	// load9SP_c->connect({ n9SP });

	line56SP->connect({ n5SP, n6SP });
	line67SP->connect({ n6SP, n7SP });
	line78_1SP->connect({ n7SP, n8SP });
	line78_2SP->connect({ n7SP, n8SP });
	line89_1SP->connect({ n8SP, n9SP });
	line89_2SP->connect({ n8SP, n9SP });
	line910SP->connect({ n9SP, n10SP });
	line1011SP->connect({ n10SP, n11SP });
	// faultDP->connect({DP::SimNode::GND , n1DP }); //terminal of generator 1
	// faultDP->connect({DP::SimNode::GND , n2DP }); //terminal of generator 2
	// faultDP->connect({DP::SimNode::GND , n3DP }); //Load bus

	auto systemSP = SystemTopology(60,
			SystemNodeList{n1SP, n2SP, n3SP, n4SP, n5SP, n6SP, n7SP, n8SP, n9SP, n10SP, n11SP},
			SystemComponentList{gen1SP, gen2SP, gen3SP, gen4SP, trafo15SP, trafo26SP, trafo311SP, trafo410SP, load7SP, load9SP, line56SP, line67SP, line78_1SP, line78_2SP, line89_1SP, line89_2SP, line910SP, line1011SP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameSP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemSP);

	// Logging
	auto loggerSP = DataLogger::make(simNameSP);
	loggerSP->addAttribute("v1", n1SP->attribute("v"));
	loggerSP->addAttribute("v2", n2SP->attribute("v"));
	loggerSP->addAttribute("v3", n3SP->attribute("v"));
	loggerSP->addAttribute("v4", n4SP->attribute("v"));
	loggerSP->addAttribute("v5", n5SP->attribute("v"));
	loggerSP->addAttribute("v6", n6SP->attribute("v"));
	loggerSP->addAttribute("v7", n7SP->attribute("v"));
	loggerSP->addAttribute("v8", n8SP->attribute("v"));
	loggerSP->addAttribute("v9", n9SP->attribute("v"));
	loggerSP->addAttribute("v10", n10SP->attribute("v"));
	loggerSP->addAttribute("v11", n11SP->attribute("v"));
	loggerSP->addAttribute("v_line56", line56SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line56", line56SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line67", line67SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line67", line67SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line78_1", line78_1SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line78_1", line78_1SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line78_2", line78_2SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line78_2", line78_2SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line89_1", line89_1SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line89_1", line89_1SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line89_2", line89_2SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line89_2", line89_2SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line910", line910SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line910", line910SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line1011", line1011SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line1011", line1011SP->attribute("i_intf"));
	loggerSP->addAttribute("Ep_gen1", gen1SP->attribute("Ep_mag"));
	loggerSP->addAttribute("v_gen1", gen1SP->attribute("v_intf"));
	loggerSP->addAttribute("i_gen1", gen1SP->attribute("i_intf"));
	loggerSP->addAttribute("wr_gen1", gen1SP->attribute("w_r"));
	loggerSP->addAttribute("wref_gen1", gen1SP->attribute("w_ref"));
	loggerSP->addAttribute("delta_gen1", gen1SP->attribute("delta_r"));
	loggerSP->addAttribute("deltaref_gen1", gen1SP->attribute("delta_ref"));
	loggerSP->addAttribute("Ep_gen2", gen2SP->attribute("Ep_mag"));
	loggerSP->addAttribute("v_gen2", gen2SP->attribute("v_intf"));
	loggerSP->addAttribute("i_gen2", gen2SP->attribute("i_intf"));
	loggerSP->addAttribute("wr_gen2", gen2SP->attribute("w_r"));
	loggerSP->addAttribute("wref_gen2", gen2SP->attribute("w_ref"));
	loggerSP->addAttribute("delta_gen2", gen2SP->attribute("delta_r"));
	loggerSP->addAttribute("deltaref_gen2", gen2SP->attribute("delta_ref"));
	loggerSP->addAttribute("Ep_gen3", gen3SP->attribute("Ep_mag"));
	loggerSP->addAttribute("v_gen3", gen3SP->attribute("v_intf"));
	loggerSP->addAttribute("i_gen3", gen3SP->attribute("i_intf"));
	loggerSP->addAttribute("wr_gen3", gen3SP->attribute("w_r"));
	loggerSP->addAttribute("delta_gen3", gen3SP->attribute("delta_r"));
	loggerSP->addAttribute("Ep_gen4", gen4SP->attribute("Ep_mag"));
	loggerSP->addAttribute("v_gen4", gen4SP->attribute("v_intf"));
	loggerSP->addAttribute("i_gen4", gen4SP->attribute("i_intf"));
	loggerSP->addAttribute("wr_gen4", gen4SP->attribute("w_r"));
	loggerSP->addAttribute("wref_gen4", gen4SP->attribute("w_ref"));
	loggerSP->addAttribute("delta_gen4", gen4SP->attribute("delta_r"));
	loggerSP->addAttribute("deltaref_gen4", gen4SP->attribute("delta_ref"));
	loggerSP->addAttribute("v_load7", load7SP->attribute("v_intf"));
	loggerSP->addAttribute("i_load7", load7SP->attribute("i_intf"));
	loggerSP->addAttribute("v_load9", load9SP->attribute("v_intf"));
	loggerSP->addAttribute("i_load9", load9SP->attribute("i_intf"));
	loggerSP->addAttribute("P_mech1", gen1SP->attribute("P_mech"));
	loggerSP->addAttribute("P_mech2", gen2SP->attribute("P_mech"));
	loggerSP->addAttribute("P_elec1", gen1SP->attribute("P_elec"));
	loggerSP->addAttribute("P_elec2", gen2SP->attribute("P_elec"));
	loggerSP->addAttribute("P_mech3", gen3SP->attribute("P_mech"));
	loggerSP->addAttribute("P_mech4", gen4SP->attribute("P_mech"));
	loggerSP->addAttribute("P_elec3", gen3SP->attribute("P_elec"));
	loggerSP->addAttribute("P_elec4", gen4SP->attribute("P_elec"));

	// trafo
	loggerSP->addAttribute("i_trafo15", trafo15SP->attribute("i_intf"));
	loggerSP->addAttribute("v_trafo15", trafo15SP->attribute("v_intf"));
	loggerSP->addAttribute("i_trafo26", trafo26SP->attribute("i_intf"));
	loggerSP->addAttribute("v_trafo26", trafo26SP->attribute("v_intf"));
	loggerSP->addAttribute("i_trafo311", trafo311SP->attribute("i_intf"));
	loggerSP->addAttribute("v_trafo311", trafo311SP->attribute("v_intf"));
	loggerSP->addAttribute("i_trafo410", trafo410SP->attribute("i_intf"));
	loggerSP->addAttribute("v_trafo410", trafo410SP->attribute("v_intf"));

	Simulation simSP(simNameSP, Logger::Level::debug);
	simSP.setSystem(systemSP);
	simSP.setTimeStep(timeStep);
	simSP.setFinalTime(finalTime);
	simSP.setDomain(Domain::SP);
	simSP.addLogger(loggerSP);

	simSP.run();
}

int main(int argc, char* argv[]) {


	//Simulation parameters
	String simName="SP_SynGenTrStab_KRK_TwoAreaTrafo_SteadyState";
	Real finalTime = 30;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Real startTimeFault=10;
	Real endTimeFault=10.2;
	Real cmdInertia_G1 = KRK_TwoArea.H_G1;
	Real cmdInertia_G2 = KRK_TwoArea.H_G2;
    Real cmdInertia_G3 = KRK_TwoArea.H_G3;
    Real cmdInertia_G4 = KRK_TwoArea.H_G4;
	Real cmdDamping_G1=1.0;
	Real cmdDamping_G2=1.0;
    Real cmdDamping_G3=1.0;
    Real cmdDamping_G4=1.0;

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA_G1") != args.options.end())
			cmdInertia_G1 = args.options["SCALEINERTIA_G1"];
		if (args.options.find("SCALEINERTIA_G2") != args.options.end())
			cmdInertia_G2 = args.options["SCALEINERTIA_G2"];
        if (args.options.find("SCALEINERTIA_G3") != args.options.end())
			cmdInertia_G3 = args.options["SCALEINERTIA_G3"];
        if (args.options.find("SCALEINERTIA_G4") != args.options.end())
			cmdInertia_G4 = args.options["SCALEINERTIA_G4"];
		if (args.options.find("SCALEDAMPING_G1") != args.options.end())
			cmdDamping_G1 = args.options["SCALEDAMPING_G1"];
		if (args.options.find("SCALEDAMPING_G2") != args.options.end())
			cmdDamping_G2 = args.options["SCALEDAMPING_G2"];
        if (args.options.find("SCALEDAMPING_G3") != args.options.end())
			cmdDamping_G3 = args.options["SCALEDAMPING_G3"];
        if (args.options.find("SCALEDAMPING_G4") != args.options.end())
			cmdDamping_G4 = args.options["SCALEDAMPING_G4"];
		if (args.options.find("STARTTIMEFAULT") != args.options.end())
			startTimeFault = args.options["STARTTIMEFAULT"];
		if (args.options.find("ENDTIMEFAULT") != args.options.end())
			endTimeFault = args.options["ENDTIMEFAULT"];
	}

	SP_SynGenTrStab_KRK_TwoAreaTrafo_SteadyState(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia_G1, cmdInertia_G2, cmdInertia_G3, cmdInertia_G4, cmdDamping_G1, cmdDamping_G2, cmdDamping_G3, cmdDamping_G4);
}
