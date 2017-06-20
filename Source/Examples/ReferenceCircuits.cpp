#include "ReferenceCircuits.h"

#include "../Simulation.h"
#include "../Utilities.h"


using namespace DPsim;

void DPsim::simulationExample1()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, 10, 0, 1));
	circElements0.push_back(new Inductor("l_1", 1, 2, 0.02));
	circElements0.push_back(new Inductor("l_2", 2, 0, 0.1));
	circElements0.push_back(new Inductor("l_3", 2, 3, 0.05));
	circElements0.push_back(new LinearResistor("r_2", 3, 0, 2));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() <<  std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExample1_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExample1L2()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, 10, 0, 1));
	circElements0.push_back(new Inductor("l_1", 1, 2, 0.02));
	circElements0.push_back(new Inductor("l_2", 2, 0, 0.1));
	circElements0.push_back(new Inductor("l_3", 2, 3, 0.05));
	circElements0.push_back(new LinearResistor("r_2", 3, 0, 2));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExample1L2_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExample2()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, 10, 0, 1));
	circElements0.push_back(new Inductor("l_1", 1, 2, 0.02));
	circElements0.push_back(new Inductor("l_2", 2, 0, 0.1));
	
	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExample2_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExample3()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, 10, 0, 1));
	circElements0.push_back(new Capacitor("c_1", 1, 2, 0.001));
	circElements0.push_back(new Inductor("l_1", 2, 0, 0.001));
	circElements0.push_back(new LinearResistor("r_2", 2, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExample3_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleIdealVS()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_in", 1, 2, 10, 0, 1));
	circElements0.push_back(new LinearResistor("r_1", 1, 0, 1));
	circElements0.push_back(new LinearResistor("r_2", 2, 0, 1));
	circElements0.push_back(new LinearResistor("r_3", 2, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExampleIdealVS_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleIdealVS2()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_in", 1, 0, 10, 0, 1));
	circElements0.push_back(new LinearResistor("r_1", 1, 2, 1));
	circElements0.push_back(new Capacitor("c_1", 2, 3, 0.001));
	circElements0.push_back(new Inductor("l_1", 3, 0, 0.001));
	circElements0.push_back(new LinearResistor("r_2", 3, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExampleIdealVS2_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}


void DPsim::simulationExampleIdealVS3()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, 10, 0, 1));
	circElements0.push_back(new LinearResistor("r_1", 1, 2, 1));
	circElements0.push_back(new LinearResistor("r_2", 2, 0, 1));
	circElements0.push_back(new LinearResistor("r_3", 2, 3, 1));
	circElements0.push_back(new LinearResistor("r_4", 3, 0, 1));
	circElements0.push_back(new LinearResistor("r_5", 3, 4, 1));
	circElements0.push_back(new IdealVoltageSource("v_2", 4, 0, 20, 0, 2));




	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExampleIdealVS3_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleRXLine()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, 10, 0, 1));
	circElements0.push_back(new RxLine("Line_1", 1, 2, 3, 0.1, 0.001));
	circElements0.push_back(new LinearResistor("r_1", 2, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExampleRXLine_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleRXLine2()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, 10, 0, 1));

	circElements0.push_back(new Inductor("l_L", 2, 3, 0.001));
	circElements0.push_back(new LinearResistor("r_L", 1, 2, 0.1));
	circElements0.push_back(new LinearResistor("r_1", 3, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExampleRXLine2_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleRXLine3()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, 10, 0, 1));
	circElements0.push_back(new RxLine("Line_1", 1, 2, 0.1, 0.001));
	circElements0.push_back(new LinearResistor("r_1", 2, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExampleRXLine3_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExamplePiLine()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, 345, 0, 1));
	circElements0.push_back(new LinearResistor("r1", 1, 2, 5));
	circElements0.push_back(new PiLine("PiLine1", 2, 3, 4, 6.4, 0.186, 0.004));
	circElements0.push_back(new LinearResistor("r_load", 3, 0, 150));



	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExamplePiLine_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExamplePiLine2()
{
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, 345, 0, 1));
	circElements0.push_back(new LinearResistor("r1", 1, 2, 5));
	circElements0.push_back(new Capacitor("c_1", 2, 0, 0.002));
	circElements0.push_back(new LinearResistor("r_load", 2, 4, 6.4));
	circElements0.push_back(new Inductor("l_1", 4, 3, 0.186));
	circElements0.push_back(new Capacitor("c_2", 3, 0, 0.002));
	circElements0.push_back(new LinearResistor("r_load", 3, 0, 150));



	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "SimulationExamplePiLine2_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
	for (auto elem : circElements0)
		delete elem;
}
