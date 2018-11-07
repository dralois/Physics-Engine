#include "MassSpringSystemSimulator.h"

/*
A: Rendering
B: UI und Interaktion
C: Simulation
*/

#pragma region Declarations

#pragma endregion

#pragma region Properties

//C
const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return nullptr;
}

//C
void MassSpringSystemSimulator::setMass(float mass)
{
}

//C
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
}

//C
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
}

//C
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return 0;
}

//C
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return 0;
}

//C
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return Vec3();
}

//C
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return Vec3();
}

#pragma endregion

#pragma region Events

//B
void MassSpringSystemSimulator::onClick(int x, int y)
{
}

//B
void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

#pragma endregion

#pragma region Functions

//B
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
}

//B
void MassSpringSystemSimulator::reset()
{
}

//A
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
}

//B
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
}

//C
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

//C
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
}

//C
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	return 0;
}

//C
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
}

//C
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

#pragma endregion

#pragma region Initialisation

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
}

#pragma endregion

#pragma region Finalization

#pragma endregion
