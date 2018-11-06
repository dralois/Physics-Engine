#include "MassSpringSystemSimulator.h"

//A
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
}

//A
const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return nullptr;
}

//B
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
}
//A
void MassSpringSystemSimulator::reset()
{
}
//A
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
}
//A
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
//B
void MassSpringSystemSimulator::onClick(int x, int y)
{
}
//B
void MassSpringSystemSimulator::onMouse(int x, int y)
{
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
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	return 0;
}
//C
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
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

//C
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}
