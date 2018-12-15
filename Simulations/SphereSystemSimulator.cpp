#include "pch.h"

#include "SphereSystemSimulator.h"

#pragma region Properties

// Get Testcases
const char * SphereSystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3";
}

#pragma endregion

#pragma region Events

// Mausbewegung während Klicken
void SphereSystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// Mausbewegung normal
void SphereSystemSimulator::onMouse(int x, int y)
{
	m_v2Oldtrackmouse.x = x;
	m_v2Oldtrackmouse.y = y;
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// Kernel Funktion
std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },									// Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },							// Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); },	// Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },				// Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },		// Electric Charge, m_iKernel = 4
};

#pragma endregion

// Initialisiere UI je nach Demo
// TODO
void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
		break;
	default:
		break;
	}
}

// Setzte Simulation zurück
// TODO
void SphereSystemSimulator::reset()
{
	m_fForceScaling = m_fDamping = m_fRadius = m_fMass = 0.0f;
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	m_v3ExternalForce = Vec3(0.0f);
	m_iNumSpheres = 0;
}

// Rendere Simulation
// TODO
void SphereSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
		break;
	default:
		break;
	}
}

// Aktiviere verschiedene Simulationen
// TODO
void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Setzte Simulation zurück
	reset();
	// Je nach Case
	switch (m_iTestCase)
	{
	case 0:
	{
		cout << "Demo 1!" << endl;
		break;
	}
	case 1:
	{
		cout << "Demo 2!" << endl;
		break;
	}
	case 2:
	{
		cout << "Demo 3!" << endl;
		break;
	}
	default:
		cout << "Empty Demo!" << endl;
		break;
	}
}

// Externe Kräfte updaten
// TODO
void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}

// Simulation updaten
// TODO
void SphereSystemSimulator::simulateTimestep(float timeStep)
{

}

#pragma endregion

#pragma region Initialisation

// Initialisiert neuen Simulator
SphereSystemSimulator::SphereSystemSimulator()
{
	srand(time(NULL));
	m_iTestCase = 0;
	reset();
}

#pragma endregion
