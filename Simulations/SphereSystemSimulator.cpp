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

// Erstellt Szene je nach Demo
void SphereSystemSimulator::X_SetupDemo()
{
	// Je nach Case
	switch(m_iTestCase)
	{
		case 0:
			m_pSphereSystem = new SphereSystem(NAIVEACC, 100, m_fRadius, m_fMass);
			break;
		case 1:
			m_pSphereSystem = new SphereSystem(GRIDACC, 100, m_fRadius, m_fMass);
			break;
		case 2:
			m_pSphereSystem = new SphereSystem(NAIVEACC, 100, m_fRadius, m_fMass);
			m_pSphereSystemGrid = new SphereSystem(GRIDACC, 100, m_fRadius, m_fMass);
			break;
		default:
			break;
	}
}

#pragma endregion

// Initialisiere UI je nach Demo
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
void SphereSystemSimulator::reset()
{
	m_iKernel = 1;
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	// Ballsystem löschen
	if(m_pSphereSystem)
	{
		delete m_pSphereSystem;
		m_pSphereSystem = NULL;
	}
	if (m_pSphereSystemGrid)
	{
		delete m_pSphereSystemGrid;
		m_pSphereSystemGrid = NULL;
	}
}

// Rendere Simulation
void SphereSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
	case 1:
		m_pSphereSystem->drawFrame(DUC, Vec3(1.0f, 0.0f, 0.0f));
		break;
	case 2:
		m_pSphereSystem->drawFrame(DUC, Vec3(1.0f, 0.0f, 0.0f));
		m_pSphereSystemGrid->drawFrame(DUC, Vec3(0.0f, 1.0f, 0.0f));
		break;
	default:
		break;
	}
}

// Aktiviere verschiedene Simulationen
void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Setzte Simulation zurück
	reset();
	// Erstelle Szene
	X_SetupDemo();
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
void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	Vec3 mouseForce(0.0f);
	// Berechne Differenz
	mouseDiff.x = m_v2Trackmouse.x - m_v2Oldtrackmouse.x;
	mouseDiff.y = m_v2Trackmouse.y - m_v2Oldtrackmouse.y;
	// Falls linke Maustaste gedrückt
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		// Berechne benötigte Matrix
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix()).inverse();
		// Vektor bestehend aus Mausverschiebung
		Vec3 inputView = Vec3((float)-mouseDiff.x, (float)mouseDiff.y, 0);
		// Bestimme Kraft im Worldspace mit Faktor
		mouseForce = worldViewInv.transformVectorNormal(inputView) * -0.001f;
	}
	m_pSphereSystem->externalForcesCalculations(timeElapsed, mouseForce);
	if(m_iTestCase == 2)
		m_pSphereSystemGrid->externalForcesCalculations(timeElapsed, mouseForce);
}

// Simulation updaten
void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	// Für Midpoint muss man zuerst halben Zeitschritt simulieren
	switch (m_iTestCase)
	{
	case 0:
	case 1:
		m_pSphereSystem->simulateHalfTimestep(timeStep);
		m_pSphereSystem->collisionResolve(m_Kernels[m_iKernel], m_fForceScaling);
		m_pSphereSystem->simulateTimestep(timeStep);
		break;
	case 2:
		m_pSphereSystem->simulateHalfTimestep(timeStep);
		m_pSphereSystem->collisionResolve(m_Kernels[m_iKernel], m_fForceScaling);
		m_pSphereSystem->simulateTimestep(timeStep);
		m_pSphereSystemGrid->simulateHalfTimestep(timeStep);
		m_pSphereSystemGrid->collisionResolve(m_Kernels[m_iKernel], m_fForceScaling);
		m_pSphereSystemGrid->simulateTimestep(timeStep);
		break;
	default:
		break;
	}
}

#pragma endregion

#pragma region Initialisation

// Initialisiert neuen Simulator
SphereSystemSimulator::SphereSystemSimulator() :
	m_pSphereSystem(NULL),
	m_pSphereSystemGrid(NULL)
{
	srand(time(NULL));
	m_iTestCase = 0;
	reset();
}

// Aufräumen
SphereSystemSimulator::~SphereSystemSimulator()
{
	if(m_pSphereSystem)
		delete m_pSphereSystem;
	if (m_pSphereSystemGrid)
		delete m_pSphereSystemGrid;
}

#pragma endregion
