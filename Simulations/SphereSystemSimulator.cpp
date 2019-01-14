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

// Kernel Funktionen
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
			m_pSphereSystem = new SphereSystem(NAIVEACC, m_iBallNumber, m_fRadius, m_fMass, m_fDamping, 
													m_fForceScaling, m_fGravity, Vec3(0.0f));
			break;
		case 1:
			m_pSphereSystem = new SphereSystem(GRIDACC, m_iBallNumber, m_fRadius, m_fMass, m_fDamping,
													m_fForceScaling, m_fGravity, Vec3(0.0f));
			break;
		case 2:
			m_pSphereSystem = new SphereSystem(NAIVEACC, m_iBallNumber, m_fRadius, m_fMass, m_fDamping,
													m_fForceScaling, m_fGravity, m_v3ShiftingLeft);
			m_pSphereSystemGrid = new SphereSystem(GRIDACC, m_iBallNumber, m_fRadius, m_fMass, m_fDamping, 
													m_fForceScaling, m_fGravity, m_v3ShiftingRight);
			break;
		default:
			break;
	}
}

#pragma endregion

#pragma region Speed Measure

void SphereSystemSimulator::speedComparisonSimulateTimeStep(float timeStep)
{
	m_pSphereSystemmeasure->simulateHalfTimestep(timeStep);
	m_pSphereSystemmeasure->collisionResolve(m_Kernels[m_iKernel], m_fForceScaling);
	m_pSphereSystemmeasure->simulateTimestep(timeStep);
}

void TW_CALL SphereSystemSimulator::startMeasure(void * pi_pMyClass)
{
	((SphereSystemSimulator*)pi_pMyClass)->speedComparison();
}

void SphereSystemSimulator::speedComparison()
{
	// time step ist 0.001 Sekunde, die Anzahl davon ist 30, und Anzahl von Balls ist n
	int timeSteps = 10;
	float timeStep = 0.001f;
	int n = 0;
	cout << "time step:              " << timeStep << endl;
	cout << "number of time steps:   " << timeSteps << endl;
	cout << "unit of time:           " << "second" << endl << endl;

	// für n = 100, naive collision detection
	n = 100;
	m_pSphereSystemmeasure = new SphereSystem(NAIVEACC, n, m_fRadius, m_fMass, m_fDamping,
		m_fForceScaling, m_fGravity, m_v3ShiftingLeft);
	clock_t begin100_1 = clock();
	for (int i = 0; i < timeSteps; i++)
	{
		speedComparisonSimulateTimeStep(timeStep);
	}
	clock_t end100_1 = clock();
	double elapsed_secs100_1 = double(end100_1 - begin100_1) / CLOCKS_PER_SEC;
	cout << "n = 100 with naive collision detection:       " << elapsed_secs100_1 << endl;
	delete m_pSphereSystemmeasure;

	// für n = 100, accelerated collision detection
	m_pSphereSystemmeasure = new SphereSystem(GRIDACC, n, m_fRadius, m_fMass, m_fDamping,
		m_fForceScaling, m_fGravity, m_v3ShiftingRight);
	clock_t begin100_2 = clock();
	for (int i = 0; i < timeSteps; i++)
	{
		speedComparisonSimulateTimeStep(timeStep);
	}
	clock_t end100_2 = clock();
	double elapsed_secs100_2 = double(end100_2 - begin100_2) / CLOCKS_PER_SEC;
	cout << "n = 100 with accelerated collision detection: " << elapsed_secs100_2 << endl << endl;

	// für n = 1000, naive collision detection
	n = 1000;
	delete m_pSphereSystemmeasure;
	m_pSphereSystemmeasure = new SphereSystem(NAIVEACC, n, m_fRadius, m_fMass, m_fDamping,
		m_fForceScaling, m_fGravity, m_v3ShiftingLeft);
	clock_t begin1000_1 = clock();
	for (int i = 0; i < timeSteps; i++)
	{
		speedComparisonSimulateTimeStep(timeStep);
	}
	clock_t end1000_1 = clock();
	double elapsed_secs1000_1 = double(end1000_1 - begin1000_1) / CLOCKS_PER_SEC;
	cout << "n = 1000 with naive collision detection:       " << elapsed_secs1000_1 << endl;

	// für n = 1000, accelerated collision detection
	delete m_pSphereSystemmeasure;
	m_pSphereSystemmeasure = new SphereSystem(GRIDACC, n, m_fRadius, m_fMass, m_fDamping,
		m_fForceScaling, m_fGravity, m_v3ShiftingRight);
	clock_t begin1000_2 = clock();
	for (int i = 0; i < timeSteps; i++)
	{
		speedComparisonSimulateTimeStep(timeStep);
	}
	clock_t end1000_2 = clock();
	double elapsed_secs1000_2 = double(end1000_2 - begin1000_2) / CLOCKS_PER_SEC;
	cout << "n = 1000 with accelerated collision detection: " << elapsed_secs1000_2 << endl << endl;

	// für n = 10000, naive collision detection
	n = 10000;
	// Glaub niemand möchte darauf warten...
	if (timeSteps > 2)
	{
		cout << "Processing time for n = 10000 is very long for number of timesteps: " << timeSteps << endl;
		cout << "You may choose another number of timesteps\n";
		delete m_pSphereSystemmeasure;
		return;
	}
	delete m_pSphereSystemmeasure;
	m_pSphereSystemmeasure = new SphereSystem(NAIVEACC, n, m_fRadius, m_fMass, m_fDamping,
		m_fForceScaling, m_fGravity, m_v3ShiftingLeft);
	clock_t begin10000_1 = clock();
	for (int i = 0; i < timeSteps; i++)
	{
		speedComparisonSimulateTimeStep(timeStep);
	}
	clock_t end10000_1 = clock();
	double elapsed_secs10000_1 = double(end10000_1 - begin10000_1) / CLOCKS_PER_SEC;
	cout << "n = 10000 with naive collision detection:       " << elapsed_secs10000_1 << endl;

	// für n = 10000, accelerated collision detection
	delete m_pSphereSystemmeasure;
	m_pSphereSystemmeasure = new SphereSystem(GRIDACC, n, m_fRadius, m_fMass, m_fDamping,
		m_fForceScaling, m_fGravity, m_v3ShiftingRight);
	clock_t begin10000_2 = clock();
	for (int i = 0; i < timeSteps; i++)
	{
		speedComparisonSimulateTimeStep(timeStep);
	}
	clock_t end10000_2 = clock();
	double elapsed_secs10000_2 = double(end10000_2 - begin10000_2) / CLOCKS_PER_SEC;
	cout << "n = 10000 with accelerated collision detection: " << elapsed_secs10000_2 << endl << endl;
	delete m_pSphereSystemmeasure;
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
		TwAddVarRW(DUC->g_pTweakBar, "Number of Balls", TW_TYPE_INT32, &m_iBallNumber, "min=32 step=10");
		TwAddVarRW(DUC->g_pTweakBar, "Force Scale", TW_TYPE_FLOAT, &m_fForceScaling, "min=1.00 step=5.00");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.00 step=0.10");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 step=0.10");
		TwAddButton(DUC->g_pTweakBar, "Demo 3: Measure Performance (Reset Scene after)", &(startMeasure), this, NULL);
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
