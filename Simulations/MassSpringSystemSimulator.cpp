#include "pch.h"

#include "MassSpringSystemSimulator.h"

#pragma region Properties

// Get verfübare Testcases
const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 2,Demo 3,Demo 4 Euler,Demo 4 Midpoint";
}

// Set Masse des Systems
void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
	for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
	{
		masspoint->Mass = m_fMass;
	}
}

// Set Federkraft der Federn
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
	{
		spring->Stiffness = m_fStiffness;
	}
}

// Set Dämpfung
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

// Get Anzahl Massepunkte
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_MassPoints.size();
}

// Get Anzahl Federn
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_Springs.size();
}

// Get Position eines Massepunktes
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_MassPoints[index].Position;
}

// Get Geschwindigkeit eines Massepunktes
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_MassPoints[index].Velocity;
}

#pragma endregion

#pragma region Events

// Mausbewegung während Klicken
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// Mausbewegung normal
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_v2Oldtrackmouse.x = x;
	m_v2Oldtrackmouse.y = y;
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// System darf bestimmte Bereiche nicht verlassen
void MassSpringSystemSimulator::X_ApplyBounding()
{
	for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
	{
		// Boden darf nicht unterschritten werden
		if (masspoint->Position.y < 0.0f)
			masspoint->Position.y = 0.0f;
	}
}

// Berechnet Federkraft
Vec3 MassSpringSystemSimulator::X_CalcSpringForce(Spring &spring, const Vec3 &point1, const Vec3 &point2)
{
	// Aktualisiere Länge
	spring.CurrentLength = sqrt((float)point1.squaredDistanceTo(point2));
	// Breche ab falls P1 ca. P2
	if (spring.CurrentLength <= FLT_EPSILON) return Vec3(0.0f);
	// F_ij = -k * (l - L) * (x_i - x_j) / l
	return -1.0f * spring.Stiffness * (spring.CurrentLength - spring.InitLenght) *
		((point1 - point2) / spring.CurrentLength);
}

// Interne Kräfte berechnen, also Federnkraft
void MassSpringSystemSimulator::X_InternalForcesCalculations()
{
	for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
	{
		Vec3 addForce = X_CalcSpringForce(*spring, spring->Point1.Position, spring->Point2.Position);
		spring->Point1.Force += addForce;
		spring->Point2.Force -= addForce;
		// ForceTilde braucht man nur für Midpoint Berechnung
		if (m_iIntegrator == MIDPOINT)
		{
			Vec3 addForceTilde = X_CalcSpringForce(*spring, spring->Point1.PositionTilde, spring->Point2.PositionTilde);
			spring->Point2.ForceTilde -= addForceTilde;
			spring->Point1.ForceTilde += addForceTilde;
		}
	}
}

// Demo Szenen Setup für Demo 2,3
void MassSpringSystemSimulator::X_SetupDefaultDemo()
{
	setMass(.01f);
	setDampingFactor(0.1f);
	setStiffness(25.0f);
	m_fSphereSize = .01f;
	applyExternalForce(Vec3(0.0f));
	m_bRunningManualTest = false;
	int p0 = addMassPoint(Vec3(0.0f, .2f, 0.0f), Vec3(0.5f, 0.0f, 0.0f), false);
	int p1 = addMassPoint(Vec3(0.0f, .3f, 0.0f), Vec3(0.0f, 0.0f, 0.5f), false);
	int p2 = addMassPoint(Vec3(0.0f, .4f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), true);
	addSpring(p0, p1, .1f);
	addSpring(p1, p2, .1f);
}

// Demo Szene für Demo 4
void MassSpringSystemSimulator::X_SetupComplexDemo()
{
	setMass(.01f);
	setDampingFactor(0.1f);
	setStiffness(25.0f);
	m_fSphereSize = .01f;
	applyExternalForce(Vec3(0.0f));
	m_bRunningManualTest = false;
	// Matrix Parameter
	int size = 8;
	float gridSize = .1f;
	float height = .5f;
	// Seeden
	srand(time(NULL));
	// Massepunkte erstellen (Zufällig Punkte fixieren)
	for(int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			addMassPoint(Vec3((i * gridSize) - (.5f * gridSize * size),
												height,
												(j * gridSize) - (.5f * gridSize * size)),
									Vec3(0.0f),
									rand() % 10 > 8);
		}
	}
	// Federn aufspannen
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			if (i < size - 1)
			{
				addSpring(i + (j*size), i + (j*size) + 1, gridSize);
			}
			if (j < size - 1)
			{
				addSpring(i + (j * size), i + (j + 1) * size, gridSize);
			}
		}
	}
}

#pragma endregion

// Initialisiere UI je nach Demo
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.00 max=1.00 step=0.01");
		break;
	default:
		break;
	}
}

// Setzte Simulation zurück
void MassSpringSystemSimulator::reset()
{
	m_fDamping = m_fMass = m_fSphereSize = m_fStiffness = 0.0f;
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	m_v3ExternalForce = Vec3(0.0f);
	m_bRunningManualTest = true;
	m_fGravity = 9.81f;
	m_MassPoints.clear();
	m_Springs.clear();
}

// Rendere Simulation
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
		// Zeichne Lines für Federn
		for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
		{
			DUC->beginLine();
			// Bestimme Differenz (Max. 1.0)
			float diff = min(abs((spring->CurrentLength - spring->InitLenght) / spring->InitLenght), 1.0f);
			// Linie wird roter je nach dem wie hoch die Abweichung von der InitLength ist
			DUC->drawLine(
				spring->Point1.Position.toDirectXVector(),
				Vec3(diff, 1 - diff, 0),
				spring->Point2.Position.toDirectXVector(),
				Vec3(diff, 1 - diff, 0));
			DUC->endLine();
		}
		// Zeichne Kugeln für Massepunkte
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			DUC->drawSphere(masspoint->Position.toDirectXVector(), Vec3(m_fSphereSize));
		}
		break;
	default:
		break;
	}
}

// Aktiviere verschiedene Demos
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Setzte Simulation zurück
	reset();
	// Je nach Case
	switch (m_iTestCase)
	{
	case 0:
	{
		cout << "Demo 2!" << endl;
		setIntegrator(EULER);
		X_SetupDefaultDemo();
		break;
	}
	case 1:
	{
		cout << "Demo 3!" << endl;
		setIntegrator(MIDPOINT);
		X_SetupDefaultDemo();
		break;
	}
	case 2:
	{
		cout << "Demo 4 Euler!" << endl;
		setIntegrator(EULER);
		X_SetupComplexDemo();
		break;
	}
	case 3:
	{
		cout << "Demo 4 Midpoint!" << endl;
		setIntegrator(MIDPOINT);
		X_SetupComplexDemo();
		break;
	}
	default:
		cout << "Empty Demo!" << endl;
		m_bRunningManualTest = true;
		break;
	}
}

// Externe Kräfte und Dämpfung berechnen
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	Vec3 mouseForce(0.0f);
	// Diese Methode wird nur durchgeführt, wenn demo 1 nicht läuft
	if (m_bRunningManualTest) return;
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
	// Wende Gravitation und Mausinteraktion an
	for(auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
	{
		// Gravity einsetzen, Damping einsetzen
		masspoint->Force += Vec3(0, -1.0f * m_fGravity * masspoint->Mass, 0);
		masspoint->Force += mouseForce;
		masspoint->Force += m_v3ExternalForce;
		masspoint->Force -= m_fDamping * masspoint->Velocity;
		if (m_iIntegrator == MIDPOINT) 
		{
			// ForceTilde braucht man nur eigentlich für Midpoint Berechnung
			masspoint->ForceTilde += Vec3(0, -1.0f * m_fGravity * masspoint->Mass, 0);
			masspoint->ForceTilde += mouseForce;
			masspoint->ForceTilde += m_v3ExternalForce;
			// Damping für Midpoint muss man später berechnen, da man braucht noch Midpoint Velocity
		}
	}
}

// Simuliere einen Schritt
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// Je nach Integrationsverfahren
	switch (m_iIntegrator)
	{
	case EULER:
	{
		// Interne Kräfte berechnen
		X_InternalForcesCalculations();
		// Aktualisiere Geschwindigkeit/Position
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			// Nur für bewegliche Punkte
			if (!masspoint->Fixed)
			{
				// Aktualisiere zuerst Position
				masspoint->Position += timeStep * masspoint->Velocity;
				// Dann Geschwindigkeit
				masspoint->Velocity += (masspoint->Force / masspoint->Mass) * timeStep;
			}
			// Kraft zurücksetzen
			masspoint->Force = Vec3(0, 0, 0);
		}
		break;
	}
	case MIDPOINT:
	{
		//Berechnen aller Positionen nach h/2 Schritt
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			if (!masspoint->Fixed)
			{
				masspoint->PositionTilde = masspoint->Position + masspoint->Velocity * timeStep / 2.0f;
			}
		}
		// Dann berechne interne Kräfte
		X_InternalForcesCalculations();
		// Aktualisiere Geschwindigkeit/Position
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			if (!masspoint->Fixed)
			{
				Vec3 midPointVelocity = masspoint->Velocity + (masspoint->Force / masspoint->Mass) * timeStep / 2.0f;
				masspoint->Position += timeStep * midPointVelocity;
				masspoint->ForceTilde -= m_fDamping * midPointVelocity;
				masspoint->Velocity += timeStep * (masspoint->ForceTilde / masspoint->Mass);
			}
			// Kraft zurücksetzen
			masspoint->Force = Vec3(0.0f);
			masspoint->ForceTilde = Vec3(0.0f);
		}
		break;
	}
	case LEAPFROG:
	{
		break;
	}
	default:
		break;
	}
	// Bounding Test
	X_ApplyBounding();
}

// Fügt Massepunkt hinzu, gibt dessen Position im Array zurück
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	// Erstelle neuen Massepunkt
	Masspoint newMass;
	newMass.Force = m_v3ExternalForce;
	newMass.ForceTilde = Vec3(0.0f);
	newMass.Position = position;
	newMass.PositionTilde = position;
	newMass.Velocity = Velocity;
	newMass.Fixed = isFixed;
	newMass.Mass = m_fMass;
	// Speichere im Array und gebe Position zurück
	m_MassPoints.push_back(newMass);
	return m_MassPoints.size() - 1;
}

// Fügt neue Feder zw. Massepunkten hinzu
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	// Erstelle Feder
	Spring newSpring(m_MassPoints[masspoint1], m_MassPoints[masspoint2]);
	newSpring.InitLenght = newSpring.CurrentLength = initialLength;
	newSpring.Stiffness = m_fStiffness;
	// Speichere im Array
	m_Springs.push_back(newSpring);
}

// Füge Kraft hinzu
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_v3ExternalForce = force;
}

#pragma endregion

#pragma region Initialisation

// Initialisiert neuen Simulator
MassSpringSystemSimulator::MassSpringSystemSimulator() :
	m_iIntegrator(0)
{
	m_iTestCase = 0;
	reset();
}

#pragma endregion
