#include "MassSpringSystemSimulator.h"

#pragma region Properties

// Get verfübare Testcases
const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 2,Demo 3,Demo 4";
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

// Speichere Klickposition
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// Speichere Mausbewegung
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_v2Oldtrackmouse.x = m_v2Trackmouse.x;
	m_v2Oldtrackmouse.y = m_v2Trackmouse.y;
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
	return spring.Stiffness * (spring.CurrentLength - spring.InitLenght) *
		((point1 - point2) / spring.CurrentLength);
}

// Demo Szenen Setup für Demo 2,3
void MassSpringSystemSimulator::X_SetupDefaultDemo()
{
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	m_fSphereSize = .05f;
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0.0f), Vec3(-1.0, 0.0f, 0.0f), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0.0f), Vec3(1.0, 0.0f, 0.0f), false);
	addSpring(p0, p1, 1.0);
}

// Demo Szene für Demo 4
void MassSpringSystemSimulator::X_SetupComplexDemo()
{
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(10.0f);
	m_fSphereSize = .05f;
	applyExternalForce(Vec3(0, 0, 0));
	// Massepunkte
	int p0 = addMassPoint(Vec3(0.0f, 3.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), true);
	int p1 = addMassPoint(Vec3(1.0f, 3.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p2 = addMassPoint(Vec3(2.0f, 3.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p3 = addMassPoint(Vec3(0.0f, 3.0f, 1.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p4 = addMassPoint(Vec3(1.0f, 3.0f, 1.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p5 = addMassPoint(Vec3(2.0f, 3.0f, 1.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p6 = addMassPoint(Vec3(0.0f, 3.0f, 2.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p7 = addMassPoint(Vec3(1.0f, 3.0f, 2.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p8 = addMassPoint(Vec3(2.0f, 3.0f, 2.0f), Vec3(0.0f, 0.0f, 0.0f), true);
	// Federn
	addSpring(p0, p1, 1.0f);
	addSpring(p0, p3, 1.0f);
	addSpring(p1, p2, 1.0f);
	addSpring(p1, p4, 1.0f);
	addSpring(p2, p5, 1.0f);
	addSpring(p3, p6, 1.0f);
	addSpring(p3, p4, 1.0f);
	addSpring(p4, p7, 1.0f);
	addSpring(p4, p5, 1.0f);
	addSpring(p5, p8, 1.0f);
	addSpring(p6, p7, 1.0f);
	addSpring(p7, p8, 1.0f);
}

#pragma endregion

// Initialisiere HUD je nach Demo
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.00 max=1.00 step=0.01");
		break;
	case 2:
		// TODO
		break;
	default:
		break;
	}
}

// Setzte Simulation zurück
void MassSpringSystemSimulator::reset()
{
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	m_fDamping = m_fMass = m_fSphereSize = m_fStiffness = 0;
	m_v3ExternalForce = Vec3(0.0f);
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
		X_SetupComplexDemo();
		break;
	}
	case 1:
	{
		cout << "Demo 3!" << endl;
		setIntegrator(MIDPOINT);
		X_SetupComplexDemo();
		break;
	}
	case 2:
	{
		// TODO
		cout << "Demo 4!" << endl;
		break;
	}
	default:
		cout << "Empty Demo!" << endl;
		break;
	}
}

// Externe Kräfte und Dämpfung berechnen
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Wende Gravitation, Dämpfung und Interaktion an
	for(auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
	{
		masspoint->Force = Vec3(0.0f, masspoint->Mass * m_fGravity, 0.0f) -
			(m_fDamping * masspoint->Velocity) + m_v3ExternalForce;
		masspoint->ForceTilde = Vec3(0.0f, masspoint->Mass * m_fGravity, 0.0f) -
			(m_fDamping * masspoint->Velocity) + m_v3ExternalForce;
	}
}

// OPT.: Leapfrog
// Simuliere einen Schritt
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// Je nach Integrationsverfahren
	switch (m_iIntegrator)
	{
	case EULER:
	{
		// Aktualisiere Längen und Kräfte
		for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
		{
			// Akkumuliere Federkraft
			spring->Point1.Force += X_CalcSpringForce(*spring, spring->Point1.Position, spring->Point2.Position);
			// Analog für Punkt 2
			spring->Point2.Force += -1.0f * spring->Point1.Force;
		}
		// Aktualisiere Geschwindigkeit/Position
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			// Nur für bewegliche Punkte
			if (!masspoint->Fixed)
			{
				// Aktualisiere zuerst Position
				masspoint->Position += timeStep * masspoint->Velocity;
				// Dann Geschwindigkeit
				masspoint->Velocity = masspoint->Velocity + (((-1.0f * masspoint->Force) / masspoint->Mass) * timeStep);
			}
			// Kraft zurüksetzen
			masspoint->Force = Vec3(0.0f);
		}
		break;
	}
	case MIDPOINT:
	{
		//Berechnen aller Positionen nach h/2 Schritt
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++) {
			if (!masspoint->Fixed) {
				masspoint->PositionTilde = masspoint->Position + (1.0f * timeStep / 2) * masspoint->Velocity;
			}
		}
		// Aktualisiere Längen und Kräfte (incl. h/2)
		for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
		{
			float l_fLengthTilt = sqrt((float)spring->Point1.PositionTilde.squaredDistanceTo(spring->Point2.PositionTilde));
			// Akkumuliere Federkraft
			spring->Point1.Force += X_CalcSpringForce(*spring, spring->Point1.Position, spring->Point2.Position);
			spring->Point1.ForceTilde += spring->Stiffness* (l_fLengthTilt - spring->InitLenght)*
				(spring->Point1.PositionTilde - spring->Point2.PositionTilde) / l_fLengthTilt;
			// Analog für Punkt 2
			spring->Point2.Force += -1.0f * spring->Point1.Force;
			spring->Point2.ForceTilde += -1.0f * spring->Point1.ForceTilde;
		}
		// Aktualisiere Geschwindigkeit/Position
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			if (!masspoint->Fixed) {
				masspoint->Position += timeStep * (masspoint->Velocity + (1.0f * timeStep / 2) *
					(((-1.0f * masspoint->Force) / masspoint->Mass)));
				masspoint->Velocity += timeStep * (((-1.0f * masspoint->ForceTilde) / masspoint->Mass));
			}
			// Kraft zurücksetzten
			masspoint->Force = Vec3(0.0f);
			masspoint->ForceTilde = Vec3(0.0f);
		}
		break;
	}
	case LEAPFROG:
	{
		// OPT.
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

// TODO
// Füge Kraft hinzu (bswp. durch Mausinteraktion)
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{

}

#pragma endregion

#pragma region Initialisation

MassSpringSystemSimulator::MassSpringSystemSimulator() :
	m_v3ExternalForce(Vec3(0.0f)),
	m_fSphereSize(0.0f),
	m_fStiffness(0.0f),
	m_fGravity(9.81f),
	m_v2Oldtrackmouse{},
	m_fDamping(0.0f),
	m_iIntegrator(0),
	m_v2Trackmouse{},
	m_MassPoints{},
	m_fMass(0.0f),
	m_Springs{}
{
	m_iTestCase = 0;
}

#pragma endregion
