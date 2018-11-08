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
	for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
	{
		masspoint->Damping = m_fDamping;
	}
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
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Speichere Mausbewegung
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	/* !!!!!!
	 logischerweise soll es so sein:
		m_oldtrackmouse.x = m_trackmouse.x;
		m_oldtrackmouse.y = m_trackmouse.y;
		m_trackmouse.x = x;
		m_trackmouse.y = y;
	*/
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// Berechnet Federkraft
Vec3 MassSpringSystemSimulator::X_CalcSpringForce(Spring &spring, const Vec3 &point1, const Vec3 &point2)
{
	// Aktualisiere Länge
	spring.CurrentLength = sqrt((float)point1.squaredDistanceTo(point2));
	// F_ij = -k * (l - L) * (x_i - x_j) / l


	/* !!!!!!
		Problem mit dieser Methode wenn zwei Punkte überlappe sich:
		1. point1 - point2 ist ein null Vektor
		2. CurrentLength = 0 als Nenner
	*/
	return spring.Stiffness * (spring.CurrentLength - spring.InitLenght) *
		((point1 - point2) / spring.CurrentLength);
}

// Demo Szenen Setup für Demo 1-3
void MassSpringSystemSimulator::X_SetupDefaultDemo()
{
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	m_fSphereSize = .05f;
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);
}

#pragma endregion

// TODO
// Initialisiere HUD je nach Demo
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	default:
		break;
	}
}

// Setzte Simulation zurück
void MassSpringSystemSimulator::reset()
{
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_fDamping = m_fMass = m_fSphereSize = m_fStiffness = 0;
	m_externalForce = Vec3(0.0f);
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

// TODO
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
		cout << "Demo 4!" << endl;
		break;
	}
	default:
		cout << "Empty Demo!" << endl;
		break;
	}
}

// TODO
// Gravitation etc. berechnen
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}

// TODO: Midpoint, Damping, External forces, ggf. Leapfrog
// Simuliere einen Schritt
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// Je nach Integrationsverfahren
	switch (m_iIntegrator)
	{

	/*
		Richtlinien hier:
		for all points:
			1. clear force
			2. add external force
		for all springs:
			1. computer force
			2. add force to points
		for all points:
			1. integrate position
			2. integrate velocity

		!!!!! nur möglich, wenn: 
		m_Masspoints und m _Springs speichern Zeiger auf dieselben Puntke!!!
	*/
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
		Vec3 Vel_tilt;
		Vec3 Acc;
		Vec3 Acc_tilt;
		float length_tilt;

		//Berechnen aller Positionen nach h/2 Schritt
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++) {
			if (!masspoint->Fixed) {
				masspoint->Pos_tilt = masspoint->Position + (1.0f * timeStep / 2) * masspoint->Velocity;
			}
		}

		// Aktualisiere Längen und Kräfte (incl. h/2)
		for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
		{
			length_tilt = sqrt((float)spring->Point1.Pos_tilt.squaredDistanceTo(spring->Point2.Pos_tilt));
			// Akkumuliere Federkraft
			spring->Point1.Force += X_CalcSpringForce(*spring, spring->Point1.Position, spring->Point2.Position);
			spring->Point1.Force_tilt += spring->Stiffness* (length_tilt - spring->InitLenght)*
				(spring->Point1.Pos_tilt - spring->Point2.Pos_tilt) / length_tilt;

			// Analog für Punkt 2
			spring->Point2.Force += -1.0f * spring->Point1.Force;
			spring->Point2.Force_tilt += -1.0f * spring->Point1.Force_tilt;
		}

		// Aktualisiere Geschwindigkeit/Position
		for (auto masspoint = m_MassPoints.begin(); masspoint != m_MassPoints.end(); masspoint++)
		{
			if (!masspoint->Fixed) {
				Acc = ((-1.0f * masspoint->Force) / masspoint->Mass);
				Vel_tilt = masspoint->Velocity + (1.0f * timeStep / 2) * Acc;
				Acc_tilt = ((-1.0f * masspoint->Force_tilt) / masspoint->Mass);
				masspoint->Position += timeStep * Vel_tilt;
				masspoint->Velocity += timeStep * Acc_tilt;
			}
			masspoint->Force = Vec3(0.0f);
			masspoint->Force_tilt = Vec3(0.0f);
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
}

// Fügt Massepunkt hinzu, gibt dessen Position im Array zurück
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	// Erstelle neuen Massepunkt
	Masspoint newMass;
	newMass.Force = m_externalForce;
	newMass.Force_tilt = Vec3(0.0f);
	newMass.Damping = m_fDamping;
	newMass.Position = position;
	newMass.Pos_tilt = position;
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
	m_externalForce(Vec3(0.0f)),
	m_fSphereSize(0.0f),
	m_fStiffness(0.0f),
	m_oldtrackmouse{},
	m_fDamping(0.0f),
	m_iIntegrator(0),
	m_trackmouse{},
	m_MassPoints{},
	m_fMass(0.0f),
	m_Springs{}
{
	m_iTestCase = 0;
}

#pragma endregion
