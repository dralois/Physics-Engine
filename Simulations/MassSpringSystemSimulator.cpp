#include "MassSpringSystemSimulator.h"

#pragma region Properties

// Get verfügbare Testcases
const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
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
	for(auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
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
		m_oldtrackmouse.x = x;
		m_oldtrackmouse.y = y;
		m_trackmouse.x = x;
		m_trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

// TODO
// Initialisiere HUD je nach Demo
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	default:break;
	}
}

// TODO
// Setzte Simulation zurück?
void MassSpringSystemSimulator::reset()
{
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
}

// TODO
// Rendere Simulation
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{

}

// TODO
// Aktiviere verschiedene Demos
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1 !\n";
		break;
	case 1:
		cout << "Demo 2 !\n";
		break;
	case 2:
		cout << "Demo 3 !\n";
		break;
	case 3:
		cout << "Demo 4 !\n";
		break;
	default:
		cout << "Empty Test!\n";
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
		case EULER:
		{
			// Aktualisiere Längen und Kräfte
			for (auto spring = m_Springs.begin(); spring != m_Springs.end(); spring++)
			{
				spring->CurrentLenght = sqrt((float) spring->Point1.Position.squaredDistanceTo(spring->Point2.Position));
				// F_ij = -k * (l - L) * (x_i - x_j) / l
				spring->Point1.Force = -1.0f * spring->Stiffness * (spring->CurrentLenght - spring->InitLenght) *
					((spring->Point1.Position - spring->Point2.Position) / spring->CurrentLenght);
				// Analog für Punkt 2
				spring->Point2.Force = -1.0f * spring->Point1.Force;
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
					masspoint->Velocity = masspoint->Velocity + ((masspoint->Force/masspoint->Mass) * timeStep);
				}
			}

			break;
		}
		case MIDPOINT:
		{

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
	newMass.Damping = m_fDamping;
	newMass.Position = position;
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
	newSpring.InitLenght = newSpring.CurrentLenght = initialLength;
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
