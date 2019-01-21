#include "pch.h"
#include "SPHSystemSimulator.h"

#pragma region Properties

// Get Testcases
const char * SPHSystemSimulator::getTestCasesStr()
{
	return "Demo 1";
}

#pragma endregion

#pragma region Events

// Mausbewegung während Klicken
void SPHSystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// Mausbewegung normal
void SPHSystemSimulator::onMouse(int x, int y)
{
	m_v2Oldtrackmouse.x = x;
	m_v2Oldtrackmouse.y = y;
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// Was ist dieser Umlaut??? Gl鋞tungsoperator W
std::function<float(Vec3, Vec3)> SPHSystemSimulator::m_W = [](Vec3 x, Vec3 xi)
{
	float q = norm(x - xi) / (0.5f * KERNELRADIUS);
	if (q >= 0.0f && q < 1.0f)
	{
		return 0.6666f - powf(q, 2.0f) + (0.5f * powf(q, 3.0f));
	}
	else if (q >= 1.0f && q < 2.0f)
	{
		return 0.1666f * powf(2.0f - q, 3.0f);
	}
	else
	{
		return 0.0f;
	}
};

// Nabla Operator \/W
std::function<Vec3(Vec3, Vec3)> SPHSystemSimulator::m_Nabla = [](Vec3 x, Vec3 xi)
{
	float q = norm(x - xi) / (0.5f * KERNELRADIUS);
	Vec3 dist = (x - xi) / norm(x - xi);
	if (q >= 0.0f && q < 1.0f)
	{
		return dist * ((q - 1.3333f) * q * (0.5f * KERNELRADIUS));
	}
	else if (q >= 1.0f && q < 2.0f)
	{
		return dist * (-pow(2.0f - q, 2.0f) * ((KERNELRADIUS * 0.5f) / 3.0f));
	}
	else
	{
		return Vec3(0.0f);
	}
};

// Erstellt Demo Szene
void SPHSystemSimulator::X_SetupDemo()
{
	// Anzahl halbieren
	int particleCount = PARTICLECOUNT / 2;
	// Zellenanzahl bestimmen
	int gridDim = ceil(sqrtf(particleCount));
	// Als Box speichern
	m_v3BoxSize = Vec3(gridDim * GRIDRADIUS * 2.0f);
	m_v3BoxPos = Vec3(-(gridDim * GRIDRADIUS));
	// Grid erstellen
	m_iGridWidth = gridDim;
	m_GridOcc.resize(gridDim * gridDim);
	m_ParticleGrid.resize(gridDim * gridDim * MAXCOUNT);
	// Bällle erstellen
	for (int i = 0; i < particleCount; i++)
	{
		// Zelle berechnen
		int cellX = i % gridDim;
		int cellY = i / gridDim;
		// Erster Layer
		Particle newBall;
		newBall.Density = newBall.Pressure = 0.0f;
		newBall.Force =  Vec3(0.0f);
		newBall.Position = Vec3(cellX * GRIDRADIUS * 2.0f + GRIDRADIUS,
														gridDim * GRIDRADIUS * 2.0f,
														cellY * GRIDRADIUS * 2.0f + GRIDRADIUS);
		m_Particles.push_back(newBall);
		// Zweiter Layer
		Particle newBall2;
		newBall2.Density = newBall2.Pressure = 0.0f;
		newBall2.Force = Vec3(0.0f);
		newBall2.Position = Vec3(cellX * GRIDRADIUS * 2.0f + GRIDRADIUS,
														(gridDim - 1) * GRIDRADIUS * 2.0f,
														cellY * GRIDRADIUS * 2.0f + GRIDRADIUS);
		m_Particles.push_back(newBall2);
	}
}

// Sortiert alle Bälle in ensprechende Zellen ein
vector<int> SPHSystemSimulator::X_SortBalls()
{
	vector<int> notEmpty;
	// Alle Bälle sortieren
	for (auto ball = m_Particles.begin(); ball != m_Particles.end(); ball++)
	{
		int x = fminf(floorf(ball->Position.x / (GRIDRADIUS * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int y = fminf(floorf(ball->Position.z / (GRIDRADIUS * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int index = ((y * m_iGridWidth) + x);
		// In passender Zelle speichern falls möglich
		if (m_GridOcc[index] < MAXCOUNT)
		{
			m_ParticleGrid[(index * MAXCOUNT) + m_GridOcc[index]++] = &*ball;
			// Belegten Index ggf. speichern
			if (find(notEmpty.begin(), notEmpty.end(), index) == notEmpty.end())
				notEmpty.push_back(index);
		}
	}
	// Gebe belegte Zellen zurück
	return notEmpty;
}

// überprüfe Zellennachbarn auf Überschneidungen
vector<int> SPHSystemSimulator::X_CheckNeighbors(int pi_iCell, int pi_iNeighborRadius)
{
	vector<int> neighbors;
	// Index berechnen
	int cellX = pi_iCell % m_iGridWidth;
	int cellY = pi_iCell / m_iGridWidth;
	// überprüfe Nachbarn
	for (int x = -pi_iNeighborRadius; x <= pi_iNeighborRadius; x++)
	{
		for (int y = -pi_iNeighborRadius; y <= pi_iNeighborRadius; y++)
		{
			// Randfälle behandeln
			if (cellX + x < 0 || cellX + x >= m_iGridWidth)
				continue;
			if (cellY + y < 0 || cellY + y >= m_iGridWidth)
				continue;
			// Speichere alle Nachbarn in Array
			int index = ((cellY + y) * m_iGridWidth) + (cellX + x);
			neighbors.push_back(index);
		}
	}
	// Gebe Nachbarnliste zurück
	return neighbors;
}

// Bälle dürfen Box nicht verlassen
void SPHSystemSimulator::X_ApplyBoundingBox(Particle & particle)
{
	// Positive Richtung clampen und Ball zurückspringen
	if (particle.Position.x > m_v3BoxSize.x)
	{
		particle.Position.x = m_v3BoxSize.x;
	}
	if (particle.Position.y > m_v3BoxSize.y)
	{
		particle.Position.y = m_v3BoxSize.y;
	}
	if (particle.Position.z > m_v3BoxSize.z)
	{
		particle.Position.z = m_v3BoxSize.z;
	}

	// Negative Richtung clampen
	if (particle.Position.x < 0.0f)
	{
		particle.Position.x = 0.0f;
	}
	if (particle.Position.y < 0.0f)
	{
		particle.Position.y = 0.0f;
	}
	if (particle.Position.z < 0.0f)
	{
		particle.Position.z = 0.0f;
	}
}

// Druckkraft berechnen
void SPHSystemSimulator::X_CalcPressureForce()
{
	// Grid zurücksetzen
	for (int i = 0; i < m_GridOcc.size(); i++)
	{
		m_GridOcc[i] = 0;
	}

	// Sortiere Partikel in Zellen, speichere belegte Zellen
	vector<int> toCheck = X_SortBalls();

	// Bestimme Dichte
	for (int currCell = 0; currCell < toCheck.size(); currCell++)
	{
		// Bestimme Nachbarzellen
		vector<int> neighbors = X_CheckNeighbors(toCheck[currCell], KERNELRADIUS);
		for (int currParticle = 0; currParticle < m_GridOcc[toCheck[currCell]]; currParticle++)
		{
			for (int currNeighborCell = 0; currNeighborCell < neighbors.size(); currNeighborCell++)
			{
				for (int currNeighbor = 0; currNeighbor < m_GridOcc[neighbors[currNeighborCell]]; currNeighbor++)
				{
					// Bestimme Indizes
					int ownParticle = (toCheck[currCell] * MAXCOUNT) + currParticle;
					int neighborParticle = (neighbors[currNeighborCell] * MAXCOUNT) + currNeighbor;
					// Berechne Dichte
					m_ParticleGrid[ownParticle]->Density += PARTICLEMASS *
						m_W(m_ParticleGrid[ownParticle]->Position, m_ParticleGrid[neighborParticle]->Position);
				}
			}
		}
	}

	// Bestimme Druck
	for (int currCell = 0; currCell < toCheck.size(); currCell++)
	{
		for (int currParticle = 0; currParticle < m_GridOcc[toCheck[currCell]]; currParticle++)
		{
			int ownParticle = (toCheck[currCell] * MAXCOUNT) + currParticle;
			// Druck berechnen
			m_ParticleGrid[ownParticle]->Pressure =
				FLUIDSTIFFNESS * (powf(m_ParticleGrid[ownParticle]->Density / RESTDENSITY, 7.0f) - 1.0f);
		}
	}

	// Bestimme Druckkraft
	for (int currCell = 0; currCell < toCheck.size(); currCell++)
	{
		// Bestimme Nachbarzellen
		vector<int> neighbors = X_CheckNeighbors(toCheck[currCell], KERNELRADIUS);
		for (int currParticle = 0; currParticle < m_GridOcc[toCheck[currCell]]; currParticle++)
		{
			Vec3 pressureForce = Vec3(0.0f);
			int ownParticle = (toCheck[currCell] * MAXCOUNT) + currParticle;
			// Gehe durch Nachbarn
			for (int currNeighborCell = 0; currNeighborCell < neighbors.size(); currNeighborCell++)
			{
				for (int currNeighbor = 0; currNeighbor < m_GridOcc[neighbors[currNeighborCell]]; currNeighbor++)
				{
					int neighborParticle = (neighbors[currNeighborCell] * MAXCOUNT) + currNeighbor;
					// überspringe Interaktion mit selbst
					if (ownParticle == neighborParticle)
						continue;
					// Druckkraft bestimmen
					pressureForce += (PARTICLEMASS / m_ParticleGrid[neighborParticle]->Density) *
						((m_ParticleGrid[neighborParticle]->Pressure + m_ParticleGrid[ownParticle]->Pressure) / 2.0f) *
						m_Nabla(m_ParticleGrid[ownParticle]->Position, m_ParticleGrid[neighborParticle]->Position);
				}
			}
			// Addiere invertierte Kraft
			m_ParticleGrid[ownParticle]->Force -= pressureForce;
		}
	}
}

#pragma endregion

// Initialisiere UI
void SPHSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

// Setzte Simulation zurück
void SPHSystemSimulator::reset()
{
	m_v3BoxPos = m_v3BoxSize = m_v3Shifting = Vec3(0.0f);
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	m_iGridWidth = 0;
	m_ParticleGrid.clear();
	m_GridOcc.clear();
	m_Particles.clear();
}

// Rendere Simulation
void SPHSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	// Farbe einrichten
	DUC->setUpLighting(Vec3(), 0.6f*Vec3(1.0f), 100.0f, Vec3(0,0,1));
	// Bälle rendern
	for (auto ball = m_Particles.begin(); ball != m_Particles.end(); ball++)
	{
		DUC->drawSphere(ball->Position + m_v3BoxPos + m_v3Shifting, Vec3(GRIDRADIUS));
	}
}

// Aktiviere verschiedene Simulationen
void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	// Setzte Simulation zurück
	reset();
	// Erstelle Szene
	X_SetupDemo();
}

// Externe Kräfte updaten
void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
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
	// Kraft anwenden
	for (auto particle = m_Particles.begin(); particle != m_Particles.end(); particle++)
	{
		//particle->Force = Vec3(0, -1.0f * m_fGravity * PARTICLEMASS, 0);
		//particle->Force += mouseForce;
	}
}

// Simulation updaten
void SPHSystemSimulator::simulateTimestep(float timeStep)
{
	// Berechne Druck
	X_CalcPressureForce();
	// Aktualisiere Geschwindigkeit/Position (Leap Frog)
	for (auto particle = m_Particles.begin(); particle != m_Particles.end(); particle++)
	{
		particle->Velocity += timeStep * (particle->Force / PARTICLEMASS);
		particle->Position += timeStep * particle->Velocity;
		// Zurücksetzen
		particle->Density = particle->Pressure = 0.0f;
		// Als letztes Position clampen
		X_ApplyBoundingBox(*particle);
	}
}

#pragma endregion

#pragma region Initialisation

// Erstelle Simulator
SPHSystemSimulator::SPHSystemSimulator()
{
	srand(time(NULL));
	m_iTestCase = 0;
	// Setzte Simulation zurück
	reset();
	// Erstelle Szene
	X_SetupDemo();
}

// Aufräumen
SPHSystemSimulator::~SPHSystemSimulator()
{
	if (m_ParticleGrid.size() > 0)
	{
		m_ParticleGrid.clear();
		m_GridOcc.clear();
	}
	if (m_Particles.size() > 0)
		m_Particles.clear();
}

#pragma endregion
