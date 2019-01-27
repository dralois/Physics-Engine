#include "pch.h"
#include "SPHSystemSimulator.h"

#pragma region Properties

// Get Testcases
const char * SPHSystemSimulator::getTestCasesStr()
{
	return "Naive Collision Handling, RB Collision Handling with Impulse";
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

// Smooth function W
std::function<float(Vec3, Vec3)> SPHSystemSimulator::m_W = [](Vec3 x, Vec3 xi)
{
	// Stabilität: Distanz zwischen x und xi ist logischerweise immer größer als GRIDRADIUS
	float distance = fmaxf(norm(x - xi), 2.0f * GRIDRADIUS);
	float q = distance / KERNELRADIUS;
	float factor = 1.5f / (powf(KERNELRADIUS, 3.0f) *PI);
	if (q >= 0.0f && q < 1.0f)
	{
		float unfactored = 0.66667f - powf(q, 2.0f) + (0.5f * powf(q, 3.0f));
		return factor * unfactored;
	}
	else if (q >= 1.0f && q < 2.0f)
	{
		float unfactored = 0.1666f * powf(2.0f - q, 3.0f);
		return factor * unfactored;
	}
	else
	{
		return 0.0f;
	}
};

// Nabla Operator ▽W
std::function<Vec3(Vec3, Vec3)> SPHSystemSimulator::m_Nabla = [](Vec3 x, Vec3 xi)
{
	// Stabilität: Distanz zwischen x und xi ist logischerweise immer größer als GRIDRADIUS
	float distance = fmaxf(norm(x - xi), 2.0f * GRIDRADIUS);
	float q = distance / KERNELRADIUS;
	Vec3 dist = (x - xi) / distance;
	float factor = 2.25f / (PI * powf(KERNELRADIUS, 5.0f));
	if (q >= 0.0f && q < 1.0f)
	{
		float unfactored = (q - 1.3333f) * q * KERNELRADIUS;
		return dist * factor * unfactored;
	}
	else if (q >= 1.0f && q < 2.0f)
	{
		float unfactored = -pow(2.0f - q, 2.0f) * KERNELRADIUS / 3.0f;
		return dist * factor * unfactored;
	}
	else
	{
		return Vec3(0.0f);
	}
};

// Kernel Funktionen
std::function<float(float)> SPHSystemSimulator::m_CollisionKernels[5] = {
	[](float x) {return 1.0f; },									// Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },							// Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); },	// Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },				// Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },		// Electric Charge, m_iKernel = 4
};

// Erstellt Demo Szene
void SPHSystemSimulator::X_SetupDemo()
{
	// Seeden
	srand(time(NULL));
	// Anzahl halbieren
	int particleCount = PARTICLECOUNT / LAYERCOUNT;
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
	for (int layer = 0; layer < LAYERCOUNT; layer++)
	{
		for (int i = 0; i < particleCount; i++)
		{
			// Zelle berechnen
			int cellX = i % gridDim;
			int cellY = i / gridDim;
			// Erster Layer
			Particle newBall;
			newBall.Density = newBall.Pressure = 0.0f;
			newBall.Force = newBall.Velocity = newBall.OldPosition = newBall.OldVelocity = Vec3(0.0f);
			newBall.Position = Vec3(		cellX * GRIDRADIUS * 2.0f + GRIDRADIUS,
										layer * GRIDRADIUS * 2.0f,
										cellY * GRIDRADIUS * 2.0f + GRIDRADIUS);
			m_Particles.push_back(newBall);
		}
	}
}

// Sortiert alle Bälle in ensprechende Zellen ein
vector<int> SPHSystemSimulator::X_SortBalls()
{
	// Zuerst vectors aufräumen
	m_GridOcc.clear();
	m_GridOcc.resize(m_iGridWidth * m_iGridWidth);
	m_ParticleGrid.clear();
	m_ParticleGrid.resize(m_iGridWidth * m_iGridWidth * MAXCOUNT);

	vector<int> notEmpty;
	// Alle Bälle sortieren
	for (auto ball = m_Particles.begin(); ball != m_Particles.end(); ball++)
	{
		int x = fminf(floorf(ball->Position.x / (GRIDRADIUS * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int y = fminf(floorf(ball->Position.z / (GRIDRADIUS * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		if (x < 0) x = 0;
		if (y < 0) y = 0;
		int index = ((y * m_iGridWidth) + x);
		// In passender Zelle speichern falls möglich
		if (m_GridOcc[index] < MAXCOUNT)
		{
			m_ParticleGrid[(index * MAXCOUNT) + m_GridOcc[index]++] = &*ball;
			// Wenn index noch nicht in notEmpty gespeichert ist...
			if (find(notEmpty.begin(), notEmpty.end(), index) == notEmpty.end())
				notEmpty.push_back(index);
		}
	}
	// Gebe belegte Zellen zurück
	return notEmpty;
}

// überprüfe Zellennachbarn auf Überschneidungen
// pi_iNeighborRadius: 1 für Kollision Detektion, KERNELRADIUS / (2*GRIDRADIUS) für SPH Simulation
vector<int> SPHSystemSimulator::X_CheckNeighbors(int pi_iCell, int pi_iNeighborRadius, vector<int> notEmpty)
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

			// Nur wenn die Nachbarzelle nicht leer ist
			if (find(notEmpty.begin(), notEmpty.end(), index) != notEmpty.end())
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
	if (particle.Position.x > m_v3BoxSize.x - GRIDRADIUS)
	{
		particle.Position.x = m_v3BoxSize.x - GRIDRADIUS;
		particle.Velocity.x = -0.5f * particle.Velocity.x;
	}
	if (particle.Position.y > m_v3BoxSize.y - GRIDRADIUS)
	{
		particle.Position.y = m_v3BoxSize.y - GRIDRADIUS;
		particle.Velocity.y = -0.9f * particle.Velocity.y;
	}
	if (particle.Position.z > m_v3BoxSize.z - GRIDRADIUS)
	{
		particle.Position.z = m_v3BoxSize.z - GRIDRADIUS;
		particle.Velocity.z = -0.5f * particle.Velocity.z;
	}

	// Negative Richtung clampen
	if (particle.Position.x < GRIDRADIUS)
	{
		particle.Position.x = GRIDRADIUS;
		particle.Velocity.x = -0.5f * particle.Velocity.x;
	}
	if (particle.Position.y < 0.0f)
	{
		particle.Position.y = 0.0f;
		particle.Velocity.y = -0.2f * particle.Velocity.y;
	}
	if (particle.Position.z < GRIDRADIUS)
	{
		particle.Position.z = GRIDRADIUS;
		particle.Velocity.z = -0.5f * particle.Velocity.z;
	}
}

void SPHSystemSimulator::X_CalcPressureForceNaive()
{
	// Density
	for (int ownParticle = 0; ownParticle < m_Particles.size(); ownParticle++)
	{
		for (int neighborParticle = 0; neighborParticle < m_Particles.size(); neighborParticle++)
		{
			m_Particles[ownParticle].Density += m_fParticleMass *
				m_W(m_Particles[ownParticle].Position, m_Particles[neighborParticle].Position);
		}
	}
	// Pressure
	for (int particle = 0; particle < m_Particles.size(); particle++)
	{
		m_Particles[particle].Pressure =
			m_fFluidStiffness * (powf(m_Particles[particle].Density / RESTDENSITY, PRESSUREPOWER) - 1.0f);
	}
	// Force
	for (int ownParticle = 0; ownParticle < m_Particles.size(); ownParticle++)
	{
		Vec3 pressureForce = Vec3(0.0f);
		for (int neighborParticle = 0; neighborParticle < m_Particles.size(); neighborParticle++)
		{
			if (neighborParticle == ownParticle)
				continue;

			pressureForce += (m_fParticleMass / m_Particles[neighborParticle].Density) *
				((m_Particles[neighborParticle].Pressure + m_Particles[ownParticle].Pressure) / 2.0f) *
				m_Nabla(m_Particles[ownParticle].Position, m_Particles[neighborParticle].Position);
		}
		m_Particles[ownParticle].Force -= pressureForce;
	}
}

// Druckkraft berechnen
void SPHSystemSimulator::X_CalcPressureForce(vector<int> toCheck)
{
	// Bestimme Dichte
	for (int currCell = 0; currCell < toCheck.size(); currCell++)
	{
		// Bestimme Nachbarzellen
		// KERNELRADIUS ist 1, GRIDRADIUS ist 0.1, d.h. maximal 25 Nachbarzellen
		// Übrigens, GRIDRADIUS = 0.1 ist eventuell ein schlechter Wert, es soll ein bisschen größer sein...
		vector<int> neighbors = X_CheckNeighbors(toCheck[currCell], ceil(KERNELRADIUS / (GRIDRADIUS * 2.0f)), toCheck);
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
					m_ParticleGrid[ownParticle]->Density += m_fParticleMass *
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
				m_fFluidStiffness * (powf(m_ParticleGrid[ownParticle]->Density / RESTDENSITY, 7.0f) - 1.0f);
		}
	}

	// Bestimme Druckkraft
	for (int currCell = 0; currCell < toCheck.size(); currCell++)
	{
		// Bestimme Nachbarzellen
		vector<int> neighbors = X_CheckNeighbors(toCheck[currCell], ceil(KERNELRADIUS / (GRIDRADIUS * 2.0f)), toCheck);
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
					pressureForce += (m_fParticleMass / m_ParticleGrid[neighborParticle]->Density) *
						((m_ParticleGrid[neighborParticle]->Pressure + m_ParticleGrid[ownParticle]->Pressure) / 2.0f) *
						m_Nabla(m_ParticleGrid[ownParticle]->Position, m_ParticleGrid[neighborParticle]->Position);
				}
			}
			// Addiere invertierte Kraft
			m_ParticleGrid[ownParticle]->Force -= pressureForce;
		}
	}
}

// Kollisionsbehandlung aus Exercise 3, super einfach
void SPHSystemSimulator::X_ApplyCollisionEx3(Particle & p1, Particle & p2, function<float(float)> & kernel, float fScaler)
{
	// Zum Achten: man braucht nur die Kraft für Ball 1 aktualisieren, nicht für Ball 2, sonst ist jede Kraft zweimal berechnet
	float dist = sqrtf(p1.Position.squaredDistanceTo(p2.Position));
	if (dist <= 2.0f * GRIDRADIUS)
	{
		Vec3 collNorm = getNormalized(p1.Position - p2.Position);
		p1.Force += -fScaler * kernel(dist / GRIDRADIUS * 2.0f) * collNorm;
	}
}

// Kollisionsbehandlung aus Exercise 2, mit Impulse
void SPHSystemSimulator::X_ApplyCollisionEx2(Particle & p1, Particle & p2)
{
	float dist = sqrtf(p1.Position.squaredDistanceTo(p2.Position));
	if (dist <= 2.0f * GRIDRADIUS)
	{
		// Für Kollisionen zwischen Bällen die Normale und Inertia Tensor sind sehr einfach
		Vec3 relV = p1.Velocity - p2.Velocity;
		Vec3 n = normalize(p1.Position - p2.Position);
		float inertiaTensor = 0.66667f * m_fParticleMass * powf(GRIDRADIUS, 2.0f);
		float tensorInverse = 1.0f / inertiaTensor;

		Vec3 helpA = cross(tensorInverse * cross(p1.Position, n), p1.Position);
		Vec3 helpB = cross(tensorInverse * cross(p2.Position, n), p2.Position);

		Vec3 J = (-(1.0f + m_fImpulseCoefficient) * relV * n) / ((2.0f / m_fParticleMass) + ((helpA + helpB) * n));
		
		p1.Velocity += J * n / m_fParticleMass;
	}
}

// Löst Kollisionen auf
void SPHSystemSimulator::collisionResolve(function<float(float)> & kernel, float fScaler, vector<int> toCheck)
{
	// Für alle belegten Zellen
	for (int i = 0; i < toCheck.size(); i++)
	{
		// Bestimme Nachbarindizes
		vector<int> neighbors = X_CheckNeighbors(toCheck[i], 1, toCheck);
		// Für alle Bälle in der Zelle
		for (int k = 0; k < m_GridOcc[toCheck[i]]; k++)
		{
			// Für alle Nachbarn
			for (int j = 0; j < neighbors.size(); j++)
			{
				for (int l = 0; l < m_GridOcc[neighbors[j]]; l++)
				{
					int own = (toCheck[i] * MAXCOUNT) + k;
					int neighbor = (neighbors[j] * MAXCOUNT) + l;
					// Überspringe Kollision mit selbst
					if (own == neighbor)
						continue;
					// Löse Kollision auf
					if(m_bRBCollision)
						X_ApplyCollisionEx2(*m_ParticleGrid[own], *m_ParticleGrid[neighbor]);
					else
						X_ApplyCollisionEx3(*m_ParticleGrid[own], *m_ParticleGrid[neighbor], kernel, fScaler);
					
				}
			}
		}
	}	
}

#pragma endregion

// Initialisiere UI
void SPHSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Particle Mass", TW_TYPE_FLOAT, &m_fParticleMass, "min=0.0001 step=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "Fluid Stiffness", TW_TYPE_FLOAT, &m_fFluidStiffness, "min=0.05 step=0.5");
	TwAddVarRW(DUC->g_pTweakBar, "Collision Factor", TW_TYPE_FLOAT, &m_fCollisionScale, "min=0.05 step=0.5");
	TwAddVarRW(DUC->g_pTweakBar, "Impulse Coefficient", TW_TYPE_FLOAT, &m_fImpulseCoefficient, "min=-0.95 step=0.02");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.05 step=0.05");
}

// Setzte Simulation zurück
void SPHSystemSimulator::reset()
{
	m_fParticleMass = PARTICLEMASS;
	m_fFluidStiffness = FLUIDSTIFFNESS;
	m_fCollisionScale = COLLISIONSCALE;
	m_fImpulseCoefficient = IMPULSECOEFFICIENT;
	m_fDamping = DAMPING;
	m_v3BoxPos = m_v3BoxSize = Vec3(0.0f);
	m_v3Shifting = Vec3(0.0f, GRIDRADIUS, 0.0f);
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
		// DUC->drawSphere(ball->Position + m_v3BoxPos + m_v3Shifting, Vec3(KERNELRADIUS));
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

	switch (testCase) 
	{
	case 0: 
	{
		cout << "Using naive collision handling!" << endl;
		m_bRBCollision = false;
		break;
	}
	case 1:
	{
		cout << "Using RB collision hanlding from Ex2!" << endl;
		m_bRBCollision = true;
		break;
	}
	default:
		cout << "WTF test case are you using?" << endl;
		m_bRBCollision = false;
		break;
	}
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
		mouseForce = worldViewInv.transformVectorNormal(inputView) * -MOUSEFORCESCALE;
	}
	m_v3MousForce = mouseForce;
}

// Simulation updaten
void SPHSystemSimulator::simulateTimestep(float timeStep)
{
	Vec3 gravity = Vec3(0, -GRAVITY * m_fParticleMass, 0);
	Vec3 damping = Vec3(0.0f);
	
	// erster halber Zeitschritt
	// Position und Velocity kopieren
	for (auto particle = m_Particles.begin(); particle != m_Particles.end(); particle++)
	{
		particle->OldPosition = particle->Position;
		particle->OldVelocity = particle->Velocity;
	}
	// Bälle in Grid sortieren
	vector<int> toCheck = X_SortBalls();
	// Berechne Druck
	// X_CalcPressureForceNaive();
	X_CalcPressureForce(toCheck);
	// Kollisionen behandeln
	collisionResolve(m_CollisionKernels[1], m_fCollisionScale, toCheck);
	// Aktualisiere Geschwindigkeit/Position (Midpoint)
	for (auto particle = m_Particles.begin(); particle != m_Particles.end(); particle++)
	{
		// Zuerst Position aktualisieren, danach Velocity
		damping = -m_fDamping * particle->Velocity;
		particle->Position += 0.5f * timeStep * particle->Velocity;
		particle->Velocity += 0.5f * timeStep * (particle->Force + gravity + m_v3MousForce + damping) / m_fParticleMass;
		// Zurücksetzen
		particle->Density = particle->Pressure = 0.0f;
		particle->Force = Vec3(0.0f);
	}	

	// zweiter halber Zeitschritt
	// Bälle in Grid sortieren
	toCheck = X_SortBalls();
	// Berechne Druck
	// X_CalcPressureForceNaive();
	X_CalcPressureForce(toCheck);
	// Kollisionen behandeln
	collisionResolve(m_CollisionKernels[1], m_fCollisionScale, toCheck);
	// Aktualisiere Geschwindigkeit/Position (Midpoint)
	for (auto particle = m_Particles.begin(); particle != m_Particles.end(); particle++)
	{
		// Zuerst Position aktualisieren, danach Velocity
		damping = -m_fDamping * particle->Velocity;
		particle->Position = particle->OldPosition + timeStep * particle->Velocity;
		particle->Velocity = particle->OldVelocity + timeStep * (particle->Force + gravity + m_v3MousForce + damping) / m_fParticleMass;
		// Zurücksetzen
		particle->Density = particle->Pressure = 0.0f;
		particle->Force = particle->OldPosition = particle->OldVelocity = Vec3(0.0f);
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
