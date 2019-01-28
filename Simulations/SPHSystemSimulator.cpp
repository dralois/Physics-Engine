#include "pch.h"
#include "SPHSystemSimulator.h"

#pragma region Properties

// Get Testcases
const char * SPHSystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2";
}

#pragma endregion

#pragma region Events

// Mausbewegung während Klicken
void SPHSystemSimulator::onClick(int x, int y)
{
	if (rbSim)
		rbSim->onClick(x, y);
}

// Mausbewegung normal
void SPHSystemSimulator::onMouse(int x, int y)
{
	if(rbSim)
		rbSim->onMouse(x, y);
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// Hilfsfunktion
double getRandIn01()
{
	return (1.0 * rand()) / (1.0 * RAND_MAX) * 0.9;
}

// Initialisiert die Demo
void SPHSystemSimulator::initDemo()
{
	// Rigidbody Partikel
	vector<Vec3> curr = rbSim->getCurrParticles();
	for(auto rbp = curr.begin(); rbp != curr.end(); rbp++)
	{
		addObstacleParticle(*rbp);
	}
	// Flüssigkeit Partikel
	vector<Vec3> positions;
	// Zufällige Positionen berechnen
	for(int i = 0; i < PARTICLECOUNT; i++)
	{
		positions.push_back(Vec3(getRandIn01(), getRandIn01(), getRandIn01()));
	}
	// Erstellen
	addFluidParticles(positions);
}

// Erstellt einen Partikel
Particle* SPHSystemSimulator::createParticle(Vec3 pos, Vec3 velocity)
{
	Particle *s = new Particle();

	s->position = pos;
	s->velocity = velocity;
	s->velocityDelta = Vec3(0.0, 0.0, 0.0);
	s->isLeapFrogInitialized = false;
	s->acceleration = Vec3(0.0, 0.0, 0.0);
	s->density = initialDensity;
	s->mass = particleMass;
	s->pressure = 0.0;

	return s;
}

// Erstellt einen Hinderniss Partikel
Particle* SPHSystemSimulator::createObstacleParticle(Vec3 pos)
{
	Particle *p = createParticle(pos, Vec3(0.0, 0.0, 0.0));
	return p;
}

// Sortiert alle Bälle in ensprechende Zellen ein
vector<nVec3i> SPHSystemSimulator::updateGrid()
{
	vector<nVec3i> notEmpty;
	// Grid leeren
	m_Grid.clear();
	// Grid neu aufbauen
	m_Grid.resize(GRIDRESOLUTION);
	for (int i = 0; i < GRIDRESOLUTION; i++)
	{
		m_Grid[i].resize(GRIDRESOLUTION);
		for(int j = 0; j < GRIDRESOLUTION; j++)
		{
			m_Grid[i][j].resize(GRIDRESOLUTION);
		}
	}
	// Alle Partikel sortieren
	for (auto particle = allParticles.begin(); particle != allParticles.end(); particle++)
	{
		int x = fmaxf(0, fminf(floorf((*particle)->position.x / (xmax / GRIDRESOLUTION)), GRIDRESOLUTION - 1));
		int y = fmaxf(0, fminf(floorf((*particle)->position.y / (ymax / GRIDRESOLUTION)), GRIDRESOLUTION - 1));
		int z = fmaxf(0, fminf(floorf((*particle)->position.z / (zmax / GRIDRESOLUTION)), GRIDRESOLUTION - 1));
		m_Grid[x][y][z].push_back(*particle);
		// Speichere Zelle
		(*particle)->cell = nVec3i(x, y, z);
		notEmpty.push_back(nVec3i(x, y, z));
	}
	// Vektor zurückgeben
	return notEmpty;
}

// Überprüfe Zellennachbarn auf Überschneidungen
vector<nVec3i> SPHSystemSimulator::getNeighbors(nVec3i cell, int pi_iNeighborRadius, vector<nVec3i> notEmpty)
{
	vector<nVec3i> neighbors;
	// überprüfe Nachbarn
	for (int x = cell.x - pi_iNeighborRadius; x <= cell.x + pi_iNeighborRadius; x++)
	{
		for (int y = cell.y - pi_iNeighborRadius; y <= cell.y + pi_iNeighborRadius; y++)
		{
			for (int z = cell.z - pi_iNeighborRadius; z <= cell.z + pi_iNeighborRadius; z++)
			{
				// Randfälle behandeln
				if (x < 0 || x >= m_Grid.size())
					continue;
				if (y < 0 || y >= m_Grid[x].size())
					continue;
				if (z < 0 || z >= m_Grid[x][y].size())
					continue;
				// Nur wenn die Nachbarzelle nicht leer ist in den Vektor
				if (find(notEmpty.begin(), notEmpty.end(), nVec3i(x,y,z)) != notEmpty.end())
					neighbors.push_back(nVec3i(x,y,z));
			}
		}
	}
	// Gebe Nachbarnliste zurück
	return neighbors;
}

// Aktualisiert Nachbarnlisten
void SPHSystemSimulator::updateNearestNeighbours(vector<nVec3i> notEmpty)
{
	Particle *sp;
	// Alle Partikel durchgehen (auch Rigidbody)
	for (int i = 0; i < allParticles.size(); i++)
	{
		sp = allParticles[i];
		sp->neighbours.clear();
		// Hole Nachbarzellen
		vector<nVec3i> nCells = getNeighbors(sp->cell, SEARCHRADIUS, notEmpty);
		// Gehe durch die Nachbarzellen
		for (auto nCell = nCells.begin(); nCell != nCells.end(); nCell++)
		{
			// Gehe durch die Partikel darin
			for (auto n = m_Grid[nCell->x][nCell->y][nCell->z].begin(); n != m_Grid[nCell->x][nCell->y][nCell->z].end(); n++)
			{
				// Füge alle Nachbarn hinzu
				sp->neighbours.push_back(*n);
			}
		}
	}
}

// Aktualisiert Dichte und Druck
void SPHSystemSimulator::updateFluidDensityAndPressure()
{
	Vec3 r;
	Particle *pi, *pj;
	// Alle Partikel durchgehen (auch Rigidbody)
	for (int i = 0; i < allParticles.size(); i++)
	{
		pi = allParticles[i];
		double density = 0.0;
		// Alle Nachbarn durchgehen
		for (int j = 0; j < pi->neighbours.size(); j++)
		{
			pj = pi->neighbours[j];
			// Radius berechnen
			r = pi->position - pj->position;
			// Dichte aufsummieren
			density += pj->mass*poly6Coefficient*pow(pow(smoothingRadius, 2.0) - dot(r, r), 3.0);
		}
		// Dichte darf nicht unter die minimale Dichte fallen (-> Negativer Druck)
		pi->density = fmax(density, initialDensity);
		// Druck berechnen
		pi->pressure = fluidStiffness * (pi->density - restDensity);
	}
}

// Aktualisiert Beschleunigung
void SPHSystemSimulator::updateFluidAcceleration()
{
	Particle *pi, *pj;
	Vec3 acc, r, vdiff;
	// Alle "echten" Partikel durchgehen
	for (int i = 0; i < fluidParticles.size(); i++)
	{
		pi = fluidParticles[i];
		acc = Vec3(0.0, 0.0, 0.0);
		// Gehe durch die Nachbarn des Partikels
		for (int j = 0; j < pi->neighbours.size(); j++)
		{
			pj = pi->neighbours[j];
			r = pi->position - pj->position;
			double dist = norm(r);
			// Nicht selbst miteinbeziehen
			if (dist == 0.0)
				continue;
			// Norm berechnen
			float inv = 1 / dist;
			r = inv * r;
			// Nabla berechnen
			float spikey = spikeyGradCoefficient * pow(smoothingRadius - dist, 2.0);
			float massRatio = pj->mass / pi->mass;
			float pterm = (pi->pressure + pj->pressure) / (2 * pi->density*pj->density);
			// Beschleunigung berechnen
			acc -= massRatio * pterm * spikey * r;
		}
		// Gravitation hinzufügen
		acc += gravityForce;
		// Speichern
		pi->acceleration = acc;
	}
}

// Positions / Geschwindigkeit Clamping
void SPHSystemSimulator::enforceFluidParticlePositionBounds(Particle *p)
{
	double eps = 0.001;
	float damping = boundaryDamping;
	// In X Position und Geschwindigkeit clampen
	if (p->position.x < xmin) {
		p->position = Vec3(xmin + eps, p->position.y, p->position.z);
		p->velocity = Vec3(-damping * p->velocity.x, p->velocity.y, p->velocity.z);
	}
	else if (p->position.x > xmax) {
		p->position = Vec3(xmax - eps, p->position.y, p->position.z);
		p->velocity = Vec3(-damping * p->velocity.x, p->velocity.y, p->velocity.z);
	}
	// In Y Position und Geschwindigkeit clampen
	if (p->position.y < ymin) {
		p->position = Vec3(p->position.x, ymin + eps, p->position.z);
		p->velocity = Vec3(p->velocity.x, -damping * p->velocity.y, p->velocity.z);
	}
	else if (p->position.y > ymax) {
		p->position = Vec3(p->position.x, ymax - eps, p->position.z);
		p->velocity = Vec3(p->velocity.x, -damping * p->velocity.y, p->velocity.z);
	}
	// In Z Position und Geschwindigkeit clampen
	if (p->position.z < zmin) {
		p->position = Vec3(p->position.x, p->position.y, zmin + eps);
		p->velocity = Vec3(p->velocity.x, p->velocity.y, -damping * p->velocity.z);
	}
	else if (p->position.z > zmax) {
		p->position = Vec3(p->position.x, p->position.y, zmax - eps);
		p->velocity = Vec3(p->velocity.x, p->velocity.y, -damping * p->velocity.z);
	}
}

// Aktualisiert Position mit Leap Frog Intergration
void SPHSystemSimulator::updateFluidPosition(double timestep)
{
	Particle *p;
	for (int i = 0; i < fluidParticles.size(); i++)
	{
		// Nur "echte" Partikel werden geupdatet
		p = fluidParticles[i];
		// Falls Leap Frog initialisiert ist
		if (p->isLeapFrogInitialized)
		{
			// Update Geschwindigkeit
			p->velocityDelta += timestep * p->acceleration;
		}
		else
		{
			// Initialisiere 1/2 Timestep beim ersten Asufruf
			p->velocityDelta = p->velocity + (0.5*timestep)*p->acceleration;
			p->isLeapFrogInitialized = true;
		}
		// Position updaten mit 1/2 Timestep Geschwindigkeit
		p->position += timestep * p->velocityDelta;
		// Geschwindigkeit updaten
		p->velocity = p->velocityDelta + (0.5*timestep) * p->acceleration;
		// Clampen
		enforceFluidParticlePositionBounds(p);
	}

}

#pragma endregion

// Fügt Hindernis Partikel hinzu
Particle* SPHSystemSimulator::addObstacleParticle(Vec3 pos)
{
	Particle *sp = createObstacleParticle(pos);
	sp->isObstacle = true;

	obstacleParticles.push_back(sp);
	allParticles.push_back(sp);

	return sp;
}

// Fügt "echte" Partikel hinzu
void SPHSystemSimulator::addFluidParticles(std::vector<Vec3> points)
{
	for (int i = 0; i < points.size(); i++)
	{
		addFluidParticle(points[i], Vec3(0.0, 0.0, 0.0));
	}
}

// Fügt "echten" Partikel hinzu
void SPHSystemSimulator::addFluidParticle(Vec3 pos, Vec3 velocity)
{
	Particle *sp = createParticle(pos, velocity);
	sp->isObstacle = false;

	fluidParticles.push_back(sp);
	allParticles.push_back(sp);
}

// Initialisiert UI
void SPHSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	if(rbSim)
		rbSim->DUC = DUC;
}

// Wenn sich der Testcase ändert
void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Setze Simulation zurück
	reset();
	// Initialisiere Demo
	initDemo();
	// Für Gravity Demo
	if (testCase == 1)
	{
		delete rbSim;
		rbSim = NULL;
		initialDensity = 100;
		restDensity = 50;
		gravityForce = Vec3(0.0, -0.02, 0.0);
	}
}

// Externe Kräfte berechen (Maus etc.)
void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	if(rbSim)
	{
		// Interaktion nur mit RB
		rbSim->externalForcesCalculations(timeElapsed);
	}
}

// Simulationsschritt
void SPHSystemSimulator::simulateTimestep(float timestep)
{
	if (rbSim)
	{
		// RB updaten
		rbSim->simulateTimestep(timestep);
		// RB Partikel updaten
		int currCount = 0;
		vector<Vec3> curr = rbSim->getCurrParticles();
		for (auto obstacle = obstacleParticles.begin(); obstacle != obstacleParticles.end(); obstacle++)
		{
			(*obstacle)->position = curr[currCount++];
		}
	}
	// Grid und Nachbarlisten updaten
	vector<nVec3i> notEmpty = updateGrid();
	updateNearestNeighbours(notEmpty);
	// Druck, Dichte und Beschleunigung aktualisieren
	updateFluidDensityAndPressure();
	updateFluidAcceleration();
	// Positionen aktualisieren
	updateFluidPosition(timestep);
}

// Simulation zurücksetzen
void SPHSystemSimulator::reset()
{
	// Rigidbody erstellen
	if(rbSim)
		delete rbSim;
	rbSim = new RigidBodySystemSimulator();
	rbSim->addRigidBody(Vec3(0.0), Vec3(0.5), 10);
	// Konstanten setzen
	initialDensity = 10.0;
	fluidStiffness = 0.7;
	restDensity = 10.0;
	particleMass = 1.0;
	boundaryDamping = 0.1;
	gravityForce = Vec3(0.0, 0.0, 0.0);
	poly6Coefficient = 315.0 / (64.0 * 3.141 * powf(smoothingRadius, 9.0));
	spikeyGradCoefficient = -45.0 / (3.141 * powf(smoothingRadius, 6.0));
	// Lösche alle Partikel
	for (auto particle = allParticles.begin(); particle != allParticles.end(); )
	{
		delete *particle;
		particle = allParticles.erase(particle);
	}
	// Lösche Rest
	obstacleParticles.clear();
	fluidParticles.clear();
	allParticles.clear();
}

// Rendering
void SPHSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmedateContext)
{
	if(rbSim)
		rbSim->drawFrame(pd3dImmedateContext);
	DUC->setUpLighting(Vec3(), 0.6f*Vec3(1.0f), 100.0f, Vec3(0.0, 0.0, 1.0));
	for(auto particle = fluidParticles.begin(); particle != fluidParticles.end(); particle++)
	{
		DUC->drawSphere((*particle)->position, Vec3(0.25));
	}
}

#pragma endregion

#pragma region Initialisation

// Erstelle Simulator
SPHSystemSimulator::SPHSystemSimulator() :
	rbSim(NULL)
{
	srand(time(NULL));
	m_iTestCase = 0;
	smoothingRadius = 1.0;
}

// Aufräumen
SPHSystemSimulator::~SPHSystemSimulator()
{
	// Lösche RB
	if (rbSim)
		delete rbSim;
	// Lösche alle Partikel
	for (auto particle = allParticles.begin(); particle != allParticles.end(); )
	{
		delete *particle;
		particle = allParticles.erase(particle);
	}
	// Lösche Rest
	obstacleParticles.clear();
	fluidParticles.clear();
	allParticles.clear();
}

#pragma endregion