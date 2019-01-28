#pragma once

#include "pch.h"
#include "RigidBodySystemSimulator.h"

#define PARTICLECOUNT 100
#define GRIDRESOLUTION 10
#define SEARCHRADIUS 1

struct Particle
{
	nVec3i cell;
	Vec3 position;
	Vec3 velocity;
	Vec3 velocityDelta;
	Vec3 acceleration;
	double mass;
	double density;
	double pressure;
	vector<Particle*> neighbours;
	bool isLeapFrogInitialized;
	bool isObstacle;
};

class SPHSystemSimulator:public Simulator
{
public:
	// Construtors
	SPHSystemSimulator();
	~SPHSystemSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void addFluidParticles(std::vector<Vec3> points);
	void addFluidParticle(Vec3 pos, Vec3 velocity);
	Particle* addObstacleParticle(Vec3 pos);

private:
	// Rigidbody
	RigidBodySystemSimulator * rbSim;

	// Partikel Speicher
	vector<vector<vector<vector<Particle*>>>> m_Grid;
	std::vector<Particle*> fluidParticles;
	std::vector<Particle*> obstacleParticles;
	std::vector<Particle*> allParticles;

	// Konstanten
	double	smoothingRadius;
	Vec3		gravityForce;
	double	fluidStiffness;
	double	restDensity;
	double	initialDensity;
	double	particleMass;
	double	boundaryDamping;
	double	poly6Coefficient;
	double	spikeyGradCoefficient;

	// Clamping
	double xmin = 0.0;
	double xmax = 1.0;
	double ymin = 0.0;
	double ymax = 1.0;
	double zmin = 0.0;
	double zmax = 1.0;

	// Funktionen
	void initDemo();
	vector<nVec3i> updateGrid();
	vector<nVec3i> getNeighbors(nVec3i cell, int pi_iNeighborRadius, vector<nVec3i> notEmpty);
	Particle* createParticle(Vec3 pos, Vec3 velocity);
	Particle* createObstacleParticle(Vec3 pos);
	void enforceFluidParticlePositionBounds(Particle *p);
	void updateNearestNeighbours(vector<nVec3i> notEmpty);
	void updateFluidDensityAndPressure();
	void updateFluidAcceleration();
	void updateFluidPosition(double dt);
};
