#pragma once

#include "pch.h"
#include "spheresystem.h"

#define MAXCOUNT 15
#define PARTICLECOUNT 108
#define LAYERCOUNT 3				// wie viele Schichten von Bällen gibt es
#define KERNELRADIUS 0.18f		// Achtung! GRIDRADIUS soll nicht größer als halb KERNELRADIUS sein!
#define GRIDRADIUS 0.082f		// GRIDRADIUS ist auch Halbmesser für Particle
#define PRESSUREPOWER 7.0f
#define RESTDENSITY 1.0f
#define DAMPING 0.3f
#define MOUSEFORCESCALE 0.3f
#define PI 3.141592653f
#define GRAVITY 9.81f
// Variablen, die den Effekt stark beeinflussen können
#define PARTICLEMASS 0.001f
#define FLUIDSTIFFNESS 8.0f
#define COLLISIONSCALE 8.0f
#define IMPULSECOEFFICIENT 0.2f


struct Particle
{
	Vec3 Position;
	Vec3 Velocity;
	Vec3 Force;
	Vec3 OldPosition;
	Vec3 OldVelocity;
	float Density;
	float Pressure;
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
private:
	// Attributes
	vector<Particle>		m_Particles;				// Speichter: alle Bälle
	Vec3					m_v3BoxSize;
	Vec3					m_v3BoxPos;
	Vec3					m_v3Shifting = Vec3(0.0f, GRIDRADIUS, 0.0f);
	Vec3					m_v3MousForce;

	// UI Attributes
	Point2D				m_v2Oldtrackmouse;
	Point2D				m_v2Trackmouse;
	Point2D				m_v2Mouse;

	// Other
	vector<Particle*>	m_ParticleGrid;			// Speichert: Zelleninformationen
	vector<int>			m_GridOcc;				// Speichert: wie viele Bälle gibts in der Zelle mit index
	int					m_iGridWidth;
	bool					m_bRBCollision;
	float				m_fParticleMass = PARTICLEMASS;
	float				m_fFluidStiffness = FLUIDSTIFFNESS;
	float				m_fCollisionScale = COLLISIONSCALE;
	float				m_fImpulseCoefficient = IMPULSECOEFFICIENT;
	float				m_fDamping = DAMPING;
	static				std::function<float(Vec3, Vec3)> m_W;
	static				std::function<Vec3(Vec3, Vec3)> m_Nabla;
	static				std::function<float(float)> m_CollisionKernels[5];

	//Functions
	void	X_SetupDemo();
	vector<int> X_SortBalls();
	vector<int> X_CheckNeighbors(int pi_iCell, int pi_iNeighborRadius, vector<int> notEmpty);
	void X_ApplyCollisionEx3(Particle & p1, Particle & p2, function<float(float)> & kernel, float fScaler);
	void X_ApplyCollisionEx2(Particle & p1, Particle & p2);
	void collisionResolve(function<float(float)> & kernel, float fScaler, vector<int> toCheck);
	void X_ApplyBoundingBox(Particle& ball);
	void X_CalcPressureForce(vector<int> toCheck);
	void X_CalcPressureForceNaive();
};
