#pragma once

#include "pch.h"
#include "spheresystem.h"

#define MAXCOUNT 20
#define PARTICLECOUNT 128
// Achtung! GRIDRADIUS soll nicht größer als halb KERNELRADIUS sein!
#define KERNELRADIUS 1.0f
#define GRIDRADIUS 0.4f
#define PARTICLEMASS 1.0f
#define FLUIDSTIFFNESS 15.0f
#define RESTDENSITY 3.0f
#define PI 3.141592653f

struct Particle
{
	Vec3 Position;
	Vec3 Velocity;
	Vec3 Force;
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
	float				m_fGravity = 9.81f;
	Vec3					m_v3BoxSize;
	Vec3					m_v3BoxPos;
	Vec3					m_v3Shifting;

	// UI Attributes
	Point2D				m_v2Oldtrackmouse;
	Point2D				m_v2Trackmouse;
	Point2D				m_v2Mouse;

	// Other
	vector<Particle*>	m_ParticleGrid;			// Speichert: Zelleninformationen
	vector<int>			m_GridOcc;				// Speichert: wie viele Bälle gibts in der Zelle mit index
	int					m_iGridWidth;
	static				std::function<float(Vec3, Vec3)> m_W;
	static				std::function<Vec3(Vec3, Vec3)> m_Nabla;

	//Functions
	void	X_SetupDemo();
	vector<int> X_SortBalls();
	vector<int> X_CheckNeighbors(int pi_iCell, int pi_iNeighborRadius, vector<int> notEmpty);
	void X_ApplyBoundingBox(Particle& ball);
	void X_CalcPressureForce();
	void X_CalcPressureForceNaive();
};
