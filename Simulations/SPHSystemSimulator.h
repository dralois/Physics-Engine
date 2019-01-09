#pragma once

#include "pch.h"
#include "spheresystem.h"

#define MAXCOUNT 10

struct Particle
{
	Vec3 Position;
	Vec3 PositionTilde;
	Vec3 Velocity;
	Vec3 Force;
	Vec3 ForceTilde;
	float Mass;
	float Radius;
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
	float					m_fGravity = 9.81f;
	int						m_iBallNumber = 72;
	vector<Particle>	m_Balls;
	Vec3				m_v3BoxSize;
	Vec3				m_v3BoxPos;
	Vec3				m_v3Shifting;

	// UI Attributes
	Point2D			m_v2Oldtrackmouse;
	Point2D			m_v2Trackmouse;
	Point2D			m_v2Mouse;

	// Other
	vector<Particle*>	m_GridAccelerator;
	vector<int>		m_GridOccupation;
	int						m_iGridWidth;

	//Functions
	void	X_SetupDemo();
	vector<int> X_SortBalls();
	vector<int> X_CheckNeighbors(int pi_iCell);
	void X_ApplyBoundingBox(Particle& ball);
};

