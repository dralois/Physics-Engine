#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h

#include "pch.h"

#include "spheresystem.h"

#define NAIVEACC 0
#define GRIDACC 1

class SphereSystemSimulator:public Simulator{
public:
	// Construtors
	SphereSystemSimulator();
	~SphereSystemSimulator();
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
	float			m_fMass = 1.0f;
	float			m_fRadius = 0.1f;
	float			m_fForceScaling = 15.0f;

	// UI Attributes
	Point2D			m_v2Oldtrackmouse;
	Point2D			m_v2Trackmouse;
	Point2D			m_v2Mouse;

	// Other
	int						m_iKernel;
	int						m_iAccelerator;
	SphereSystem*	m_pSphereSystem;
	SphereSystem*	m_pSphereSystemGrid;
	static				std::function<float(float)> m_Kernels[5];

	//Functions
	void	X_SetupDemo();
};

#endif