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
	float		m_fForceScaling;
	Vec3		m_v3ExternalForce;
	int			m_iNumSpheres;
	float		m_fRadius;
	float		m_fMass;

	// UI Attributes
	Point2D		m_v2Oldtrackmouse;
	Point2D		m_v2Trackmouse;
	Point2D		m_v2Mouse;

	// Other
	int							m_iKernel;
	static					std::function<float(float)> m_Kernels[5];
	int							m_iAccelerator;
	SphereSystem*		m_pSphereSystem;
	// Für Demo 3
	SphereSystem*		m_pSphereSystemGrid;

	//Functions
	void	X_SetupDemo();
};

#endif