#ifndef SPHERESYSTEMSIMULATOR_h
#define SPHERESYSTEMSIMULATOR_h

#include "pch.h"
#include "spheresystem.h"
#include "ctime"

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
	void speedComparison();
	void speedComparisonSimulateTimeStep(float timeStep);
	static void TW_CALL startMeasure(void * pi_pMyClass);

private:
	// Attributes
	float			m_fMass = 1.0f;
	float			m_fRadius = 0.1f;
	float			m_fForceScaling = 35.0f;
	float			m_fDamping = 0.5f;
	float			m_fGravity = 9.81f;
	int				m_iBallNumber = 72;
	int				m_iCase3 = 0;
	
	// UI Attributes
	Point2D			m_v2Oldtrackmouse;
	Point2D			m_v2Trackmouse;
	Point2D			m_v2Mouse;

	// Other
	int						m_iKernel;
	int						m_iAccelerator;
	SphereSystem*	m_pSphereSystem;
	SphereSystem*	m_pSphereSystemGrid;
	SphereSystem*	m_pSphereSystemmeasure;
	static				std::function<float(float)> m_Kernels[5];
	Vec3				m_v3ShiftingLeft  = Vec3(-0.8f, 0.0f, 0.0f);
	Vec3				m_v3ShiftingRight = Vec3(0.8f, 0.0f, 0.0f);

	//Functions
	void	X_SetupDemo();
};

#endif