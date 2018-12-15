#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h

#include "pch.h"

//#include "spheresystem.h", add your sphere system header file

#define NAIVEACC 0
#define GRIDACC 1

struct Ball
{
	Vec3 Position;
	Vec3 PositionTilde;
	Vec3 Velocity;
	Vec3 Force;
	Vec3 ForceTilde;
	float Mass;
	float Radius;
};

class SphereSystemSimulator:public Simulator{
public:
	// Construtors
	SphereSystemSimulator();
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

protected:
	// Attributes
	float		m_fForceScaling;
	Vec3		m_v3ExternalForce;
	int			m_iNumSpheres;
	float		m_fDamping;
	float		m_fRadius;
	float		m_fMass;
	// UI Attributes
	Point2D	m_v2Oldtrackmouse;
	Point2D	m_v2Trackmouse;
	Point2D	m_v2Mouse;

	int			m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	static	std::function<float(float)> m_Kernels[5];

	int			m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)

	//SphereSystem * m_pSphereSystem; // add your own sphere system member!
	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	// SphereSystem * m_pSphereSystemGrid; 

};

#endif