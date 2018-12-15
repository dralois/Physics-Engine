#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h

#include "pch.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct Masspoint
{
	Vec3 Position;
	Vec3 PositionTilde;
	Vec3 Velocity;
	Vec3 Force;
	Vec3 ForceTilde;
	float Mass;
	bool Fixed;
};

struct Spring
{
	Masspoint &Point1;
	Masspoint &Point2;
	float Stiffness;
	float InitLenght;
	float CurrentLength;
	// Standartkonstruktor
	Spring::Spring(Masspoint &ref1, Masspoint &ref2) :
		Point1(ref1),
		Point2(ref2),
		Stiffness(0),
		InitLenght(0),
		CurrentLength(0) {}
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator)
	{
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float							m_fMass;
	float							m_fStiffness;
	float							m_fDamping;
	float							m_fGravity;
	int								m_iIntegrator;
	bool							m_bRunningManualTest;

	// Simulation Data
	vector<Masspoint>	m_MassPoints;
	vector<Spring>		m_Springs;

	// UI Attributes
	float							m_fSphereSize;
	Vec3							m_v3ExternalForce;
	Point2D						m_v2Oldtrackmouse;
	Point2D						m_v2Trackmouse;

	// Functions
	Vec3 X_CalcSpringForce(Spring &spring, const Vec3 &point1, const Vec3 &point2);
	void X_InternalForcesCalculations();
	void X_SetupDefaultDemo();
	void X_SetupComplexDemo();
	void X_ApplyBounding();
};

#endif