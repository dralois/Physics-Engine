#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h

#include "pch.h"

#define TESTCASEUSEDTORUNTEST 2

struct Rigidbody
{
	Mat4 Translation;
	Mat4 Rotation;
	Mat4 Scale;
	Vec3 LinVel;
	Vec3 AngVel;
	Vec3 AngMom;
	Vec3 Torque;
	Vec3 Force;
	Mat4 InertiaTensorInv;
	float Mass;
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
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

	// Extra functions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, float mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Data Attributes
	bool							m_HasPrinted;
	Vec3							m_v3ExternalForce;
	float							m_fCollisionCoefficient;

	// Simulation Data
	vector<Rigidbody>	m_Rigidbodies;

	// UI Attributes
	Point2D						m_v2Oldtrackmouse;
	Point2D						m_v2Trackmouse;
	Point2D						m_v2Mouse;

	// Functions
	void X_SetupDemo(int demoNr);
	void X_ApplyForceOnBody(Rigidbody & rb, Vec3 loc, Vec3 force);
	void X_CalculateInertiaTensor(Rigidbody & rb);
	void X_CalculateImpulse(Rigidbody & rb_A, Rigidbody & rb_B, CollisionInfo & coll);
};

#endif
