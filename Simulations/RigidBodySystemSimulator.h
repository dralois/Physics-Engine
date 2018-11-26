#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h

#include "Simulator.h"
#include "util/quaternion.h"
#include "util/matrixbase.h"
#include <algorithm>
#include <vector>

#define TESTCASEUSEDTORUNTEST 2

struct Rigidbody
{
	Mat4 Translation;
	Mat4 Rotation;
	Mat4 Scale;
	Vec3 LinVel;
	Vec3 AngVel;
	Vec3 Torque;
	Mat4 InertiaTensor;
	int Mass;
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

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Data Attributes
	Vec3							m_v3ExternalForce;

	// Simulation Data
	vector<Rigidbody>	m_Ridigbodies;

	// UI Attributes
	Point2D						m_v2Oldtrackmouse;
	Point2D						m_v2Trackmouse;
	Point2D						m_v2Mouse;

	// Functions
	void X_SetupDemo(int demoNr);
	Mat4 X_CalculateInertiaTensor(Rigidbody & rb);
};

#endif
