#pragma once

#include "pch.h"

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

class SphereSystem
{
public:
	// Construtors
	SphereSystem(int pi_iAccelerator, int pi_iNumSpheres,
		float pi_fRadius, float pi_fMass);
	// Functions
	void drawFrame(DrawingUtilitiesClass* DUC, const Vec3& v3Color);
	void externalForcesCalculations(float timeElapsed, Vec3 v3MouseForce);
	void simulateTimestep(float timeStep);
	void collisionResolve(const function<float(float)>& kernel, float fScaler);
private:
	// Attributes
	vector<Ball>	m_Balls;
	int						m_iAccelerator;
	Vec3					m_v3BoxSize;
	
	// Other

	// Functions
	void X_ApplyBoundingBox(Ball& ball);
	void X_ApplyCollision(Ball& ball1, Ball& ball2,
		const function<float(float)>& kernel, float fScaler);
};
