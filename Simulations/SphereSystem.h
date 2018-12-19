#pragma once

#include "pch.h"

#define MAXCOUNT 10

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
	SphereSystem(int pi_iAccelerator, int pi_iNumSpheres, float pi_fRadius, float pi_fMass, 
				 float pi_fDamping, float pi_fForceScaling, float pi_fGravity, Vec3 pi_v3Shifting);
	~SphereSystem();

	// Functions
	void drawFrame(DrawingUtilitiesClass* DUC, const Vec3& v3Color);
	void externalForcesCalculations(float timeElapsed, Vec3 v3MouseForce);
	void simulateTimestep(float timeStep);
	void simulateHalfTimestep(float timeStep);
	void collisionResolve(function<float(float)> & kernel, float fScaler);
private:
	// Attributes
	vector<Ball>m_Balls;
	int					m_iAccelerator;
	Vec3				m_v3BoxSize;
	Vec3				m_v3BoxPos;
	Vec3				m_v3Shifting;
	float			m_fDamping ;
	float			m_fGravity;
	Point2D			m_v2Oldtrackmouse;
	Point2D			m_v2Trackmouse;
	bool				m_Deleted = false;

	// Other
	vector<Ball*>	m_GridAccelerator;
	vector<int>		m_GridOccupation;
	int						m_iGridWidth;

	// Functions
	vector<int> X_SortBalls();
	vector<int> X_CheckNeighbors(int pi_iCell);
	void X_ApplyBoundingBox(Ball& ball);
	void X_ApplyCollision(Ball& ball1, Ball& ball2, function<float(float)> & kernel, float fScaler);
};
