#include "pch.h"

#include "SphereSystem.h"

#pragma region Functions

#pragma region Internal

// Bälle dürfen Box nicht verlassen
void SphereSystem::X_ApplyBoundingBox(Ball & ball)
{
	// Positive Richtung clampen
	if (ball.Position.x > m_v3BoxSize.x)
		ball.Position.x = m_v3BoxSize.x;
	if (ball.Position.y > m_v3BoxSize.y)
		ball.Position.y = m_v3BoxSize.y;
	if (ball.Position.z > m_v3BoxSize.z)
		ball.Position.z = m_v3BoxSize.z;
	// Negative Richtung clampen
	if (ball.Position.x < 0.0f)
		ball.Position.x = 0.0f;
	if (ball.Position.y < 0.0f)
		ball.Position.y = 0.0f;
	if (ball.Position.z < 0.0f)
		ball.Position.z = 0.0f;
}

// Überprüft auf Kollision und updatet Kräfte
void SphereSystem::X_ApplyCollision(Ball & ball1, Ball & ball2, const function<float(float)>& kernel, float fScaler)
{
	float dist = sqrtf(ball1.Position.squaredDistanceTo(ball2.Position));
	// Radius kleiner Abstand?
	if(dist < ball1.Radius)
	{
		Vec3 collNorm = getNormalized(ball1.Position - ball2.Position);
		ball1.Force += fScaler * kernel(dist / ball1.Radius * 2.0f) * collNorm;
	}
}

#pragma endregion

// Rendert Bälle in übergebener Farbe
void SphereSystem::drawFrame(DrawingUtilitiesClass* DUC, const Vec3& v3Color)
{
	// Farbe einrichten
	DUC->setUpLighting(Vec3(), 0.6f*Vec3(1.0f), 100.0f, v3Color);
	// Bälle rendern
	for(auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		DUC->drawSphere(ball->Position, Vec3(ball->Radius));
	}
}

// Externe Kräfte anwenden (z.B. Maus, Gravitation, Damping)
// TODO Demo 1,2,3
void SphereSystem::externalForcesCalculations(float timeElapsed, Vec3 v3MouseForce)
{

}

// Simuliert einen Zeitschritt
// TODO Demo 1,2,3
void SphereSystem::simulateTimestep(float timeStep)
{
	for(auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{

		// Als letztes Position clampen
		X_ApplyBoundingBox(*ball);
	}
}

// Löst Kollisionen auf
void SphereSystem::collisionResolve(const function<float(float)>& kernel, float fScaler)
{
	switch (m_iAccelerator)
	{
	case 0:
	{
		for(auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
		{
			for (auto coll = m_Balls.begin(); coll != m_Balls.end(); coll++)
			{
				// Verhindere Kollision mit selbst
				if (ball != coll)
				{
					// Löse Kollision auf falls es eine gibt
					X_ApplyCollision(*ball, *coll, kernel, fScaler);
				}
			}
		}
		break;
	}
	case 1:
	{
		break;
	}
	case 2:
	{
		cout << "KD currently not supported!" << endl;
		break;
	}
	default:
		cout << "No valid accelerator was given!" << endl;
		break;
	}
}

#pragma endregion

#pragma region Initialisation

// Initialsiert neues Ballsystem
SphereSystem::SphereSystem(int pi_iAccelerator,
	int pi_iNumSpheres, float pi_fRadius, float pi_fMass) :
	m_iAccelerator(pi_iAccelerator)
{
	// Zellengröße bestimmen
	float gridDim = ceilf(sqrtf((float) pi_iNumSpheres));
	// Als Box speichern
	m_v3BoxSize = Vec3(gridDim);
	// Bälle erstellen
	for(int i = 0; i < pi_iNumSpheres; i++)
	{
		Ball newBall;
		// Erstelle Ball
		newBall.Force = newBall.ForceTilde = newBall.PositionTilde = Vec3(0.0f);
		// Spawne Bälle in einem Matrixraster
		newBall.Position = Vec3(floorf(((float) i) / gridDim),
			((float) i) - (floorf(((float) i) / gridDim) * gridDim), gridDim);
		newBall.Radius = pi_fRadius;
		newBall.Mass = pi_fMass;
		// Im Array speichern
		m_Balls.push_back(newBall);
	}
}

#pragma endregion
