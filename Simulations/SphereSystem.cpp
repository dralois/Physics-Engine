#include "pch.h"

#include "SphereSystem.h"

#pragma region Functions

#pragma region Internal

// Sortiert alle Bälle in ensprechende Zellen ein
vector<int> SphereSystem::X_SortBalls()
{
	vector<int> notEmpty;
	// Alle Bälle sortieren
	for(auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		int x = floorf(ball->Position.x / ball->Radius);
		int y = floorf(ball->Position.z / ball->Radius);
		int index = ((y * m_iGridWidth) + x) * MAXCOUNT;
		// In passender Zelle speichern falls möglich
		if(m_GridOccupation[index] < MAXCOUNT)
		{
			m_GridAccelerator[index + m_GridOccupation[index]++] = &*ball;
			// Belegten Index ggf. speichern
			if (find(notEmpty.begin(), notEmpty.end(), index) == notEmpty.end())
				notEmpty.push_back(index);
		}
	}
	// Gebe belegte Zellen zurück
	return notEmpty;
}

// überprüfe Zellennachbarn auf Kollisionen
vector<int> SphereSystem::X_CheckNeighbors(int pi_iCell)
{
	vector<int> neighbors;
	// Index berechnen
	int cellX = pi_iCell % m_iGridWidth;
	int cellY = pi_iCell / m_iGridWidth;
	// überprüfe Nachbarn
	for(int x = -1; x < 2; x++)
	{
		for(int y = -1; y < 2; y++)
		{
			// Eigene Zelle ignorieren
			if (x == y == 0)
				continue;
			// Randfälle behandeln
			if (cellX + x < 0 || cellX + x >= m_iGridWidth)
				continue;
			if (cellY + y < 0 || cellY + y >= m_GridOccupation.size() / m_iGridWidth)
				continue;
			// Speichere alle Nachbarn in Array
			int index = ((cellY + y) * m_iGridWidth) + (cellX + x);
			for(int j = 0; j < m_GridOccupation[index]; j++)
			{
				neighbors.push_back(index + j);
			}
		}
	}
	// Gebe Nachbarnliste zurück
	return neighbors;
}

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

// überprüft auf Kollision und updatet Kräfte
void SphereSystem::X_ApplyCollision(Ball & ball1, Ball & ball2, const function<float(float)>& kernel, float fScaler)
{
	float dist = sqrtf(ball1.Position.squaredDistanceTo(ball2.Position));
	// Radius kleiner Abstand?
	if(dist <= ball1.Radius + ball2.Radius)
	{
		Vec3 collNorm = getNormalized(ball1.Position - ball2.Position);
		Vec3 collForce = fScaler * kernel(dist / ball1.Radius * 2.0f) * collNorm;
		ball1.Force += collForce;
		ball2.Force -= collForce;
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

// Externe Kräte anwenden (z.B. Maus, Gravitation, Damping)
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
	// Naiver Ansatz: Alles mit allem
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
	// Grid Acceleration
	case 1:
	{
		// Sortiere Bälle in Zellen, speichere belegte Zellen
		vector<int> toCheck = X_SortBalls();
		// Für alle belegten Zellen
		for(int i = 0; i < toCheck.size(); i++)
		{
			// Bestimme Nachbarindizes
			vector<int> neighbors = X_CheckNeighbors(toCheck[i]);
			// Für alle Bälle in der Zelle
			for(int k = 0; k < m_GridOccupation[toCheck[i]]; k++)
			{
				// Für alle Nachbarn
				for(int j = 0; j < neighbors.size(); j++)
				{
					// Löse Kollision auf
					X_ApplyCollision(	*m_GridAccelerator[toCheck[i] + k],
														*m_GridAccelerator[neighbors[j]],
														kernel, fScaler);
				}
			}
		}
		break;
	}
	case 2:
	{
		cout << "KD currently not supported!" << endl;
		break;
	}
	default:
		cout << "No valid accelerator selected!" << endl;
		break;
	}
}

#pragma endregion

#pragma region Initialisation

// Initialsiert neues Ballsystem
SphereSystem::SphereSystem(	int pi_iAccelerator, int pi_iNumSpheres,
														float pi_fRadius, float pi_fMass) :
	m_iAccelerator(pi_iAccelerator)
{
	// Zellengröße bestimmen
	float gridDim = ceilf(sqrtf((float) pi_iNumSpheres));
	// Als Box speichern
	m_v3BoxSize = Vec3(gridDim * pi_fRadius);
	// Grid erstellen
	m_iGridWidth = gridDim;
	m_GridOccupation.resize(gridDim * gridDim);
	m_GridAccelerator.resize(gridDim * gridDim * MAXCOUNT);
	// Bälle erstellen
	for(int i = 0; i < pi_iNumSpheres; i++)
	{
		Ball newBall;
		// Erstelle Ball
		newBall.Force = newBall.ForceTilde = newBall.PositionTilde = Vec3(0.0f);
		// Spawne Bälle in einem Matrixraster
		newBall.Position = Vec3(floorf(((float) i) / gridDim),
			((float)i) - (floorf(((float)i) / gridDim) * gridDim), gridDim);
		newBall.Radius = pi_fRadius;
		newBall.Mass = pi_fMass;
		// Im Array speichern
		m_Balls.push_back(newBall);
	}
}

// Aufräumen
SphereSystem::~SphereSystem()
{
	m_GridAccelerator.clear();
	m_GridOccupation.clear();
	m_Balls.clear();
}

#pragma endregion
