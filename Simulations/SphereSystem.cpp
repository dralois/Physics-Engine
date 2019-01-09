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
		int x = fminf(floorf(ball->Position.x / (ball->Radius * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int y = fminf(floorf(ball->Position.z / (ball->Radius * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int index = ((y * m_iGridWidth) + x);
		// In passender Zelle speichern falls möglich
		if(m_GridOccupation[index] < MAXCOUNT)
		{
			m_GridAccelerator[(index * MAXCOUNT) + m_GridOccupation[index]++] = &*ball;
			// Belegten Index ggf. speichern
			if (find(notEmpty.begin(), notEmpty.end(), index) == notEmpty.end())
				notEmpty.push_back(index);
		}
	}
	// Gebe belegte Zellen zurück
	return notEmpty;
}

// Überprüfe Zellennachbarn auf Kollisionen
vector<int> SphereSystem::X_CheckNeighbors(int pi_iCell)
{
	vector<int> neighbors;
	// Index berechnen
	int cellX = pi_iCell % m_iGridWidth;
	int cellY = pi_iCell / m_iGridWidth;
	// Überprüfe Nachbarn
	for(int x = -1; x < 2; x++)
	{
		for(int y = -1; y < 2; y++)
		{
			// Randfälle behandeln
			if (cellX + x < 0 || cellX + x >= m_iGridWidth)
				continue;
			if (cellY + y < 0 || cellY + y >= m_iGridWidth)
				continue;
			// Speichere alle Nachbarn in Array
			int index = ((cellY + y) * m_iGridWidth) + (cellX + x);
			neighbors.push_back(index);
		}
	}
	// Gebe Nachbarnliste zurück
	return neighbors;
}

// Bälle dürfen Box nicht verlassen
void SphereSystem::X_ApplyBoundingBox(Ball & ball)
{
	// Positive Richtung clampen und Ball zurückspringen
	if (ball.Position.x > m_v3BoxSize.x)
	{
		ball.Position.x = m_v3BoxSize.x;
		ball.Velocity = Vec3(-1.0 * ball.Velocity.x, ball.Velocity.y, ball.Velocity.z);
	}
	if (ball.Position.z > m_v3BoxSize.z)
	{
		ball.Position.z = m_v3BoxSize.z;
		ball.Velocity = Vec3(ball.Velocity.x, ball.Velocity.y, -1.0f * ball.Velocity.z);
	}
	
	// Negative Richtung clampen
	if (ball.Position.x < 0.0f)
	{
		ball.Position.x = 0.0f;
		ball.Velocity = Vec3(-1.0 * ball.Velocity.x, ball.Velocity.y, ball.Velocity.z);
	}
	if (ball.Position.y < 0.0f)
	{
		ball.Position.y = 0.0f;
		ball.Velocity = Vec3(ball.Velocity.x, -1.0f * ball.Velocity.y, ball.Velocity.z);
	}
	if (ball.Position.z < 0.0f)
	{
		ball.Position.z = 0.0f;
		ball.Velocity = Vec3(ball.Velocity.x, ball.Velocity.y, -1.0f * ball.Velocity.z);
	}
}

// überprüft auf Kollision und updatet Kräfte
void SphereSystem::X_ApplyCollision(Ball & ball1, Ball & ball2, function<float(float)> & kernel, float fScaler)
{
	// Zum Achten: man braucht nur die Kraft für Ball 1 aktualisieren, nicht für Ball 2, sonst ist jede Kraft zweimal berechnet
	// Für Kollisionen am Anfang vom Zeitschritt
	float dist = sqrtf(ball1.Position.squaredDistanceTo(ball2.Position));
	if(dist <= ball1.Radius + ball2.Radius)
	{
		Vec3 collNorm = getNormalized(ball1.Position - ball2.Position);
		ball1.Force += -fScaler * kernel(dist / ball1.Radius * 2.0f) * collNorm;
	}

	// Für Kollisionen in der Mitte vom Zeitschritt
	float distTilde = sqrtf(ball1.PositionTilde.squaredDistanceTo(ball2.PositionTilde));
	if (distTilde <= ball1.Radius + ball2.Radius)
	{
		Vec3 collNormTilde = getNormalized(ball1.Position - ball2.Position);
		ball1.ForceTilde += -fScaler * kernel(dist / ball1.Radius * 2.0f) * collNormTilde;
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
		DUC->drawSphere(ball->Position + m_v3BoxPos + m_v3Shifting, Vec3(ball->Radius));
	}
}

// Externe Kräte anwenden (z.B. Maus, Gravitation, Damping)
void SphereSystem::externalForcesCalculations(float timeElapsed, Vec3 v3MouseForce)
{
	for (auto ball = m_Balls.begin(); ball != m_Balls.end() && !m_Deleted; ball++)
	{
		ball->Force += Vec3(0, -1.0f * m_fGravity * ball->Mass, 0);
		ball->Force += v3MouseForce;
		ball->Force -= m_fDamping * ball->Velocity;

		ball->ForceTilde += Vec3(0, -1.0f * m_fGravity * ball->Mass, 0);
		ball->ForceTilde += v3MouseForce;
	}
}

// Simuliert einen Zeitschritt
void SphereSystem::simulateHalfTimestep(float timeStep)
{
	//Berechnen aller Positionen nach h/2 Schritt
	for (auto ball = m_Balls.begin(); ball != m_Balls.end() && !m_Deleted; ball++)
	{
		ball->PositionTilde = ball->Position + ball->Velocity * timeStep / 2.0f;
	}
}

// Simuliert einen Zeitschritt
void SphereSystem::simulateTimestep(float timeStep)
{
	// Aktualisiere Geschwindigkeit/Position
	for (auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		Vec3 midPointVelocity = ball->Velocity + (ball->Force / ball->Mass) * timeStep / 2.0f;
		ball->Position += timeStep * midPointVelocity;
		ball->ForceTilde -= m_fDamping * midPointVelocity;
		ball->Velocity += timeStep * (ball->ForceTilde / ball->Mass) ;
		// Kraft zurücksetzen
		ball->Force = Vec3(0.0f);
		ball->ForceTilde = Vec3(0.0f);
		// Als letztes Position clampen
		X_ApplyBoundingBox(*ball);
	}
}

// Löst Kollisionen auf
void SphereSystem::collisionResolve(function<float(float)> & kernel, float fScaler)
{
	if (m_Deleted)
		return;

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
		for(int i = 0; i < m_GridOccupation.size(); i++)
		{
			m_GridOccupation[i] = 0;
		}
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
					for(int l = 0; l < m_GridOccupation[neighbors[j]]; l++)
					{
						int own = (toCheck[i] * MAXCOUNT) + k;
						int neighbor = (neighbors[j] * MAXCOUNT) + l;
						// Überspringe Kollision mit selbst
						if (own == neighbor)
							continue;
						// Löse Kollision auf
						X_ApplyCollision(	*m_GridAccelerator[own],
															*m_GridAccelerator[neighbor],
															kernel, fScaler);
					}
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
SphereSystem::SphereSystem(	int pi_iAccelerator, int pi_iNumSpheres, float pi_fRadius, float pi_fMass, 
							float pi_fDamping, float pi_fForceScaling, float pi_fGravity, Vec3 pi_v3Shifting)
{
	m_iAccelerator = pi_iAccelerator;
	m_fDamping = pi_fDamping;
	m_v3Shifting = pi_v3Shifting;
	m_fGravity = pi_fGravity;

	// Anzahl halbieren
	pi_iNumSpheres /= 2;
	// Zellenanzahl bestimmen
	int gridDim = ceil(sqrtf(pi_iNumSpheres));
	// Als Box speichern
	m_v3BoxSize = Vec3(gridDim * pi_fRadius * 2.0f);
	m_v3BoxPos = Vec3(-(gridDim * pi_fRadius));
	// Grid erstellen
	m_iGridWidth = gridDim;
	m_GridOccupation.resize(gridDim * gridDim);
	m_GridAccelerator.resize(gridDim * gridDim * MAXCOUNT);
	// Bälle erstellen
	for(int i = 0; i < pi_iNumSpheres; i++)
	{
		// Zelle berechnen
		int cellX = i % gridDim;
		int cellY = i / gridDim;
		// Erstelle Ball
		Ball newBall1;
		newBall1.Force = newBall1.ForceTilde = newBall1.PositionTilde = Vec3(0.0f);
		newBall1.Position = Vec3(	cellX * pi_fRadius * 2.0f + pi_fRadius,
															gridDim * pi_fRadius * 2.0f,
															cellY * pi_fRadius * 2.0f + pi_fRadius);
		newBall1.Radius = pi_fRadius;
		newBall1.Mass = pi_fMass;
		// Im Array speichern
		m_Balls.push_back(newBall1);

		// Andere Balllayer
		Ball newBall2;
		newBall2.Force = newBall2.ForceTilde = newBall2.PositionTilde = Vec3(0.0f);
		newBall2.Position = Vec3(	cellX * pi_fRadius * 2.0f + pi_fRadius,
															(gridDim - 1) * pi_fRadius * 2.0f,
															cellY * pi_fRadius * 2.0f + pi_fRadius);
		newBall2.Radius = pi_fRadius;
		newBall2.Mass = pi_fMass;
		// Im Array speichern
		m_Balls.push_back(newBall2);
	}
}

// Aufräumen
SphereSystem::~SphereSystem()
{
	m_Deleted = true;
	if (m_GridAccelerator.size() > 0)
	{
		m_GridAccelerator.clear();
		m_GridOccupation.clear();
	}
	if(m_Balls.size() > 0)
		m_Balls.clear();
}

#pragma endregion
