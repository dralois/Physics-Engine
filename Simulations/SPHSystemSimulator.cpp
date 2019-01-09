#include "pch.h"
#include "SPHSystemSimulator.h"

#pragma region Properties

// Get Testcases
const char * SPHSystemSimulator::getTestCasesStr()
{
	return "Demo 1";
}

#pragma endregion

#pragma region Events

// Mausbewegung während Klicken
void SPHSystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// Mausbewegung normal
void SPHSystemSimulator::onMouse(int x, int y)
{
	m_v2Oldtrackmouse.x = x;
	m_v2Oldtrackmouse.y = y;
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// TODO
// Erstellt Demo Szene
void SPHSystemSimulator::X_SetupDemo()
{

}

// Sortiert alle Bälle in ensprechende Zellen ein
vector<int> SPHSystemSimulator::X_SortBalls()
{
	vector<int> notEmpty;
	// Alle Bälle sortieren
	for (auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		int x = fminf(floorf(ball->Position.x / (ball->Radius * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int y = fminf(floorf(ball->Position.z / (ball->Radius * 2.0f)), (m_iGridWidth - 1) * 1.0f);
		int index = ((y * m_iGridWidth) + x);
		// In passender Zelle speichern falls möglich
		if (m_GridOccupation[index] < MAXCOUNT)
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

// Überprüfe Zellennachbarn auf Überschneidungen
vector<int> SPHSystemSimulator::X_CheckNeighbors(int pi_iCell)
{
	vector<int> neighbors;
	// Index berechnen
	int cellX = pi_iCell % m_iGridWidth;
	int cellY = pi_iCell / m_iGridWidth;
	// Überprüfe Nachbarn
	for (int x = -1; x < 2; x++)
	{
		for (int y = -1; y < 2; y++)
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
void SPHSystemSimulator::X_ApplyBoundingBox(Particle & ball)
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

#pragma endregion

// TODO
// Initialisiere UI
void SPHSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

// TODO
// Setzte Simulation zurück
void SPHSystemSimulator::reset()
{
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
}

// Rendere Simulation
void SPHSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	// Farbe einrichten
	DUC->setUpLighting(Vec3(), 0.6f*Vec3(1.0f), 100.0f, Vec3(0,0,1));
	// Bälle rendern
	for (auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		DUC->drawSphere(ball->Position + m_v3BoxPos + m_v3Shifting, Vec3(ball->Radius));
	}
}

// Aktiviere verschiedene Simulationen
void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	// Setzte Simulation zurück
	reset();
	// Erstelle Szene
	X_SetupDemo();
}

// TODO
// Externe Kräfte updaten
void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	Vec3 mouseForce(0.0f);
	// Berechne Differenz
	mouseDiff.x = m_v2Trackmouse.x - m_v2Oldtrackmouse.x;
	mouseDiff.y = m_v2Trackmouse.y - m_v2Oldtrackmouse.y;
	// Falls linke Maustaste gedrückt
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		// Berechne benötigte Matrix
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix()).inverse();
		// Vektor bestehend aus Mausverschiebung
		Vec3 inputView = Vec3((float)-mouseDiff.x, (float)mouseDiff.y, 0);
		// Bestimme Kraft im Worldspace mit Faktor
		mouseForce = worldViewInv.transformVectorNormal(inputView) * -0.001f;
	}
	// Kraft anwenden
	for (auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		ball->Force += Vec3(0, -1.0f * m_fGravity * ball->Mass, 0);
		ball->Force += mouseForce;
		ball->Force -= ball->Velocity;

		ball->ForceTilde += Vec3(0, -1.0f * m_fGravity * ball->Mass, 0);
		ball->ForceTilde += mouseForce;
	}
}

// TODO
// Simulation updaten
void SPHSystemSimulator::simulateTimestep(float timeStep)
{
	//Berechnen aller Positionen nach h/2 Schritt
	for (auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		ball->PositionTilde = ball->Position + ball->Velocity * timeStep / 2.0f;
	}

	// TODO
	// Grid updaten
	for (int i = 0; i < m_GridOccupation.size(); i++)
	{
		m_GridOccupation[i] = 0;
	}
	// Sortiere Bälle in Zellen, speichere belegte Zellen
	vector<int> toCheck = X_SortBalls();
	// Für alle belegten Zellen
	for (int i = 0; i < toCheck.size(); i++)
	{
		// Bestimme Nachbarindizes
		vector<int> neighbors = X_CheckNeighbors(toCheck[i]);
		// Für alle Bälle in der Zelle
		for (int k = 0; k < m_GridOccupation[toCheck[i]]; k++)
		{
			// Für alle Nachbarn
			for (int j = 0; j < neighbors.size(); j++)
			{
				for (int l = 0; l < m_GridOccupation[neighbors[j]]; l++)
				{
					int own = (toCheck[i] * MAXCOUNT) + k;
					int neighbor = (neighbors[j] * MAXCOUNT) + l;
					// Überspringe Interaktion mit selbst
					if (own == neighbor)
						continue;
					// TODO
				}
			}
		}
	}

	// Aktualisiere Geschwindigkeit/Position
	for (auto ball = m_Balls.begin(); ball != m_Balls.end(); ball++)
	{
		Vec3 midPointVelocity = ball->Velocity + (ball->Force / ball->Mass) * timeStep / 2.0f;
		ball->Position += timeStep * midPointVelocity;
		ball->ForceTilde -=  midPointVelocity;
		ball->Velocity += timeStep * (ball->ForceTilde / ball->Mass);
		// Kraft zurücksetzen
		ball->Force = Vec3(0.0f);
		ball->ForceTilde = Vec3(0.0f);
		// Als letztes Position clampen
		X_ApplyBoundingBox(*ball);
	}
}

#pragma endregion

#pragma region Initialisation

// TODO
SPHSystemSimulator::SPHSystemSimulator()
{
}

// TODO?
SPHSystemSimulator::~SPHSystemSimulator()
{
	if (m_GridAccelerator.size() > 0)
	{
		m_GridAccelerator.clear();
		m_GridOccupation.clear();
	}
	if (m_Balls.size() > 0)
		m_Balls.clear();
}

#pragma endregion
