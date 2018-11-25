#include "RigidBodySystemSimulator.h"

#pragma region Properties

// Demos f¨¹r Antweakbar
const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

// Get Anzahl
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_Ridigbodies.size();
}

// Get Position
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	Vec3 l_v3Position, l_v3Scale, l_v3Rotation, l_v3Shear;
	m_Ridigbodies[i].Translation.decompose(l_v3Position, l_v3Scale, l_v3Rotation, l_v3Shear);
	return l_v3Position;
}

// Get Geschwindigkeit
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_Ridigbodies[i].LinVel;
}

// Get Winkelgeschwindigkeit
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_Ridigbodies[i].AngVel;
}

// Set Rotation
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_Ridigbodies[i].Rotation = orientation.getRotMat();
}

// Set Geschwindigkeit
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_Ridigbodies[i].LinVel = velocity;
}

#pragma endregion

#pragma region Events

// TODO
void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// TODO
void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_v2Oldtrackmouse.x = x;
	m_v2Oldtrackmouse.y = y;
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

#pragma endregion

#pragma region Functions

#pragma region Internal

// Demo Szenen Setup
void RigidBodySystemSimulator::X_SetupDemo(int demoNr)
{
	switch (demoNr)
	{
	case 0:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat::Quaternion(Vec3(0,0,1), (float)(M_PI)* 0.5f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	case 1:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		break;
	case 2:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		addRigidBody(Vec3(2.0f), Vec3(1, 0.6, 0.5), 2);
		break;
	case 3:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		addRigidBody(Vec3(1.0f), Vec3(1, 0.6, 0.5), 2);
		addRigidBody(Vec3(2.0f), Vec3(1, 0.6, 0.5), 2);
		addRigidBody(Vec3(3.0f), Vec3(1, 0.6, 0.5), 2);
		break;
	default:
		break;
	}
}

#pragma endregion

// Initialisiere UI je nach Demo
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		break;
	default:
		break;
	}
}

// Setzte Simulation zur¨¹ck
void RigidBodySystemSimulator::reset()
{
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	m_v3ExternalForce = Vec3(0.0f);
	m_Ridigbodies.clear();
}

// Rendere Simulation
// TODO: Selbst wenn man die Kamera dreht sieht man immer das Gleiche
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
		// Zeichnet alle Rigidbodies
		for(auto rigidbody = m_Ridigbodies.begin(); rigidbody != m_Ridigbodies.end(); rigidbody++)
		{
			Mat4 obj2World = rigidbody->Scale * rigidbody->Rotation * rigidbody->Translation;
			DUC->drawRigidBody(obj2World);
		}
		break;
	default:
		break;
	}
}

// Aktiviere verschiedene Simulationen
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Setzte Simulation zur¨¹ck
	reset();
	// Erstelle Demo Szene
	X_SetupDemo(m_iTestCase);
	// Je nach Case
	switch (m_iTestCase)
	{
	case 0:
	{
		cout << "Demo 1!" << endl;
		break;
	}
	case 1:
	{
		cout << "Demo 2!" << endl;
		break;
	}
	case 2:
	{
		cout << "Demo 3!" << endl;
		break;
	}
	default:
		cout << "Empty Demo!" << endl;
		break;
	}
}

// TODO
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Problem: an welcher Ecke soll die Mauskraft wirken?
}

// TODO
// Problem: Intertia Tensor ist eigentlich 3x3 Matrix, nicht 4x4
Mat4 X_calculateInertiaTensor(int i)
{
	return Mat4(0);
}

// TODO
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	// Da wir brauchen noch Index f¨¹r vector m_Torques, hier ist es effizienter zu iterieren mit Index statt Iterator
	for (int i = 0; i < m_Ridigbodies.size(); i++)
	{
		// Rotationsmatrix zu Quaternion
		// Problem: keine vordefinierte Methode f¨¹r Matrix zu Quaternion, nur f¨¹r Quaternion zu Matrix
		Quat oldRot = m_Ridigbodies[i].Rotation.getQuat();

		// Euler Step: neue Rotation mit altem Angularvelocity zu berechnen
		// Problem: Quaternion Multiplikation
		Quat newRot = oldRot + Quat(0, m_Ridigbodies[i].AngVel) * oldRot * timestep / 2.0f;

		// Rotation aktualisieren
		m_Ridigbodies[i].Rotation = newRot.getRotMat();

		// Inertia Tensor berechnen
		Mat4 inertia = X_calculateInertiaTensor(i);

		// Angular Momentum am Anfang
		Vec3 momentum = inertia.transformVector(m_Ridigbodies[i].AngVel);

		// Angular Momentum aktualisieren
		momentum += timestep * m_Torques[i];

		// Angular Velocity aktualisieren
		m_Ridigbodies[i].AngVel = inertia.inverse().transformVector(momentum);

	}
}

// TODO
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	/* Funktioniert nur unten folgenden zwei Bedingungen:

		1. Kraft/Kollision passiert genau an einer Ecke (insgesamt 8 Ecken f¨¹r eine Box)
		2. Es kann maximal gleichzeig nur eine Kraft/Kollision an diesem Objekt passieren
	*/

	Rigidbody collider = m_Ridigbodies[i];
	
	// die Kraftsposition in local space umrechnen
	// Problem: Mat4 ist 4x4 Matrix aber Vec3 ist nur eine 3d Vektor
	Vec3 locLocal = (collider.Rotation.inverse() * collider.Translation.inverse()).transformVector(loc);
	
	// Torque berechnen
	// Problem: keine vordefinierte Cross Product Methode in Vec3
	Vec3 torque = locLocal.operator * force;

	// Torque aktualisieren
	m_Torques[i] = torque;

}

// F¨¹gt neuen Rigidbody hinzu
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	// Matrizen aufbauen (beschreibt Ridigbody)
	Mat4 trans, scale, rot;
	trans.initTranslation(position.x, position.y, position.z);
	scale.initScaling(size.x, size.y, size.z);
	rot.initRotationXYZ(0, 0, 0);
	// Rigidbody aufbauen
	Rigidbody toAdd;
	toAdd.Translation = trans;
	toAdd.Rotation = rot;
	toAdd.Scale = scale;
	toAdd.Mass = mass;
	toAdd.AngVel = Vec3(0.0f);
	toAdd.LinVel = Vec3(0.0f);
	// Zum Array hinzuf¨¹gen
	m_Ridigbodies.push_back(toAdd);

	// Torque zu dem neuen Rigidbody hinzuf¨¹gen
	Vec3 torque = Vec3(.0f, .0f, .0f);
	m_Torques.push_back(torque);
}

#pragma endregion

#pragma region Initialisation

// Initialisiert neuen Simulator
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	reset();
}

#pragma endregion
