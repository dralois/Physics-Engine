#include "pch.h"

#include "RigidBodySystemSimulator.h"

#pragma region Properties

// Demos für Antweakbar
const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

// Get Anzahl
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_Rigidbodies.size();
}

// Get Position
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	Vec3 l_v3Position, l_v3Scale, l_v3Rotation, l_v3Shear;
	m_Rigidbodies[i].Translation.decompose(l_v3Position, l_v3Scale, l_v3Rotation, l_v3Shear);
	return l_v3Position;
}

// Get Geschwindigkeit
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_Rigidbodies[i].LinVel;
}

// Get Winkelgeschwindigkeit
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_Rigidbodies[i].AngVel;
}

// Set Rotation
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_Rigidbodies[i].Rotation = orientation.unit().getRotMat();
}

// Set Geschwindigkeit
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_Rigidbodies[i].LinVel = velocity;
}

#pragma endregion

#pragma region Events

// Mausbewegung während Klicken
void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_v2Trackmouse.x = x;
	m_v2Trackmouse.y = y;
}

// Mausbewegung normal
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
	case 0:				// Demo 1
		addRigidBody(Vec3(0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setOrientationOf(0, Quat::Quaternion(Vec3(0,0,1), (float)(M_PI)* 0.5f));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	case 1:				// Demo 2
		addRigidBody(Vec3(0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setOrientationOf(0, Quat::Quaternion(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)* 0.5f));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	case 2:				// Demo 3
		addRigidBody(Vec3(0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		addRigidBody(Vec3(0.75f, 0.0f, 1.0f), Vec3(1.0f, 0.6f, 0.5f), 1.0f);
		applyForceOnBody(0, Vec3(0.0f), Vec3(0.0f, 0.0f, 5.0f));
		applyForceOnBody(1, Vec3(0.75f, 0.0f, 1.0f), Vec3(0.0f, 0.0f, -5.0f));
		break;
	case 3:				// Demo 4
		addRigidBody(Vec3(0.0f, 0.0f, 0.5f), Vec3(0.8f, 0.2f, 0.5f), 1.0f);
		setOrientationOf(0, Quat::Quaternion(Vec3(1.0f, 0.0f, 1.0f), (float)(M_PI)* 0.4f));
		addRigidBody(Vec3(-0.4f, 0.0f, 0.8f), Vec3(1.2f, 0.1f, 0.25f), 1.0f);
		setOrientationOf(1, Quat::Quaternion(Vec3(1.0f, -1.0f, 0.0f), (float)(M_PI)* 0.2f));
		addRigidBody(Vec3(0.8f, 0.0f, -0.8f), Vec3(1.3f, 0.9f, 0.5f), 1.0f);
		setOrientationOf(2, Quat::Quaternion(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI)* 0.7f));
		addRigidBody(Vec3(-0.4f, 0.0f, -0.8f), Vec3(1.0f, 0.5f, 0.25f), 1.0f);
		setOrientationOf(3, Quat::Quaternion(Vec3(1.0f, 1.0f, 1.0f), (float)(M_PI)* 0.4f));
		break;
	default:
		break;
	}
}

// Berechnet Inertia Tensor für einen Rigidbody
void RigidBodySystemSimulator::X_CalculateInertiaTensor(Rigidbody & rb)
{
	Mat4 inertia(0);
	// x = width, y = height, z = depth
	Vec3 trans, scale, rot, shear;
	rb.Scale.decompose(trans, scale, rot, shear);
	// 1/12 * m * (h^2 + d^2)
	inertia.value[0][0] = 1.0f / 12.0f * rb.Mass * (powf(scale.y,2) + powf(scale.z,2));
	// 1/12 * m * (w^2 + d^2)
	inertia.value[1][1] = 1.0f / 12.0f * rb.Mass * (powf(scale.x, 2) + powf(scale.z, 2));
	// 1/12 * m * (w^2 + h^2)
	inertia.value[2][2] = 1.0f / 12.0f * rb.Mass * (powf(scale.x, 2) + powf(scale.y, 2));
	// Wird benötigt damit man damit rechnen kann
	inertia.value[3][3] = 1.0f;
	// Speichere Inverses
	rb.InertiaTensorInv = inertia.inverse();
}

// Berechnet Impuls
void RigidBodySystemSimulator::X_CalculateImpulse(Rigidbody & rb_A, Rigidbody & rb_B, CollisionInfo & coll)
{
	// Velocity und local Position von A
	Vec3 transA, scaleA, rotA, shearA;
	rb_A.Translation.decompose(transA, scaleA, rotA, shearA);
	Vec3 xa = coll.collisionPointWorld - transA;
	Vec3 va = rb_A.LinVel + cross(rb_A.AngVel, xa);

	// Velocity und local Position von B
	Vec3 transB, scaleB, rotB, shearB;
	rb_B.Translation.decompose(transB, scaleB, rotB, shearB);
	Vec3 xb = coll.collisionPointWorld - transB;
	Vec3 vb = rb_B.LinVel + cross(rb_B.AngVel, xb);

	// Relative Geschwindigkeit
	Vec3 rVel = va - vb;

	// Bei Separierung abbrechen
	if (dot(rVel, coll.normalWorld) > 0)
		return;

	// (Ia.inverse * (Xa cross n)) cross Xa
	Mat4 rotTranspA = rb_A.Rotation;
	rotTranspA.transpose();
	Mat4 inertiaTensorInvA = rotTranspA * rb_A.InertiaTensorInv * rb_A.Rotation;
	Vec3 helpA = cross(inertiaTensorInvA.transformVectorNormal(cross(xa, coll.normalWorld)), xa);

	// (Ib.inverse * (Xb cross n)) cross Xb
	Mat4 rotTranspB = rb_B.Rotation;
	rotTranspB.transpose();
	Mat4 inertiaTensorInvB = rotTranspB * rb_B.InertiaTensorInv * rb_B.Rotation;
	Vec3 helpB = cross(inertiaTensorInvB.transformVectorNormal(cross(xb, coll.normalWorld)), xb);

	// Impuls Formel
	Vec3 J = (-1.0f * (1.0f + m_fCollisionCoefficient) * rVel * coll.normalWorld) /
		((1.0f / rb_A.Mass) + (1.0f / rb_B.Mass) + ((helpA + helpB) * coll.normalWorld));

	// Anwenden
	rb_A.LinVel += J * coll.normalWorld / rb_A.Mass;
	rb_B.LinVel -= J * coll.normalWorld / rb_B.Mass;
	rb_A.AngMom -= cross(xa, J * coll.normalWorld);
	rb_B.AngMom += cross(xb, J * coll.normalWorld);
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
		TwAddVarRW(DUC->g_pTweakBar, "Collision C", TW_TYPE_FLOAT, &m_fCollisionCoefficient, "min=0.00 max=1.00 step=0.01");
		break;
	default:
		break;
	}
}

// Setzte Simulation zurück
void RigidBodySystemSimulator::reset()
{
	m_v2Oldtrackmouse.x = m_v2Oldtrackmouse.y = 0;
	m_v2Trackmouse.x = m_v2Trackmouse.y = 0;
	m_v3ExternalForce = Vec3(0.0f);
	m_fCollisionCoefficient = 0.8f;
	m_HasPrinted = false;
	m_Rigidbodies.clear();
}

// Rendere Simulation
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
		for(auto rigidbody = m_Rigidbodies.begin(); rigidbody != m_Rigidbodies.end(); rigidbody++)
		{
			Mat4 obj2World = rigidbody->Scale * rigidbody->Rotation *
				rigidbody->Translation * DUC->g_camera.GetWorldMatrix();
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
	// Setzte Simulation zurück
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
	case 3:
	{
		cout << "Demo 4!" << endl;
		break;
	}
	default:
		cout << "Empty Demo!" << endl;
		break;
	}
}

// Externe Kräfte berechnen
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Bei Demo 1 und 3 keine Interaktion
	if (m_iTestCase == 0 || m_iTestCase == 2)
		return;

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
		mouseForce = worldViewInv.transformVectorNormal(inputView) * -0.01f;
	}

	// Wende Mausinteraktion an
	for (auto rb = m_Rigidbodies.begin(); rb != m_Rigidbodies.end(); rb++)
	{
		// Zufällige X/Y/Z Werte bestimmen -> Zufällige Ecke
		float ranX = rand() > RAND_MAX / 2 ? 1.0f : -1.0f;
		float ranY = rand() > RAND_MAX / 2 ? 1.0f : -1.0f;
		float ranZ = rand() > RAND_MAX / 2 ? 1.0f : -1.0f;
		Vec3 trans, scale, rot, shear;
		rb->Scale.decompose(trans, scale, rot, shear);
		Vec3 pos = Vec3(scale.x * ranX, scale.y * ranY, scale.z * ranZ);
		rb->Translation.decompose(trans, scale, rot, shear);
		// Kraft anwenden
		X_ApplyForceOnBody(*rb, pos + trans, mouseForce * 1.0f / rb->Mass);
	}
}

// Simuliert Positions-, Rotations- usw. Update
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	// Bei Testcase 1 abbrechen nach dem Print
	if (m_HasPrinted)
		return;

	// Timestep überschreiben für Testcase 1
	if(m_iTestCase == 0)
		timeStep = 2.0f;

	for (auto rb = m_Rigidbodies.begin(); rb != m_Rigidbodies.end(); rb++)
	{
		// Kollisionscheck
		for (auto collider = m_Rigidbodies.begin(); collider != m_Rigidbodies.end(); collider++)
		{
			// Kann nicht mit sich selbst kollidieren
			if (collider == rb)
				continue;

			// Erstelle Matrizen
			Mat4 obj2World_A = rb->Scale * rb->Rotation * rb->Translation;
			Mat4 obj2World_B = collider->Scale * collider->Rotation * collider->Translation;

			// Kollision simulieren
			CollisionInfo check = checkCollisionSAT(obj2World_A, obj2World_B);

			// Falls gültig
			if (check.isValid)
			{
				X_CalculateImpulse(*rb, *collider, check);
			}
		}

		// Geschwindigkeit- und Positionsupdate
		Mat4 transMat = rb->Translation;
		Vec3 trans, scale, rot, shear;
		transMat.decompose(trans, scale, rot, shear);
		trans += timeStep * rb->LinVel;
		transMat.initTranslation(trans.x, trans.y, trans.z);
		rb->Translation = transMat;
		rb->LinVel += timeStep * (rb->Force / rb->Mass);

		// Rotationsmatrix zu Quaternion
		Quat oldRot = Quat(rb->Rotation).unit();

		// Euler Step: Neue Rotation mit alter Winkelgeschwindigkeit berechnen
		Quat newRot = oldRot + (Quat(rb->AngVel.x, rb->AngVel.y, rb->AngVel.z, 0.0f) * oldRot) * (timeStep / 2.0f);

		// Rotation aktualisieren
		rb->Rotation = newRot.unit().getRotMat();
		// Transponieren weil Lefthanded (!!)
		rb->Rotation.transpose();

		// Drehimpuls updaten
		rb->AngMom += timeStep * rb->Torque;

		// InverseMoment updaten
		Mat4 rotTransp = rb->Rotation;
		rotTransp.transpose();
		Mat4 inertiaTensorInv = rotTransp * rb->InertiaTensorInv * rb->Rotation;

		// Winkelgeschwindigkeit aktualisieren
		rb->AngVel = inertiaTensorInv * rb->AngMom;

		// Ergebnis printen
		if(m_iTestCase == 0 && !m_HasPrinted)
		{
			m_HasPrinted = true;
			cout << "Lin Vel: " << rb->LinVel << " Ang Vel: " << rb->AngVel << endl;
			cout << "Point (0.3/0.5/0.25): " << rb->LinVel + cross(rb->AngVel, Vec3(0.3, 0.5, 0.25)) << endl;
		}

		// Reset Torque und Force
		rb->Force = Vec3(0.0f);
		rb->Torque = Vec3(0.0f);
	}
}

// Wendet Kraft auf Rigidbody an
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	X_ApplyForceOnBody(m_Rigidbodies[i], loc, force);
}

// Wendet Kraft auf Rigidbody an
void RigidBodySystemSimulator::X_ApplyForceOnBody(Rigidbody & rb, Vec3 loc, Vec3 force)
{
	// Translation holen
	Vec3 trans, scale, rot, shear;
	rb.Translation.decompose(trans, scale, rot, shear);

	// Torque & Force aktualisieren
	rb.Torque += cross(loc - trans, force);
	rb.Force += force;
}

// Fügt neuen Rigidbody hinzu
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, float mass)
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
	toAdd.Force = Vec3(0.0f);
	toAdd.AngVel = Vec3(0.0f);
	toAdd.LinVel = Vec3(0.0f);
	toAdd.Torque = Vec3(0.0f);
	toAdd.AngMom = Vec3(0.0f);
	X_CalculateInertiaTensor(toAdd);

	// Zum Array hinzufügen
	m_Rigidbodies.push_back(toAdd);
}

#pragma endregion

#pragma region Initialisation

// Initialisiert neuen Simulator
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	srand(time(NULL));
	m_iTestCase = 0;
	reset();
}

#pragma endregion
