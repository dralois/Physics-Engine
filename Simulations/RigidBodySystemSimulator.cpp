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
	case 0:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat::Quaternion(Vec3(0,0,1), (float)(M_PI)* 0.5f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	case 1:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat::Quaternion(Vec3(0, 0, 1), (float)(M_PI)* 0.5f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	case 2:
		addRigidBody(Vec3(0.0f), Vec3(1, 0.6, 0.5), 2);
		applyForceOnBody(0, Vec3(0.0f), Vec3(10, 10, 10));
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

// Berechnet Inertia Tensor für einen Rigidbody
void RigidBodySystemSimulator::X_CalculateInertiaTensor(Rigidbody & rb)
{
	Mat4 inertia(0);
	// x = width, y = height, z = depth
	Vec3 trans, scale, rot, shear;
	rb.Scale.decompose(trans, scale, rot, shear);
	// 1/12 * m * (h^2 + d^2)
	inertia.value[0][0] = 1.0f / 12 * rb.Mass * (powf(scale.y,2) + powf(scale.z,2));
	// 1/12 * m * (w^2 + d^2)
	inertia.value[1][1] = 1.0f / 12 * rb.Mass * (powf(scale.x, 2) + powf(scale.z, 2));
	// 1/12 * m * (w^2 + h^2)
	inertia.value[2][2] = 1.0f / 12 * rb.Mass * (powf(scale.x, 2) + powf(scale.y, 2));
	// vierte Dimension einfach auf 1 
	inertia.value[3][3] = 1.0f;
	// Speichere Inverses
	rb.InertiaTensorInv = inertia.inverse();
}

// Berechnet Impuls
void RigidBodySystemSimulator::X_CalculateImpulse(Rigidbody & rb_A, Rigidbody & rb_B, CollisionInfo & coll)
{
	// Relative Geschwindigkeit -> Stimmt das so? Ist Angular egal?
	Vec3 rVel = rb_A.LinVel - rb_B.LinVel;
	// Abbruch falls Separierung
	if (dot(rVel, coll.normalWorld) > 0)
		return;
	// Position der Kollision lokal in A und B
	Vec3 trans, scale, rot, shear;
	rb_A.Translation.decompose(trans, scale, rot, shear);
	Vec3 xA = coll.collisionPointWorld - trans;
	rb_B.Translation.decompose(trans, scale, rot, shear);
	Vec3 xB = coll.collisionPointWorld - trans;
	// Vorberechnung Winkelgeschwindigkeit
	Vec3 inertA = cross(rb_A.InertiaTensorInv * cross(xA, coll.normalWorld), xA);
	Vec3 inertB = cross(rb_B.InertiaTensorInv * cross(xB, coll.normalWorld), xB);
	// Impuls Formel
	Vec3 J = (-1.0f * (1.0f + m_fCollisionCoefficient) * rVel * coll.normalWorld) /
		((1.0f / rb_A.Mass) + (1.0f / rb_B.Mass) + (inertA + inertB) * coll.normalWorld);
	// Anwenden
	rb_A.LinVel += J * coll.normalWorld / rb_A.Mass;
	rb_B.LinVel -= J * coll.normalWorld / rb_B.Mass;
	rb_A.AngVel += cross(xA, J * coll.normalWorld);
	rb_B.AngVel -= cross(xB, J * coll.normalWorld);
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
	m_fCollisionCoefficient = 1.0f;
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
	default:
		cout << "Empty Demo!" << endl;
		break;
	}
}

// Tut momentan nichts
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	return;
}

// Simuliert Positions, Rotations usw. Update
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (auto rb = m_Ridigbodies.begin(); rb != m_Ridigbodies.end(); rb++)
	{
		// Kollisionscheck
		for (auto collider = m_Ridigbodies.begin(); collider != m_Ridigbodies.end(); collider++)
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

		// Geschwindigkeitsupdate und Positionsupdate
		Mat4 transMat = rb->Translation;
		Vec3 trans, scale, rot, shear;
		transMat.decompose(trans, scale, rot, shear);
		trans += timeStep * rb->LinVel;
		transMat.initTranslation(trans.x, trans.y, trans.z);
		rb->Translation = transMat;
		rb->LinVel += timeStep * (rb->Force / (rb->Mass * 1.0f));

		// Rotationsmatrix zu Quaternion
		Quat oldRot = Quat(rb->Rotation);

		// Euler Step: Neue Rotation mit alter Winkelgeschwindigkeit berechnen
		Quat newRot = oldRot + (Quat(rb->AngVel.x, rb->AngVel.y, rb->AngVel.z, 0.0f) * oldRot) * (timeStep / 2.0f);

		// Rotation aktualisieren
		rb->Rotation = newRot.getRotMat();

		// Drehimpuls updaten
		rb->AngMom += timeStep * rb->Torque;

		// Transponierte Rotationsmatrix
		Mat4 rotTransp = rb->Rotation;
		rotTransp.transpose();

		// InverseMoment updaten
		Mat4 inertiaTensorInv = rb->Rotation * rb->InertiaTensorInv * rotTransp;

		// Winkelgeschwindigkeit aktualisieren
		rb->AngVel = inertiaTensorInv * rb->AngMom;
		
		// Reset Torque und Force
		rb->Force = Vec3(0.0f);
		rb->Torque = Vec3(0.0f);
	}
}

// Wendet Kraft auf Rigidbody an
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	Rigidbody &collider = m_Ridigbodies[i];
	// World to Object Matrix
	Mat4 world2Obj = (collider.Rotation * collider.Translation).inverse();
	// Die Kraftsposition und Kraftrichtung in local space umrechnen

	// semi-local space
	Mat4 transMat = collider.Translation;
	Vec3 trans, scale, rot, shear;
	transMat.decompose(trans, scale, rot, shear);
	Vec3 v3PosLocal = loc - trans; // world2Obj.transformVector(loc);
	Vec3 v3ForceLocal = force; // world2Obj.transformVector(force);

	// Torque & Force aktualisieren
	collider.Torque += cross(v3PosLocal, v3ForceLocal);
	collider.Force += force;
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
	m_Ridigbodies.push_back(toAdd);
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
