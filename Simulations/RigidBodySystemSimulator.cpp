#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_dGravity = 0;
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Simple one-step,Single body,Two body,Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_DOUBLE, &m_dGravity, "step=0.1 min=0");
	//TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Leap Forg,Midpoint");
	switch (m_iTestCase) {
	case 0:break;
	case 1:break;
	case 2:break;
	case 3:break;
	default:break;
	}
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	for (const auto& body : rigidbodys) {
		body->draw(DUC);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	rigidbodys.clear();
	switch (m_iTestCase) {
	case 0:
		cout << "Single one-step!\n";
		m_dGravity = 0.;
		rigidbodys.emplace_back(make_unique<Box>(2, Vec3(),
			Quat(Vec3(0, 0, 1), M_PI_2), Vec3(), Vec3(), Vec3(1, 0.6, 0.5)));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	case 1:
		cout << "Single!\n";
		m_dGravity = 0.;
		rigidbodys.emplace_back(make_unique<Box>(2, Vec3(),
			Quat(Vec3(0, 0, 1), M_PI_2), Vec3(), Vec3(), Vec3(1, 0.6, 0.5)));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(0.2, 0.3, 0.4));
		break;
	case 2:
		cout << "Two!\n";
		m_dGravity = 0.;
		addRigidBody(Vec3(-0.1, -0.2, 0.1), Vec3(0.4, 0.2, 0.2), 100.0);
		addRigidBody(Vec3(0.0, 0.2, 0.0), Vec3(0.4, 0.2, 0.2), 100.0);
		setOrientationOf(1, Quat(Vec3(0.0, 0.0, 1.0), M_PI_4));
		setVelocityOf(1, Vec3(0.0, -0.1, 0.05));
		break;
	case 3:
		cout << "Complex!\n";
		m_dGravity = 0.;
		addRigidBody(Vec3(0.0, 0.2, 0.0), Vec3(0.4, 0.2, 0.2), 100.0);
		setOrientationOf(0, Quat(Vec3(0.0, 0.0, 1.0), M_PI_4));
		setVelocityOf(0, Vec3(0.0, -0.1, 0.05));
		addRigidBody(Vec3(-0.1, -0.2, 0.1), Vec3(0.4, 0.2, 0.2), 100.0);
		setVelocityOf(1, Vec3(0.0, 0.1, 0.0));
		addRigidBody(Vec3(-0.5, 0., 0.), Vec3(0.3, 0.3, 0.3), 100.0);
		setVelocityOf(2, Vec3(0.1, 0., 0.0));
		addRigidBody(Vec3(0.4, 0., 0.), Vec3(0.2, 0.2, 0.2), 100.0);
		setVelocityOf(3, Vec3(-0.15, 0., 0.0));
		break;
	default:
		cout << "Empty Test!\n";
		assert(false);
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		for (auto& body : rigidbodys) {
			body->applyForce(inputWorld, 0.001 * inputWorld);
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	applyGravity();
	for (auto& body : rigidbodys) {
		if (body->is_fixed)continue;
		body->x += body->v * timeStep;
		body->v += body->f / body->mass * timeStep;
		Mat4 R = body->q.getRotMat();
		Mat4 I = body->Ibody * R;
		R.transpose();
		I = R * I;
		Vec3 tmp = body->omega * timeStep / 2;
		body->q += Quat(tmp.x, tmp.y, tmp.z, 0) * body->q;
		body->q = body->q.unit();
		body->omega += I.inverse() * body->torque * timeStep;
	}
	if (m_iTestCase == 0) {
		for (auto& body : rigidbodys) {
			std::cout << body->x << std::endl;
			std::cout << body->v << std::endl;
			std::cout << body->omega << std::endl;
		}
	}
	int n = getNumberOfRigidBodies();
	for (int i = 0; i < n; ++i) {
		for (int j = i + 1; j < n; ++j) {
			auto& body1 = rigidbodys[i];
			auto& body2 = rigidbodys[j];
			Mat4 M1 = body1->getWorldMat();
			Mat4 M2 = body2->getWorldMat();
			CollisionInfo collision = checkCollisionSAT(M1, M2);
			Vec3 n = collision.normalWorld;
			Vec3 loc = collision.collisionPointWorld;
			if (!collision.isValid) continue;
			Vec3 va = body1->velOnBody(loc);
			Vec3 vb = body2->velOnBody(loc);
			double vrel = dot(n, (va - vb));
			if (vrel >= 0)continue;
			constexpr double c = 0;
			double J = -(1 + c) * vrel;
			J /= 1. / body1->mass + 1. / body2->mass +
				dot(cross(body1->Ibody.inverse() * cross(loc - body1->x, n), loc - body1->x) +
					cross(body2->Ibody.inverse() * cross(loc - body2->x, n), loc - body2->x), n);
			body1->applyImpulse(J, n, loc);
			body2->applyImpulse(-J, n, loc);
		}
	}
	for (auto& body : rigidbodys) {
		body->f = Vec3();
		body->torque = Vec3();
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	// should have detect whether loc is inside the body
	auto& body = rigidbodys.at(i);
	body->applyForce(loc, force);
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return static_cast<int>(rigidbodys.size());
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return rigidbodys.at(i)->x;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return rigidbodys.at(i)->v;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return rigidbodys.at(i)->omega;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass, bool is_fixed) {
	rigidbodys.emplace_back(make_unique<Box>(mass, position,
		Quat(0, 0, 0, 1), Vec3(), Vec3(), size, is_fixed));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	rigidbodys.at(i)->q = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	rigidbodys.at(i)->v = velocity;
}

void RigidBodySystemSimulator::applyGravity() {
	for (auto& body : rigidbodys) {
		if (!body->is_fixed) {
			body->f += Vec3(0, -m_dGravity, 0) * body->mass;
		}
	}
}