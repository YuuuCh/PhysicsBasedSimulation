#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_fGravity = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Simple one-step,Single body,Two body,Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.1 min=0");
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Leap Forg,Midpoint");
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
		rigidbodys.emplace_back(make_unique<Box>(2, Vec3(),
			Quat(Vec3(0, 0, 1), M_PI_2), Vec3(), Vec3(), Vec3(1, 0.6, 0.5)));
		break;
	case 1:
		cout << "Single!\n";
		rigidbodys.emplace_back(make_unique<Box>(2, Vec3(),
			Quat(Vec3(0, 0, 1), M_PI_2), Vec3(), Vec3(), Vec3(1, 0.6, 0.5)));
		break;
	case 2:
		cout << "Two!\n";
		break;
	case 3:
		cout << "Complex!\n";
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		// m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		// m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	for (auto& body : rigidbodys) {
		body->v += body->f / body->mass * timeStep;
		body->x += body->v * timeStep;
		Mat4 R = body->q.getRotMat();
		Mat4 I = body->Ibody * R;
		R.transpose();
		I = R * I;
		body->omega += I.inverse() * body->torque * timeStep;
		Vec3 tmp = body->omega * timeStep / 2;
		body->q += Quat(tmp.x, tmp.y, tmp.z, 0) * body->q;
		body->q = body->q.unit();
	}
	if (m_iTestCase == 0) {
		for (auto& body : rigidbodys) {
			std::cout << body->v << std::endl;
			std::cout << body->omega << std::endl;
		}
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
	body->f += force;
	body->torque += cross(loc - body->x, force);
}