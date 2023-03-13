#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
	setMass(10);
	setStiffness(40);
	setDampingFactor(0);
	setIntegrator(0);
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Single Spring,Complex";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Leap Forg,Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	DUC->beginLine();
	for (const auto& spring : springs) {
		DUC->drawLine(getPositionOfMassPoint(spring.point1), 
			0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)), 
			getPositionOfMassPoint(spring.point2), 
			0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase){
	case 0:
		cout << "Single Spring!\n";
		points.clear();
		springs.clear();
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		break;
	case 1:
		cout << "Complex Springs!\n";
		points.clear();
		springs.clear();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
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

void MassSpringSystemSimulator::advanceEuler(float timeStep) {
	applyExternalForce(Vec3());
	for (const auto& spring : springs) {
		int p1 = spring.point1;
		int p2 = spring.point2;
		Vec3 x12 = getPositionOfMassPoint(p1) - getPositionOfMassPoint(p2);
		Vec3 force = m_fStiffness * (norm(x12) - spring.restlength) * getNormalized(x12);
		points[p1].force += force;
		points[p2].force -= force;
	}
	for (auto& point : points) {
		if (!point.isFixed) {
			point.position += point.velocity * timeStep;
			point.velocity += point.force * timeStep / m_fMass;
		}
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	switch (m_iIntegrator){
	case EULER:
		advanceEuler(timeStep);
		break;
	case LEAPFROG:
		break;
	case MIDPOINT:
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	int cnt = getNumberOfMassPoints();
	points.emplace_back(position, Velocity, isFixed);
	return cnt;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	springs.emplace_back(masspoint1, masspoint2, initialLength);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return static_cast<int>(points.size());
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return static_cast<int>(springs.size());
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return points.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return points.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	for (auto& point : points) {
		point.force = force;
	}
}