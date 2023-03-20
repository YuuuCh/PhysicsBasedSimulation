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
	constexpr int nodes = 11;
	constexpr float restlen = 0.2;
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
		setMass(1);
		setStiffness(100);
		for (int i = 0; i < nodes; ++i) {
			for (int j = 0; j < nodes; ++j) {
				addMassPoint(Vec3(i * restlen - 1, 1, j * restlen - 1), Vec3(), 
					(i == 0 && j == 0) || (i == 0 && j == nodes - 1) /*|| 
					(i == nodes - 1 && j == 0) || (i == nodes - 1 && j == nodes - 1)*/
				);
			}
		}
		for (int i = 0; i < nodes - 1; ++i) {
			for (int j = 0; j < nodes - 1; ++j) {
				addSpring(nodes * i + j, nodes * i + j + 1, restlen);
				addSpring(nodes * i + j, nodes * i + j + nodes, restlen);
				addSpring(nodes * i + j, nodes * i + j + nodes + 1, restlen * std::sqrt(2));
				addSpring(nodes * i + j + 1, nodes * i + j + nodes, restlen * std::sqrt(2));
			}
		}
		for (int j = 0; j < nodes - 1; ++j) {
			addSpring(nodes * (nodes - 1) + j, nodes * (nodes - 1) + j + 1, restlen);
		}
		for (int i = 0; i < nodes - 1; ++i) {
			addSpring(nodes * i + (nodes - 1), nodes * i + (nodes - 1) + nodes, restlen);
		}
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
	applyExternalForce();
	for (const auto& spring : springs) {
		int p1 = spring.point1;
		int p2 = spring.point2;
		Vec3 x12 = getPositionOfMassPoint(p1) - getPositionOfMassPoint(p2);
		Vec3 force = m_fStiffness * (norm(x12) - spring.restlength) * getNormalized(x12);
		points[p1].force -= force;
		points[p2].force += force;
	}
	for (auto& point : points) {
		if (!point.isFixed) {
			point.position += point.velocity * timeStep;
			point.velocity += point.force / m_fMass * timeStep;
		}
	}
}

void MassSpringSystemSimulator::advanceMidpoint(float timeStep) {
	applyExternalForce();
	std::vector<Vec3> tmppos;
	for (const auto& point : points) tmppos.emplace_back(point.position);
	for (const auto& spring : springs) {
		int p1 = spring.point1;
		int p2 = spring.point2;
		Vec3 x12 = getPositionOfMassPoint(p1) - getPositionOfMassPoint(p2);
		Vec3 force = m_fStiffness * (norm(x12) - spring.restlength) * getNormalized(x12);
		points[p1].force -= force;
		points[p2].force += force;
	}
	for (auto& point : points) {
		if (!point.isFixed) {
			point.position += point.velocity * timeStep / 2;
			//point.velocity += point.force * timeStep / m_fMass / 2;
		}
	}
	applyExternalForce();
	for (const auto& spring : springs) {
		int p1 = spring.point1;
		int p2 = spring.point2;
		Vec3 x12 = getPositionOfMassPoint(p1) - getPositionOfMassPoint(p2);
		Vec3 force = m_fStiffness * (norm(x12) - spring.restlength) * getNormalized(x12);
		points[p1].force -= force;
		points[p2].force += force;
	}
	for (int i = 0; i < points.size(); ++i) {
		if (!points[i].isFixed) {
			points[i].position = tmppos[i] + points[i].velocity * timeStep;
			points[i].velocity += points[i].force * timeStep / m_fMass;
		}
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	switch (m_iIntegrator) {
	case EULER:
		advanceEuler(timeStep);
		break;
	case LEAPFROG:
		break;
	case MIDPOINT:
		advanceMidpoint(timeStep);
		break;
	default:
		break;
	}
	// boundary condition
	for (auto& point : points) {
		if (point.position.y < -1) {
			point.position.y = -1;
		}
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

void MassSpringSystemSimulator::applyExternalForce() {
	switch (m_iTestCase) {
	case 0:
		for (auto& point : points) {
			point.force = Vec3();
		}
		break;
	case 1:
		for (auto& point : points) {
			point.force = Vec3(0,-0.3 * m_fMass,0);
		}
		break;
	default:
		assert(false);
	}
}