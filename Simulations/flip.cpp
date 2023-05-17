#include "flip.h"

FlipSimulator::FlipSimulator() {
	m_fRatio = 0.;
	m_iTestCase = 0;
}

const char* FlipSimulator::getTestCasesStr() {
	return "PIC,FLIP95,Ball";
}

void FlipSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "FLIP/PIC ratio", TW_TYPE_FLOAT, &m_fRatio, "step=0.05 min=0");
	//TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Leap Forg,Midpoint");
	switch (m_iTestCase) {
	case 0:break;
	case 1:break;
	case 2:break;
	default:break;
	}
}

void FlipSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void FlipSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	for (int i = 0; i < m_iNumSpheres; ++i) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, m_particleColor[i]);
		DUC->drawSphere(m_particlePos[i], Vec3(m_particleRadius));
	}
	if (m_iTestCase == 2) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(1.0, 0.0, 0.0));
		DUC->drawSphere(m_obstaclePos, Vec3(0.1));
	}
}

void FlipSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	switch (m_iTestCase) {
	case 0:
		cout << "PIC!\n";
		m_fRatio = 0.f;
		setupScene(32);
		break;
	case 1:
		cout << "FLIP95!\n";
		m_fRatio = 0.95f;
		setupScene(32);
		break;
	case 2:
		cout << "Interaction!\n";
		setupScene(32);
		setObstacle(Vec3(0.8, 0.6, 0.8));
		break;
	case 3:
		cout << "PCG!\n";

		break;
	default:
		cout << "Empty Test!\n";
		assert(false);
	}
}

void FlipSimulator::externalForcesCalculations(float timeElapsed) {
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
		m_obstaclePos = m_obstacleFinalPos + inputWorld;
	}
	else {
		m_obstacleFinalPos = m_obstaclePos;
	}
	setObstacle(m_obstaclePos);
}

void FlipSimulator::simulateTimestep(float timeStep) {
	int numSubSteps = 1;
	int numParticleIters = 2;
	int numPressureIters = 30;
	bool separateParticles = false;
	float overRelaxation = 1.9;
	bool compensateDrift = true;

	float flipRatio = m_fRatio; 
	Vec3 obstaclePos(m_obstaclePos);
	Vec3 obstacleVel((m_obstaclePos - m_obstacleFinalPos) / timeStep);

	float sdt = timeStep / numSubSteps;

	for (int step = 0; step < numSubSteps; step++) {
		integrateParticles(sdt);
		if (separateParticles)
			pushParticlesApart(numParticleIters);
		handleParticleCollisions(obstaclePos, 0.1, obstacleVel);
		transferVelocities(true, flipRatio);
		updateParticleDensity();
		solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
		transferVelocities(false, flipRatio);
	}
	updateParticleColors();
}

void FlipSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FlipSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FlipSimulator::setupScene(int res) {
	// an example to set up a breaking dam scene
	float tankHeight = 1.0;
	float tankWidth = 1.0;
	float tankDepth = 1.0;

	float _h = tankHeight / res;
	float point_r = 0.3 * _h;	// particle radius w.r.t. cell size

	float relWaterHeight = 0.8;
	float relWaterWidth = 0.6;
	float relWaterDepth = 0.6;

	// dam break
	// compute number of particles	
	float dx = 2.0 * point_r;
	float dy = sqrt(3.0) / 2.0 * dx;
	float dz = dx;

	int numX = floor((relWaterWidth * tankWidth - 2.0 * _h - 2.0 * point_r) / dx);
	int numY = floor((relWaterHeight * tankHeight - 2.0 * _h - 2.0 * point_r) / dy);
	int numZ = floor((relWaterDepth * tankDepth - 2.0 * _h - 2.0 * point_r) / dz);

	// update object member attributes
	m_iNumSpheres = numX * numY * numZ;
	m_iCellX = res + 1;
	m_iCellY = res + 1;
	m_iCellZ = res + 1;
	m_h = 1.0 / float(res);
	m_fInvSpacing = float(res);
	m_iNumCells = m_iCellX * m_iCellY * m_iCellZ;
	m_particleRadius = 0.3 * point_r;

	// update particle array
	m_particlePos.clear(); m_particlePos.resize(m_iNumSpheres, Vec3(0.0f));
	m_particleColor.clear(); m_particleColor.resize(m_iNumSpheres, Vec3(1.0f));
	m_particleVel.clear(); m_particleVel.resize(m_iNumSpheres, Vec3(0.0f));

	// update grid array
	m_vel.clear(); m_vel.resize(m_iNumCells, Vec3(0.0f));
	m_pre_vel.clear(); m_pre_vel.resize(m_iNumCells, Vec3(0.0f));
	m_p.clear();  m_p.resize(m_iNumCells, 0.0);
	m_s.clear(); m_s.resize(m_iNumCells, 0.0);
	m_type.clear(); m_type.resize(m_iNumCells, 0);
	m_particleDensity.clear(); m_particleDensity.resize(m_iNumCells, 0.0f);

	// the rest density can be assigned after scene initialization
	m_particleRestDensity = 0.0;

	// create particles
	int p = 0;
	for (int i = 0; i < numX; i++) {
		for (int j = 0; j < numY; j++) {
			for (int k = 0; k < numZ; k++) {
				m_particlePos[p++] = Vec3(m_h + point_r + dx * i + (j % 2 == 0 ? 0.0 : point_r), m_h + point_r + dy * j, m_h + point_r + dz * k + (j % 2 == 0 ? 0.0 : point_r)) /* + Vec3(-0.5f)*/;
			}
		}
	}
	// setup grid cells for tank
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;

	for (int i = 0; i < m_iCellX; i++) {
		for (int j = 0; j < m_iCellY; j++) {
			for (int k = 0; k < m_iCellZ; k++) {
				float s = 1.0;	// fluid
				if (i == 0 || i == m_iCellX - 1 || j == 0 || k == 0 || k == m_iCellZ - 1)
					s = 0.0f;	// solid
				m_s[i * n + j * m + k] = s;
			}
		}
	}
}

void FlipSimulator::setObstacle(Vec3 pos) {
	m_obstaclePos = pos;
	Vec3 vel(0.f);
	float r = 0.1f;
	for (int i = 1; i < m_iCellX - 1; ++i) {
		for (int j = 1; j < m_iCellY - 1; ++j) {
			for (int k = 1; k < m_iCellZ - 1; ++k) {
				float dx = (i + 0.5) * m_h - pos[0];
				float dy = (j + 0.5) * m_h - pos[1];
				float dz = (k + 0.5) * m_h - pos[2];
				int cell = i * m_iCellY * m_iCellZ + j * m_iCellZ + k;
				if (dx * dx + dy * dy + dz * dz < r * r) {
					m_s[cell] = 0.f;
					m_vel[cell] = vel;
					m_vel[(i + 1) * m_iCellY * m_iCellZ + j * m_iCellZ + k][0] = vel[0];
					m_vel[i * m_iCellY * m_iCellZ + (j + 1) * m_iCellZ + k][1] = vel[1];
					m_vel[i * m_iCellY * m_iCellZ + j * m_iCellZ + k + 1][2] = vel[2];
				}
			}
		}
	}
}

void FlipSimulator::integrateParticles(float timeStep) {
	for (int i = 0; i < m_iNumSpheres; ++i) {
		m_particleVel[i] += -Vec3(0.f, 9.8f, 0.f) * timeStep;
		m_particlePos[i] += m_particleVel[i] * timeStep;
	}
}

void FlipSimulator::pushParticlesApart(int numIters) {
	for (int iter = 0; iter < numIters; ++iter) {
		for (int j = 0; j < m_iNumSpheres; ++j) {
			for (int k = j + 1; k < m_iNumSpheres; ++k) {
				Vec3 d = m_particlePos[j] - m_particlePos[k];
				float dist = norm(d);
				if (dist < 2 * m_particleRadius && dist != 0.f) {
					Vec3 s = 0.5 * (2 * m_particleRadius - dist) * d / dist;
					m_particlePos[j] += s;
					m_particlePos[k] -= s;
				}
			}
		}
	}
}

void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel) {
	float min = m_h + m_particleRadius;
	float max = (m_iCellX - 1) * m_h - m_particleRadius;
	for (int i = 0; i < m_iNumSpheres; ++i) {
		if (m_iTestCase == 2) {
			Vec3 d = m_particlePos[i] - obstaclePos;
			if (norm(d) < obstacleRadius + m_particleRadius) {
				m_particleVel[i] = obstacleVel;
			}
		}

		for (int j = 0; j < 3; ++j) {
			if (m_particlePos[i][j] < min) {
				m_particlePos[i][j] = min;
				m_particleVel[i][j] = 0.f;
			}
			if (m_particlePos[i][j] > max) {
				m_particlePos[i][j] = max;
				m_particleVel[i][j] = 0.f;
			}
		}
	}
}

void FlipSimulator::updateParticleDensity() {
	m_particleDensity.clear(); m_particleDensity.resize(m_iNumCells, 0.0f);
	for (int i = 0; i < m_iNumSpheres; ++i) {
		int x = std::floor(m_particlePos[i][0] * m_fInvSpacing);
		int y = std::floor(m_particlePos[i][1] * m_fInvSpacing);
		int z = std::floor(m_particlePos[i][2] * m_fInvSpacing);
		assert(x >= 0 && x < m_iCellX&& y >= 0 && y < m_iCellY&& z >= 0 && z < m_iCellZ);
		float tx = m_particlePos[i][0] * m_fInvSpacing - x;
		float ty = m_particlePos[i][1] * m_fInvSpacing - y;
		float tz = m_particlePos[i][2] * m_fInvSpacing - z;
		float w[8] = { (1 - tx) * (1 - ty) * (1 - tz),
							tx * (1 - ty) * (1 - tz),
							(1 - tx) * ty * (1 - tz),
							tx * ty * (1 - tz),
							(1 - tx) * (1 - ty) * tz,
							tx * (1 - ty) * tz,
							(1 - tx) * ty * tz,
							tx * ty * tz };
		int c[8] = { x * m_iCellY * m_iCellZ + y * m_iCellZ + z,
					(x + 1) * m_iCellY * m_iCellZ + y * m_iCellZ + z,
					x * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z,
					(x + 1) * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z,
					x * m_iCellY * m_iCellZ + y * m_iCellZ + z + 1,
					(x + 1) * m_iCellY * m_iCellZ + y * m_iCellZ + z + 1,
					x * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z + 1,
					(x + 1) * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z + 1 };
		for (int j = 0; j < 8; ++j) {
			m_particleDensity[c[j]] += w[j];
		}
	}
	if (m_particleRestDensity == 0.f) {
		float sum = 0;
		int numFluidcells = 0;
		for (int i = 0; i < m_iNumCells; ++i) {
			if (m_type[i] == FLUID_CELL) {
				sum += m_particleDensity[i];
				numFluidcells++;
			}
		}
		if (numFluidcells > 0)
			m_particleRestDensity = sum / numFluidcells;
	}
}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio) {
	if (toGrid) {
		m_pre_vel = std::move(m_vel);
		m_vel.resize(m_iNumCells, Vec3(0.f));
		for (int i = 0; i < m_iNumCells; ++i) {
			m_type[i] = m_s[i] == 0. ? SOLID_CELL : EMPTY_CELL;
		}
		for (const auto& pos : m_particlePos) {
			int x = std::floor(pos[0] * m_fInvSpacing);
			int y = std::floor(pos[1] * m_fInvSpacing);
			int z = std::floor(pos[2] * m_fInvSpacing);
			assert(x >= 0 && x < m_iCellX&& y >= 0 && y < m_iCellY&& z >= 0 && z < m_iCellZ);
			int cell = x * m_iCellY * m_iCellZ + y * m_iCellZ + z;
			if (m_type.at(cell) == EMPTY_CELL)
				m_type[cell] = FLUID_CELL;
		}
	}
	for (int dim = 0; dim < 3; ++dim) {
		float dx = dim == 0 ? 0 : m_h / 2;
		float dy = dim == 1 ? 0 : m_h / 2;
		float dz = dim == 2 ? 0 : m_h / 2;
		vector<float> ws(m_iNumCells);
		for (int i = 0; i < m_iNumSpheres; ++i) {
			Vec3 new_pos = m_particlePos[i] - Vec3(dx, dy, dz);
			int x = std::floor(new_pos[0] * m_fInvSpacing);
			int y = std::floor(new_pos[1] * m_fInvSpacing);
			int z = std::floor(new_pos[2] * m_fInvSpacing);
			assert(x >= 0 && x < m_iCellX&& y >= 0 && y < m_iCellY&& z >= 0 && z < m_iCellZ);
			float tx = new_pos[0] * m_fInvSpacing - x;
			float ty = new_pos[1] * m_fInvSpacing - y;
			float tz = new_pos[2] * m_fInvSpacing - z;

			float w[8] = {  (1 - tx) * (1 - ty) * (1 - tz),
							tx * (1 - ty) * (1 - tz),
							(1 - tx) * ty * (1 - tz),
							tx * ty * (1 - tz),
							(1 - tx) * (1 - ty) * tz,
							tx * (1 - ty) * tz,
							(1 - tx) * ty * tz,
							tx * ty * tz };
			int c[8] = {x * m_iCellY * m_iCellZ + y * m_iCellZ + z,
						(x + 1) * m_iCellY * m_iCellZ + y * m_iCellZ + z,
						x * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z,
						(x + 1) * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z,
						x * m_iCellY * m_iCellZ + y * m_iCellZ + z + 1,
						(x + 1) * m_iCellY * m_iCellZ + y * m_iCellZ + z + 1,
						x * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z + 1,
						(x + 1) * m_iCellY * m_iCellZ + (y + 1) * m_iCellZ + z + 1 };
			if (toGrid) {
				for (int j = 0; j < 8; ++j) {
					m_vel.at(c[j])[dim] += m_particleVel[i][dim] * w[j];
					ws.at(c[j]) += w[j];
				}
			}
			else {
				int offset = 0;
				if (dim == 0)
					offset = m_iCellY * m_iCellZ;
				else if (dim == 1)
					offset = m_iCellZ;
				else
					offset = 1;
				float valid[8] = {};
				float weight_sum = 0.f;
				for (int j = 0; j < 8; ++j) {
					valid[j] = m_type[c[j]] != EMPTY_CELL || m_type.at(c[j] - offset) != EMPTY_CELL ? 1.f : 0.f;
					weight_sum += valid[j] * w[j];
				}
				if (weight_sum > 0.) {
					float picV = 0.f;
					float correct = 0.f;
					for (int j = 0; j < 8; ++j) {
						picV += valid[j] * w[j] * m_vel[c[j]][dim] / weight_sum;
						correct += valid[j] * w[j] * (m_vel[c[j]][dim] - m_pre_vel[c[j]][dim]) / weight_sum;
					}
					float flipV = m_particleVel[i][dim] + correct;
					m_particleVel[i][dim] = (1.f - flipRatio) * picV + flipRatio * flipV;
				}
			}
		}
		if (toGrid) {
			for (int j = 0; j < m_iNumCells; ++j) {
				if (ws.at(j) > 0.)
					m_vel.at(j)[dim] /= ws.at(j);
			}
		}
	}
	if (toGrid) {
		// restore solid cells
		for (int i = 0; i < m_iCellX; ++i) {
			for (int j = 0; j < m_iCellY; ++j) {
				for (int k = 0; k < m_iCellZ; ++k) {
					int cell = i * m_iCellY * m_iCellZ + j * m_iCellY + k;
					bool solid = m_type[cell] == SOLID_CELL;
					if (solid || (i > 0 && m_type.at((i - 1) * m_iCellY * m_iCellZ + j * m_iCellY + k) == SOLID_CELL))
						m_vel[cell][0] = m_pre_vel[cell][0];
					if (solid || (j > 0 && m_type.at(i * m_iCellY * m_iCellZ + (j - 1) * m_iCellY + k) == SOLID_CELL))
						m_vel[cell][1] = m_pre_vel[cell][1];
					if (solid || (k > 0 && m_type.at(i * m_iCellY * m_iCellZ + j * m_iCellY + k - 1) == SOLID_CELL))
						m_vel[cell][2] = m_pre_vel[cell][2];
				}
			}
		}
	}
}

void FlipSimulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
	m_p.clear();  m_p.resize(m_iNumCells, 0.0);
	m_pre_vel = m_vel;
	for (int iter = 0; iter < numIters; ++iter) {
		for (int i = 1; i < m_iCellX - 1; ++i) {
			for (int j = 1; j < m_iCellY - 1; ++j) {
				for (int k = 1; k < m_iCellZ - 1; ++k) {
					int cell = i * m_iCellY * m_iCellZ + j * m_iCellZ + k;
					if (m_type[cell] != FLUID_CELL)
						continue;
					int left = (i - 1) * m_iCellY * m_iCellZ + j * m_iCellZ + k;
					int right = (i + 1) * m_iCellY * m_iCellZ + j * m_iCellZ + k;
					int bottom = i * m_iCellY * m_iCellZ + (j - 1) * m_iCellZ + k;
					int up = i * m_iCellY * m_iCellZ + (j + 1) * m_iCellZ + k;
					int front = cell - 1;
					int back = cell + 1;
					float sx0 = m_s[left];
					float sx1 = m_s[right];
					float sy0 = m_s[bottom];
					float sy1 = m_s[up];
					float sz0 = m_s[front];
					float sz1 = m_s[back];
					float s = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
					if (s == 0.f)
						continue;
					float d = m_vel[right][0] - m_vel[cell][0] + m_vel[up][1] - m_vel[cell][1] + m_vel[back][2] - m_vel[cell][2];
					d *= overRelaxation;
					if (m_particleRestDensity > 0.f && compensateDrift) {
						float compression = m_particleDensity[cell] - m_particleRestDensity;
						if (compression > 0.f)
							d -= compression;
					}
					m_p[cell] -= d / s * 1000.f * m_h / dt;
					m_vel[cell][0] += d * sx0 / s;
					m_vel[right][0] -= d * sx1 / s;
					m_vel[cell][1] += d * sy0 / s;
					m_vel[up][1] -= d * sy1 / s;
					m_vel[cell][2] += d * sz0 / s;
					m_vel[back][2] -= d * sz1 / s;
				}
			}
		}
	}
}

void FlipSimulator::updateParticleColors() {
	for (int i = 0; i < m_iNumSpheres; ++i) {
		float s = 0.01;
		Vec3 new_color = m_particleColor[i] + Vec3(-s, -s, s);
		for (int j = 0; j < 3; ++j) {
			if (new_color[j] < 0.f)
				new_color[j] = 0.f;
			if (new_color[j] > 1.f)
				new_color[j] = 1.f;
		}
		int x = std::floor(m_particlePos[i][0] * m_fInvSpacing);
		int y = std::floor(m_particlePos[i][1] * m_fInvSpacing);
		int z = std::floor(m_particlePos[i][2] * m_fInvSpacing);
		assert(x >= 0 && x < m_iCellX&& y >= 0 && y < m_iCellY&& z >= 0 && z < m_iCellZ);
		int cell = x * m_iCellY * m_iCellZ + y * m_iCellZ + z;
		if (m_particleRestDensity > 0.f) {
			float reldens = m_particleDensity[cell] / m_particleRestDensity;
			if (reldens < 0.7) {
				new_color = Vec3(0.8f, 0.8f, 1.0f);
			}
		}
		m_particleColor[i] = new_color;
	}
}