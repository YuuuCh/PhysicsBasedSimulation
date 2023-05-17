#ifndef FLIP_H
#define FLIP_H
#include "Simulator.h"

#define SOLID_CELL 0
#define FLUID_CELL 1
#define EMPTY_CELL 2

class FlipSimulator :public Simulator {
public:
	// Construtors
	FlipSimulator();

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fForceScaling;	
	// FLIP/PIC ratio
	float m_fRatio;

	// grid property
	int m_iCellX;
	int m_iCellY;
	int m_iCellZ;
	float m_h; 			 // grid spacing, m_h = 1.0 / (m_iCellX-1)
	float m_fInvSpacing; // grid inverse spacing, m_fInvSpacing = 1.0/m_h
	int m_iNumCells;	 // m_iCellX * m_iCellY * m_iCellZ

	// particle property
	int m_iNumSpheres;
	float m_particleRadius;

	// particle data arrays
	std::vector<Vec3> m_particlePos;		// Particle Positions
	std::vector<Vec3> m_particleColor;		// Particle Color for visualization
	std::vector<Vec3> m_particleVel;		// Particle Velocity

	// grid data arrays
	std::vector<Vec3>  m_vel;	  	// Velocity array
	std::vector<Vec3>  m_pre_vel; 	// Hold the previous velocity for flip update
	std::vector<float> m_p; 		// Pressure array
	std::vector<float> m_s; 		// 0.0 for solid cells, 1.0 for fluid cells, used to update m_type
	std::vector<int>  m_type; 		// Flags array (const int EMPTY_CELL = 0; const int FLUID_CELL = 1; const int SOLID_CELL = 2;)
									// m_type = SOLID_CELL if m_s == 0.0; 
									// m_type = FLUID_CELL if has particle and m_s == 1; 
									// m_type = EMPTY_CELL if has No particle and m_s == 1; 
	std::vector<float> m_particleDensity;	// Particle Density per cell, saved in the grid cell
	float m_particleRestDensity;

	// Simulation Functions
	void integrateParticles(float timeStep);
	void pushParticlesApart(int numIters);
	void handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel);
	void updateParticleDensity();

	void transferVelocities(bool toGrid, float flipRatio);
	void solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift);
	void updateParticleColors();

	// UI functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// two given functions:
	void simulateTimestep(float dt);

	// ui functions
	void setupScene(int res);
	Vec3 m_obstacleFinalPos;
	Vec3 m_obstaclePos;
	void setObstacle(Vec3 pos);
};
#endif