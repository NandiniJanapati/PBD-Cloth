#include "Spring.h"
#include "Particle.h"

using namespace std;
using namespace Eigen;

Spring::Spring(shared_ptr<Particle> p0, shared_ptr<Particle> p1, double alpha)
{
	assert(p0);
	assert(p1);
	assert(p0 != p1);
	this->p0 = p0;
	this->p1 = p1;
	this->alpha = alpha;
	
	// TODO: Compute L
	L = (p1->x0 - p0->x0).norm();
}

Spring::~Spring()
{
	
}

//void Spring::springConstraint(double& C, MatrixXd& gradC) {
//
//	Vector3d deltax = p1->x - p0->x;
//	double l = deltax.norm();
//	C = l - L;
//
//	Vector3d gradC0 = -(deltax / l);
//	Vector3d gradC1 = (deltax / l);
//
//	gradC.block<3, 1>(0, 0) = gradC0;
//	gradC.block<3, 1>(0, 1) = gradC1;
//
//	return;
//}

void Spring::springConstraint(double& C, vector<vector<double>>& gradC) {
	
	Vector3d deltax = p1->x - p0->x;
	double l = deltax.norm();
	C = l - L;

	Vector3d gradC0 = -(deltax / l);
	Vector3d gradC1 = (deltax / l);

	vector<double> C0 = { gradC0(0), gradC0(1), gradC0(2) };
	vector<double> C1 = { gradC1(0), gradC1(1), gradC1(2) };

	gradC.push_back(C0);
	gradC.push_back(C1);  //2x3

	return;
}
