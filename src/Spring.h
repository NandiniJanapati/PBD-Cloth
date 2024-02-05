#pragma once
#ifndef Spring_H
#define Spring_H

#include <memory>
#include <Eigen/Dense>

class Particle;

class Spring
{
public:
	Spring(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1, double alpha);
	virtual ~Spring();

	//void springConstraint(double& C, Eigen::MatrixXd& gradC);

	void springConstraint(double& C, std::vector<std::vector<double>>& gradC);

	std::shared_ptr<Particle> p0;
	std::shared_ptr<Particle> p1;
	double L;
	double alpha;
};

#endif
