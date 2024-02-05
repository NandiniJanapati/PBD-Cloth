#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

int Cloth::coordsToindex(float i, float j) {
	int newi = round(i);
	int newj = round(j);
	return newi * cols + newj;
}

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double alpha,
			 double damping,
			 double pradius)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(alpha >= 0.0);
	assert(damping >= 0.0);
	assert(pradius >= 0.0);
	
	this->rows = rows;
	this->cols = cols;
	
	// TODO: Create cloth
	
	// Create particles
	int nVerts = rows*cols; // Total number of vertices
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = pradius;
			p->d = damping;
			
			// This if/else code only works for 2x2.
			// Replace with your code.
			/*
			if(i == 0) {
				if(j == 0) {
					p->x = x00;
				} else {
					p->x = x01;
				}
				p->fixed = true;
			} else {
				if(j == 0) {
					p->x = x10;
				} else {
					p->x = x11;
				}
				p->fixed = false;
			}
			*/

			double s = (1.0 / ((double)cols - 1.0)) * (double)j; //s0 and s1 are the same
			Vector3d x_0 = x00 + s * (x01 - x00);
			Vector3d x_1 = x10 + s * (x11 - x10);
			double t = 0;
			t = (1.0 / ((double)rows - 1.0)) * ((double)rows - 1.0 - (double)i);   //(1 / (rows-1)) * (rows - 1 - i);
			p->x = x_1 + t * (x_0 - x_1);

			if (abs(1 - t) < 0.001) {
				//t is very close to 1
				if (abs(s) < 0.001) {
					//s is very close to 0
					p->fixed = true;
				}
				else if (abs(1 - s) < 0.001) {
					//s is very close to 1
					p->fixed = true;
				}
				else {
					p->fixed = false;
				}
			}
			else {
				p->fixed = false;
			}


			// Populate the other member variables of p here
			p->m = (mass / nVerts);
			p->x0 = p->x;
			p->v0.setZero();
			p->v.setZero();
			p->p = p->x;
		}
	}

	// Create x springs (replace with your code)
	//springs.push_back(make_shared<Spring>(particles[0], particles[1], alpha));
	//springs.push_back(make_shared<Spring>(particles[2], particles[3], alpha));

	for (float i = 0.5; i < (rows - 1); i++) {
		for (float j = 0.5; j < (cols - 1); j++) {

			int topleft = coordsToindex(floor(i), floor(j));
			int topright = coordsToindex(floor(i), ceil(j));
			int bottomleft = coordsToindex(ceil(i), floor(j));
			int bottomright = coordsToindex(ceil(i), ceil(j));

			//make the top x spring
			springs.push_back(make_shared<Spring>(particles[topleft], particles[topright], alpha));

			if ((i + 1.0) >= (rows - 1)) { //if we are on our last row
				//make the bottom x spring;
				springs.push_back(make_shared<Spring>(particles[bottomleft], particles[bottomright], alpha));
			}
		}
	}
	
	// Create y springs (replace with your code)
	//springs.push_back(make_shared<Spring>(particles[0], particles[2], alpha));
	//springs.push_back(make_shared<Spring>(particles[1], particles[3], alpha));

	for (float i = 0.5; i < (rows - 1); i++) {
		for (float j = 0.5; j < (cols - 1); j++) {

			int topleft = coordsToindex(floor(i), floor(j));
			int topright = coordsToindex(floor(i), ceil(j));
			int bottomleft = coordsToindex(ceil(i), floor(j));
			int bottomright = coordsToindex(ceil(i), ceil(j));

			//make the left y spring
			springs.push_back(make_shared<Spring>(particles[topleft], particles[bottomleft], alpha));

			if ((j + 1.0) >= (cols - 1)) { //if we are on our last iteration for this row
				//make the right y spring for this set
				springs.push_back(make_shared<Spring>(particles[topright], particles[bottomright], alpha));
			}

		}
	}

	// Create shear springs
	//springs.push_back(make_shared<Spring>(particles[0], particles[3], alpha));
	//springs.push_back(make_shared<Spring>(particles[1], particles[2], alpha));
	for (float i = 0.5; i < (rows - 1); i++) {
		for (float j = 0.5; j < (cols - 1); j++) {

			int topleft = coordsToindex(floor(i), floor(j));
			int topright = coordsToindex(floor(i), ceil(j));
			int bottomleft = coordsToindex(ceil(i), floor(j));
			int bottomright = coordsToindex(ceil(i), ceil(j));

			//make the \ diagonal spring (p0 is top p1 is bottom)
			springs.push_back(make_shared<Spring>(particles[topleft], particles[bottomright], alpha));
			//make the / diagonal spring (p0 is top p1 is bottom)
			springs.push_back(make_shared<Spring>(particles[topright], particles[bottomleft], alpha));

		}
	}

	// Create x bending springs
	for (int i = 1; i < (rows - 1); i++) {
		for (int j = 1; j < (cols - 1); j++) { //only the middle points' coordinates, not the edges
			int topleft = coordsToindex(i - 1, j - 1);
			int topright = coordsToindex(i - 1, j + 1);
			int bottomleft = coordsToindex(i + 1, j - 1);
			int bottomright = coordsToindex(i + 1, j + 1);
			//make the top x bending spring
			springs.push_back(make_shared<Spring>(particles[topleft], particles[topright], alpha));

			if ((i + 1) >= (rows - 1)) { //we're on our last iteration of i
				//make 2 x bending springs that cover the middle and bottom edge of this block
				int middleleft = coordsToindex(i, j - 1);
				int middleright = coordsToindex(i, j + 1);
				springs.push_back(make_shared<Spring>(particles[middleleft], particles[middleright], alpha)); //middle
				springs.push_back(make_shared<Spring>(particles[bottomleft], particles[bottomright], alpha)); //edge
			}
		}
	}

	// Create y bending springs
	for (int i = 1; i < (rows - 1); i++) {
		for (int j = 1; j < (cols - 1); j++) { //only the middle points' coordinates, not the edges
			int topleft = coordsToindex(i - 1, j - 1);
			int topright = coordsToindex(i - 1, j + 1);
			int bottomleft = coordsToindex(i + 1, j - 1);
			int bottomright = coordsToindex(i + 1, j + 1);

			//make the left y bending spring
			springs.push_back(make_shared<Spring>(particles[topleft], particles[bottomleft], alpha));

			if ((j + 1) >= (cols - 1)) {//we're on our last iteration of j
				//make 2 y bending springs that cover the middle and right edge of this block
				int topmiddle = coordsToindex(i - 1, j);
				int bottommiddle = coordsToindex(i + 1, j);
				springs.push_back(make_shared<Spring>(particles[topmiddle], particles[bottommiddle], alpha)); //middle
				springs.push_back(make_shared<Spring>(particles[topright], particles[bottomright], alpha)); //edge
			}

		}
	}
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();
	
	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}
	
	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(auto p : particles) {
		p->tare();
	}
}

void Cloth::reset()
{
	for(auto p : particles) {
		p->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x; // updated position
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     / | \
			// u0 /__|__\ u1
			//    \  |  /
			//     \ | /
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}

}

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	// TODO: Step
	for (int i = 0; i < particles.size(); i++) {
		if (!particles[i]->fixed) {
			Vector3d fi = particles[i]->m * grav - particles[i]->d * particles[i]->v;
			particles[i]->v += (h / particles[i]->m) * fi;
			particles[i]->p = particles[i]->x;
			particles[i]->x += h * particles[i]->v;
		}
	}

	for (int i = 0; i < springs.size(); i++) {
		double Cj;
		//MatrixXd gradCj(3, 2);
		vector<vector<double>> gradCj;
		springs[i]->springConstraint(Cj, gradCj);
		//Vector3d gradCj0 = gradCj.block<3, 1>(0, 0);
		Vector3d gradCj0; 
		gradCj0 << gradCj[0][0], gradCj[0][1], gradCj[0][2];
		//Vector3d gradCj1 = gradCj.block<3, 1>(0, 1);
		Vector3d gradCj1;
		gradCj1 << gradCj[1][0], gradCj[1][1], gradCj[1][2];
		double w0 = 1 / springs[i]->p0->m;
		double w1 = 1 / springs[i]->p1->m;

		//double lambda = -Cj / ( (w0 * pow(gradCj0.norm(), 2)) + (w1 * pow(gradCj1.norm(), 2)) + (springs[i]->alpha / (h * h)) );
		double lambda = (-1 * Cj) / (w0 + w1 + (springs[i]->alpha / (h * h)));
		if (!springs[i]->p0->fixed) {
			springs[i]->p0->x += lambda * w0 * gradCj0;
		}
		if (!springs[i]->p1->fixed) {
			springs[i]->p1->x += lambda * w1 * gradCj1;
		}

	}

	//sphere collisions
	for (int i = 0; i < particles.size(); i++) {
		//x is the particle poisiton, r is the particle radius
		Vector3d x = particles[i]->x;
		double r = particles[i]->r;

		for (int j = 0; j < spheres.size(); j++) {
			//each particle i has a radius, 
			//xs is the sphere position rs is the sphere particle radius
			Vector3d xs = spheres[j]->x;
			double rs = spheres[j]->r;

			Vector3d deltax = xs - x;
			double l = deltax.norm();
			double d = r + rs - l;
			//if d > 0, then there is a collision
			if (d > 0) {
				Vector3d n = (deltax / l); //collision normal
				particles[i]->x = x + d * (-n);
			}
		}
	}

	for (int i = 0; i < particles.size(); i++) {
		if (!particles[i]->fixed) {
			particles[i]->v = (1 / h) * (particles[i]->x - particles[i]->p);
		}
	}

	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3f(p->getUniform("kdFront"), 0.894f, 0.882f, 0.792f);
	glUniform3f(p->getUniform("kdBack"),  0.776f, 0.843f, 0.835f);
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_tex = p->getAttribute("aTex");
	if(h_tex >= 0) {
		glEnableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	if(h_tex >= 0) {
		glDisableVertexAttribArray(h_tex);
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
