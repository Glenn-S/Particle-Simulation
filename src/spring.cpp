/**
 * Filename: spring.cpp
 *
 * Represents a spring connecting two particles together.
 *
 * Author: Glenn Skelton
 * Last Modified: March 18, 2019
 */

#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

#include "particle.h"
#include "spring.h"
#include <vector>

using namespace std;
using namespace glm;

// class: spring
////////////////////////////// CONSTRUCTORS //////////////////////////
Spring::Spring(float a_l,
               signed int a_p1,
               signed int a_p2,
               float a_k,
               float a_b) {
    setSpringLength(a_l);
    setParticle1(a_p1);
    setParticle2(a_p2);
    setStiffness(a_k);
    setDamping(a_b);
}

Spring::~Spring() {}





//////////////////////////// GETTERS/SETTERS //////////////////////////
float Spring::getSpringLength() { return this->m_l; }
void Spring::setSpringLength(float a_l) { this->m_l = a_l > 0.0f ? a_l : 0.0f; }

signed int Spring::getParticle1() { return this->m_p1; }
void Spring::setParticle1(signed int a_p1) { this->m_p1 = a_p1 >= 0 ? a_p1 : -1; }

signed int Spring::getParticle2() { return this->m_p2; }
void Spring::setParticle2(signed int a_p2) { this->m_p2 = a_p2 >= 0 ? a_p2 : -1; }

float Spring::getStiffness() { return this->m_k; }
void Spring::setStiffness(float a_k) { this->m_k = a_k > 0.0f ? a_k : 0.0f; }

float Spring::getDamping() { return this->m_b; }
void Spring::setDamping(float a_b) { this->m_b = a_b > 0.0f ? a_b : 0.0f; }






////////////////////////// MEMBER FUNCTIONS ///////////////////////////
/**
 * Calculate the spring force exerted by each particle on the other and add to their force accumulator
 *
 * @brief Spring::calculateSpringForce
 * @param a_particles
 */
void Spring::calculateSpringForce(vector<Particle*> *a_p) {
    // F = -kx - bv
    Particle *p1 = a_p->at(getParticle1());
    Particle *p2 = a_p->at(getParticle2());

    vec3f p1p2 = p2->getPosition() - p1->getPosition(); // from p1 to p2
    float x = length(p1p2); // length of vector p1p2 stored in
    p1p2 = normalize(p1p2); // get unit length vector in direction of vector p1p2
    float k = this->getStiffness();
    float l = this->getSpringLength();
    float deltaX = x - l;
    float b = this->getDamping();
    vec3f F_s; // force of spring
    vec3f F_d; // force of damping
    vec3 F_p(0.0f, 0.0f, 0.0f); // net force

    // get projected velocity of both particles
    // borrowed idea from lecture/tutorial material for vector projection
    F_d = -b * ( dot((p2->getVelocity() - p1->getVelocity()), p1p2) / dot(p1p2, p1p2) ) * p1p2;
    F_s = -k * deltaX * p1p2;
    F_p = F_s + F_d; // Force applied to the particle from spring and damper  + F_d

    vec3f F_net;

    if (!p1->m_fixed) { // update if not fixed
        F_net = a_p->at(getParticle1())->getNetForce();
        F_net = F_net - F_p;
        a_p->at(getParticle1())->setNetForce(F_net);
    }
    if (!p2->m_fixed) { // update if not fixed
        F_net = a_p->at(getParticle2())->getNetForce();
        F_net = F_net + F_p;
        a_p->at(getParticle2())->setNetForce(F_net);
    }

}
