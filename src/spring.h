/**
 * Filename: spring.h
 *
 * Author: Glenn Skelton
 * Last Modified: March 18, 2019
 */

#ifndef SPRING_H
#define SPRING_H

#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include "particle.h"

using namespace std;
using namespace glm;
using namespace givr;

class Spring {
// public functions
public:
    Spring(float a_l,
           signed int a_p1,
           signed int a_p2,
           float a_k,
           float a_b);
    ~Spring();

    // getter/setters
    float getSpringLength();
    void setSpringLength(float a_l);

    signed int getParticle1();
    void setParticle1(signed int a_p1);

    signed int getParticle2();
    void setParticle2(signed int a_p2);

    float getStiffness();
    void setStiffness(float a_k);

    float getDamping();
    void setDamping(float a_b);

// member functions
public:
    void calculateSpringForce(vector<Particle*> *a_p);

// private member variables
private:
    float m_l; // natural spring length
    signed int m_p1; // index of particle 1
    signed int m_p2; // index of particle 2
    float m_k; // spring stiffness coefficient
    float m_b; // spring damping coefficient
};
#endif // SPRING_H
