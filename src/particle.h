/**
 * Filename: particle.h
 *
 * Author: Glenn Skelton
 * Last Modified: March 18, 2019
 */

#ifndef PARTICLE_H
#define PARTICLE_H

#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

using namespace std;
using namespace glm;
using namespace givr;

class Particle {
// public functions
public:
    Particle(unsigned int a_ID,
             float a_mass,
             vec3f  a_p,
             vec3f  a_v,
             vec3f  a_F,
             bool a_fixed = false);
    virtual ~Particle();

    signed int getID();
    // no setter

    vec3f getGravity();
    // no setter

    float getMass();
    void setMass(float a_mass);

    vec3f getPosition();
    void setPosition(vec3f a_p);

    vec3f getVelocity();
    void setVelocity(vec3f a_v);

    vec3f getNetForce();
    void setNetForce(vec3f a_F);

// member functions
public:
    void updateParticleForce();
    void updateParticlePosition(float a_t);

// public member variables
public:
    vec3f m_F; // net force of the particle
    bool  m_fixed; // to know if a particle should be moved or if it is anchored

// private member variables
private:
    signed int m_ID;
    float m_mass; // mass of the particle in kg
    vec3f  m_p; // position of the particle
    vec3f  m_v; // velocity of the particle


};
#endif // PARTICLE_H
