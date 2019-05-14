/**
 * Filename: particle.cpp
 *
 * Represents a single particle and its attributes.
 *
 * Author: Glenn Skelton
 * Last Modified: March 18, 2019
 */


#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include "particle.h"

using namespace std;
using namespace glm;
using namespace givr;


// class: Particle
////////////////////////////// CONSTRUCTORS //////////////////////////
Particle::Particle(unsigned int a_ID,
                   float a_mass,
                   vec3f a_p,
                   vec3f a_v,
                   vec3f a_F,
                   bool a_fixed) {
    this->m_ID = (signed int)a_ID; // any particle assigned an ID must be 0 or greater otherwise it is an error
    this->m_fixed = a_fixed;
    setMass(a_mass);
    setPosition(a_p);
    setVelocity(a_v);
    setNetForce(a_F);
}

Particle::~Particle() {}


//////////////////////////// GETTERS/SETTERS //////////////////////////
signed int Particle::getID() { return m_ID; }
vec3f Particle::getGravity() { return this->m_mass * vec3f(0.0f, -9.81, 0.0f); } // F = mg

float Particle::getMass() { return this->m_mass; }
void Particle::setMass(float a_mass) { this->m_mass = a_mass; }

vec3f Particle::getPosition() { return this->m_p; }
void Particle::setPosition(vec3f a_p) { this->m_p = a_p; }

vec3f Particle::getVelocity() { return this->m_v; }
void Particle::setVelocity(vec3f a_v) { this->m_v = a_v; }

vec3f Particle::getNetForce() { return this->m_F; }
void Particle::setNetForce(vec3f a_F) { this->m_F = a_F; }


////////////////////////// MEMBER FUNCTIONS ///////////////////////////
/**
 * Add gravity to the net force
 *
 * @brief Particle::updateParticle
 * @param a_t
 */
void Particle::updateParticleForce() {
    if (!this->m_fixed) { // only update if not a fixed particle
        vec3f F_p = this->getNetForce() + this->getGravity();
        this->setNetForce(F_p);
    }
}

/**
 * Update the particles position using the net force accumulated
 *
 * @brief Particle::updateParticlePosition
 * @param a_t
 */
void Particle::updateParticlePosition(float a_t) {
    if (!this->m_fixed) { // only update if not a fixed particle
        vec3f acceleration = this->getNetForce() / this->getMass(); // a = F / m
        this->setVelocity(this->getVelocity() + (acceleration * a_t)); // V = V + a(delta_t)
        this->setPosition(this->getPosition() + (this->getVelocity() * a_t)); // X = x + v(delta_t)
    }
    this->setNetForce(vec3f(0.0f, 0.0f, 0.0f)); // reset force accumulator
}

