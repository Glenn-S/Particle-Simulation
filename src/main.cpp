//------------------------------------------------------------------------------
// Filename: main.cpp
//
// CPSC 587, Assignment 3
// Glenn Skelton
//
// Resources Used:
// -used for learning about good method of spring damping and wind force
// Cloth Simulation: CSE 169: Computer Animation - Steve Rotenburg
// -lecture notes and tutorial notes from CPSC 587
//
//------------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////////
#include "particle.h"
#include "spring.h"
#include <vector>

#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include "io.h"
#include "turntable_controls.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
void createSprings(vector<Particle*> *a_p,
                   vector<Spring*> *a_s,
                   const float a_k,
                   const float a_b,
                   const float a_searchRadius,
                   const float a_minLen = 0);

void updateParticles(vector<Particle*> *a_p,
                     vector<unsigned int> *a_t,
                     vector<Spring*> *a_s,
                     float a_time,
                     void (*const checkCollisions)(vector<Particle*>*, vector<unsigned int>*, Particle*)); // takes a collision detection function
void updateParticles(vector<Particle*> *a_p,
                     vector<Spring*> *a_s,
                     float a_time,
                     void (*const checkCollisions)(vector<Particle*>*, Particle*));
void updateParticles(vector<Particle*> *a_p,
                     vector<Spring*> *a_s,
                     float a_time);

void calculateCollisionsPart1(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p);
void calculateCollisionsPart2(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p);
void calculateCollisionsPart3(vector<Particle*> *a_p, Particle *p);
void calculateCollisionsPart4(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p);
void calculateCollisionsBonus1(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p);
void calculateCollisionsBonus2(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p);
void calculateWindForce(vector<Particle*> *a_p, unsigned int a_size, vec3f a_wind);

void resetParticles(vector<Particle*> *a_p,
                    vector<Spring*> *a_s,
                    unsigned int,
                    void (*const init)(vector<Particle*>*, vector<Spring*>*, unsigned int));
void resetParticles(vector<Particle*> *a_p,
                    vector<Spring*> *a_s,
                    void (*const init)(vector<Particle*>*, vector<Spring*>*));
void cleanup(vector<Particle*> *a_p, vector<Spring*> *a_s);

// PROCEDURES
void part1_Init(vector<Particle*> *a_p, vector<Spring*> *a_s);
void part2_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int num_particles);
void part3_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int cube_size);
void part4_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int cloth_size);
void bonus1_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int flag_size);
void bonus2_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int cloth_size);

void generateTriangles(vector<Particle*> *a_p, vector<unsigned int> *a_t, unsigned int a_size);
void updateCloth(vector<unsigned int> *a_t, vector<Particle*> *a_p, TriangleSoupGeometry *a_mesh);
void updateJello(vector<Particle*> *a_p, TriangleSoupGeometry *a_mesh, unsigned int a_size);

///////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL CONSTANTS
constexpr float ROOF_HEIGHT  =  10.0f; // height of the roof for part 1 and 2
constexpr float FLOOR_HEIGHT = -15.0f; // height of the floor for part 3

constexpr float DELTA_T                 = 0.005;   // seconds (time step) 0.05
constexpr float AIR_DAMPING             = 0.06f;   // in Ns/m
constexpr unsigned int INTEGRATION_STEP = 32;      // number of times to integrate before rendering / 10
constexpr float AIR_DENSITY             = 1.2041f; // density of the air at 20 degrees celcius (wikipedia)
constexpr float COEFFICIENT_DRAG        = 1.2f;    // ceofficient of drag (wikipedia)


// SHADERS
auto red_down    = Phong( Colour(1.0f, 0.2f, 0.2f), LightPosition(15.0f, -15.0f, 15.0f) );
auto red_up      = Phong( Colour(1.0f, 0.2f, 0.2f), LightPosition(15.0f,  15.0f, 15.0f) );
auto purple_down = Phong( Colour(0.5f, 0.3f, 1.0f), LightPosition(15.0f, -15.0f, 15.0f) );
auto purple_up   = Phong( Colour(0.5f, 0.3f, 1.0f), LightPosition(15.0f,  15.0f, 15.0f) );
auto white_down  = Phong( Colour(1.0f, 0.5f, 0.5f), LightPosition(15.0f, -15.0f, 15.0f) );
auto white_up    = Phong( Colour(1.0f, 0.5f, 0.5f), LightPosition(15.0f,  15.0f, 15.0f) );
auto grey_down   = Phong( Colour(0.5f, 0.5f, 0.5f), LightPosition(15.0f, -15.0f, 15.0f) );
auto grey_up     = Phong( Colour(0.5f, 0.5f, 0.5f), LightPosition(15.0f,  15.0f, 15.0f) );
auto shader4     = Phong( Colour(1.0f, 0.0f, 1.0f), LightPosition(15.0f,  15.0f, 15.0f) );









//---------------------------------------------------------------------------------------------------
// SIMULATION
//---------------------------------------------------------------------------------------------------
/**
 * Simulation of a mass particle system modelling masses suspended by springs
 *
 * @brief main
 * @return
 */
int main(void)
{
/////////////////////////////////////////// SETUP /////////////////////////////////////////////
    io::GLFWContext windows;
    auto window = windows.create(io::Window::dimensions{800, 600}, "Assignment 3"); // generate window instance
    window.enableVsync(true);

    auto view = View(TurnTable(), Perspective()); // create view matrix
    TurnTableControls controls(window, view.camera); // init controls for window

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f); // clear screen

    // additional geometry
    auto plane1 = Triangle(Point1( 100.0f, 0.0f,  100.0f),
                           Point2( 100.0f, 0.0f, -100.0f),
                           Point3(-100.0f, 0.0f, -100.0f));
    auto plane2 = Triangle(Point1(-100.0f, 0.0f, -100.0f),
                           Point2(-100.0f, 0.0f,  100.0f),
                           Point3( 100.0f, 0.0f,  100.0f));



////////////////////////////////// CREATE GEOMETRY /////////////////////////////////////////////
    // part 1
    vector<Particle*> *part1_particles = new vector<Particle*>();
    vector<Spring*> *part1_spring = new vector<Spring*>();
    part1_Init(part1_particles, part1_spring); // init all particles and spring

    // create particle geometry
    auto part1_p = createRenderable(Sphere(), red_down); // init part1_particle

////////////////////////////////////////////////////////////////////////////////////////////////
    // part 2
    constexpr unsigned int num_particles = 10; // 10 particles in the chain (part 2)

    vector<Particle*> *part2_particles = new vector<Particle*>();
    vector<Spring*> *part2_springs = new vector<Spring*>();
    part2_Init(part2_particles, part2_springs, num_particles); // init all particles and spring

    // need array of spheres
    auto part2_p = createInstancedRenderable(Sphere(), red_down); // create instanced paricle for copying

////////////////////////////////////////////////////////////////////////////////////////////////
    // part 3
    constexpr unsigned int cube_size = 10; // part 3 size of jelloy cube

    vector<Particle*> *part3_particles = new vector<Particle*>();
    vector<Spring*> *part3_springs = new vector<Spring*>();
    part3_Init(part3_particles, part3_springs, cube_size); // init all particles and spring

    auto meshJello = TriangleSoup();
    updateJello(part3_particles, &meshJello, cube_size);
    auto jMesh = createRenderable(meshJello, red_up);

////////////////////////////////////////////////////////////////////////////////////////////////
    // part 4
    constexpr unsigned int cloth_size = 80; // number of particles per row / column (part 4)
    vec3f part4_windDirection(-15.0, 0.0, -15.0); // direction and velocity of wind

    // create triangles for quick search
    vector<unsigned int> *part4_triangles = new vector<unsigned int>(); // array of vertices in order of triangles
    vector<Particle*> *part4_particles = new vector<Particle*>();
    vector<Spring*> *part4_springs = new vector<Spring*>();
    part4_Init(part4_particles, part4_springs, cloth_size); // init all particles and spring
    generateTriangles(part4_particles, part4_triangles, cloth_size); // populate triangle array with indices of particle indexes

    // create the triangles
    auto meshCloth = TriangleSoup();
    updateCloth(part4_triangles, part4_particles, &meshCloth);
    auto cMesh = createRenderable(meshCloth, red_down);

/////////////////////////////////////////////////////////////////////////////////////////////////
    // bonus 1
    constexpr unsigned int flag_size = 80; // number of particles per row / column (bonus 1)
    vec3f bonus1_windDirection(50.0, 20.0, 1.0); // direction and velocity of wind

    vector<unsigned int> *bonus1_triangles = new vector<unsigned int>(); // array of vertices in order of triangles
    vector<Particle*> *bonus1_particles = new vector<Particle*>();
    vector<Spring*> *bonus1_springs = new vector<Spring*>();
    bonus1_Init(bonus1_particles, bonus1_springs, flag_size); // init all particles and spring
    generateTriangles(bonus1_particles, bonus1_triangles, flag_size); // populate triangle array with indices of particle indexes

    // create the triangles
    auto meshFlag = TriangleSoup();
    updateCloth(bonus1_triangles, bonus1_particles, &meshFlag);
    auto fMesh = createRenderable(meshFlag, red_down);

    // flag poll
    float offset = ((float)flag_size * (20.0 / (float)flag_size)) / 2.0f;
    auto flagPole = Cylinder(Point1(-offset, 10.5f, 0.0f),
                             Point2(-offset, -20.0f, 0.0f),
                             Radius(0.25f));
    auto cap = createRenderable(Sphere(Centroid(-offset, 11.5, 0.0f)), grey_down);
    auto pole = createRenderable(flagPole, grey_down);


////////////////////////////////////////////////////////////////////////////////////////////////
    // bonus 2
    constexpr unsigned int sheet_size = 80; // number of particles per row / column (bonus 2) used to be 80

    vector<Particle*> *bonus2_particles = new vector<Particle*>();
    vector<unsigned int> *bonus2_triangles = new vector<unsigned int>(); // array of vertices in order of triangles
    vector<Spring*> *bonus2_springs = new vector<Spring*>();
    bonus2_Init(bonus2_particles, bonus2_springs, sheet_size); // init all particles and spring
    generateTriangles(bonus2_particles, bonus2_triangles, sheet_size);

    // create the triangles
    auto meshSheet = TriangleSoup();
    updateCloth(bonus1_triangles, bonus1_particles, &meshSheet);
    auto sMesh = createRenderable(meshSheet, red_up);

    // ball to have cloth fall onto (I prefered over table)
    auto ballObject = Sphere(Centroid(0.0f, FLOOR_HEIGHT + 3.0f, 0.0f),
                             Radius(3.0f),
                             AzimuthPoints(80),
                             AltitudePoints(80));

    auto ball = createRenderable(ballObject, grey_up);


////////////////////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------------------------
// GRAPHICS LOOP
//----------------------------------------------------------------------------------------------
    window.run([&](float) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the screen
        view.projection.updateAspectRatio(window.width(), window.height()); // update view matrix
        mat4f model{1.0f};

        switch (scene) {
        case SCENE_1:
        {
            // reset particles if scene changed
            if (RESET) {
                resetParticles(part1_particles, part1_spring, part1_Init);
                RESET = !RESET;
            }

            // render roof
            auto roof1 = createRenderable(plane1, purple_down);
            auto roof2 = createRenderable(plane2, purple_down);
            model = translate(mat4{1.f}, vec3f(0.0f, ROOF_HEIGHT, 0.0f));
            draw(roof1,  view, model);
            draw(roof2,  view, model);

            // render particle
            for (unsigned int i = 0; i < INTEGRATION_STEP; i++)
                updateParticles(part1_particles, part1_spring, DELTA_T); // calculate forces and update particles
            model = translate(mat4f{1.f}, part1_particles->at(1)->getPosition());
            draw(part1_p, view, model);

            // render string
            auto line = createRenderable(Cylinder(Point1(part1_particles->at(0)->getPosition()),
                                                  Point2(part1_particles->at(1)->getPosition()),
                                                  Radius(0.05f)),
                                         white_down);
            draw(line, view, mat4{1.f});
        }
            break;
        case SCENE_2:
        {
            // reset particles if scene changed
            if (RESET) {
                resetParticles(part2_particles, part2_springs, num_particles, part2_Init);
                RESET = !RESET;
            }

            // render roof
            auto roof1 = createRenderable(plane1, purple_down);
            auto roof2 = createRenderable(plane2, purple_down);
            model = translate(mat4{1.f}, vec3f(0.0f, ROOF_HEIGHT, 0.0f));
            draw(roof1,  view, model);
            draw(roof2,  view, model);

            // render particles
            for (unsigned int i = 0; i < INTEGRATION_STEP; i++)
                updateParticles(part2_particles, part2_springs, DELTA_T); // calculate forces and update particles

            for (Particle *p : *part2_particles) { // update model for each particle
                model = translate(mat4f{1.f}, p->getPosition());
                addInstance(part2_p, model);
            }
            draw(part2_p, view);

            // update lines
            for (Spring *s : *part2_springs) {
                Particle *a = part2_particles->at(s->getParticle1());
                Particle *b = part2_particles->at(s->getParticle2());
                auto line = createRenderable(Cylinder(Point1(a->getPosition()),
                                                      Point2(b->getPosition()),
                                                      Radius(0.05f)),
                                             white_down);
                draw(line, view, mat4{1.f});
            }
        }
            break;
        case SCENE_3:
        {
            // reset particles if scene changed
            if (RESET) {
                resetParticles(part3_particles, part3_springs, cube_size, part3_Init);
                RESET = !RESET;
            }

            // render floor
            auto floor1 = createRenderable(plane1, purple_up);
            auto floor2 = createRenderable(plane2, purple_up);
            model = translate(mat4{1.f}, vec3f(0.0f, FLOOR_HEIGHT, 0.0f));
            draw(floor1, view, model);
            draw(floor2, view, model);

            // render particles
            for (unsigned int i = 0; i < INTEGRATION_STEP; i++)
                updateParticles(part3_particles, part3_springs, DELTA_T, calculateCollisionsPart3); // calculate forces and update particles

            updateJello(part3_particles, &meshJello, cube_size);
            updateRenderable(meshJello, red_up, jMesh);
            draw(jMesh, view, mat4{1.f});

        }
            break;
        case SCENE_4:
        {
            // reset particles if scene changed
            if (RESET) {
                resetParticles(part4_particles, part4_springs, cloth_size, part4_Init);
                RESET = !RESET;
            }

            // render particles
            for (unsigned int i = 0; i < INTEGRATION_STEP; i++) {
                calculateWindForce(part4_particles, cloth_size, part4_windDirection); // calculate wind on hanging sheet
                updateParticles(part4_particles, part4_triangles, part4_springs, DELTA_T, calculateCollisionsPart4); // calculate forces and update particles
            }

            updateCloth(part4_triangles, part4_particles, &meshCloth);
            updateRenderable(meshCloth, red_down, cMesh);
            draw(cMesh, view, mat4{1.f});
        }
            break;
        case BONUS_1:
        {
            // reset particles if scene changed
            if (RESET) {
                resetParticles(bonus1_particles, bonus1_springs, flag_size, bonus1_Init);
                RESET = !RESET;
            }

            // render pole
            draw(cap, view, mat4{1.f});
            draw(pole, view, mat4{1.f});

            // render particles
            for (unsigned int i = 0; i < INTEGRATION_STEP; i++) {
                calculateWindForce(bonus1_particles, flag_size, bonus1_windDirection); // calculate wind on hanging sheet
                updateParticles(bonus1_particles, bonus1_triangles, bonus1_springs, DELTA_T, calculateCollisionsBonus1); // calculate forces and update particles
            }

            updateCloth(bonus1_triangles, bonus1_particles, &meshFlag);
            updateRenderable(meshFlag, red_down, fMesh);
            draw(fMesh, view, mat4{1.f});
        }
            break;
        case BONUS_2:
        {
            // reset particles if scene changed
            if (RESET) {
                resetParticles(bonus2_particles, bonus2_springs, sheet_size, bonus2_Init);
                RESET = !RESET;
            }

            // render floor
            auto floor1 = createRenderable(plane1, purple_up);
            auto floor2 = createRenderable(plane2, purple_up);
            model = translate(mat4{1.f}, vec3f(0.0f, FLOOR_HEIGHT, 0.0f));
            draw(floor1, view, model);
            draw(floor2, view, model);

            // render particles
            for (unsigned int i = 0; i < INTEGRATION_STEP; i++)
                updateParticles(bonus2_particles, bonus2_triangles, bonus2_springs, DELTA_T, calculateCollisionsBonus2); // calculate forces and update particles

            updateCloth(bonus2_triangles, bonus2_particles, &meshSheet);
            updateRenderable(meshSheet, red_up, sMesh);
            draw(sMesh, view, mat4{1.f});

            // draw sphere
            draw(ball, view, mat4{1.f});
        }
            break;
        }
    });

    cleanup(part1_particles, part1_spring);
    cleanup(part2_particles, part2_springs);
    cleanup(part3_particles, part3_springs);
    cleanup(part4_particles, part4_springs);
    exit(EXIT_SUCCESS);
}

///////////////////////////////////// END OF MAIN ///////////////////////////////////////////







//////////////////////////////////// PROCEDURES/INIT ////////////////////////////////////////
/**
 * Initialize the state for part 1
 *
 * @brief part1_Init
 * @param a_p
 * @param a_s
 */
void part1_Init(vector<Particle*> *a_p, vector<Spring*> *a_s) {
    constexpr float PARTICLE_MASS = 0.5f;

    // define particles
    a_p->push_back(new Particle(0, // index
                                PARTICLE_MASS, // mass
                                vec3f(0.0f, ROOF_HEIGHT, 0.0f), // position
                                vec3f(0.0f, 0.0f, 0.0f), // velocity
                                vec3f(0.0f, 0.0f, 0.0f), // force
                                true) // anchored particle
                                );
    a_p->push_back(new Particle(1, // index
                                PARTICLE_MASS, // mass
                                vec3f(0.0f, ROOF_HEIGHT - 3.0f, 0.0f), // position
                                vec3f(0.0f, 0.0f, 0.0f), // velocity
                                vec3f(0.0f, 0.0f, 0.0f)) // force
                                // non anchored particle
                                );
    // create springs
    float searchRadius = length(a_p->at(0)->getPosition() - a_p->at(1)->getPosition());
    createSprings(a_p, // array of particles
                  a_s, // array of springs
                  10.0f, // spring constant
                  0.5f, // damping coefficient
                  searchRadius, // search radius for particles
                  15.0f); // natural spring length
}

/**
 * Initialize the state for part 2
 *
 * @brief part2_Init
 * @param a_p
 * @param a_s
 */
void part2_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int num_particles) {
    // create array of particles for chain pendulum
    constexpr float PARTICLE_MASS = 0.5f;
    constexpr float dist_between = 0.5f;
    constexpr float dist_multiplier = 5.0f;
    float xPos = dist_between * dist_multiplier;
    vec3f chainStartPos = vec3f(0.0f, ROOF_HEIGHT - 1.0f, 0.0f);

    // create all particles
    for (unsigned int i = 0; i <= num_particles; i++) { // 11 particles in total, 1 anchored
        if (i != 0) { // non anchored particle
            a_p->push_back(new Particle(i, // index
                                        PARTICLE_MASS, // mass
                                        vec3f(xPos * i, 0.0f, 0.0f) + chainStartPos, // position vec3f(1, 0, 0)
                                        vec3f(0.0f, 0.0f, 0.0f), // velocity
                                        vec3f(0.0f, 0.0f, 0.0f)) // force
                                        );
        } else { // anchored particle
            a_p->push_back(new Particle(i, // index
                                        PARTICLE_MASS, // mass
                                        chainStartPos, // position
                                        vec3f(0.0f, 0.0f, 0.0f), // velocity
                                        vec3f(0.0f, 0.0f, 0.0f), // force
                                        true) // is anchored particle
                                        );
        }
    }

    // create all springs
    createSprings(a_p, // array of particles
                  a_s, // array of springs
                  40.0f, // spring constant
                  1.0f, // damping coefficient
                  xPos); // search radius for particles
                  // natural spring length
                  // make the springs a little closer than natural length
}

/**
 * Initialize the state for part 3
 *
 * @brief part3_Init
 * @param a_p
 * @param a_s
 */
void part3_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int cube_size) {
    constexpr float PARTICLE_MASS = 0.04f;
    const float dist_between = (5.0f / (float)cube_size);
    const float diagonal_between = sqrt(dist_between * dist_between + sqrt(2 * dist_between * dist_between)); // diagonal distance in cube
    const float offset = 0.001; // offset for making jello jiggle
    float start = (float)(cube_size * dist_between) / 2.0f;

    vec3f origin(-start, start, -start); // back left corner
    vec3f jelloDelta(0.0f, 0.0f, 0.0f);
    unsigned int ID = 0;

    // generate Jello indices and store particles
    for (unsigned int i = 0; i <= cube_size; i++) {
        jelloDelta = origin + vec3f((dist_between * i), 0.0f, 0.0f); // increment
        for (unsigned int j = 0; j <= cube_size; j++) {
            for (unsigned int k = 0; k <= cube_size; k++) {
                a_p->push_back(new Particle(ID, // index
                                            PARTICLE_MASS, // mass
                                            origin + jelloDelta + vec3f(0, offset * k, 0), // position
                                            vec3f(0.0f, 0.0f, 0.0f), // velocity
                                            vec3f(0.0f, 0.0f, 0.0f)) // force
                                            );
                ID++;
                jelloDelta = jelloDelta + vec3f(0.0f, 0.0f, dist_between);
            }
            jelloDelta = jelloDelta + vec3f(0.0f, -dist_between, -(dist_between * (float)(cube_size+1)));
        }
    }

    // create all springs
    createSprings(a_p, // array of particles
                  a_s, // array of springs
                  50.0f, // spring constant 8.0
                  0.01f, // damping coefficient 0.05
                  diagonal_between); // search radius for particles
}

/**
 * Initialize the state for part 4
 *
 * @brief part4_Init
 * @param a_p
 * @param a_s
 */
void part4_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int cloth_size) {
    constexpr float PARTICLE_MASS = 0.03f;
    const float dist_between = (20.0 / (float)cloth_size);
    const float diagonal_between = sqrt(2* dist_between*dist_between); // diagonal distance
    float start = ((float)cloth_size * dist_between) / 2.0f;

    vec3f origin(-start, 10.0f, 0.0f);
    vec3f clothDelta(0.0f, 0.0f, 0.0f);
    unsigned int ID = 0;
    float particleOffset = 0.0f;

    // generate cloth indices and particles
    for (unsigned int i = 0; i <= cloth_size; i++) {
        // update clothDelta to start next row
        clothDelta = origin + vec3f(0.0f, -dist_between * i, 0.0f); // move down one particle

        for (unsigned int j = 0; j <= cloth_size; j++) {
            if ( (i == 0 && j == 0) ||
                 (i == 0 && j == cloth_size) ||
                 (i == 0 && j == cloth_size / 2) ) { // fixed particles
                a_p->push_back(new Particle(ID, // index
                                            PARTICLE_MASS, // mass
                                            clothDelta, // position
                                            vec3f(0.0f, 0.0f, 0.0f), // velocity
                                            vec3f(0.0f, 0.0f, 0.0f), // force
                                            true)
                                            );

            } else { // free moving particle
                a_p->push_back(new Particle(ID, // index
                                            PARTICLE_MASS, // mass
                                            clothDelta + vec3f(0.0f, 0.0f, particleOffset), // position
                                            vec3f(0.0f, 0.0f, 0.0f), // velocity
                                            vec3f(0.0f, 0.0f, 0.0f)) // force
                                            );
            }
            ID++;

            // update clothDelta for next column in row
            clothDelta = clothDelta + vec3f(dist_between, 0.0f, 0.0f); // move right one particle
        }
        particleOffset += 0.00002; // move particles forward (create displacement to cause paricles to not be directly in line)

    }

    // create springs
    createSprings(a_p, // array of particles
                  a_s, // array of springs
                  250.0f, // spring constant 250 , 60
                  0.25f, // damping coefficient 0.08 , 0.002
                  diagonal_between); // search radius for particles

}

/**
 * Initialize the state for bonus 1
 *
 * @brief bonus1_Init
 * @param a_p
 * @param a_s
 * @param cloth_size
 */
void bonus1_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int flag_size) {
    constexpr float PARTICLE_MASS = 0.03f;
    const float dist_between = (20.0 / (float)flag_size);
    const float diagonal_between = sqrt(2* dist_between*dist_between); // diagonal distance
    float start = ((float)flag_size * dist_between) / 2.0f;
    float particleOffset = 0.0f;

    vec3f origin(-start, 10.0f, 0.0f); // was 10.0
    vec3f clothDelta(0.0f, 0.0f, 0.0f);
    unsigned int ID = 0;

    // generate flag indices and particles
    for (unsigned int i = 0; i <= flag_size; i++) {
        // update clothDelta to start next row
        clothDelta = origin + vec3f(0.0f, -dist_between * i, 0.0f); // move down one particle

        for (unsigned int j = 0; j <= flag_size; j++) {
            if ( (i == 0 && j == 0) ||
                 (i == flag_size && j == 0)) { // fixed particle
                a_p->push_back(new Particle(ID, // index
                                            PARTICLE_MASS, // mass
                                            clothDelta, // position
                                            vec3f(0.0f, 0.0f, 0.0f), // velocity
                                            vec3f(0.0f, 0.0f, 0.0f), // force
                                            true)
                                            );

            } else { // free moving particle
                a_p->push_back(new Particle(ID, // index
                                            PARTICLE_MASS, // mass
                                            clothDelta + vec3f(0.0f, 0.0f, particleOffset), // position
                                            vec3f(0.0f, 0.0f, 0.0f), // velocity
                                            vec3f(0.0f, 0.0f, 0.0f)) // force
                                            );
            }
            ID++;

            // update clothDelta for next column in row
            clothDelta = clothDelta + vec3f(dist_between, 0.0f, 0.0f); // move right one particle
        }
        particleOffset += 0.00002; // move particles forward to offset alignment
    }

    // create springs
    createSprings(a_p, // array of particles
                  a_s, // array of springs
                  350.0f, // spring constant 120
                  0.3f, // damping coefficient 0.8
                  diagonal_between); // search radius for particles

}

/**
 * Initialize the state for bonus 2
 *
 * @brief bonus2_Init
 * @param a_p
 * @param a_s
 * @param cloth_size
 */
void bonus2_Init(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int sheet_size) {
    constexpr float PARTICLE_MASS = 0.03f;
    const float dist_between = (20.0 / (float)sheet_size);
    const float diagonal_between = sqrt(2* dist_between*dist_between); // diagonal distance
    float start = ((float)sheet_size * dist_between) / 2.0f;

    vec3f origin(-start, -1.0f, -start);
    vec3f clothDelta(0.0f, 0.0f, 0.0f);
    unsigned int ID = 0;

    // generate indices and particles
    for (unsigned int i = 0; i <= sheet_size; i++) {
        // update clothDelta to start next row
        clothDelta = origin + vec3f(0.0f, 0.0f, dist_between * i); // move forward (towards the camera) one particle

        for (unsigned int j = 0; j <= sheet_size; j++) {
            a_p->push_back(new Particle(ID, // index
                                        PARTICLE_MASS, // mass
                                        clothDelta, // position
                                        vec3f(0.0f, 0.0f, 0.0f), // velocity
                                        vec3f(0.0f, 0.0f, 0.0f)) // force
                                        );
            ID++;

            // update clothDelta for next column in row
            clothDelta = clothDelta + vec3f(dist_between, 0.0f, 0.0f); // move right one particle
        }
    }

    // create springs
    createSprings(a_p, // array of particles
                  a_s, // array of springs
                  250.0f, // spring constant 120
                  0.25f, // damping coefficient 0.8
                  diagonal_between); // search radius for particles
}





/////////////////////////////////////// COLLISION /////////////////////////////////////////////
/**
 * @brief calculateCollisionForce
 * @param particles
 */
void calculateCollisionsPart1(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p) {
    // not necessary any more
}

/**
 * @brief calculateCollisionForce
 * @param particles
 */
void calculateCollisionsPart2(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p) {
    // not necessary right now
}

/**
 * @brief calculateCollisionForce
 * @param particles
 */
void calculateCollisionsPart3(vector<Particle*> *a_p, Particle *p) {
    float distance;
    vec3f force;
    float k = 1000; // stiffness of the ground
    float b = 0.5; // floor spring damping 40
    float particleRadius = 0.1; // arbitrarily small value for particles in cube

    // test against floor and push back
    if (p->getPosition().y < FLOOR_HEIGHT + particleRadius) {
        distance = p->getPosition().y - (FLOOR_HEIGHT + particleRadius);
        force = ((-k * distance) + (-b * p->getVelocity())) * vec3f(0.0, 1.0, 0.0); // direct force up in the positive y axis (normal to the surface)
        p->setNetForce(p->getNetForce() + force);
    }


}

/**
 * @brief calculateCollisionForce
 * @param particles
 */
void calculateCollisionsPart4(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p) {
    // not necessary right now
    // cloth - cloth intersection
}

/**
 * @brief calculateCollisionForce
 * @param particles
 */
void calculateCollisionsBonus1(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p) {
    // not necessary right now
    // cloth - cloth intersection
}

/**
 * @brief calculateCollisionForce
 * @param particles
 */
void calculateCollisionsBonus2(vector<Particle*> *a_p, vector<unsigned int> *a_t, Particle *p) {
    float distance;
    vec3f force;
    vec3f direction; // normal to the surface
    float k = 500; // stiffness of the ground
    float ballRadius = 3.0; // radius of the ball
    float particleRadius = 0.1; // arbitrarily small value for particles in cube

    // test against sphere
    direction = p->getPosition() - vec3f(0.0, FLOOR_HEIGHT + ballRadius, 0.0);
    distance = length(direction);
    direction = normalize(direction);
    if (distance < particleRadius + ballRadius) {
        // collision so apply force
        force = -k * (distance - (particleRadius + ballRadius)) * direction;
        p->setNetForce(p->getNetForce() + force);
    }

    // test against floor
    if (p->getPosition().y < FLOOR_HEIGHT + particleRadius) {
        distance = p->getPosition().y - (FLOOR_HEIGHT + particleRadius);
        force = -k * distance * vec3f(0.0, 1.0, 0.0);
        p->setNetForce(p->getNetForce() + force); // make jello bounce
    }

}

/**
 * Function to generate and apply force of wind
 *
 * @brief calculateWindForce
 * @param a_p
 * @param a_size
 * @param a_wind
 */
void calculateWindForce(vector<Particle*> *a_p, unsigned int a_size, vec3f a_wind) {
    unsigned int index = 0;
    unsigned int offset = a_size + 1;
    float area;
    vec3f F_wind;
    vec3f netVelocity;
    vec3f normal;
    Particle *vert1, *vert2, *vert3;

    if (length(a_wind) != 0) { // if a wind is blowing, calculate
        for (unsigned int i = 0; i < a_size; i++) {
            for (unsigned int j = 0; j < a_size; j++) {
                index = i * offset + j;

                // triangle 1
                vert1 = a_p->at(index);
                vert2 = a_p->at(index + 1);
                vert3 = a_p->at(index + offset);

                // calculate force of wind on triangle
                // get velocity of triangle
                netVelocity = vert1->getVelocity() + vert2->getVelocity() + vert3->getVelocity() / 3.0f;
                netVelocity = netVelocity - a_wind; // get net velocity

                // calculate triangle surface normal
                normal = cross( (vert2->getPosition() - vert1->getPosition()), (vert3->getPosition() - vert1->getPosition()) );
                area = 0.5f * length(normal); // get the area of the triangle
                area = area * dot( normalize(netVelocity), normal); // get the surface area exposed to the wind
                normal = normalize(normal);

                F_wind = -0.5f * AIR_DENSITY * (netVelocity*netVelocity) * COEFFICIENT_DRAG * area * normal;

                // divide force by 3 and assign to each particle
                F_wind = F_wind / 3.0f;

                F_wind = vec3f(0, 0, 0);

                // update the force on the particles
                vert1->setNetForce(vert1->getNetForce() + F_wind);
                vert2->setNetForce(vert2->getNetForce() + F_wind);
                vert3->setNetForce(vert3->getNetForce() + F_wind);

                // triangle 2
                vert1 = a_p->at(index + 1);
                vert2 = a_p->at(index + offset + 1);
                vert3 = a_p->at(index + offset);

                // calculate force of wind on triangle
                // get velocity of triangle
                netVelocity = vert1->getVelocity() + vert2->getVelocity() + vert3->getVelocity() / 3.0f;
                netVelocity = netVelocity - a_wind; // get net velocity

                // calculate triangle surface normal
                normal = cross( (vert2->getPosition() - vert1->getPosition()), (vert3->getPosition() - vert1->getPosition()) );
                area = 0.5f * length(normal); // get the area of the triangle
                area = area * dot( normalize(netVelocity), normal); // get the surface area exposed to the wind
                normal = normalize(normal);

                F_wind = -0.5f * AIR_DENSITY * (netVelocity*netVelocity) * COEFFICIENT_DRAG * area * normal;

                // divide force by 3 and assign to each particle
                F_wind = F_wind / 3.0f;

                // update the force on the particles
                vert1->setNetForce(vert1->getNetForce() + F_wind);
                vert2->setNetForce(vert2->getNetForce() + F_wind);
                vert3->setNetForce(vert3->getNetForce() + F_wind);

            }
        }
    }

}





////////////////////////////////////// HELPER FUNCTIONS //////////////////////////////////////
/**
 * create the triangles indices and stores it in an array
 *
 * @brief generateTriangles
 * @param a_p
 * @param a_t
 * @param a_size
 */
void generateTriangles(vector<Particle*> *a_p, vector<unsigned int> *a_t, unsigned int a_size) {
    a_t->clear();
    unsigned int index = 0;
    unsigned int offset = a_size + 1;

    // create indices for each triangle and push back
    for (unsigned int i = 0; i < a_size; i++) {
        for (unsigned int j = 0; j < a_size; j++) {
            index = i * offset + j;

            // triangle 1 of square
            a_t->push_back(index);
            a_t->push_back(index + 1);
            a_t->push_back(index + offset);
            // triangle 2 of square
            a_t->push_back(index + 1);
            a_t->push_back(index + offset + 1);
            a_t->push_back(index + offset);
        }
    }
}

/**
 * Update the jello mesh index positions
 *
 * @brief updateJello
 * @param a_p
 * @param a_mesh
 * @param a_size
 */
void updateJello(vector<Particle*> *a_p, TriangleSoupGeometry *a_mesh, unsigned int a_size) {
    a_mesh->triangles().clear();
    unsigned int index = 0;
    unsigned int offsetX = (a_size + 1) * (a_size + 1);
    unsigned int offsetY = a_size + 1;

    // loop through all x, y, and z coordinates of the jello
    for (unsigned int i = 0; i < a_size; i++) {
        for (unsigned int j = 0; j < a_size; j++) {
            for (unsigned int k = 0; k < a_size; k++) {
                index = i * offsetX + j * offsetY + k;
                if (i == 0) {
                    // left face
                    a_mesh->push_back(Triangle(Point1(a_p->at(index)->getPosition()),
                                               Point2(a_p->at(index + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetY)->getPosition())));
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + 1)->getPosition()),
                                               Point2(a_p->at(index + offsetY + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetY)->getPosition())));
                }
                if (i == a_size - 1) {
                    // right face
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + offsetX)->getPosition()),
                                               Point2(a_p->at(index + offsetX + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetX + offsetY)->getPosition())));
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + offsetX + 1)->getPosition()),
                                               Point2(a_p->at(index + offsetX + offsetY + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetX + offsetY)->getPosition())));
                }
                if (j == 0) {
                    // top face
                    a_mesh->push_back(Triangle(Point1(a_p->at(index)->getPosition()),
                                               Point2(a_p->at(index + offsetX)->getPosition()),
                                               Point3(a_p->at(index + offsetX + 1)->getPosition())));
                    a_mesh->push_back(Triangle(Point1(a_p->at(index)->getPosition()),
                                               Point2(a_p->at(index + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetX + 1)->getPosition())));
                }
                if (j == a_size - 1) {
                    // bottom face
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + offsetY)->getPosition()),
                                               Point2(a_p->at(index + offsetY + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetX + offsetY)->getPosition())));
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + offsetX + offsetY)->getPosition()),
                                               Point2(a_p->at(index + offsetX + offsetY + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetY + 1)->getPosition())));
                }
                if (k == 0) {
                    // back face
                    a_mesh->push_back(Triangle(Point1(a_p->at(index)->getPosition()),
                                               Point2(a_p->at(index + offsetX)->getPosition()),
                                               Point3(a_p->at(index + offsetX + offsetY)->getPosition())));
                    a_mesh->push_back(Triangle(Point1(a_p->at(index)->getPosition()),
                                               Point2(a_p->at(index + offsetY)->getPosition()),
                                               Point3(a_p->at(index + offsetX + offsetY)->getPosition())));
                }
                if (k == a_size - 1) {
                    // front face
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + 1)->getPosition()),
                                               Point2(a_p->at(index + offsetX + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetY + 1)->getPosition())));
                    a_mesh->push_back(Triangle(Point1(a_p->at(index + offsetX + 1)->getPosition()),
                                               Point2(a_p->at(index + offsetX + offsetY + 1)->getPosition()),
                                               Point3(a_p->at(index + offsetY + 1)->getPosition())));
                }
            }
        }
    }

}

/**
 * Update the cloth mesh coordinates for outputing to screen
 *
 * @brief updateCloth
 * @param a_p
 * @param a_mesh
 */
void updateCloth(vector<unsigned int> *a_t, vector<Particle*> *a_p, TriangleSoupGeometry *a_mesh) {
    a_mesh->triangles().clear(); // clear the current set of triangles

    // go through each triple in the triangles array
    for (unsigned int i = 0; i < a_t->size(); i += 3) {
        a_mesh->push_back(Triangle(Point1(a_p->at(a_t->at(i))->getPosition()),
                                   Point2(a_p->at(a_t->at(i + 1))->getPosition()),
                                   Point3(a_p->at(a_t->at(i + 2))->getPosition())));
    }
}

/**
 * Create springs that link to particles exclusively together
 *
 * @brief createSprings
 * @param a_p
 * @param a_s
 * @param a_minLen
 */
void createSprings(vector<Particle*> *a_p,
                   vector<Spring*> *a_s,
                   const float a_k,
                   const float a_b,
                   const float a_searchRadius,
                   const float a_len) {
    float distance; // distance between pi and pj
    float len;

    // loop through each particle and create springs as needed
    for (Particle *pi : *a_p)
        for (Particle *pj : *a_p)
            // calculate collision force and add it to net force of particle
            if (pi->getID() < pj->getID()) {
                //cout << "pi: " << pi->getID() << ", pj: " << pj->getID() << endl;
                distance = length(pi->getPosition() - pj->getPosition());

                if (distance <= a_searchRadius) { // add spring if true
                    len = a_len > 0.0f ? a_len : distance; // if min length isn't set, use the particles current distance apart

                    a_s->push_back(new Spring(len, // spring length
                                              pi->getID(), // particle 1 ID
                                              pj->getID(), // particle 2 ID
                                              a_k, // spring coefficient
                                              a_b) // spring damping
                                              );
                }
            }
}

/**
 * Go through and sum the net forces acting on each particle and then apply them to get the new positions
 * Overloaded function
 *
 * @brief updateParticles
 * @param a_p
 * @param a_s
 */
void updateParticles(vector<Particle*> *a_p, vector<unsigned int> *a_t, vector<Spring*> *a_s, float a_time, void (*const checkCollisions)(vector<Particle*>*, vector<unsigned int>*, Particle*)) {
    // update the spring force on both particles
    for (Spring *s : *a_s) {
        s->calculateSpringForce(a_p);
    }

    // update the particles force
    for (Particle *p : *a_p) {
        checkCollisions(a_p, a_t, p);
        // add air damping
        p->setNetForce(p->getNetForce() + (-AIR_DAMPING * p->getVelocity())); // add air damping to the total net force
        p->updateParticleForce(); // update particles force by adding gravity
    }

    // update particles position
    for (Particle *p : *a_p)
        p->updateParticlePosition(a_time); // update particle position based on accumulated forces
}

/**
 * Go through and sum the net forces acting on each particle and then apply them to get the new positions
 * Overloaded function
 *
 * @brief updateParticles
 * @param a_p
 * @param a_t
 * @param a_s
 * @param a_time
 */
void updateParticles(vector<Particle*> *a_p, vector<Spring*> *a_s, float a_time, void (*const checkCollisions)(vector<Particle*>*, Particle*)) {
    // update the spring force on both particles
    for (Spring *s : *a_s) {
        s->calculateSpringForce(a_p);
    }

    // update the particles force
    for (Particle *p : *a_p) {
        checkCollisions(a_p, p);
        // add air damping
        p->setNetForce(p->getNetForce() + (-AIR_DAMPING * p->getVelocity())); // add air damping to the total net force
        p->updateParticleForce(); // update particles force by adding gravity
    }

    // update particles position
    for (Particle *p : *a_p)
        p->updateParticlePosition(a_time); // update particle position based on accumulated forces
}

/**
 * Go through and sum the net forces acting on each particle and then apply them to get the new positions
 * Overloaded function
 *
 * @brief updateParticles
 * @param a_p
 * @param a_s
 * @param a_time
 */
void updateParticles(vector<Particle*> *a_p, vector<Spring*> *a_s, float a_time) {
    // update the spring force on both particles
    for (Spring *s : *a_s) {
        s->calculateSpringForce(a_p);
    }

    // update the particles force
    for (Particle *p : *a_p) {
        // no collision detection for this
        // add air damping
        p->setNetForce(p->getNetForce() + (-AIR_DAMPING * p->getVelocity())); // add air damping to the total net force
        p->updateParticleForce(); // update particles by adding gravity
    }

    // update particles position
    for (Particle *p : *a_p)
        p->updateParticlePosition(a_time); // update particle position
}

/**
 * go through and reset the particles and springs array
 * Overloaded funtion
 *
 * @brief resetParticles
 * @param a_p
 * @param a_s
 */
void resetParticles(vector<Particle*> *a_p, vector<Spring*> *a_s, unsigned int arg, void (*const init)(vector<Particle*>*, vector<Spring*>*, unsigned int)) {
    for (Particle *p : *a_p) // delete memory of particles
        delete p;
    a_p->clear();
    for (Spring *s : *a_s) // clear the springs
        delete s;
    a_s->clear();

    init(a_p, a_s, arg); // re initialize the particles and springs
}

/**
 * go through and reset the particles and springs array
 * Overloaded funtion
 *
 * @brief resetParticles
 * @param a_p
 * @param a_s
 */
void resetParticles(vector<Particle*> *a_p, vector<Spring*> *a_s, void (*const init)(vector<Particle*>*, vector<Spring*>*)) {
    for (Particle *p : *a_p) // delete memory of particles
        delete p;
    a_p->clear();
    for (Spring *s : *a_s) // clear the springs
        delete s;
    a_s->clear();

    init(a_p, a_s); // re initialize the particles and springs
}

/**
 * Delete the meory of each particle element and spring element and then delete the memory of each array
 *
 * @brief cleanup
 * @param a_p
 * @param a_s
 */
void cleanup(vector<Particle*> *a_p, vector<Spring*> *a_s) {
    for (Particle *p : *a_p)
        delete p;
    delete a_p;
    for (Spring *s : *a_s)
        delete s;
    delete a_s;
}

