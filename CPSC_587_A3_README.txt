//---------------------------------------------------------------------
// Filename: README.txt
//
// CPSC 587, Assignment 3
// Glenn Skelton
//---------------------------------------------------------------------


**INSTALL**

Load up in QT Creator and setup in release mode. Other modes will casue
the program to lag. This program was developed on a Linux environoment
using the graphics lab computer.


**USER INTERFACE**

All calls originally part of the boilerplate for GIVR

In addition, the following key calls have been added

F - Maximize / Original Size of screen
1 - Part 1
2 - Part 2
3 - Part 3
4 - Part 4
5 - Bonus 1
6 - Bonus 2

To modify constants like air damping or the time step, look under the GLOBAL
CONSTANTS header at the top of the main.cpp file.

To modify any the cloth, jello, or chain number of particles look under the
create geometry for each given part.

To modify the particles/springs specifically such as spring constants and 
damping coefficient, look under the function for each part names ie. 
part1_Init() etc.


**RESOURCES**

I used the knowledge which I have gained from the lecture notes and tutorial
notes in CPSC 587 and CPSC 599 (Haptics). As well I used a technique outlined
in the lecture slides "Cloth Simulation: CSE 169: Computer Animation" by
Steve Rotenburg. I also used values for air denisty and my drag coefficient
from Wikipedia.


**NOTES**

For part 4 I decided for fun to model m draping cloth with a bit of wind to
add some interest to the scene.


**BONUS 1**

For simulating the wind I used the equation for air resistance and applied it
to each particle making up a triangle in the mesh being effected. What the
formula does is take the surface area of the object and a drag and air density 
coefficient which is multiplied by the relative velocity squared. The formula
used goes one step further by taking a dot product between the triangles 
surface normal and the relative velocity to only apply the force along the area
of the triangle that is exposed directly to the wind. As such this formula
applies a steady force on the area exposed on the triangle which will vary
depending on the area that is actually exposed which will cause the rippling 
effect that is charactersitic of a flage flapping in the wind.

The formula is as follows and was borrowed from Steve Rotenburg's lecture 
slides.

F = 0.5 * airDensity * velocity^2 * drag * surfaceArea * -surfaceNormal

surfaceArea = 0.5 * 
			  cross(vert2 - vert1, vert3 - vert1) * 
			  dot(normalize(velocity), surfaceNormal)

The velocity is an average of all three vertex velocities and has the air 
velocity subtracted from it to get the relative velocity.

The cross product divided by 2 gives the triangle area and the dot product of 
the velocity and surface normal gives the area that is exposed to the wind.

The force calculated from this equation is then divided by 3 to split the work
acted on each triangle to be 1/3 whic is added to each particles force 
accumulator.


**BONUS 2**

For simulating a cloth falling on an object (which in the case of my assignment
I decided to model a ball instead of a table out of aesthetic) I applied Hookes
law in the same manner to which was applied in the other parts to create a 
repelling force of the cloth to the ball and floor. Looping through eaach particle
I tested to see whether the distance between the particle and the ball was less than
their combined radii. If this is the case, I calculated the interpenetration distance
of the particle to the sphere to get my delta x value and multiplied by a constant,
k which is the spring constant. All of this is multiplied by a unit vector that is
calculated by taking the vector between the centers of both the particle and the 
ball.

direction = particlePosition - ballPosition
force = -k * (length(direction) - (particleRadius + ballRadius) * normalized(direction)

The result is that the ball repels the cloth proportionatly to the penetration depth
and makes the cloth sit on the surface of the ball nicely. The folds were created from
the structural, shear, and bend springs contained in the cloth particles.

If this were a table though, I would use the same technique but test each particle against
each surface of the table and, based on the surface normal of the part of the table being
touched, create a spring force in the normal direction that would repel the cloth to
sit on the surface.




