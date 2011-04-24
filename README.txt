Authors: Ruben, Kevin
Logins: cs184-cv, cs184-cj

Title:
Populated Möbius Strip

Proposal:
We would like to recreate a famous M.C. Escher painting, possibly with some modern twists.

Summary:
We would like to generate an interactive image of a mobius strip floating floating in space upon which red ants (or possibly bipedal creatures) are walking (similar to M.C. Escher's famous woodcutting "Möbius Strip II").

Details:
We will animate the models by creating an algorithm that will provide coordinates for the creatures to walk across the mobius strip. The coordinates will change based on the position of the creature on the mobius strip. The coordinates will be used to move the legs of the creature.
The mobius strip will be a textured sweep similar to that in assignment 8, but textured to appear like the strip in the Escher painting (or possibly have some cooler texture, if we devise one). We will simulate movement of ants over the surface using inverse kinematics to determine the placement of feet on the surface. The scene will have two camera angles, first and third person, that the user can switch between. The first person camera angle will be from the perspective of an ant, and the user will be able to control the ant's direction of movement with either the arrow keys or mouse. Additionally, we may introduce additional elements to the scene if we have time, such as mesh bodies for the ants, or reflective surfaces for the mobius strip.

Potential challenges (may choose to do more than 2 of these):
1).  Create a texture mapped mobius strip so that the ants/bipeds can walk infinitely along a single surface.
2).  Realistically determine placement of the creatures' feet using inverse kinematics so that they appear to be walking along the surface of the strip.
3).	 Allow the user to have interactive control of the direction of the creatures using the mouse/arrow keys without having to explicitly place each foot of the creature.
4).	 Skin the creatures with a mesh rather than a body constructed of OpenGL primitives.
5).  Added texture mapping to the creatures so that they have non-monochromatic features.
6).  Have both multiple cameras so that the user can switch between first person (as one of the ants/bipeds) and third person.


Image Explanation:
The image is of a preliminary mobius strip and a ladybug mesh that we will try
to animate walking along the strip.
