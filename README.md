# raytracing
C++ implementation of basic ray tracer simulating the propogation of the light in vacuum.
Program takes a XML file and renders the scene. XML file contains information about materials, geometry, lights and cameras.

# How to run
In the source directory run this command in a terminal:

$make

After that, a scene is given to the program as follows:

$./raytracer path_to_input.xml

Program saves the output(s) as ppm file(s) in the same directory with the source code files.
