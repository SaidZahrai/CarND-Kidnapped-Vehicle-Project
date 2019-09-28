This is a project in Udacity Nanodegree for Self-driving car, forked from [CarND-Kidnapped-Vehicle-Project](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).


# Particle Filter
The goal of this project is to implement a particle filter and follow a car in a 2D simulation. Following tasks have been completed:

1. particle_filter.cpp is completed according to the instructions given in the file and steps as described in the lesson.
2. Two files, multiv_gauss.h and multiv_gauss.cpp, are taken from Tiffany's lessons for calculation of weights and added to src directory.
<p align="center">
<img src="./images/src.png" width="600"/>
</p>
3. These two files are added in the list of source files in CMakeLists.txt so that they will be compiled and linked.
<p align="center">
<img src="./images/cmakelists.png" width="600"/>
</p>

After the above changes, the executable can be created by use of cmake and the build.sh provided in the project as shown below.

<p align="center">
<img src="./images/make.png" width="600"/>
</p>

Once the executable is available the particle filter applicaiton can be launched using the run.sh script available in the project. Launching the application, will start the filter and wait for connection from the simulator. The connection will be established once the simulator is launched, as shown below.


<p align="center">
<img src="./images/simulator.png" width="600"/>
</p>

<p align="center">
<img src="./images/run.png" width="600"/>
</p>

Pushing the start button will start the simulation. You will see that the car is accurately followed and the simulation ends with the final message of having uccessfully passed.

<p align="center">
<img src="./images/simulation.png" width="600"/>
</p>
 
<p align="center">
<img src="./images/particle_filter.png" width="600"/>
</p>
 
