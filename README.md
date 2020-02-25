# Localization of a Kidnapped Vehicle using Particle Filters

## Writeup

This writeup is on the Udacity Kidnapped Vehicle Project (as part of the Self-Driving Car Nanodegree) in which a Kidnapped Vehicle is localized using Particle Filters. 

[//]: # (Image References)

[image1]: ./writeup_images/bayes_filter_equation.png "Bayesian Filter Equation"
[image2]: ./writeup_images/bicycle_model_equations.png "Bicyle Model Equations"
[image3]: ./writeup_images/mvgauss_equation.png "Multivariate Gaussian Probability"
[image4]: ./writeup_images/homogenous_transformation_equation.png "Homogenous Transformation Equation"
[image5]: ./writeup_images/flowchart_particle_filter.png "Flowchart for a Particle Filter"

### Project Introduction

Here is the scenario:

```A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.```

In this project, a 2 dimensional particle filter is implemented in C++ to localize the kidnapped vehicle. The particle filter is given a map and some initial localization information (similar to GPS). At each time step, the filter gets observation and control data.

### Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

It also requires installation of uWebSocketIO, cmake, make, gcc/g++. For installation instructions and help refer this [link](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/README.md) from a previous project. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively, some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

### Description of a Particle Filter

A particle filter is a **type of a Bayesian filter** that uses a set of particles (which are nothing but random samples) to represent the state of a system (in this case, the position and orientation of a robot). The current state is found by finding the **posterior distribution of these samples, given some noisy observations** (which here are lidar measurements of certain landmarks on the map). It uses the **Markovian assumption** and hence requires information only on the previous state and the current state. 

It requires a map inorder to calculate the posterior distributions. The posterior distribution is found using the prior and observations with the help of Bayes rule and Markov assumption according to the following formula:

![alt text][image1]

The **prior distribution** can be uniform or normally distributed around a GPS estimate for the initial step, and for the subsequent step, it can be found using the process model (the bicycle model, in this case) and the control inputs (velocity and yaw rate). This is called the **prediction step**. The equations for the bicycle model used in the prediction step are given below (Image source: [Udacity classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/28233e55-d2e8-4071-8810-e83d96b5b092/lessons/e5de5b5a-d706-40ae-902c-ba113b088de7/concepts/ff7658c9-6edd-498b-b066-1578ec3f97aa)):

![alt text][image2]

The **likelihood or the probability of observations given the state** is found using the sensor measurements of the distances to some landmarks around the robot. The likelihood of each particle observing all these landmarks at these distances is calculated by taking a product of the multi-variate Gaussian probabilities for each observed landmark. This product or combined likelihood is the weight of each particle, that tells how likely is the combination of these measured observations given the current position and orientation of the particle. This process of calculating the weights is called the **update step.**

The equations for the multivariate Gausian probability used to update the weights are as follows (Image source: [Udacity classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/28233e55-d2e8-4071-8810-e83d96b5b092/lessons/e3981fd5-8266-43be-a497-a862af9187d4/concepts/0d4fb3b6-7db0-4bca-b88e-fceebf8373ed)):

![alt text][image3]

The measurements are made in the vehicle coordinate system, and these need to be transformed into the global or map coordinate system before updating the weights. The transformation is done using a rotation and a translation and can be found using the below equations (Image source: [Udacity classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/28233e55-d2e8-4071-8810-e83d96b5b092/lessons/e3981fd5-8266-43be-a497-a862af9187d4/concepts/97f45106-adf4-45f1-b919-9675b3c990e0)):

![alt text][image4]
where the variables with m as subscript are the x,y positions in map coordinates; theta and the variable with p as subscripts are the orientation and x,y positions of the particle; and the variables with c as subscript are the x,y positions of the landmark as measured from the car/vehicle in the vehicle's coordinates.

The **marginal likelihood** is calculated by taking a sum of these weights and normalizing the weights of the individual particles inorder to get the posterior distribution of these particles. This is done in the **resampling step.** During resampling, we draw particles with replacement with a probability proportional to the weights. The number of particles drawn is same as that in the initial step.

This above sequence of steps is carried out in a cyclic manner for localizing the robot or vehicle. This can be represented using a flowchart as shown below:

![alt text][image5]


### Inputs to the Particle Filter

#### The Map
This can be found in the `data` directory. `map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

#### Location Initialization Data 
The initial noisy position data (similar to GPS) is given by the simulator with the following values:

["sense_x"]

["sense_y"]

["sense_theta"]

#### Control Data
The previous velocity and yaw rate are given by the simulator to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

#### Observation Data - Landmark measurements
A set of noisy observation data is obtained from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


### Implementation of the Particle Filter - Description of the Code

The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```
The code can be found in the `src` directory. Only the `particle_filter.cpp` file needs to be filled for the project. The remaining files are already filled and provided. Here is an brief explanation of each of the code files in the directory:

1. `main.cpp` - This file contains the code that will actually run the particle filter and calls the associated methods. It contains code to connect with and interface with the simulator using uWebSocketIO. It reads in the initial positions and orientation, previous control data such as velocity and yaw_rate and the sensor measurements of the observations(landmarks) and passes them to the respective function in `particle_filter.cpp`. It also contains code to read in the map data. Finally, the `main.cpp` find out the best particle from the particle filter by finding the particle with the highest weight. It then forwards the position and orientation of the best particle and some optional debugging information such as sensor observation associations and xy data to the simulator using json messages. 

2. `map.h` - This file contains the definition of the Map class, which has a vector of landmarks as its attributes.

3. `helper_function.h` - This file has definitions of struct data types used for the landmark observations, control parameters and ground truth values. It also has function definitions for calculating distances and errors and reading in map data, control data, ground truth data and observed landmark data.

4. `particle_filter.h` - This file contains the definitions for the Particle Filter class, its attributes such as the struct for the 'particle' data type, the vector of particles, the number of particles, boolean flag for initialization status and the member functions of this class.

5. `particle_filter.cpp` - This file contains the main code for the particle filter with the following functions that are executed roughly in the same order (with the exception of (d) and (f) which are called from the other functions):

    (a) `init()` - Used to initialize the particle filter by initializing particles to a Gaussian distribution around the initial position and setting all the weights to 1.
    
    (b) `prediction()` - Used to predict the state for the next time step using the process model, which here is the bicycle model, by taking in control data such as previous state velocity and yaw rate as inputs.
    
    (C) `dataAssociation()` - Used to find the actual landmarks on the map that are closest to the observations/measurements of the landmarks (as estimated from the particles' predicted positions and orientation).
    
    (d) `updateWeights()` -  Used to updates the weights for each particle based on the likelihood of the observed measurements. This is calculated by taking a product of the individual multi-variate Gaussian probabilities of the position for each of the observed landmarks. The observed landmarks are transformed into the map coordinates before calculation of the probabilities, since the actual landmarks are stored in the map coordinates.
    
    (e) `resample()` - Used to resample from the existing set of particles with probabilities based on the weights assigned to the particles, to form the new set of particles.
    
    (f) and other already filled functions either used as helper functions or for debugging, viz.`SetAssociations()`, `initialized()`,`getAssociations()` and `getSenseCoord()`.
    
    
### Output from the Particle Filter

These values are provided by the c++ program to the simulator for displaying the output, calculating the errors and also for debugging.

#### Best Particle Values
The following best particle values are used for the error evaluation:

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

#### Best Particle Sensing and Associations
This is optional message data used for debugging the particles' sensing and associations.

Landmark IDs of the sensed observations: 

["best_particle_associations"]

Respective (x,y) values of the sensed observations:

["best_particle_sense_x"] 

["best_particle_sense_y"] 


### Project Output

The grading code looks for Accuracy (such that the vehicle position and yaw is localized within specified values) and Performance (such that the execution completes within 100s.)

The particle filter passes the current grading code in the simulator and it shows the output
```
Success! Your particle filter passed!
```.