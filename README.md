# Polytechnique-Internship
This repository contains both the MATLAB code and report for my work on the development on the MAVion convertible drone and Crazyflie swarm drones. 

The work on the convertible tail-sitting drone involved the design and implementation of a lateral control system for the MAVion drone. This involved checking if decoupling was possible for all flight velocities, and then designing a suitable controller that would allow all saturation limits to be met (eg. actuator values such as elevon deflection were not pushed beyond their limits). An LQR controller was chosen, however future work would involve the design of an H infinity controller. 

The second part of this internship involved the documentation and setup of the Bitcraze Crazyflie drones. This involved working with the Loco Positioning System (LPS), allowing a swarm of drones to be programmed and controlled. The VICON system was also used to position the anchor nodes and reflective VICON balls were then mounted to the Crazyflie and the trajectory for each flight was saved on the VICON Tracker and exported to a CSV file. This then allowed the flight path to be analysed as a 3D plot in MATLAB. 
