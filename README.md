README for Model Predictive Controller Simulator
Overview
This repository contains the code for the Model Predictive Controller simulation project, including the Model Predictive Control (MPC) system for a car-like robot. The project involves developing a simulation environment where the robot follows a path while avoiding obstacles using a dynamic model.

Project Structure
src/: Contains the Python scripts for the MPC and related functionalities.
mpc_node.py: Main ROS node for MPC control.
mpc_controller.py: MPC controller implementation.
plot_error.py: Script for plotting error metrics.
launch/: Contains ROS launch files.
mpc.launch: Launch file to start the simulation with the MPC controller.
Dockerfile: Dockerfile for building the Docker image used for the simulation.
CMakeLists.txt: CMake configuration file for building the project.
package.xml: ROS package manifest file.
Installation
Clone the Repository

bash
Copy code
git clone https://github.com/victorumobi/mpc-controller-simulation.git mpc-controller-simulation
cd mpc-controller-simulation
Install Dependencies

Ensure you have ROS Noetic installed. You can install the required dependencies using:

bash
Copy code
sudo apt-get update
sudo apt-get install python3-catkin-tools
Build the Workspace

bash
Copy code
mkdir -p ~/gem_ws/src
cp -r mpc-controller-simulation ~/gem_ws/src/
cd ~/gem_ws
catkin_make
Source the Workspace

bash
Copy code
source devel/setup.bash
Running the Simulation
Launch the MPC Simulation

bash
Copy code
roslaunch mpc-controller-simulation mpc.launch
Interact with the Simulation

Use the provided ROS topics and services to interact with and monitor the simulation. Refer to the mpc_node.py and mpc_controller.py for details on available topics and services.

Docker Setup
Build the Docker Image

bash
Copy code
docker build -t mpc-controller-simulation .
Run the Docker Container

bash
Copy code
docker run -it --rm --name mpc-controller-simulation mpc-controller-simulation
Submission Guidelines
Prepare the Deliverables

Ensure your repository includes:

Updated code in the src/ directory.
Any additional scripts or files as required by the assignment.
A Dockerfile for containerization.
Update README

Ensure the README file is up-to-date with:

Installation and setup instructions.
Details on how to run and interact with the simulation.
Create a Zip Archive

Zip your entire workspace directory, ensuring the src/, launch/, and any other relevant directories are included.

bash
Copy code
cd ~/gem_ws
zip -r mpc-controller-simulation_submission.zip src launch Dockerfile CMakeLists.txt package.xml
Submit Your Assignment

Upload the ZIP file to the provided submission portal.
Ensure any additional requirements or forms are filled out as per your assignment guidelines.
Contact
For any questions or issues, please contact:

Name: Victor Umobi
Email: victorumobi@gmail.com
