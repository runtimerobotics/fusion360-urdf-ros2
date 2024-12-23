# Autodesk Fusion 360 to URDF for ROS 2

## Table of Contents
- [Introduction](#introduction)
- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
  - [Start Modeling a Sample Robot](#start-modeling-a-sample-robot)
  - [Converting Fusion 360 Model to URDF for ROS 2](#converting-fusion-360-model-to-urdf-for-ros-2)
  - [Visualizing Robot in Rviz](#visualizing-robot-in-rviz)
  - [Launching Robot Simulation in Gazebo Sim](#launching-robot-simulation-in-gazebo-sim)
- [Limitations](#limitations)
- [Contributing](#contributing)
- [License](#license)
- [Credits](#credits)
- [Conclusion](#conclusion)

## Introduction
The "Autodesk Fusion 360 to URDF for ROS 2" project aims to bridge the gap between CAD modeling and robotic simulation. 
This tool allows users to convert their Autodesk Fusion 360 models into Unified Robot Description Format (URDF) files, which can be used in ROS 2 (Robot Operating System) for visualization and simulation.
 
By providing a seamless workflow from design to simulation, this project enables roboticists and engineers to visualize, test, and iterate on their robot designs more efficiently. Whether you are developing a new robot or refining an existing one, this tool simplifies the process of integrating your CAD models into the ROS 2 ecosystem.

## Description
This project is an Add-In script for Autodesk Fusion 360 to export 3D models to a robot description package which contains URDF, Mesh files (.stl),
launch files for visualization and simulation etc. to make it work with ROS 2 (Tested in Jazzy and Humble)

Here are the list of features of the project

### Features
- **Seamless Conversion to URDF**: Easily convert Autodesk Fusion 360 models into URDF files compatible with ROS 2.
- **Visualization**: Visualize your robot models in Rviz using the launch file automatically created after conversion.
- **Simulation**: Launch and test your robot simulations in ROS 2 Humble with Gazebo Classic and ROS 2 Jazzy with latest Gazebo Sim.
- **User-Friendly Workflow**: Provides a straightforward workflow from CAD design to robotic simulation.
- **Sample Models**: Includes instructions and examples for modeling sample robots.
- **Customization**: Supports customization of URDF files to match specific robot configurations and requirements.
- **Documentation**: Comprehensive documentation to guide users through installation, usage, and troubleshooting.

By leveraging this tool, users can bridge the gap between CAD modeling and robotic simulation, making it easier to develop and refine robotic systems within the ROS 2 ecosystem.

### System Requirements

Here is the platform we used to test this plugin.

- **Operating System**: Windows 11 x64
- **Software**: Autodesk Fusion 360 2.0.20981 x86_64

## Installation

Here are the steps to install the script in Fusion 360

* **Step 1**: Download repository as Zip file and extract the file to a location.

* **Step 2**: Open Fusion 360, and press *Shift+S*, this will show the Scripts and Add-Ins like shown below

<p align="center">
  <img src="img/scripts_ui.png" alt="Scripts and Add-Ins UI">
</p>

Click on the *Green +* icon and browse the extracted script folder as shown below. 
The folder we have to browse is *Fusion_URDF_Exporter_ROS2*. 

<p align="center">
  <img src="img/script_location.png" alt="Script Location">
</p>

After selecting the folder, it will show the new script as *Fusion_URDF_Exporter_ROS2* under *My Scripts*.

* **Step 3**: Press Shift+S to see the *Scripts and Add-Ins Window* (Utilities -> ADD-INS)

<p align="center">
  <img src="img/fusion360_script.png" alt="Fusion 360 Script Location">
</p>

The installation is successfull if you are seeing the script under *My Scripts*.

## Usage
### Start Modeling a Sample Robot
Instructions on how to start modeling a sample robot using Autodesk Fusion 360.

### Converting Fusion 360 Model to URDF for ROS 2
Steps to convert a Fusion 360 model to URDF for ROS 2.

### Visualizing Robot in Rviz
Guide on how to visualize the robot in Rviz.

### Launching Robot Simulation in Gazebo Sim
Instructions on how to launch the robot simulation in Gazebo Sim.

## Limitations
List any limitations or known issues with the project.

## Contributing

Always welcome bug fixes, new features etc.

* **Create a Pull Request**: Ceate a pull request from your forked repository. Provide a clear description of the changes and any relevant information. 


## License

This project is licensed under the terms of the [MIT License](LICENSE).


## Credits
This project is an updation of repositories from [syuntoku14](https://github.com/syuntoku14/fusion2urdf) and [dheena2k2](https://github.com/dheena2k2/fusion2urdf-ros2). 


## Conclusion

In conclusion, the "Autodesk Fusion 360 to URDF for ROS 2" project provides a powerful and user-friendly tool for roboticists and engineers. By streamlining the process of converting CAD models to URDF files, this tool enhances the efficiency of robot design, visualization, and simulation within the ROS 2 ecosystem. We hope this project will significantly contribute to your robotic development efforts and look forward to your feedback and contributions.