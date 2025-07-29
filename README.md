# UR_summer2025
Summer 2025 work by Leia Hannes and Hannah Loly. 

**UR_library_2025** is a modified version of myur3e.py (https://github.com/tuftsceeo/Universal-Robots-ROS2-CEEO/blob/main/myur/myur3e.py) and includes base classes used in both the clay and violin projects. 

**clay_commands** has two classes, ClaySculpt and ClayTrajectory, that inherit from the MyUR3e and Trajectory classes found in UR_library_2025. 

**lib_testing** includes tests for UR_library commands and clay_commands. 

**play_doh** is our Jupyter notebook code for the Hackathon. This uses the libraries prior to class implementation and is slightly out of date as a result. 

**impact_commands** has two classes, ImpactTesting, and ImpactTrajectory, that inherit from the MyUR3e and Trajectory classes found in UR_library_2025. 

**websocket** has classes and functions to start the websocket in Jupyter and read data from and send to channels

**Websocket_Startup** is the Jupyter file that needs to be kept running to access channels in Jupyter

**Websocket_Example_Code** goes through the different classes and functions in websocket.py

**Impact_Testing** is the Jupyter page that runs the full impact test

**LEGO_controller** is the Jupyter page for integrating LEGOs with the arm

**RobotControlPanel.zip and DemoRobotControlPanel.zip** are the pyscript pages for controlling the robot
