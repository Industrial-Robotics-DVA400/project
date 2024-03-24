# Inverse dynamics control of 6-DOF manipulator in operational space
## Initialization 
The following steps need to be executed to initialize in preperation of simulation:
    + Open file "IDC\_OS.m"
    + Execute sections:
        - Clean up
        - Define robot
        - Matrices
        - Define positions (via points)

## Simulink Simulation
To run the simulink simulation of the Inverse dynamics controller, do one of the following:
    + Main approach: open file "IDC\_OS.slx" and execute simulation through simulink by pressing "Simulate model" (ctrl+T)
    + Alternative approach: run section "Perform simulation in simulink" in the "IDC\_OS.m" file

\section{Visualize Output}
To visualize the system output and motion, execute the following sections in the file "IDC\_OS.m":
    + Create figure 
        - Only needs to be executed one time in the beginning
    + Draw/redraw
        - Execute this section to visualize the motion
        - Execute this section again to redo the visualization
