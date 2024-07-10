# Pre-requisites
In order to properly execute the code and perform the various simulations and deployments, it is recommended to add ALL the project folders and subfolders to the Matlab path.

# Project overview
This project is organized as follows:
- `common`: in this folder are the Simulink block to read the encoder data with two different techniques, as well as the files we used for directly communicate with the microcontroller via UART, bypassing the limitations of XCP. It is strongly recommended to add this folder to the Matlab path;
- `models`: contains all the models used in this project as well as the hardware interface Simulink block;
- `identification`: folder in which are the files to perform the identification of the motor speed model;
- `position_control`: this folder contains all the Matlab scripts and Simulink models for position control. It also contains the direct coding files;
- `torque_control`: this folder presents all the Matlab scripts and Simulink models for torque control. It also contains the direct coding files;
- `admittance_control`: this folder presents all the Matlab scripts and Simulink models for the admittance control;
- `SIL_PIL`: SILs and PILs of the position and torque controllers;
- `utils`: this folder has files for making plots of the data coming from the hardware.

# Authors
- Antonio Langella
- Salvatore Paolino
