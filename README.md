In order to properly execute the code and perform the various simulations and deployments, it is recommended to add all the project folders to the Matlab path.

This project folders are divided as follows:
- `common`: in this folder are the files to read from the encoder with and without filters, and the files to use the alternative mode of execution on the microcontroller, communicating directly through the UART. It is strongly recommended to add this folder to the Matlab path.
- `models`: contains all the models used in this project.
- `identification`: folder in which are the files to perform the identification of the motor model.
- `position_control`: this folder contains all the scripts and patterns for position control. It also contains the direct coding files.
- `torque_control`: this folder presents all the scripts and schematics for torque control. It also contains the direct coding files.
- `admittance_control`: this folder presents all the scripts and schemes for admittance control.
- `SIL_PIL`: SILs and PILs of the position and torque control.
- `utils`: this folder has files for making plots using the data coming from the hardware.
