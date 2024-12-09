The code provided includes our overlay and configuration files that are specific to the nRF9161 DK. Other boards have their own overlay and config files but the main.c file 
should work regardless of the board being used. To find files for your board visit this github: https://github.com/NordicDeveloperAcademy/ncs-fund/tree/main
The files specific to this application should be found in the directory "v2.8.x-v2.7.0/l6/l6_e1_sol" and will require a small amount of editing to enable I2C functionality.
To better understand what the config and overlay files are doing in this application we recommend completing the basic Nordic DevAcademy course lesson 6.
