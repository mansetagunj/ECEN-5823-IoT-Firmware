# IoT Firmware BLE Client for Wireless Health Thermometer project
 
The folder consists of the a BLE Client used to read temperature reading from the Wireless Health Thermometer BLE Project 
#foreverbuff 
#gobuffs

Feel free to use and modify the project keeping in mind the licensing and mentions about the creators.
The BLE_Client.zip can be directly imported into the Silicon Labs Simplicity Studio using the Import option in the File menu.
This project makes use of the Silicon Labs example program and the BLE SDK + Platform, so make sure you have installed them.

The BLE Client project flushes out data using UART and the python script "temperaturePlotter.py" is used to plot the temperature data in real time.
The plotter looks like:
![Plotter Image](https://github.com/mansetagunj/ECEN-5823-IoT-Firmware/blob/master/Wireless%20Health%20Thermometer%20BLE%20Client/Plots_snap.png)

The BLE Client LCD display:
![Plotter Image](https://github.com/mansetagunj/ECEN-5823-IoT-Firmware/blob/master/Wireless%20Health%20Thermometer%20BLE%20Client/BLE_Client.jpg)

The BLE Server LCD display:
![Plotter Image](https://github.com/mansetagunj/ECEN-5823-IoT-Firmware/blob/master/Wireless%20Health%20Thermometer%20BLE%20Client/BLE_Server.jpg)

Tools and development boards used: 

1. Simplicity Studio V4.1.25
2. Silicon Labs BLE SDK(2.9.2.0) + Platform + Example codes (using GNU ARM v7.2.1 toolchain)
3. Silicon Labs EFR32BG13 + Base board

Special credits to Prof. Keith Graham for teaching this wonderful subject and covering the concepts of IoT Firmware especially BLE and Bluetooth Mesh.

Note: There are some Licence mentions in the Code for the software IP of Silicon Labs.




