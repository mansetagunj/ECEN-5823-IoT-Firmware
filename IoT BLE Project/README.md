# IoT Bluetooth Low Energy  

The folder consists of the Bluetooth Low Energy project developed during my course "ECEN 5823 IoT Embedded Firmware" at University of Colorado, Boulder.   
#foreverbuff   
#gobuffs

**Feel free to use and modify the project keeping in mind the licensing and mentions about the creators.** 

The  BLE_Thermometer.zip can be directly imported into the Silicon Labs Simplicity Studio using the Import option in the File menu.  
This project makes use of the Silicon Labs example program and the Bluetooth SDK + Platform, so make sure you have installed them.  

Tools and development boards used: 

1. Simplicity Studio V4.1.11
2. Silicon Labs Bluetooth SDK + Platform + Example codes
3. Silicon Labs BGM121 + Base board + Expansion board having BMA280 accelerometer and Joystick
4. Silicon Labs Blue Gecko application 
    Android App Link: https://play.google.com/store/apps/details?id=com.siliconlabs.bledemo&hl=en
    iOS App Link : https://itunes.apple.com/us/app/silicon-labs-blue-gecko-wstk/id1030932759?mt=8

**Special credits to** ***Prof. Keith Graham*** **for teaching this wonderful subject and covering the concepts of IoT Firmware especially BLE and Bluetooth Mesh.**

## Objective:   
To take the temperature measured by the on board Si7021 and communicate it via BLE to the Silicon Labs’ BlueGecko iPhone or Android phone app. 

## Configuration and Functionality: 

### BLE service used:   
Services :  
1. Health Thermometer
  Characteristics:   
    a. Temperature Measurement
    b. Temperature Type
2. Tx Power  
  Characteristics:   
    a. Tx Power Level
3. Start Temperature Sensor Service (Custom Service)  
  Characterictics:  
    a. Start Temperature Sensor Control Point  
4. Silicon Labs OTA service (Custom Service)  
  Characterictics:  
    a. Silicon Labs OTA control
    b. Silicon Labs OTA data
    c. Silicon Labs OTA BLE stack version
    d. Silicon Labs OTA version
  **Note: The application supports the OTA update via the Silicon Labs Blue Gecko Android/iOS application. The steps to follow are mentioned below in Section OTA Update.

### To maximize energy savings, the Bluetooth application should change its advertising, connection interval, and slave interval to what is appropriate to the application.
a. The Advertising min and max is 500mS  
b. Connection Interval minimum and maximum is 75mS    
c. Slave latency to enable it to be off the “air” is up to 375mS  

### To further maximize energy savings, the application should automatically adjust its transmit power based on the proximity of the master / client, the phone. Please use the following settings:
a. If rssi > -35db, set tx_power to BGM121 TX Min i.e. -26db  
b. If -35db > rssi > -45db, tx_power is -20db  
c. If -45db > rssi > -55db, tx_power is -15db  
d. If -55db > rssi > -65 db, tx_power is -5db  
e. If -65db > rssi > -75db, tx_power is 0db  
f. If -75db > rssi > -85db, tx_power is 5db  
g. If rssi < -85db, tx_power is BGM121 TX Max i.e. 8db  
h. Upon reset or a Bluetooth connection disconnect, TX Power is set to 0db  

### Joy Stick (Used ADC):     
a. If Joy Stick is pressed away (north), enable the BMA280 (put in normal mode)    
b. If Joy Stick is pressed toward you (south), disable the BMA280 (suspend mode)    
c. If Joy Stick is pressed to the right (east), temperature set point is increased by 5 degrees C    
d. If Joy Stick is pressed to the left (west), temperature set point is decreased by 5 degrees C    

### BMA280 functionality (Used SPI for interfacing the sensor):  
a. Upon power on reset or the Blue Gecko reset, the BMA280 should be in SUSPEND mode  
b. Single tap should enable the I2C sensor, Load Power Management ON, and no affect to LED1  
c. Double tap should disable the I2C temp sensor, Load Power Management OFF, and no effect on LED1  

### LETIMER0 is set to the following conditions at startup / reset.  
a. Period = 1.75 seconds
b. No period, so no need to have the LETIMER0 to interrupt twice per period
c. During the LETIMER0 period interrupt, it will request, receive, and process the temperature reading from the Si7021

### Si7021 I2C temp sensor (Used I2C for interfacing the sensor):  
a. It should be running at the lowest energy possible while enabled and while taking temperature measurements
b. Temperature measurements should be 14-bit and calculated in degrees C
c. The default temperature should be 15C
d. If the temperature read from the Si7021 is below the set temperature, LED1 is latched on until cleared by pressing the joy stick button down
e. The temperature set point should increase by 5 degrees C every time the joy stick is pressed to the right
f. The temperature set point should decrease by 5 degrees C every time the joy stick is pressed to the left
g. If LED1 is turned off by pressing the joy stick button down(centre) and the temperature is still below the current set point, LED1 should turn on at the next temperature measurement
h. The temperature set point is not reset when the joy stick button is pressed down. The only function pressing the joy stick down should be to turn off LED1.

## OTA Update: 

### Creating OTA update file: 
1. Build the project
2. Copy app-sign-key.pem and app-encrypt-key.txt into the Bluetooth project folder of this project (This step can be skipped as the keys are already present in the project directory)
3. Run create_bl_files.bat found in your Bluetooth project folder.  
   This will create stack-signed-encrypted.gbl and app-signed-encrypted.gbl files into the output_gbl folder.  
   These are the signed and encrypted upgrade files, which can be sent OTA to the target device

### Performing the OTA 
1. (Android) Copy the .gbl files to your phone and store it in /SiliconLabs_BGApp/OTAFiles/(ProjectFolder)/   
   (iPhone) Copy the .gbl file to your Google Drive/iCloud Drive   
2. Download the Blue Gecko App on your phone. 
    Android App Link: https://play.google.com/store/apps/details?id=com.siliconlabs.bledemo&hl=en
    iOS App Link : https://itunes.apple.com/us/app/silicon-labs-blue-gecko-wstk/id1030932759?mt=8
    
3. Go to Bluetooth Browser and Connect to your device   
4. After connecting select OTA in the menu in the top right corner.   
5. Select the OTA files and do a full or app only OTA depending on the changes in the project    

### To perform OTA update on your own app, the pdf from the below link has steps which could be of help.   
https://github.com/mansetagunj/IoT-Firmware/blob/master/IoT%20BLE%20Project/SiLabs%20BG%20OTA%20procedure.pdf  


***Note: There are some Licence mentions in the Code for the software IP of Silicon Labs.***
