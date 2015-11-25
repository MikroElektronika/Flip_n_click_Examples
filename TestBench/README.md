
> #Flip and Click Test by [MikroElektronika](http://www.mikroe.com)
>----------


## TestBench ##
This example works on the premise that you are wanting to test the following features:

 - SPI
 - I2C / I2C1 
 - UART
 - GPIO 

## What you will need to accomplish the testing proceedure: ##
### Hardware ###
 - [Flip and Click](http://www.mikroe.com/flip-n-click/)
 - [GPS3 click](http://www.mikroe.com/click/gps3/) Placed in MikroBus Socket A
 - [Flash click](http://www.mikroe.com/click/flash/) Placed in MikroBus Socket B
 - [RTC2 click](http://www.mikroe.com/click/rtc2/) Placed in MikroBus Socket C
 - [Button click](http://www.mikroe.com/click/button-r/) Placed in MikroBus Socket D
 
 ### Software ###
 - [Arduino IDE](https://www.arduino.cc/download_handler.php?f=/arduino-1.6.6-windows.exe) IDE
 - [DS1307RTC Library](http://www.pjrc.com/teensy/arduino_libraries/DS1307RTC.zip)
 - [DueTimer Library](https://github.com/ivanseidel/DueTimer/archive/master.zip)
 
 ## Instructions for Library Installation ##
 - 1. Install Arduino IDE and Start.
 - 2. Under the menu Sketch/Include Library select "Add .ZIP Library".
 - 3. Select the DS1307RTC and DueTimer Libraries to install them.
 - 4. Select "open" under the File menu and open the "TestBench.ino" project file.
 - 5. Under the Tools menu select "Board Manager".
 - 6. Scroll down and find the Arduino Due board, click on it, and click "Install".
 - 7. Once the installation is completed, choose Tools from the menu again.  Scroll down towards the bottom and select "Arduino Due Programming Port".
 - 8. Choose Tools again and select "Port" and select the Com port that the Due is attached.
 - 9. Finally, click the check mark icon to compile the application if it succeeds, then you can press the arrow icon which will upload the program to the Due.
 
 ## Instructions for Testing ## 
 Once the program is uploaded, press the reset button and the test will start.
 Testing lasts for approximately 10 seconds.  If the board passes, then the moving LEDs will stop and turn off.  If the test fails, then the LEDs will stop but remain on.

 If the board fails 1 time, press the reset and allow it one more test, if it fails the second time, the board has a failure on one of the test cases.

