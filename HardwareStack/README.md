# Hopper_Hardware

Dependencies: 
* [Teensyduino](https://www.pjrc.com/teensy/td_download.html)
* [ESP 8266](https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/)
* Run ``sudo usermod -a -G tty yourname``
* Run ``sudo usermod -a -G dialout yourname``
* Reboot the computer
* In arduino IDE, got to File->preferences, change sketchbook location to be ${PATH_TO_REPO}/src

Steps to make this work:
1. Flash the esp with the esp code
* when you are flashing the esp, COMPLETELY UNPLUG IT FROM THE ROBOT 
* plug in the USB to the computer
* Go to Tools->Board->ESP8266->NodeMCU1.0
* Tools->Port->USB0
* Upload
2. Flash the teensy with the teensy code
* Board->Teensyduino->Teensy4.1
* Port teensy port or something
3. Start the server code just before the robot reboots

Starting Sequence for the robot:
1. Turn on left most switch (enable logic)
2. If yellow light flashes two times, move on -- if red flight flashes 10 times, abandon
3. Turn middle switch on (enable elmo power) and wait 5 seconds
4. Turn on right most swtich. Motors will go through initialization sequence
5. Wait for the orange light to dissapear
6. When orange light dissapears, turn off the right most swtich
7. Wait for the Archer symbol to go green
8. Initiate server fille (./build/hopper)

Starting optitrck:
* roslaunch vrpn_client_ros fast.launch server:=192.168.1.3

Igor's troubleshooting steps:
* Make sure that the ports/boards are selected properly
* Unconnect ESP when flashing to the robot
* Make sure to check RX/TX if you are not recieving any messages
* Make sure the robot is on to flash teensy code
* CHECK THE BATTERIES (the beeping tool) -- current in cells to be at least 3.8 V
* Ping the ESP board to make sure that IP address is correct
* Close open serial ports to make sure that flashing works properly



