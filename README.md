Project description:
This project is to automate the mixing of male/female zebrafish in an off the shelf breeding tank format which allows for easier timing of embryo generation.
It uses a primary controller which hosts a website used to set all the various parameters of the tank functions. The primary connects to a main wifi router to allow devices such as laptops and smart phones to access it.
The primary connects to all the secondary (tank) devices using ESP-NOW, which is a peer to peer protocol.
Each tank has an ESP32 with custom PCB's. A power PCB which accepts a 21700 lithium cell for power and interfaces with an opto sensor for door open detection status.
A control PCB interfaces with the ESP32, a adafruit lithium charge PCB and a pololu DC motor controller. A small geared DC motor is powered to rotate a 3D printed gear.
This turns a rotating door allowing the zebrafish to mix at the appropriate time and date.
