# Lynxmotion mechDOG Web Interface

## Table of Contents
- [Control Panels](#control-panels)
- [OFF Mode](#off-mode)
- [USB Mode](#usb-mode)
- [WIFI Mode](#wifi-mode)

## Control Panels

### Top Panel

The first input allows to "Calibrate" the robot. To calibrate it the robot has to be "Limp" and the user has to position it in the calibration position (laying down) and press the "Calibrate" button. The next button is "Limp" which allows the servos to move freely. The next button is "Halt & Hold" which makes the servos stop immediately and hold the angular position. The "Teach" button allows getting feedback from the servos and showing the position of the real robot in the animation. The next button is the "Emergency Stop" which make the servos go limp and restarts them. The following input is to change the baudrate of the servos and the serial communication (this is disabled by default). The last input allows changing the control mode (OFF, USB & WIFI). On top of this panel there is also a button **˅** where you can find resources.

### Left Panel

The left panel has inputs for the direct control of the servos positions (Forward Kinematics). It also has a input field for sending direct LSS commands.

### Right Panel

The right panel has inputs for the control of the robot: Roll, Pitch & Yaw (in degrees), Frontal & Lateral offsets (in mm), Height (in mm), and it also has a button to "Reset" (R) these values. It has controls for Walking (Forward, Backward, Left & Right, as well as combinations of these), Rotation (Clockwise & Counterclockwise), Speed control (1-3 using the Static gait & 4 for Dynamic gait), Trajectory type (Circular & Square). It also has a list of predefined sequences or "special moves", such as "Sit", "Wiggle" & "Stretch", and "Jog" can be triggered using the "J" button.

The robot can also be controlled using the keyboard:

- W / ↑: Forward
- A / ←: Strafe left
- S / ↓: Backward
- D / →: Strafe right
- Q: Rotate CCW
- E: Rotate CW
- X: Stand up
- Z: Sit
- L: Lay down
- R: Go up
- F: Go down
- T: Roll (-)
- G: Roll (+)
- Y: Pitch (-)
- H: Pitch (+)
- U: Yaw (-)
- J: Yaw (+)
- C: Wiggle
- V: Give Paw
- B: Tinkle

# Web Interface

<p align="center">
  <img src="https://github.com/Lynxmotion/graphical-interfaces/blob/mechdog/desktop_gui/data/web-interface.png"/>
</p>


### OFF Mode

In this mode the animation of the robot will display every change triggered by the controls: forward kinematics (left panel) and inverse kinematic (right panel), the view can be rotated by moving the mouse and pressing the scroll wheel. The only controls that don't have a function in this mode are the "Calibrate", "Limp", "Halt & Hold", "Teach", “Emergency Stop”, "Direct Command" and “LED” functions.

### USB Mode

In this mode the code for kinematics runs on the PC, so a microcontroller is not needed. The LSS adapter has to be connected to the PC and the default baudrate for the communication is 38400 (this can be changed using the interface).

To communicate with using the serial port, the P5.js serialport library and the p5.serialcontrol app is used. The browser doesn’t have direct access to the serial port but it can communicate with another program on the computer that can exchange data with the serialport. The p5.serialcontrol is the app that connects the sketch, running in a browser, with the serial ports on the computer.

**Install the p5.Serialcontrol App**

Download the latest version of the [P5.serialcontrol app](https://github.com/p5-serial/p5.serialcontrol/releases). Open the executable file and open the port the robot is connected to. You don’t need to do anything with the app, just have it open while using the interface and close the port once you finish.

### WIFI Mode

To use this mode the LSS adapter has to be set in Arduino mode, and the interface in WIFI mode. When this mode is selected on the interface it will request the IP address, once a valid address is entered it will be able to send wireless commands to the robot.

In this mode the code for kinematics runs on the robot, so a microcontroller is needed (defalut ID: 100). The library supports Arduino boards, i.e: Arduino UNO, LSS 2IO, BotBoarduino. The WiFi communication supports the WiFiBee board. The default baudrate of the WiFiBee is 115200, but it can be changed using the example Arduino sketch, it should be set to a baudrate supported by the LSS servos, i.e. 38400.

The pins for communication with the WiFiBee can be easily changed in the Arduino sketch, and "MCU_SupportSoftwareSerial" has to be defined (MCU_SupportPPM shouldn't be defined, to avoid conflicts with the interrupts). This sketch can also be used to find the IP address of the WiFibee, to use this simply connect the board to a computer and use a serial monitor, i.e. realterm, to check the information.

* Note for users using the web interface from GitHub pages:
The WiFiBee does not support HTTPS requests, only HTTP, however as GitHub pages serves the contect as HTTPS you need to [enable mixed mode](https://experienceleague.adobe.com/docs/target/using/experiences/vec/troubleshoot-composer/mixed-content.html?lang=en) for the project URL in our browser.

If you are on a mobile using Google Chrome search for: *chrome://flags/#unsafely-treat-insecure-origin-as-secure* and add the IP address assigned to your robot, i.e. *http://192.168.0.190:1000* where 1000 is hte port used for the communication.

Another option to use the web interface is downloading the project and opening it with [Visual Studio Code](https://code.visualstudio.com/) and launch a local server with the [Live Server](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer) extension. With this option it is not necessary to enable mixed mode.

## Authors

* **Geraldine Barreto** - [geraldinebc](https://github.com/geraldinebc)
* **EDGEtronics** - [EDGE-tronics](https://github.com/EDGE-tronics)