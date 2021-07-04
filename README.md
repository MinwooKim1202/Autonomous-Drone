# Autonomous drone

## 1. DESCRIPTON
This is an autonomous quadcopter swarm project. The purpose of this project is to achieve a designated mission by multiple drone cooperation. I developed and assembled two sets of the drone as the hardware platform. One set consists of six drones modified from 3DR IRIS+ drone, the other set consists of four drones made from scratch. I also developed a drone swarm control software package, including the functions of flight control, multi-drone communication, multi-drone coordination, stereo vision, failure handle, and so on.

There are three subprojects in this repository. 1) The Drone_Matrix_Formation_Flight is the earliest developed subproject, which achieves four drones formation flight. Four drones can form any formation dynamically in flight. This drone swarm can be easily expanded, i.e., you can add more drones to this troop. 2) The Drone_Matrix_Curvature_Flight subject is developed based on Drone_Matrix_Formation_Flight. This is an improved version of Drone_Matrix_Formation_Flight, in which the functions are modularized and more functions are added. The drone troop achieves curved path flight by specified curvature radius and degrees. An execution queue is implemented to achieve complex path designation. 3) Drone_Matrix_Balloon_Hunter is the latest developed subproject. It is developed based on Drone_Matrix_Curvature_Flight, with lots of more function added. In this project, four drones cooperate to find and destroy four balloons. The four balloons are in four different colors, and each drone is intended to find the ballon with specified color and then destroy it. The advanced stereo vision camera plus sonar system is designed and equipped on each drone. With this stereo vision system, the drone is able to find the ballon and calculate the distance of the ballon. Then the drone can adjust its heading and altitude to lock the balloon.

For all the three subprojects, there is one drone acts as a leader (commander), and the other drones act as followers (soldiers). The leader drone plans the mission and sends real-time commands to all the follower drones, and the follower drones carry out required actions following the leader's instructions. All the drones in the troop are identical, which makes the decentralized swarm structure feasible. With a proper negotiation strategy, it is easy to reassign the leader drone. The mission is still achievable by other drones even if the leader fails.


## 2. HARDWARE CONFIGURATION
### 1) DRONE SWARM SET I: DRONE BIULT FROM SCRATCH BASED ON Q250 FRAME AND PIXHAWK4 MINI AUTOPILOT
#### a) MECHANICAL SYSTEM
This Drone platform is modified from 3DR iris+ drone, and the mechanical system is unchange. Pleas refer to the official [3DR Operation Manual](Hardware_Configuration/3DR_IRIS/DOCS/IRIS-Plus-Operation-Manual-vH-web.pdf) for the hardware details. Following is the basic hardware spacification of the mechanical system:
```
- UAV Type: QuadCopter
- Frame Size: Q250
- Motors: 2450 kV
- Propellers: 5.0 x 3.0 counterclockwise (2),
              5.0 x 3.0 clockwise (2)
- Battery: 3S 3.0 Ah 60C lithium polymer
```

<p align="center">
  <img src="img/q250.jpg" height="300">
</p>

#### b) AUTOPILOT SYSTEM
The autopilot system shipped with iris+ drone is [Pixhawk](Hardware_Configuration/3DR_IRIS/README_PICS/Pixhawk.jpg) with firmware ArduCopter v3.2. I upgraded the firmeare to ArduCopter v3.3.3. All the software packages are tested with ArduCopter v3.3.3, and are guaranteed to work well. Please refer to [pixhawk official website](https://pixhawk.org) for details. The firmware is from ArduPilot open source project. Please refer to [ArduPilot official website](http://ardupilot.org/) for the introduction on this project and the instruction on how to flash firmeware and set parameters.
```
- Autopilot: Pixhawk 4 mini
- Firmware: px4 v2.0
```

<p align="center">
<img src="img/pixhawk4_mini.png" height="300">
</p>

#### c) POSITIONING SYSTEM
This project uses its Realsense T265.
```
- Tracking camera: Intel® RealSense™ Tracking Camera T265
```

<p align="center">
<img src="img/t265.png" height="300">
</p>

#### d) ON BOARD COMPUTING SYSTEM (COMPANION COMPUTER)
This project need a high performance companion computer to do mission planning, computer vision task, and other computing-intensive tasks. However, the original iris+ done does not have a companion computer. After research, I found that the intel UP board is the best companion computer for this project. A [power module](Hardware_Configuration/3DR_IRIS/README_PICS/Power_module_5V_3A.jpg) is needed to power the companion computer.
```
- Single Board Computer: Nvida Jetson nano board
- Processor: Quad-core ARM® Cortex-A57 @ 1.43 GHz
- Memory: 2 GB 64-bit LPDDR4 25.6 GB/s
- GPU : 128-core Maxwell
- Storage Capacity: microSD
- OS: JetPack 4.5.1
```

<p align="center">
  <img src="img/jetson_nano.jpg" height="300">
</p>

#### e) STEREO VISION CAMERA AND SONAR SYSTEM
Original iris+ drone does not have any vision system. I designed and assembled the stereo vision camare to equipe it on each drone. Please refer to my other repository [Stereo_Vision_Camera](https://github.com/FlyingCatAlex/Stereo_Vision_Camera) for more detail. With stereo vision camera, drones are able to detect and locate objects in 3D frame. A [Maxbotix I2CXL-MaxSonar-EZ4 High Performance Sonar](https://www.maxbotix.com) is added to each iris+ drone for obstacle detection. The sonar sensor is installed in the center of the stereo vision camera. The solar sensor works with stereo vison camarea to locate objects accurately. 

<p align="center">
<img src="img/pi_cam.jpg" height="300">
</p>

#### f) COMMUNICATION SYSTEM
Original iris+ drone is shipped with RC radio and 3DR radio. An operator can control iris+ maually with RC radio, or control it through ground station installed on computer with 3DR radio. In this project, the mission planning tasks are done by on board computer (companion computer), so there is no need to use a ground station. I removed the 3DR radio and reserve the telemetry port for on board computer. The port TELEM 2 on pixhawk is connected to companion computer via [USB to serial adapter](Hardware_Configuration/3DR_IRIS/README_PICS/USB_to_Serial_Adapter.jpg), and the port TELEM 1 on pixhawk is connected to RC radio receiver. Please refer to [this guide](http://ardupilot.org/dev/docs/odroid-via-mavlink.html) to hook up pixhawk with companion computer. The communication between drones are achieved via WiFi. The communication between drones is achieved by wifi via wireless router on ground. A [USB WiFi adapter](Hardware_Configuration/3DR_IRIS/README_PICS/Panda_Wireless_USB_WiFi_Adapter.jpg) is plugged to the usb port of companion computer to send and receive messages.
```
- Communication with Companion Computer: Telem1 to serial pin
- Communication with pc : Wifi
- Manual Control: RC radio 
```
<p align="center">
<img src="img/lan_card.jpg" height="300"> <img src="img/taranis_x7.jpg" height="300">
</p>
<p align="center">
Iptime Wireless LAN Card (left) and rc radio transmitter Tranis X7 (right)
</p>

#### g) FINAL PRODUCT
The following pictures show bottom view of the modified 3DR iris+ drone. In practical, a case covers on the companion computer to protect it from damage in case of crash, and the USB wifi adapter and USB to Serial adapter are fixed on each side of the companion computer.

<p align="center">
<img src="img/20210625_055456.jpg" height="300"> <img src="img/20210625_055512.jpg" height="300"> <img src="img/20210625_055525.jpg" height="300">
</p>
