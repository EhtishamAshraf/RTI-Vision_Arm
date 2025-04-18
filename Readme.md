# 🤖🔦 RTI-Vision_Arm
It is a robotic inspection system that leverages a robotic arm as a dynamic light source and a Vimba camera to analyze object appearances using Reflectance Transformation Imaging (RTI). Designed for precision, it features an XY moving platform to position objects accurately underneath the camera for detailed surface inspection.
![Flowchart Image](https://github.com/EhtishamAshraf/RTI-Vision_Arm/blob/f470cc14dfadbd00d28bf9a9b09034069bc50e7d/assets/Flowcharts/LightBot_Flowchart.png)

##### 📺 Demo Video
You can watch the demo video of the system in action by clicking on the below image:
[![Watch the video](https://github.com/EhtishamAshraf/RTI-Vision_Arm/blob/a27c84b9d5da47ba385bdb62a4be36443c20e0c3/assets/Images/RTIBot_system.png)](https://youtu.be/8RlSMWvTJ8c)

## 🔧 Hardware Components
1. Eva Robotic Arm
2. Vimba Camera
3. Light source
4. XY platform running on Arduino Uno
5. Router
   
## 🚀 Powering ON the System
To start the setup, ensure the following devices are properly powered and connected:
1. **Power On All Devices**  
   - Eva robotic arm  
   - Vimba camera  
   - Light source  
   - XY platform  
   - Wi-Fi router  

2. **Power On the XY Platform**  
   - Use the physical **power button** on the XY platform to turn it on.
   - Use Arduino cable to connect the Arduino Uno to your laptop via one of the USB 
     ports.

3. **Establish Network Connections**  
   - Connect the **Ethernet cable** from the router to your **laptop**.  
   - Connect the **Eva robotic arm** and the **Vimba camera** to the **router** using Ethernet cables.

## 📦 Project Dependencies & Libraries
This project uses several libraries and ROS packages. Key dependencies include:
- [`evasdk`](https://pypi.org/project/evasdk/) – Python SDK for controlling the Eva robotic arm.
- [`vimbasdk`](https://www.alliedvision.com/en/products/vimba-sdk/) - SDK for controlling Vimba camera
- [`moveit`](https://moveit.github.io/moveit_tutorials/) - ROS framework for Motion planning
- [`move_group`](https://github.com/moveit/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py) - Interface for planning and executing robot motion using MoveIt.

Others are:
Opencv, Numpy, CV_Bridge and so on ...

## 📥 Cloning the Repository
Create a ros workspace, inside it, create a src folder and navigate into it and run the following command to clone the repo:
```bash
git clone https://github.com/EhtishamAshraf/RTI-Vision_Arm.git
```
Run "catkin_make" inside the main workspace, to build the workspace and compile all ROS packages, ensuring that your system recognizes the newly cloned code.
```bash
catkin_make 
```
1. **Initialize the System from Your Laptop**
- Open a new terminal on your laptop (navigate to ~/path_to_your_workspace/src/lightbot/bash_files), 
  and run the following command to turn ON the system:
```bash
./TurnOnSystem.sh
```
- Turn OFF the system by running the following command:
```bash
./KillSystem.sh
```
+ Make sure to update the path to the light_pos.lp file in both utilities.py and sample_client.py, and also modify the paths in the ./TurnOnSystem.sh script accordingly.
---

# 🌐 Wi-Fi Router
To ensure smooth communication between all devices, it's important that the **camera**, **Eva robotic arm**, and **laptop** are connected to the **same local network** via a router. This setup allows seamless interaction across different devices.
> All devices should have **static IPs**, so they can communicate reliably.
> The communication is done because: Laptop, camera and robot are in the same subnet 
  (192.168.187.x/255.255.255.0).

![Router used](https://github.com/EhtishamAshraf/RTI-Vision_Arm/blob/a27c84b9d5da47ba385bdb62a4be36443c20e0c3/assets/Images/router.png)

**Router Configuration**
-   Logged into the router (TP-Link Archer MR500 Router) settings (192.168.1.1 initially).
-   Changed the router IP to 192.168.187.254 to match my network.
-   Disabled the DHCP range so it doesn't assign random IP address

💻 **Laptop Configuration**

Navigate to your Wired Network Settings → IPv4 → set the method to Manual.
Enter the IP Address and Network Mask manually, ensuring that the IP falls within the defined subnet range mentioned earlier.

> The robot is configured to use 192.168.187.1.

> The camera is configured to use 192.168.187.102.

![Router Setup](https://github.com/EhtishamAshraf/RTI-Vision_Arm/blob/a27c84b9d5da47ba385bdb62a4be36443c20e0c3/assets/Images/router_setting.png)

---

# 🦾 Eva Robotic Arm
![Eva Frames](assets/Eva_man.jpg)

| Tilt Angle  | Missed in 30 poses | Missed in 120 poses  | 
|-------------|--------------------|----------------------|
| **-10°**    | 6                  | 28                   |
| **-20°**    | 8                  | 29                   |


The robotic arm can be controlled in a few differnt ways:
1. Chreograph - Automate
2. EvaSDK
3. ROS

> 🧠 Before controlling with ROS, it's better to control with Choreograph and EvaSDK.

![Eva Manual Image](assets/Eva_man.jpg)

### 🤖 Accessing and Controlling the Eva Robot via Choreograph

Assuming all hardware connections are set up as described earlier:

1. **Open a web browser** (e.g., Chrome) and enter the **IP address** of the Eva robot into the address bar.
2. This will launch the **Choreograph interface**. Log in using **username** and **password**. {username: ehtishamashraf67@gmail.com, password: eva_roboticarm}
3. Once logged in, you’ll be prompted to **load an available toolpath**. Select the provided toolpath (currently named `bend`).
4. The main Choreograph window will appear, displaying the Eva robot model. 
5. Click **Lock Robot**, then **Upload Toolpath**, and finally **Play** the toolpath in **Teach Mode** to execute it.
6. The physical Eva robotic arm will execute movements precisely as defined in the selected toolpath.

🖼️ Take a look at the image below to get familiar with the Choreograph interface.
![Choreograph Interface]()

🛠️ **Want to create your own toolpaths?** 

- To set a waypoint: Move the Eva arm to the desired position by pressing the Backdriving button on the head of the physical robot to release the brakes. Once the arm is in place, click the end effector in Choreograph or press the waypoint button on the head, to save the position as a waypoint.
- After saving the waypoint, click and drag its number into the main timeline panel at the bottom of the Choreograph window. To ensure smooth execution, add a small delay between the waypoints to give the robot time to reach each position accurately.

Check out this helpful video tutorial from **13:30 to 23:30** for a detailed guide:  
👉 [Automata – An Introduction to Eva](https://knowhow.distrelec.com/webinars/automata-an-introduction-to-eva/)

### 🤖 Controlling Eva with EvaSDK
First of all, install EvaSDK using
```bash
pip3 install evasdk
```
[`Eva SDK GitHub Repository`](https://github.com/automata-tech/eva_python_sdk.git) - Refer for more details

Once installed, navigate to the package eva_control > Scripts > evaSDK_test.py and run the script, the arm will go to a certain pose.

### 🤖 Controlling Eva with ROS
Eva can be controlled with ROS in Gazebo Simulation as well as hardware.

📚 **Details of the packages:**
1. **eva_description** – Contains the URDF and Xacro files defining the physical and visual model of the Eva robotic arm.  
2. **eva_control** – Includes configurations for controlling the Eva robot in both Gazebo simulation and real hardware setups.  
3. **sim_eva_without_rail_moveit_config** – MoveIt configuration package specifically for simulating the Eva arm in Gazebo (without the linear rail).  
4. **eva_without_rail_moveit_config** – MoveIt configuration package used for planning and controlling the real Eva arm (without the rail) in hardware.  
5. **lightbot** – Main package for operating the RTI-Bot system: moves the robotic arm to predefined light poses and captures images for reflectance transformation imaging (RTI).
6. The **eva_ikfast_plugin** – package contains an analytically generated inverse kinematics (IK) solver for the Eva robotic arm using IKFast. It enables precise and efficient computation of joint angles for a desired end-effector pose, enhancing MoveIt's planning performance.

🎯 **MoveIt Framework for EVA Arm Control**

The MoveIt framework is used for motion planning, and control of robotic arms. 
In this project, MoveIt plays a central role in controlling the EVA robotic arm, both in simulation and on real hardware.

Using MoveIt, we are:
- Planning complex motion paths automatically while avoiding obstacles in the environment (if any, defined before doing motion planning (static obstacles)).
> The box, xy platform, and a virtual cylinder underneath the camera can be put as an obstacle by running the script "moveit_obstacles.py"
```bash
rosrun eva_control moveit_obstacles.py
```

The eva_ikfast_plugin provides a fast analytical inverse kinematics solver for the Eva arm, used by MoveIt through the kinematics.yaml file to enable efficient and real-time motion planning.

In simulation, MoveIt communicates with Gazebo and RViz to visualize and test planned trajectories without needing the physical robot. On hardware, it sends those planned trajectories to the real EVA arm using the eva SDK, ensuring accurate and safe movements.

### Gazebo Simulation
To view the simulation of the Eva arm and xy platform, run the command:
```bash
roslaunch eva_control spawn_gazebo_sim.launch
```
> The file eva_moveit_IK.py contains the moveit script for controlling the Eva arm in simulation.

🕹️ This (eva_controllers.yaml) YAML configuration file defines the **ROS controllers** used for simulating both the **EVA robotic arm** and the **XY Plotter** within RViz and Gazebo environments. The EVA arm is controlled using an `effort_controllers/JointTrajectoryController`, allowing precise trajectory planning and execution via the MoveIt framework. Each joint is individually configured with PID gains and motion constraints to ensure smooth and safe movement. A `JointGroupEffortController` is also defined for applying direct effort commands to all arm joints collectively. Additionally, the XY Plotter is simulated using `position_controllers/JointPositionController` for both its X and Y axes, representing the platform that moves an object beneath a fixed camera. This setup enables seamless simulation and testing of integrated robotic workflows—combining motion planning and object positioning in a fully virtual environment before deployment to real hardware.

⚠️ Note: The Gazebo simulation is intentionally launched in a paused state to allow all components and controllers to initialize properly before starting.

✅ Remember to manually unpause the simulation in the Gazebo GUI once everything is loaded.

The demo demonstrates the Eva robotic arm autonomously reaching a specified pose using MoveIt, while simultaneously executing a predefined motion on the XY platform.
![Demo in Action](assets/demo.gif)

### Hardware 
🔓 Remember to turn OFF the choreograph robot lock before running ROS nodes else arm won't move.

Eva can also be controlled without using MoveIt, by directly sending joint angle goals through an action client. This provides more direct and lightweight control over the arm.

Start the Action Server:
```bash
rosrun eva_control eva_sdk_control_server.py
```
Run the Action Client:
```bash
rosrun eva_control eva_control_sample_client.py
```
To reach various light poses using MoveIt and perform data acquisition, simply run the provided bash script as outlined in the "cloning the Repository" section.

Here's an overview of the core components involved:

1.   main.py - acts as action client (sends goal points to action server) and service server (receives service requests from sample_client.py to read the light positions, setting camera parameters and acquiring data)

2.   eva_ros_control_server.py - ROS Action Server node (receives the goal points from the action client and sends them to eva_service.py for execution on the real eva arm, hence it also acts as Service client node.

3.   eva_service.py - acts as Sevice server and it directly controls the real Eva arm.

The following image illustrates the light poses arranged in a circular ring around the object.

![Circular Light Poses](assets/circular_light_poses.png)

The following image shows the light poses distributed in a spherical configuration around the object.

![Spherical Light Poses](assets/spherical_light_poses.png)

---

# ↔️↕️ XY Platform
An XY platform is used to position objects precisely beneath vimba camera for inspection. The platform operates using two stepper motors that control horizontal (X-axis) and vertical (Y-axis) movements. These motors are driven by a TB6600 motor driver and controlled via an Arduino Uno, providing accurate and repeatable motion control essential for the task.
_LED Indicators for Home Position:_
Two LEDs on the breadboard light up when the limit switches are triggered, indicating that the platform has reached the home position. These LEDs serve as visual indicators to confirm successful homing.

The circuit diagram of the XY platform is shown below:
![Fritzing Circuit](assets/spherical_light_poses.png)

Components used are:
| Component              | Quantity |
|------------------------|----------|
| Arduino Uno            | x1       |
| TB6600 Motor Driver    | x2       |
| Stepper Motors         | x2       |
| Magnetic Limit Switch  | x2       |
| Breadboard             | x1       |
| Wires (bundle)         | x1       |

🧭 The limit switch plays a crucial role in homing the platform by defining the reference or zero position.

## Platform Control:
The corresponding ROS package can be found in the workspace with the name "xy_platform_control".

Before controlling the platform using Python or ROS, ensure that the appropriate firmware has been uploaded to the Arduino Uno.

### Controlling with Arduino: 
To control the platform using Arduino:
1. Upload the `Arduino_xyplatform.ino` sketch to your Arduino Uno.
2. Open the **Serial Monitor** in the Arduino IDE.
3. Send simple text commands to control the platform:

   - `mx5` → Move the platform 5 steps in the X direction  
   - `my5` → Move the platform 5 steps in the Y direction

These commands allow for manual testing of the XY motion.
> DIR Pin of the stepper is used for controlling the direction, and the Pulse Pin is used for controlling the steps.

### Controlling with Python:
To control the platform with python without ROS, run the python script "Python_Arduino_Communication.py". 

### Controlling with ROS:
The platform can be controlled with ROS both in Simulation and Hardware.

#### 1.   Gazebo/Rviz Simulation:
To simulate the XY platform within a ROS environment, launch the xy_plotter.launch file. This setup utilizes the xyplatform_sim_config.yaml file, which defines the necessary controllers for a simulated XY plotter using ROS Control.
```bash
roslaunch xy_platform_control xy_plotter.launch
```
The controller configuration includes:

A joint_state_controller for publishing joint states to ROS topics.

Two JointPositionController instances to independently manage movement along the 
X and Y axes.

This configuration enables the simulated platform to:

Receive and execute precise position commands.
    
Provide real-time joint feedback.
    
🎮 Controllers play a crucial role in simulating the platform. They take position commands and move the platform accordingly. This is similar to how a motor controller (e.g., L298N) operates, converting input commands into motor movements, allowing for precise control over the platform's position and movement.

![Demo in Action](assets/demo.gif)

#### 2.   Hardware:
ROS service (xy_platform_service.srv) is used to control the platform using ROS. 
Run the Server node first:
```bash
rosrun xy_platform_control xy_platform_server.py
```
Then run the Client node:
```bash
rosrun xy_platform_control xy_platform_client.py
```
**Control logic:**
-   The client sends a target x or y position, server calculates how far the platform needs to move by comparing it with the current x_position or y_position respectively.

-   If the desired x is 0, it sends an "ix" command, to initialize or home the X-axis or Y-axis respectively.

-   After sending the movement command, internal state (self.x_position or self.y_position) is updated to reflect the new position.

-   Finally, it publishes the updated X position or Y position to the ROS topic.

🖼️ Below is an image of the used XY Platform:
![platform](images/platform.jpg)

---

# 📷 Vimba Camera
The project makes use of a Vimba camera (from Allied Vision) to capture images of the object.

![Vimba camera](assets/xy_platform.jpg)

## ⚙️ Setting up the Camera:
### 🪟 Windows OS

**Note:**
1. Step:4 is not mandatory, if step 3 has been done.
2. MAC address of the camera could be found on the camera itself.
3. Change of IP address is important to have the Eva Robotic Arm, Laptop, and the camera in the same subnet.

First of all, **Download and Install** VimbaSDK from the official website on Windows OS (I have used Vimba_6_0)

1.  Make sure the connections are done as explained in the beginning.
2.  Open **VimbaViewer**, Go to Detected Cameras/GigE and you should see the detected camera there. - - Click on the camera and a new window will pop up. **Note:** the camera will open in the CONFIG Mode 
  and that's fine for now.

3. In the newly opened window, on the right hand side, you will see GigE under the Features Tab.
Go to GigE/Persistent, change the IP address, after setting persistent IP, Go to GigE/Configuration/IP Configuration Mode. Set IP Configuration Mode =
Persistent; and execute IP Configuration Apply command.

4. In the first window (from Step: 2), Find "Force IP" from the left bar, and click on it. Fill in all the details as shown below and click Send. This will change the current IP address of the camera. **Note:** It won't be a permanent change untill you set the Persistent IP address as explained in Step:3.

![Vimba Window Setup](assets/xy_platform.jpg)

### 🐧 Linux OS
Before proceeding, make sure all the steps in the Windows OS section are completed successfully.
1. Install Vimba SDK from allied vision website --- Vimba_6_0 for linux.

2. Then open a terminal and head to Vimba_6_0/VimbaPython and run this command: sudo ./Install.sh to install python wrapper. {Give executable permission beforehand}

3. Then head to Vimba_6_0/VimbaGigETL and run this command: sudo ./Install.sh to install vimba. {Give executable permission beforehand}

**Remember,** to get rid of Transport layer and some other errors update the "bashrc" file by adding the below lines at the end of the file:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/path-to-your-workspace/src/camera/Vimba_6_0/VimbaC/DynamicLib/x86_64bit

export GENICAM_GENTL64_PATH=~/path-to-your-workspace/src/camera/Vimba_6_0/VimbaGigETL/CTI/x86_64bit
```

5. Then go to Vimba_6_0/VimbaPython/Examples$ and run the examples to see if vimba is successfully installed or not. If successfully installed, running:
```bash
python3 list_cameras.py
```
should list the **Serial Number** of the camera along with other details and capture an image.

> running the below command will capture an image if the camera is available:
```bash
python3 camera.py
```

6. run vimbaviewer from: Vimba_6_0/Tools/Viewer/Bin/x86_64bit to have a GUI to check the detected cameras, if camera is detected then clicking on it should now open the camera in "FULL Access Mode".
   
7. And in the newly opened windown, Click on **Freerun** button and you should be able to get the live feed from the camera.

![Vimba Window Setup](assets/xy_platform.jpg)

🔭 **Image Acquisition**

Place an object underneath the camera and in the **Brigthness tab**, set the parameters like Exposure time, gain, gamma and remember to change the focus of the camera to ensure the object is clearly visible by the camera for accurate inspection.
Currently the values are: 
-   Exposure Time:   3503
-   Gain:            12

## 🧿 Integrating ROS with the Camera:
The repo already have the necessary ROS driver for the camera inside the package "avt_vimba_camera".

🚗 A ROS driver serves as the interface between physical hardware and the ROS framework, allowing devices to communicate using standard ROS messages. For example, the avt_vimba_camera driver connects the Vimba-compatible camera to ROS, publishing image and camera info topics so the data can be processed or visualized within the ROS ecosystem.

Open the "mono_camera.launch" file and write the correct IP address and the serial number:
<param name="ip" value="192.168.187.102"/>
<param name="guid" value="50-0503385000"/> 

1. Open a new terminal and run the following command: 
```bash
roslaunch avt_vimba_camera mono_camera.launch
```
To test that everything is working correctly, There are two python scripts included in the /scripts folder, open a new terminal and run to see if the camera is working with ROS or not.
```bash
rosrun avt_vimba_camera img_vimba.py
```
or
```bash
rosrun avt_vimba_camera Bbox_vimba.py
```

🖼️ Below is an image captured with the Vimba camera:
![Vimba Image](images/eva_arm_demo.jpg)

---
## 🧠 Key Insights & Notes

### ⚠️ Important to Know

- MoveIt is used to generate the joint trajectory. The waypoints are executed on the Eva arm using Eva SDK.
- In the current setup, only the **first and last waypoints** generated by MoveIt are sent to the real Eva arm to avoid **jerky movements**.
- The **object’s position** is defined **relative to the world coordinate frame**.
Below is the visualization of the world as displayed in the TF frame from RViz.
![World Visualization](assets/world_visualization.png)
- Interpolation between waypoints was attempted but did **not result in smoother motion**.
- After data acquisition, the **Eva arm returns to its Home position**.
- On startup, the **XY platform first homes itself**, then moves beneath the camera for image acquisition.
- 🛑 If you close the Vimba camera launch file and immediately rerun it, you might encounter an error. Wait a few seconds for the camera to restart.
- The current camera used is **monochromatic**, meaning it captures images in grayscale only.
- Calibration is crucial for ensuring that the camera’s intrinsic parameters, such as focal length, optical center, and distortion coefficients, are accurate. For example, in the case of the Vimba camera, the calibration file (calibration_50-0503343289.yaml) is used to load these parameters, which are essential for accurate image processing, undistortion, and proper camera pose estimation.
- A **greedy search algorithm** is implemented to prioritize the **nearest light poses**, optimizing execution time.
  ![Greedy Search Image](images/eva_arm_demo.jpg)
- The missed poses are turned RED in RVIZ by checking if the position of a marker is within 0.01 units of the target pose's position, to determine if it corresponds to a missed waypoint.
Mathematically:∣Marker Position(𝑥,𝑦,𝑧) − Pose Position(𝑥,𝑦,𝑧)∣≤ 0.01
- The current setup defines **semi-circular (Theta (θ): -70° to 58°) and semi-spherical (Phi (ϕ): 65° to 0°, and Theta (θ): -60° to 35°) light poses** around the object.
- **Circular waypoints**

   To generate waypoints in a circular pattern around an object, we use **polar to 
   Cartesian coordinate conversion**. Given the object's position (x₀, y₀), a fixed 
   **radius** (r), and an **angle** (theta) measured from the x-axis, we compute 
   each waypoint's position in the global frame.
   
   This positions the waypoint at a distance (r) from the object, at angle (θ) along 
   the circular arc.

   Mathematically:
   x = x₀ + r · cos(θ)
   y = y₀ + r · sin(θ)
   z = z₀
![circular_points Visualization](assets/world_visualization.png)
> Code is available in src/assets/python_codes.
- **Spherical waypoints**

   To generate waypoints in a spherical pattern around an object, we use spherical- 
   to-Cartesian coordinate conversion. Given the object's center position (x₀, y₀, 
   z₀) a fixed radius (ρ) and two angles — the azimuthal angle (𝜃) measured in the 
   XY-plane from the X-axis, and the elevation angle (𝜙) measured from the 
   horizontal plane (XY-plane) — the waypoint's global position is computed as:

   Mathematically:
    x = x₀ + ρ * math.cos(ϕ) * math.cos(θ)
    y = y₀ + ρ * math.cos(ϕ) * math.sin(θ)
    z = z₀ +  ρ * math.sin(ϕ)  

   This positions each waypoint at a constant radial distance from the object, 
   distributed over a spherical surface defined by the varying angles 𝜃 and 𝜙.

⚠️ Note on Convention Differences

   While above formulas resemble standard spherical coordinate equations, there is 
   an intentional deviation in the definition of the angle 𝜙.

Standard (Literature) Convention:

   👉 Refer for lit review: [Math Insight explanation of spherical coordinates](https://mathinsight.org/spherical_coordinates#:~:text=In%20summary%2C%20the%20formulas%20for,%CE%B8z%3D%CF%81cos%CF%95)
   x = x₀ + ρsinϕcosθ 
   y = y₀ + ρsinϕsinθ 
   z = z₀ + ρcosϕ

   In this form, ϕ is the angle from the positive Z-axis, measured downward toward 
   the vector ρ.

In current Implementation:

   𝜙 is instead defined as the angle from the XY-plane upward toward the vector ρ, 
   which causes the sine and cosine terms in the 𝑧 and 𝑥/𝑦 components to be 
   effectively swapped.

🔍 This change in convention was made intentionally to simplify visualization and 
   integration with Robotic Arm as it operate relative to the Ground Plane (XY).
   As in current setup, EVA is mounted on a table or ground, so the "base frame" 
   assumes:
.   XY plane is the workspace (horizontal plane).

.   Z axis is the height (vertical movement).

Defining ϕ as an angle from the XY-plane:

   Means ϕ = 0° → arm moves horizontally outward.

   Means ϕ increases as the arm raises upward, which matches how robotic arm is 
   programmed or visualized.

   This is more intuitive than imagining an angle measured from the vertical Z-axis 
   downward (as in the math literature).


![spherical_points Visualization](assets/world_visualization.png)
> Code is available in src/assets/python_codes.

- Y-axis is the default forward axis for the end effector, meaning that the direction along the Y-axis is considered the "forward" or "front" direction of the end effector. To ensure the correct orientation for pointing light at the object, we align the end effector's Y-axis with the direction vector.
![Y-axis Visualization](assets/world_visualization.png)

> In RVIZ: X-axis is represented by Red | Y-axis by Green | Z-axis by Blue
> The Y-axis is defined as the forward axis in the current setup, but it can vary in the URDF/Xacro file depending on the Roll, Pitch, and Yaw (RPY) values of the joints, starting from the world frame to the end effector link.

- **Light Pose Orientation:**

  The direction vector can be found by taking the difference between the 
  object's position (x₀, y₀, z₀) and the waypoint's position.

  direction = [x₀ - xg, y₀ - yg, z₀ - zg]

  To rotate direction vector downward in 3D space, we need to find the 
  axis (of rotation) that is perpendicular to both the current direction 
  vector and the Z-axis (as the requirement is to tilt the vector 
  downward, meaning the vertical (Z) component will be changed 
  slightly, making the vector "look" lower).

* The cross product of the direction vector and Z-axis will give us the 
  axis of rotation. 
![Direction Vector Tilt](assets/world_visualization.png)

  As the default forward direction of the EE is along the Y-axis so we 
  align the Y-axis with the tilted direction vector to get the rotation 
  and from rotation we find the orientation in quaternion.

![Direction Vector ALignment](assets/world_visualization.png)

**Light Pose Orientation:**

The direction vector `direction` can be found by taking the difference between the object's position `(x₀, y₀, z₀)` and the waypoint's position `(xg, yg, zg)`:

direction = [x₀ - xg, y₀ - yg, z₀ - zg]

To rotate the direction vector downward in 3D space, we need to find the axis of rotation that is perpendicular to both the current direction vector and the Z-axis (since the goal is to tilt the vector downward, i.e., modify the Z component to "look" lower).

*Rotation Axis Calculation:*

The cross product of the direction vector and Z-axis will give us the axis of rotation:

axis = direction × Z-axis

Where `×` represents the cross product, and `axis` is the axis around which the rotation occurs.

**Note:** The rotation axis is perpendicular to both vectors `direction` and `Z-axis`, ensuring that we rotate around the correct axis to achieve the desired downward tilt.

![Direction Vector Tilt](assets/world_visualization.png)

**Aligning the Direction Vector:**

Since the default forward direction of the End-Effector (EE) is along the Y-axis, we align the Y-axis with the tilted direction vector to achieve the desired rotation. Once we find the rotation, we convert it into a quaternion to represent the final orientation.

This gives us the desired light pose orientation that is aligned with the new downward-tilted direction. 
![Direction Vector Alignment](assets/world_visualization.png)
> Code is available in src/assets/python_codes.

- The system takes approximately 70 seconds for startup.
- For processing 120 light poses, the system requires around 12 minutes and 33 
  seconds.
- When processing 30 light poses, the time required is approximately 3 minutes and 
  50 seconds.

### 📊 Pose Capture Stats with Tilt for semi-spherical setup

| Tilt Angle  | Missed in 30 poses | Missed in 120 poses  | 
|-------------|--------------------|----------------------|
| **-10°**    | 6                  | 28                   |
| **-20°**    | 8                  | 29                   |

- **Acquired Data Visualization from Lightbot**

The images below show the actual acquisition data collected from the Lightbot. Notice the significant changes based on variations in the theta and phi parameters:

Changing Theta (Phi Fixed):
    When phi is held constant and theta is varied, the result is shown in the image below:
![theta_changed](images/eva_arm_demo.jpg)

Changing Phi (Theta Fixed):
    When theta is kept constant and phi is varied, the resulting image is captured as follows:
    ![phi changed](images/eva_arm_demo.jpg)



### 🧩 MoveIt Setup

- For MoveIt **simulation configuration**, the description file used is: `eva_without_rail.urdf`
- For MoveIt **hardware configuration**, the file used is: `eva.xacro`

---

## 🛠️ Troubleshooting

- If images are not as expected:
  - **Remove the camera service call** from `sample_client.py`
  - Run the rest of the system
  - Open **Vimba Viewer** separately to monitor the camera’s live feed
  - Adjust the camera parameters and observe improvements before re-integrating into 
    the script

---

## 🚀 Future Work

- Eva’s current workspace is limited. Future improvements may include:
  - Placing the robot on a **linear or circular rail**
  - Adding rotation capability to the **XY platform** to allow varied object orientations
