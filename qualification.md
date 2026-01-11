To build a robot that matches the BarelangFC robot in the video and qualifies for the **RoboCup Humanoid KidSize League (2025/2026 rules)**, you need to follow this specific roadmap.

The video shows a robot with high torque, a stable walking engine, and autonomous vision. Here is your checklist to replicate those features and qualify.

### **Phase 1: Hardware & Mechanics (The Body)**

* **Meet Dimension Rules:**
* [ ] **Height:** Build between **40cm and 90cm** (BarelangFC is ~69cm).
* [ ] **Foot Size:** Ensure feet fit within the specific area ratios defined in the rulebook (approx. maximum size limits apply).


* **Actuators (Motors):**
* [ ] **Legs:** Use high-torque smart servos (e.g., **Dynamixel MX-106** or XM-540) for the 10-12 leg joints (knee/ankle pitch need the most power).
* [ ] **Arms/Head:** Use lighter servos (e.g., **Dynamixel MX-64** or MX-28) for the 6-8 arm/head joints.
* [ ] **Total DOF:** Ensure at least 20 Degrees of Freedom (2 Head, 6 Left Leg, 6 Right Leg, 3 Left Arm, 3 Right Arm).


* **Sensors:**
* [ ] **IMU:** Install a 6-axis or 9-axis IMU (Gyroscope + Accelerometer) in the torso for balance (e.g., MPU-6050 or BNO055).
* [ ] **Vision:** Mount a webcam (e.g., Logitech C920) or global shutter camera in the head. **No LIDAR or depth sensors allowed.**


* **Computing:**
* [ ] Onboard PC: Install a Mini PC (Intel NUC i5) or Embedded AI board (Jetson Xavier/Orin or Rock 5B) inside the torso.



### **Phase 2: Software & Control (The Brain)**

* **Framework:**
* [ ] Install **ROS 2** (Robot Operating System) on Ubuntu. This is the standard for modern RoboCup teams.


* **Vision System (YOLO):**
* [ ] Train a model (like YOLOv8) to detect:
* **The Ball:** Standard FIFA Size 1 ball (often with a custom design).
* **Goal Posts:** Usually white posts.
* **Field Lines:** White lines on green turf.




* **Walking Engine:**
* [ ] Implement a **ZMP (Zero Moment Point)** or **LIPM (Linear Inverted Pendulum Mode)** walking algorithm. The robot in the video uses this to stay balanced while lifting legs.
* [ ] Implement "Active Balance" using IMU data to adjust ankle pitch if the robot starts tipping.



### **Phase 3: Essential Behaviors (Must match the video)**

To qualify, your robot **must** autonomously demonstrate these specific skills shown in the BarelangFC video:

1. **Autonomous Get-Up (CRITICAL):**
* [ ] **Front Get-Up:** Create a motion to stand up from lying on its stomach.
* [ ] **Back Get-Up:** Create a motion to stand up from lying on its back.
* *Note: In KidSize, if your robot falls and cannot get up by itself, you are disqualified.*


2. **Search & Approach:**
* [ ] **Ball Search:** Robot spins head/body until vision detects the ball.
* [ ] **Walk to Ball:** Robot aligns its body and walks toward the detected ball.


3. **The Kick:**
* [ ] **Alignment:** Robot stops at the correct distance from the ball.
* [ ] **Dynamic Kick:** Robot balances on one leg and swings the other to kick the ball firmly.



### **Phase 4: The Qualification Deliverables**

To actually enter the league, you must submit these items (usually in January/February):

* [ ] **Team Description Paper (TDP):** A 5-10 page academic paper explaining your robot's hardware and software.
* [ ] **Qualification Video:** A generic unedited video (approx. 3-5 mins) showing:
* Sensing the ball (put the ball in random spots).
* Walking to it.
* Kicking it into a goal.
* **Getting up from a fall.**
* Goalkeeper diving (optional but good).



This video from the BarelangFC team demonstrates the "Get Up" motions (at 0:52) and the vision-based kicking (at 1:30) that are mandatory for qualification.

[BarelangFC Qualification Video for RoboCup 2025 Humanoid Kid Size League](https://www.youtube.com/watch?v=X1pooE_HAc0)