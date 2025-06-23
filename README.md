# Real world of digital twin-based autonomous driving robot system using turtlebot3 waffle-pi

## 1. Project Overview

<img width="557" alt="image" src="https://github.com/user-attachments/assets/aaf7a9ed-fe61-4c66-954a-5d338d66db44" />


🔹 Background of Topic Selection

As autonomous driving technology continues to expand, the importance of camera-based visual recognition has increased.
This project was initiated with the goal of implementing lane detection and driving control using image processing techniques.

🔹 Objective

Simulate real-world road environments using TurtleBot3 and ROS2.
Implement key autonomous driving functionalities such as lane detection, speed control, obstacle avoidance, and adaptation to lighting changes.

🔹 Equipment Used

TurtleBot3 Waffle Pi, DRGO Webcam, Dynamixel Manipulator, Nvidia Jetson Nano, ArUco Marker, OpenCV, ROS2 Humble

🔹 Expected Outcomes

Develop driving technologies applicable to various service robots such as logistics, guidance, and surveillance. Enhance understanding of image processing and autonomous driving algorithms.


## 🛣️ 2. Project Execution – Lane Detection & Control
### ✅ A. Overall Lane Detection Flow
In turtle_pubimg.py, the camera input undergoes the following steps:

HSV Conversion → Bird’s Eye View Transformation → Masking → Polynomial Fitting → Center Line Calculation → ROS2 Topic Publishing → Control Logic Execution
#### 🧪 1) Image Preprocessing & HSV Masking
▶ HSV Color Space Conversion

Used to detect white and yellow lanes. Applied color filtering using cv2.inRange().

<img width="341" alt="image" src="https://github.com/user-attachments/assets/821f3815-be4d-409b-a4ca-37f4cc80fd1d" />


▶ Trackbar (Control Panel)

GUI to adjust HSV ranges and Region of Interest (ROI) in real-time. Allows users to tune optimal lane detection parameters interactively.

<img width="189" alt="image" src="https://github.com/user-attachments/assets/c5cb74d4-e86a-4ebe-a504-b3a5d363ad3b" />


#### 🧭 2) Bird’s Eye View Transformation
Removes perspective distortion for lane alignment. Uses cv2.findHomography and cv2.warpPerspective. Improves the accuracy of curvature calculation and path estimation.

<img width="266" alt="image" src="https://github.com/user-attachments/assets/81465f92-42bc-4b46-9228-4f0b9df7f0f8" />


#### 🌟 3) Histogram Equalization + CLAHE
▶ Histogram Equalization

V channel (brightness): Enhances contrast.

<img width="422" alt="image" src="https://github.com/user-attachments/assets/dcdce56f-4add-42a2-b8ca-cc491c84c924" />
<img width="353" alt="image" src="https://github.com/user-attachments/assets/e2161cf4-62ee-4ea4-9dae-0ee389b1f046" />

S channel (saturation): Improves color clarity.

<img width="385" alt="image" src="https://github.com/user-attachments/assets/c3624b9b-3dbc-45b6-a626-30475b71fdd3" />

<img width="302" alt="image" src="https://github.com/user-attachments/assets/1e55dcca-4703-40dc-9d22-4f31b7330dfa" />

▶ CLAHE (Contrast Limited Adaptive Histogram Equalization)

Solves over-brightening issue in global equalization by applying localized corrections.

Ensures robust recognition under varying lighting and reflections.

<img width="443" alt="image" src="https://github.com/user-attachments/assets/54b63166-4633-47bd-9c46-0d036f6eb44b" />

#### 🧼 4) Noise Removal Techniques
▶ Overlapping Mask Removal

If yellow and white masks overlap due to reflections, prioritize the yellow mask.

<img width="284" alt="image" src="https://github.com/user-attachments/assets/8c25223a-e05c-4f0f-a6e1-e624c64d49d4" />


▶ Morphology – Erosion

Removes small noise (e.g., crosswalks) using a structural kernel to erode unnecessary areas.

<img width="418" alt="image" src="https://github.com/user-attachments/assets/ecea8ebc-e7d5-4b7c-aed6-96ca68e7edf4" />


▶ Labeling

Keeps only the largest connected component, effectively removing noise missed by morphology.

<img width="287" alt="image" src="https://github.com/user-attachments/assets/c117477f-8462-4507-b768-732541842429" />


#### 📈 5) Polynomial Fitting & Center Line Calculation
Used a second-degree polynomial curve fitting in the form of x = Ay² + By + C using np.polyfit().
To enable smoother and gentler cornering, we reduced the A coefficient slightly and increased the B coefficient, making the turning behavior more gradual and stable.

<img width="202" alt="image" src="https://github.com/user-attachments/assets/aa960a3e-6e01-45b1-85c0-9a668b58f795" />

The red and light blue lanes represent the curve before coefficient adjustment, while the orange and light blue lanes represent the curve after applying the adjusted coefficients.

Also if both lanes are detected, the midpoint is calculated. If only one lane is detected, the path is estimated 350px from that lane.

#### 🧠 6) Dynamic Threshold Adjustment
Automatically adjusts the V (brightness) value based on the number of masked pixels.

Ensures stable lane recognition even under changing lighting conditions.

### 🤖 B. Control Logic (control_lane.py)
▶ ROS2-Based Control Node Configuration

Subscribers: /control/lane, /control/max_vel, /avoid_control, /robot_state

Publisher: /control/cmd_vel (velocity command)

<img width="589" alt="image" src="https://github.com/user-attachments/assets/28995864-a10c-4a4e-bbbe-858452fc5017" />



▶ PD Controller-Based Lane Following

📐 Error Calculation

error = center - 500  # based on image center

⚙️ Control Formula

angular.z = Kp * error + Kd * (error - last_error)
Kp = 0.0025, Kd = 0.007

→ Ensures smooth turning and fast directional adjustment.

## 3. ArUco Marker Detection & Manipulation
🔹 Marker Detection
Utilized OpenCV’s ArUco library. Detected markers → Pose Estimation → Homography → Transformed to robot base coordinates.

🔹 Manipulator Control
Upon marker detection:

Issue /robot_state = slow if marker is at a distance.

Issue /robot_state = stop if marker is close.

Perform pick → place with arm_client.send_request.

Resume driving: /robot_state = drive.

## 📷 4. Additional Testing & Dataset Collection
Captured a total of 13 test images under different driving environments.

Designed to evaluate performance under lighting reflections and shadows.

Adjusted HSV ranges, applied histogram equalization, CLAHE, and removed mask overlaps.

<img width="307" alt="image" src="https://github.com/user-attachments/assets/4a2debea-aa70-4d05-9d6e-aa424c9940a1" />


### 🚗 Speed Control
linear.x = min(((1 - abs(error)/500) ** 2.2), 0.1)

→ As error increases, speed decreases sharply to ensure safe cornering.

### 🔄 Obstacle Avoidance Mode Integration
When ArUco marker is detected:

→ /robot_state = slow      # Reduce speed at a distance
→ /robot_state = stop      # Stop near the marker
→ /robot_state = drive     # Resume driving after manipulation

## 5. Demo
### Lane-following driving video

https://drive.google.com/file/d/1BRyIH8yQ3AaU35Ih1dRi5DRQY31I70fs/view?usp=drive_link

### Pick and place operation after ArUco marker detection

https://drive.google.com/file/d/1hqbYXowimGbrBbTyqLsK_fHKh3g4IQIR/view?usp=drive_link

## 🔧 How to Run
> ROS2 Humble + Gazebo11 + TurtleBot3 environment
```bash
# 1. Launch the turtlebot3_manipulation_bringup
$ ros2 launch turtlebot_manipulation_bringup hardware.launch.py

# 2. Launch the turtlebot3_manipulation_moveit_config
$ ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

# 3. Run the arm_controller
$ ros2 run turtlebot_moveit turtlebot_arm_controller

# 4. Execute Lane Detection
$ python3 turtle_pubimg.py

# 5. Execute Lane Control
$ ros2 launch turtlebot3_autorace_mission control_lane.launch.py

# 6. Run Aruco_detector
$ ros2 run aruco_yolo aruco_detector

# 7. Run task_aruco
$ python3 task_aruco.py

