# Real world of digital twin-based autonomous driving robot system using turtlebot3 waffle-pi

## 1. Project Overview

🔹 Background of Topic Selection

As autonomous driving technology continues to expand, the importance of camera-based visual recognition has increased.
This project was initiated with the goal of implementing lane detection and driving control using image processing techniques.

🔹 Objective

Simulate real-world road environments using TurtleBot3 and ROS2.
Implement key autonomous driving functionalities such as lane detection, speed control, obstacle avoidance, and adaptation to lighting changes.

🔹 Equipment Used

TurtleBot3 Waffle Pi, DRGO Webcam, Dynamixel Manipulator, Nvidia Jetson Nano, ArUco Marker, OpenCV, ROS2 Humble

🔹 Expected Outcomes

Develop driving technologies applicable to various service robots such as logistics, guidance, and surveillance.

Enhance understanding of image processing and autonomous driving algorithms.


## 🛣️ 2. Project Execution – Lane Detection & Control
### ✅ A. Overall Lane Detection Flow
In turtle_pubimg.py, the camera input undergoes the following steps:

HSV Conversion → Bird’s Eye View Transformation → Masking → Polynomial Fitting → Center Line Calculation → ROS2 Topic Publishing → Control Logic Execution
#### 🧪 1) Image Preprocessing & HSV Masking
▶ HSV Color Space Conversion

Used to detect white and yellow lanes.

Applied color filtering using cv2.inRange().

▶ Trackbar (Control Panel)

GUI to adjust HSV ranges and Region of Interest (ROI) in real-time.

Allows users to tune optimal lane detection parameters interactively.

#### 🧭 2) Bird’s Eye View Transformation
Removes perspective distortion for lane alignment.

Uses cv2.findHomography and cv2.warpPerspective.

Improves the accuracy of curvature calculation and path estimation.

#### 🌟 3) Histogram Equalization + CLAHE
▶ Histogram Equalization

V channel (brightness): Enhances contrast.

S channel (saturation): Improves color clarity.

▶ CLAHE (Contrast Limited Adaptive Histogram Equalization)

Solves over-brightening issue in global equalization by applying localized corrections.

Ensures robust recognition under varying lighting and reflections.

#### 🧼 4) Noise Removal Techniques
▶ Overlapping Mask Removal

If yellow and white masks overlap due to reflections, prioritize the yellow mask.

▶ Morphology – Erosion

Removes small noise (e.g., crosswalks) using a structural kernel to erode unnecessary areas.

▶ Labeling

Keeps only the largest connected component, effectively removing noise missed by morphology.

#### 📈 5) Polynomial Fitting & Center Line Calculation
Used np.polyfit() to model lanes with a 2nd-degree polynomial:

a: curvature, b: slope, c: intercept.

If both lanes are detected, the midpoint is calculated.

If only one lane is detected, the path is estimated 350px from that lane.

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

-> Ensures smooth turning and fast directional adjustment.

## 3. ArUco Marker Detection & Manipulation
🔹 Marker Detection
Utilized OpenCV’s ArUco library.

Detected markers → Pose Estimation → Homography → Transformed to robot base coordinates.

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

### 🚗 Speed Control
linear.x = min(((1 - abs(error)/500) ** 2.2), 0.1)
As error increases, speed decreases sharply to ensure safe cornering.

### 🔄 Obstacle Avoidance Mode Integration
When ArUco marker is detected:

→ /robot_state = slow      # Reduce speed at a distance
→ /robot_state = stop      # Stop near the marker
→ /robot_state = drive     # Resume driving after manipulation
