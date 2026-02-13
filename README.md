# Waiter Robot - User Guide

This guide provides step-by-step instructions for operating the waiter robot, including mapping, navigation, and table management.

---

## 2.2 Powering On the Robot

After turning on the UPS HAT, press the power button located on the bottom side of the robot to start the main system.

---

## 2.3 Accessing the Raspberry Pi Desktop Remotely

To view and control the Raspberry Pi from your laptop, follow these steps:

1. Open the **Remote Desktop** application on your laptop.
2. Enter the **IP address** of the Raspberry Pi.
3. Ensure that both your laptop and the Raspberry Pi are connected to the **same network**.
4. Click **Connect** to establish the remote session.
5. When prompted, enter the following credentials:
   - **Username:** `luai`
   - **Password:** `1`
6. Once logged in, you will see the Raspberry Pi's screen on your laptop, allowing you to control it remotely.

---

## 2.4 Running the Mapping Process

### Step 1: Open RViz on Another Laptop

1. On a Linux-based laptop, open a terminal.
2. Start RViz by running the following command:
   ```bash
   rviz2
   ```
3. This laptop is used to visualize the ROS2 environment and see the mapping process in real-time.

### Step 2: Enable Mapping in ROS2 on the Raspberry Pi Laptop

1. On the laptop connected to the Raspberry Pi, open **Visual Studio Code** and navigate to:
   ```
   ros2_jazzy → launchall.launch.py
   ```
2. In the code, go to **line 10** and remove the comment sign (`#`) from the first line.
3. Save the file and close Visual Studio Code.

### Step 3: Launch the Mapping Process

1. Open a terminal on the same laptop and run the following command:
   ```bash
   ros2 launch waiter_robot_bringup launchall.launch.py
   ```
2. To verify that mapping is running, check the RViz screen on the other laptop—you should see the map being created.

### Step 4: Control the Robot for Mapping

1. On the Raspberry Pi laptop, open a new terminal (without closing the first one).
2. Run this command to manually control the robot:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. Use the keyboard controls to move the robot around the environment and generate a complete map. You will see the map forming on the RViz screen.

### Step 5: Save the Map

1. Once the mapping is complete, open a new terminal on the Raspberry Pi laptop and start RViz:
   ```bash
   rviz2
   ```
2. In RViz, go to the toolbar, click on **Panels**, and select **Add New Panel**.
3. Choose **SlamToolboxPlugin**, then click **OK**.
4. On the left side of RViz, locate the **Save Map** and **Serialize Map** options.
5. Enter a name for the map (e.g., `floor2`).
6. Click on **Save Map** and then **Serialize Map** to store the map in different formats.

---

## 2.5 Setting up Localization and Navigation

### Step 1: Revert the Mapping Changes

1. Open **Visual Studio Code** on the Raspberry Pi laptop and navigate to:
   ```
   ros2_jazzy → launchall.launch.py
   ```
2. Add back the comment sign (`#`) to **line 10** to disable the mapping feature.
3. Save the file and close Visual Studio Code.

### Step 2: Restart ROS2 and RViz

1. End all running terminals by pressing `Ctrl + C` and close them.
2. Open a new terminal and run:
   ```bash
   ros2 launch waiter_robot_bringup launchall.launch.py
   ```
3. On the other laptop running RViz, close RViz and restart it:
   ```bash
   rviz2
   ```

### Step 3: Load the Saved Map and Start Localization

1. On the Raspberry Pi laptop, open a new terminal and run:
   ```bash
   ros2 launch nav2_bringup localization_launch.py map:=floor2.yaml
   ```
   > **Note:** Replace `floor2.yaml` with the actual name of your saved map.

2. On the other laptop running RViz, do the following:
   - Go to **Left Panel → Map**
   - Click on **Topic**
   - Set **Durability Policy** to **Transient Local**

### Step 4: Set the Robot's Position on the Map

- In RViz, use the **"2D Pose Estimate"** tool to set the initial position of the robot based on the map.
- Click on the map at the robot's location and drag in the direction it's facing.

### Step 5: Start Navigation with Ultrasonic Safety

1. On the Raspberry Pi laptop, open a new terminal and run:
   ```bash
   ros2 launch waiter_robot_bringup nav2_with_ultrasonic.launch.py map_subscribe_transient_local:=true
   ```

2. You should now see the cost map displayed in RViz, indicating that navigation is active.

---

## 2.6 Sending Navigation Goals to Tables

The robot supports **two methods** for sending navigation goals to table positions:

### **Option A: Table Manager Application (GUI)**

#### Step 1: Run the Application

1. On the Raspberry Pi laptop, open a terminal and navigate to the application directory:
   ```bash
   cd ~/application/TableManager-Tinker-ROS-main
   ```
2. Activate the virtual environment:
   ```bash
   source myenv/bin/activate
   ```
3. Start the application:
   ```bash
   python3 main.py
   ```
4. Use the application interface to send the robot to specific tables.

---

### **Option B: VC-02 Voice Module (Voice Commands)**

The VC-02 voice recognition module allows you to send the robot to tables using voice commands.

#### Step 1: Configure Table Positions

1. Open the table configuration file in Visual Studio Code:
   ```
   waiter_robot_bringup → config → tables.yaml
   ```

2. Edit the file to define your table positions. The format is:
   ```yaml
   tables:
     table1: {x: 1.20, y: 0.50, z: 0.0, qz: 0.0, qw: 1.0}
     table2: {x: 2.10, y: -0.30, z: 0.0, qz: 0.707, qw: 0.707}
     table3: {x: 0.40, y: 1.80, z: 0.0, qz: 1.0, qw: 0.0}
   ```

3. The parameters are:
   - `x, y`: Position coordinates (in meters)
   - `z`: Height (usually 0.0)
   - `qz, qw`: Orientation (quaternion z and w components)

#### Step 2: Determine Table Positions

1. On the RViz laptop, open a new terminal and run:
   ```bash
   ros2 topic echo /goal_pose
   ```
   This will listen for goal positions when they are selected in RViz.

2. In RViz, use the **"2D Goal Pose"** tool:
   - Click on the map at the table's location
   - Drag to set the direction the robot should face
   - The terminal will display position coordinates (x, y, z, w)

3. Copy these coordinates and update the `tables.yaml` file with the correct values for each table.

4. Save the file after adding all table positions.

#### Step 3: Using Voice Commands

1. Ensure the VC-02 voice module is properly connected to the Raspberry Pi's serial port (`/dev/serial0`).

2. The robot will automatically listen for voice commands when the navigation system is running.

3. Speak the table name clearly to the VC-02 module:
   - **"Go to Table 1"** → Robot navigates to table1
   - **"Go to Table 2"** → Robot navigates to table2
   - **"Go to Table 3"** → Robot navigates to table3

4. The voice module accepts various formats:
   - "TABLE 1", "table1", or just "1"
   - Commands are automatically normalized

5. A cooldown of 2 seconds prevents accidental duplicate commands.

#### Step 4: Verify Voice Commands

1. Monitor the robot's behavior in RViz to confirm it receives and executes the goal.

2. Check the terminal output for messages like:
   ```
   VC-02 cmd: 'TABLE 1' -> 'table1'
   Goal published to /goal_pose: table1 -> x=1.20, y=0.50
   ```

---

## 2.7 Safety Features

The navigation system includes three layers of safety:

1. **Ultrasonic Obstacle Detection**: Detects low obstacles that might not be visible to the main sensors
2. **Detour Supervisor**: Automatically navigates around detected obstacles
3. **Plate Safety Monitor**: Stops the robot immediately if no plate is detected on the tray (includes buzzer alert)

All safety features are automatically active when using the `nav2_with_ultrasonic.launch.py` launch file.

---

## 2.8 Shutting Down the Robot

1. Stop all running ROS2 nodes by pressing `Ctrl + C` in each terminal.
2. Close all terminals.
3. Press the power button located at the bottom of the robot to turn it off safely.

---



## Additional Notes

- Always ensure both laptops (Raspberry Pi and RViz) are on the same network and can communicate.
- The robot's safety systems will automatically prevent collisions, but manual supervision is recommended.
- Keep the map file and `tables.yaml` backed up to prevent data loss.
- For best voice recognition results, speak clearly and minimize background noise.
