# Multi-Robot Navigation Using Centralized and Decentralized Monte Carlo Tree Search

🚀 **A comparative study of centralized vs. decentralized Monte Carlo Tree Search (MCTS) for multi-robot navigation in Gazebo simulation.**

## 📝 Overview
This project explores **Monte Carlo Tree Search (MCTS)** as a decision-making framework for **four TurtleBot3 Waffle robots** navigating through a custom-built Gazebo environment. The goal is to analyze the efficiency of **Centralized MCTS** (where a global planner coordinates all robots) versus **Decentralized MCTS** (where each robot makes independent decisions).

## 🔹 Approach
- **Centralized MCTS** 📡: A single controller plans paths for all robots, optimizing movement collectively.
- **Decentralized MCTS** 🤖: Each robot runs its own independent MCTS instance, navigating autonomously.
- **Heuristics Implemented**:
  - Obstacle-aware path planning 🛑
  - Priority-based decision-making 🏆
  - Efficiency heuristics for shorter, collision-free paths ⚡

## 🎯 Key Objectives
✅ Ensure all robots reach their assigned goals in the least amount of time.  
✅ Avoid collisions using efficient decision-making heuristics.  
✅ Compare performance metrics (e.g., time-to-goal, success rate).  

## 📊 Results & Insights
- **Centralized MCTS** provides better **global coordination** but is computationally intensive.  
- **Decentralized MCTS** is **more scalable** but may lead to **suboptimal coordination**.  
- The study provides valuable insights for real-world **robotic applications** such as **warehouse automation, delivery fleets, and autonomous exploration**.

## 🛠️ Technologies Used
- **Simulation:** Gazebo + ROS  
- **Robots:** TurtleBot3 Waffle  
- **Algorithm:** Monte Carlo Tree Search (MCTS)  
- **Programming Language:** Python

## Pre-requisites to Run the Package without issues.
- Make sure have the **Ubuntu-22.04.5 (Jammy Jellyfish)** LTS installed 
- This Project used **ROS2 Humble Hawksbill** so make sure to install that too.
   Link for ROS2 Humble Documentation - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Gazebo Installation - Enter this Command in the Terminal
```bash
   sudo apt install gazebo
   sudo apt install ros-humble-gazebo-ros-pkgs
```
- Install Turtlebot3 Packages
```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot3*
```
You can use the TurtleBot model of your choice but make sure to export them well before 
## Steps to setup and run the package
- Create a workspace with the name of your choice with a `src` folder in it.For Example.
```bash
mkdir -p ~/ MCTS_ws/src
```

- Place the Packages `mcts` and `turtlebot3_multi_robot` in the `src` folder of your workspace and run the following commands in your workspace.
```bash
colcon build --symlink-install 
source install/setup.bash
```

- To Open the Gazebo Environment with multiple robots at origin and with obstacles.
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_robot_custom_world.launch.py enable_drive:=False
```

- To launch the Decentralized MCTS which open 4 terminals runnning the nodes for each TurtleBot3.
```bash
ros2 launch mcts Decentralized_MCTS_multi_window.py
```

- To launch Centralized MCTS.
```bash
ros2 run mcts Centralized_MCTS
```

## 📌 Future Improvements
- Implement **adaptive communication strategies** for decentralized MCTS.  
- Optimize **computational efficiency** for centralized MCTS.  
- Extend to **dynamic environments** with moving obstacles.  

---

### 📂 **Project Repository**
[🔗 GitHub Repository](https://github.com/varunlakshmanan11/Multi-Robot-Naviagtion-Using-Centralized-and-Decentralized-Monte-Carlo-Tree-Search)

---

### 📬 Contact & Contributions
Contributions and feedback are welcome! Feel free to open an issue or reach out.  

---

### Acknowledgements
The gazebo world for this project is created using some of the files from the repository **TurtleBot3 multi-robot environment** provided by [arshadlab/turtlebot3_multi_robot](https://github.com/arshadlab/turtlebot3_multi_robot). Special thanks to the original authors for their contributions to multi-robot navigation research and simulation.  
