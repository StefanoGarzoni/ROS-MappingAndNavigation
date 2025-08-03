# ROS Mapping & Navigation System

Re-creation of a 2D SLAM + navigation pipeline in **ROS1 (C/C++)** from a recorded bag and a **Stage** simulation. It fuses **front/back LaserScan** for full 360° coverage, converts robot **odometry → TF**, builds a map, and navigates via **move\_base** with goals read from CSV using **actionlib**.&#x20;

---

🔍 Introduction
This project enables:

* Building a **2D occupancy map** from a ROS bag (odometry + dual LaserScan)
* Publishing **TF** from noisy odometry and combining scans into a single 360° stream
* Running a **Stage** simulation with the reconstructed map
* Configuring **move\_base** to localize and avoid obstacles
* Driving the robot via a **goal publisher** that reads a CSV of waypoints and sends action goals
  All tasks are launched with dedicated files (mapping + navigation) and visualized in **RViz** (global frame: `map`).&#x20;

---

🛠️ Features

✔ **Bag-Based Mapping** – Play `robotics2.bag` with simulated time and build the map using your preferred SLAM package (e.g., gmapping, Karto, SLAM Toolbox).&#x20;

✔ **Odometry → TF** – Convert `/odometry` (noisy) into a proper TF tree required by the mapper.&#x20;

✔ **360° Scan Fusion** – Merge `/scan_front` and `/scan_back` into a single LaserScan; **filter out robot self-points** before mapping.&#x20;

✔ **Navigation on Stage** – Simulate a robot (0.54 m × 0.40 m; choose Ackermann / Skid-steer / Omnidirectional) on the reconstructed map; configure **move\_base** to receive goals and avoid obstacles.&#x20;

✔ **CSV Goal Publisher (Actionlib)** – A node reads goals from `second_project/csv/goals.csv` (`x,y,theta`) and sends a new goal when the previous is reached or aborted.&#x20;

✔ **RViz Preset** – Launch RViz to view map, TFs, scans, paths, goals, and (if AMCL is used) particle cloud; global frame set to `map`.&#x20;

---

⚙️ How It Works?

1️⃣ **Bag Replay & Time**
Play the provided bag with simulated time:
`rosbag play --clock robotics2.bag` and set `use_sim_time=true`. This provides:

* `/odometry` (robot odometry, noisy)
* `/scan_front` and `/scan_back` (front/back LIDAR)
* `/tf` (dynamic) and `/tf_static` (sensor poses).&#x20;

2️⃣ **Odometry to TF**
A small C/C++ node publishes the TF needed by the mapper (e.g., `odom → base_link`) from `/odometry`, enabling consistent frame transformations.&#x20;

3️⃣ **Scan Fusion & Filtering**
The two LaserScans are synchronized and merged into a **single 360°** scan. Points that hit the robot body are **masked/filtered** to avoid mapping self-obstacles.&#x20;

4️⃣ **Mapping**
Run your preferred SLAM node to output a `map` (and optionally a serialized state if using SLAM Toolbox). RViz is launched preconfigured (global frame `map`) to visualize the map, LIDAR and TF.&#x20;

5️⃣ **Stage Simulation**
Set up a realistic **Stage** world using the robot dimensions **0.54 m × 0.40 m** and one chosen kinematics model. Load the reconstructed map into the world.&#x20;

6️⃣ **Navigation Stack**
Configure **move\_base** (and AMCL if used) to localize in the provided map and plan around obstacles. RViz shows the map, TFs, path, goals, and particle cloud.&#x20;

7️⃣ **CSV Goal Publisher**
A controller node reads `csv/goals.csv` with lines `x,y,theta` and sends each goal via **actionlib**; it dispatches the next goal when the current one succeeds or aborts.&#x20;

---

🔒 Benefits

✅ **End-to-End** – From bag-based mapping to simulated navigation with goals
✅ **360° Perception** – Dual-scan fusion for complete coverage
✅ **Reproducible** – Deterministic bag replay and ready-made launch files
✅ **Didactic** – Clear split between mapping and navigation tasks, ideal for coursework

---

NOTE:

* Use **simulated time** in launch files (`rosparam set use_sim_time true` or `<param name="/use_sim_time" value="true"/>`).&#x20;
* Provide a **map folder** with `map.png` (mandatory) and a serialized map if SLAM Toolbox is used.&#x20;
* The **CSV** path must be `second_project/csv/goals.csv`; an example file is included.&#x20;
* **Code:** C/C++ only; **package name:** `second_project`. Avoid absolute paths. Do **not** include `build/` or `devel/` or bag files in the submission archive.&#x20;

---

📧 Contact
Questions or contributions? Open an **Issue** or reach out on GitHub! 🚀

