# 🤖 Text-Driven Gesture Generation to Humanoid Robots

> Generating expressive body gestures for the **Pepper robot** from text descriptions,  
> using BVH motion retargeting in a ROS + Gazebo simulation environment.

---

## 📖 Overview

This project bridges **text-driven gesture generation** and **humanoid robot animation**.  
Given a text prompt describing an emotion or action, the pipeline generates a BVH skeleton  
animation and retargets it to the Pepper robot, animating it live in Gazebo via ROS.
```
Text Prompt  →  BVH Motion File  →  Retargeting  →  Pepper in Gazebo
```

---

## 🗂️ Repository Structure
```
.
├── BVH-generated-by-text-only/                    # BVH motion files generated from text prompts
│   └── Tesis-2025/
│       ├── Sad_1.bvh
│       └── Happy_1.bvh
├── Motion-Analyzer/                               # Scripts to analyze and compare motions
│   ├── bvh_analyzer.py                            # Plots BVH joint trajectories
│   ├── pepper_analyzer.py                         # Records and plots Pepper joint trajectories
│   ├── motion_comparison.py                       # Compares BVH vs Pepper motion
│   ├── bvh_analysis/                              # Output folder: BVH analysis plots
│   └── pepper_analysis/                           # Output folder: Pepper analysis plots
├── Do_odom_son_of_map.py                          # Makes odom a child of map (fixed world frame)
├── bvh_broadcaster_v2_with_frame_map_added.py     # Broadcasts BVH skeleton as TF frames in RViz
└── Retargeting-from-bvh_to_pepper.py              # Retargets BVH skeleton → Pepper joints
```

---

## 🧠 Pipeline
```
┌───────────────────────┐
│     Text Prompt        │  e.g. "A person expressing sadness"
└──────────┬────────────┘
           ▼
┌───────────────────────┐
│  Text-to-Motion Model  │  Generates BVH skeleton animation
└──────────┬────────────┘
           ▼
┌───────────────────────┐
│   BVH Retargeting     │  Maps BVH joints → Pepper joint angles
│   Script              │
└──────────┬────────────┘
           ▼
┌───────────────────────┐
│   Pepper in Gazebo    │  Animates robot via ROS joint controllers
└───────────────────────┘
```

---

## 🚀 Running the Simulation

### Prerequisites

- ROS Melodic
- Gazebo
- `pepper_gazebo_plugin` package
- Python with ROS bindings (`rospy`)

---

### Terminal 1 — Launch Pepper in Gazebo

Launches the Pepper robot simulation inside an office environment and initializes
the full TF (transform) tree.
```bash
roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch
```

---

### Terminal 2 — Fix the Odometry Frame

Makes `odom` a child of `map`, anchoring the robot to a fixed world reference frame.
```bash
cd pepper_sim_ws/src/scripts-funcionais/
python Do_odom_son_of_map.py
```

---

### Terminal 3 — Retarget BVH to Pepper

Reads a BVH file and drives Pepper's joints in the Gazebo simulation accordingly.
```bash
cd pepper_sim_ws/src/scripts-funcionais/

# Run animation once
python Retargeting-from-bvh_to_pepper.py ./BVH-generated-by-text-only/Tesis-2025/Sad_1.bvh

# Run animation in loop
python Retargeting-from-bvh_to_pepper.py ./BVH-generated-by-text-only/Tesis-2025/Sad_1.bvh -l
```

| Flag | Description |
|------|-------------|
| *(none)* | Play animation once |
| `-l` | Loop animation continuously |

---

## 👁️ Visualizing the BVH Skeleton in RViz

Useful for **inspecting and understanding the joint structure** of the generated human motion
before retargeting it to Pepper. You can verify joint names, hierarchy, and animation
directly in RViz — without needing the full Gazebo simulation.

### Terminal 1 — Launch RViz
```bash
rosrun rviz rviz -d `rospack find pepper_gazebo_plugin`/config/pepper_sensors.rviz
```

### Terminal 2 — Broadcast the BVH skeleton
```bash
cd pepper_sim_ws/src/scripts-funcionais/

# Play once
python bvh_broadcaster_v2_with_frame_map_added.py Happy_1.bvh world

# Loop continuously
python bvh_broadcaster_v2_with_frame_map_added.py Happy_1.bvh world -l
```

| Argument | Description |
|----------|-------------|
| `Happy_1.bvh` | BVH file to visualize |
| `world` | Reference frame to publish the skeleton under |
| `-l` | Loop animation continuously |

---

## 📊 Motion Analyzer

The `Motion-Analyzer/` folder contains scripts to **plot, record, and compare** the joint
trajectories of both the BVH skeleton and the Pepper robot. This is useful for evaluating
the quality of the retargeting and understanding how faithfully Pepper reproduces the
generated human motion.
```bash
cd pepper_sim_ws/src/scripts-funcionais/Motion-Analyzer/

# Step 1: Plot BVH joint trajectories and save to output folder
python bvh_analyzer.py ../BVH-generated-by-text-only/Tesis-2025/Happy_1.bvh --output-dir ./bvh_analysis/

# Step 2: Record and plot Pepper joint trajectories and save to output folder
python pepper_analyzer.py --output-dir ./pepper_analysis_1/

# Step 3: Compare BVH motion vs Pepper motion
python motion_comparison.py --bvh-data ./bvh_analysis/ --pepper-data ./pepper_analysis/
```

| Script | Description |
|--------|-------------|
| `bvh_analyzer.py` | Plots joint angle trajectories from a BVH file |
| `pepper_analyzer.py` | Records and plots Pepper's joint trajectories from ROS topics |
| `motion_comparison.py` | Side-by-side comparison of BVH vs Pepper motion curves |

---

## 📦 BVH Files

BVH motion files live under `BVH-generated-by-text-only/` and were generated using a
text-to-motion model. Each file encodes a full-body skeleton animation corresponding to
a specific emotion or gesture described in natural language.

| File | Description |
|------|-------------|
| `Tesis-2025/Sad_1.bvh` | Sadness gesture sequence |
| `Tesis-2025/Happy_1.bvh` | Happiness gesture sequence |

---

## 📚 Citation

If you use this work, please cite:
```bibtex
@misc{inofuente2025gestures,
  title  = {Text-Driven Gesture Generation to Humanoid Robots},
  author = {Kevin Adier Inofuente Colque},
  year   = {2025},
  url    = {https://github.com/AI-Unicamp/Text-Driven-Gesture-Generation-to-Humanoid-Robots}
}
```

---

## 🏫 Affiliation

Developed at **[AI-Unicamp](https://github.com/AI-Unicamp)** —  
Artificial Intelligence research group at the University of Campinas (UNICAMP), Brazil.