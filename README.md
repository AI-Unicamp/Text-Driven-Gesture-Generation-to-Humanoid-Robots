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
│       └── Sad_1.bvh                              # Example: "Sad" gesture sequence
├── Do_odom_son_of_map.py                          # Makes odom a child of map (fixed world frame)
├── Retargeting-from-bvh_to_pepper.py              # Retargets BVH skeleton → Pepper joints
└── bvh_broadcaster_v2_with_frame_map_added.py     # Broadcasts BVH skeleton as TF frames in RViz
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

This is useful for **inspecting and understanding the joint structure** of the generated
human motion before retargeting it to Pepper. You can verify joint names, hierarchy,
and animation directly in RViz — without needing the full Gazebo simulation.

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
@misc{aiunicamp2025gestures,
  title  = {Text-Driven Gesture Generation to Humanoid Robots},
  author = {AI-Unicamp},
  year   = {2025},
  url    = {https://github.com/AI-Unicamp/Text-Driven-Gesture-Generation-to-Humanoid-Robots}
}
```

---

## 🏫 Affiliation

Developed at **[AI-Unicamp](https://github.com/AI-Unicamp)** —  
Artificial Intelligence research group at the University of Campinas (UNICAMP), Brazil.