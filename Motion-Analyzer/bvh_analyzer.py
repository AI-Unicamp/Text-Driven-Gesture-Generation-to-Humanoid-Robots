#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Enhanced BVH file visualizer with separate wrist analysis and image export.
- Basic visualization:
  python bvh_analyzer.py PATH_TO_FILE.bvh
- With wrist tracking and loop:
  python bvh_analyzer.py PATH_TO_FILE.bvh --track-wrist -l
- Export plots only (no animation):
  python bvh_analyzer.py PATH_TO_FILE.bvh --export-only
- Custom output directory:
  python bvh_analyzer.py PATH_TO_FILE.bvh --output-dir ./analysis_results/

The script will automatically save high-resolution plots of X, Y, Z vs Time for each wrist separately.
"""
from __future__ import print_function
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import math
import os
from collections import deque
import argparse
import traceback
plt.rcParams['font.family'] = 'DejaVu Sans'

# ===================================================================
# ===== NODE AND BVHREADER CLASSES (Same as before) ===============
# ===================================================================

class Node(object):
    def __init__(self, name=None, parent=None):
        self.name, self.parent = name, parent
        self.channels, self.offset, self.children = [], np.array([0.0,0.0,0.0]), []
    def isEndSite(self): return not self.children
    def __repr__(self): return "Node('{}')".format(self.name)

class BVHReader(object):
    def __init__(self, filename):
        self.filename = filename
        self.root, self.nodestack, self.num_channels = None, [], 0
        self.frames, self.frame_time, self.ordered_nodes = [], 0.0, []

    def read(self):
        with open(self.filename, 'r') as f: self.lines = [l for l in (l.strip() for l in f) if l]
        self.line_idx = 0
        self._read_hierarchy(); self._read_motion(); self._order_nodes(self.root)
        print("BVH file '{}' read: {} frames, {}s/frame, {} joints.".format(self.filename, len(self.frames), self.frame_time, len(self.ordered_nodes)))

    def _read_hierarchy(self):
        if not self.lines[self.line_idx].upper().startswith("HIERARCHY"): raise ValueError("HIERARCHY not found")
        self.line_idx += 1; tokens = self.lines[self.line_idx].split()
        if not tokens[0].upper().startswith("ROOT"): raise ValueError("ROOT not found")
        self.root = Node(name=tokens[1]); self.nodestack.append(self.root); self.line_idx += 2
        while self.nodestack:
            tokens = self.lines[self.line_idx].split(); keyword = tokens[0].upper(); self.line_idx += 1
            if keyword == "JOINT":
                node = Node(name=tokens[1], parent=self.nodestack[-1]); self.nodestack[-1].children.append(node)
                self.nodestack.append(node); self.line_idx += 1
            elif keyword == "END":
                node = Node(name="{}_EndSite".format(self.nodestack[-1].name), parent=self.nodestack[-1])
                self.nodestack[-1].children.append(node); self.nodestack.append(node); self.line_idx += 1
            elif keyword == "OFFSET": self.nodestack[-1].offset = np.array([float(v) for v in tokens[1:]])
            elif keyword == "CHANNELS":
                num = int(tokens[1]); self.nodestack[-1].channels = tokens[2:]; self.num_channels += num
            elif keyword == "}": self.nodestack.pop()
            elif keyword == "MOTION": break

    def _read_motion(self):
        if not self.lines[self.line_idx].upper().startswith("MOTION"): raise ValueError("MOTION not found")
        self.line_idx += 1; self.num_frames = int(self.lines[self.line_idx].split()[1]); self.line_idx += 1
        self.frame_time = float(self.lines[self.line_idx].split()[2]); self.line_idx += 1
        for _ in range(self.num_frames): self.frames.append([float(v) for v in self.lines[self.line_idx].split()]); self.line_idx += 1

    def _order_nodes(self, node):
        self.ordered_nodes.append(node)
        for child in node.children: self._order_nodes(child)

# ===================================================================
# ===== ENHANCED BVH ANALYZER CLASS WITH SEPARATED WRISTS ==========
# ===================================================================

class BVHAnalyzer(object):
    def __init__(self, bvh_reader, trail_length=150, track_wrist=False, loop=False, output_dir='./bvh_analysis'):
        self.bvh = bvh_reader
        self.trail_length = trail_length
        self.track_wrist = track_wrist
        self.loop = loop
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print("Created output directory: {}".format(self.output_dir))
        
        if self.track_wrist: print("Wrist tracking mode activated.")
        if self.loop: print("Loop mode activated.")
        
        self._prepare_data()
        self._setup_plot()

    def _prepare_data(self):
        print("Preparing animation data...")
        self.skeleton_connections = self._get_skeleton_connections()
        self.r_wrist_name = self._find_joint_name(['b_r_wrist', 'RightHand', 'RightWrist'])
        self.l_wrist_name = self._find_joint_name(['b_l_wrist', 'LeftHand', 'LeftWrist'])
        if not self.r_wrist_name: raise ValueError("Right wrist joint not found.")
        print("Tracking joints found: '{}' and '{}'".format(self.r_wrist_name, self.l_wrist_name))
        self.all_frame_positions = self._calculate_all_frame_positions()
        self._apply_root_transform()
        self.r_wrist_full_history = np.array([f[self.r_wrist_name] for f in self.all_frame_positions])
        self.l_wrist_full_history = np.array([f[self.l_wrist_name] for f in self.all_frame_positions])

    def _setup_plot(self):
        print("Setting up Matplotlib figure with separated wrist analysis...")
        self.fig = plt.figure(figsize=(20, 16))
        
        # Main 3D skeleton view (top-left)
        self.ax_skeleton = self.fig.add_subplot(3, 4, 1, projection='3d')
        
        # Complete trajectory 3D (top-right)
        self.ax_trajectory = self.fig.add_subplot(3, 4, 2, projection='3d')
        
        # Right wrist time series (left column)
        self.ax_r_x = self.fig.add_subplot(3, 4, 5)  # Right X vs Time
        self.ax_r_y = self.fig.add_subplot(3, 4, 9)  # Right Y vs Time
        self.ax_r_z = self.fig.add_subplot(3, 4, 6)  # Right Z vs Time
        
        # Left wrist time series (right column)
        self.ax_l_x = self.fig.add_subplot(3, 4, 7)  # Left X vs Time
        self.ax_l_y = self.fig.add_subplot(3, 4, 11) # Left Y vs Time
        self.ax_l_z = self.fig.add_subplot(3, 4, 8)  # Left Z vs Time
        
        # Top-down views
        self.ax_top_r = self.fig.add_subplot(3, 4, 10) # Right wrist top view
        self.ax_top_l = self.fig.add_subplot(3, 4, 12) # Left wrist top view
        
        # 3D trajectories separate
        self.ax_3d_r = self.fig.add_subplot(3, 4, 3, projection='3d')  # Right 3D
        self.ax_3d_l = self.fig.add_subplot(3, 4, 4, projection='3d')  # Left 3D
        
        # Calculate limits
        all_positions = np.vstack([self.r_wrist_full_history, self.l_wrist_full_history])
        min_c = all_positions.min(axis=0)
        max_c = all_positions.max(axis=0)
        center = (max_c + min_c) / 2.0
        max_range = (max_c - min_c).max() * 0.7 or 1.0
        
        self.xlim_static = [center[0] - max_range, center[0] + max_range]
        self.ylim_static = [center[1] - max_range, center[1] + max_range]
        self.zlim_static = [min_c[2] - 0.2, max_c[2] + 0.2]

        # Setup skeleton lines
        self.skeleton_lines = [self.ax_skeleton.plot([], [], [], c='blue', lw=2, alpha=0.8)[0] for _ in self.skeleton_connections]
        self.r_wrist_trail_line, = self.ax_skeleton.plot([], [], [], c='red', lw=2, alpha=0.7, label='Right Wrist - Generated Gesture')
        self.l_wrist_trail_line, = self.ax_skeleton.plot([], [], [], c='green', lw=2, alpha=0.7, label='Left Wrist - Generated Gesture')
        self.frame_text = self.ax_skeleton.set_title('')

        # Setup trajectory plots
        self.full_traj_line_r, = self.ax_trajectory.plot([], [], [], c='red', lw=2, alpha=0.7, label='Right - Generated Gesture')
        self.full_traj_line_l, = self.ax_trajectory.plot([], [], [], c='green', lw=2, alpha=0.7, label='Left - Generated Gesture')

        # Setup RIGHT WRIST time series plots with labels
        self.r_x_line, = self.ax_r_x.plot([], [], 'red', lw=2, label='Generated Gesture')
        self.r_y_line, = self.ax_r_y.plot([], [], 'red', lw=2, label='Generated Gesture')
        self.r_z_line, = self.ax_r_z.plot([], [], 'red', lw=2, label='Generated Gesture')
        
        # Setup LEFT WRIST time series plots with labels
        self.l_x_line, = self.ax_l_x.plot([], [], 'green', lw=2, label='Generated Gesture')
        self.l_y_line, = self.ax_l_y.plot([], [], 'green', lw=2, label='Generated Gesture')
        self.l_z_line, = self.ax_l_z.plot([], [], 'green', lw=2, label='Generated Gesture')

        # Setup top-down views with labels
        self.top_r_line, = self.ax_top_r.plot([], [], 'red', lw=2, label='Generated Gesture Trajectory')
        self.top_r_marker, = self.ax_top_r.plot([], [], 'ro', markersize=8)
        self.top_l_line, = self.ax_top_l.plot([], [], 'green', lw=2, label='Generated Gesture Trajectory')
        self.top_l_marker, = self.ax_top_l.plot([], [], 'go', markersize=8)
        
        # Setup 3D individual trajectories with labels
        self.traj_3d_r_line, = self.ax_3d_r.plot([], [], [], 'red', lw=2, alpha=0.7, label='Generated Gesture')
        self.traj_3d_l_line, = self.ax_3d_l.plot([], [], [], 'green', lw=2, alpha=0.7, label='Generated Gesture')

    def _init_animation(self):
        # Setup 3D skeleton view
        self.ax_skeleton.set_xlim(self.xlim_static); self.ax_skeleton.set_ylim(self.ylim_static); self.ax_skeleton.set_zlim(self.zlim_static)
        self.ax_skeleton.set_xlabel('X (m)', fontsize=10); self.ax_skeleton.set_ylabel('Y (m)', fontsize=10); self.ax_skeleton.set_zlabel('Z (m)', fontsize=10)
        self.ax_skeleton.legend(fontsize=8, loc='upper right')
        self.ax_skeleton.set_title('BVH Skeleton - Generated Gesture', fontsize=12, fontweight='bold')
        
        # Setup complete trajectory
        self.ax_trajectory.set_xlim(self.xlim_static); self.ax_trajectory.set_ylim(self.ylim_static); self.ax_trajectory.set_zlim(self.zlim_static)
        self.ax_trajectory.set_xlabel('X (m)', fontsize=10); self.ax_trajectory.set_ylabel('Y (m)', fontsize=10); self.ax_trajectory.set_zlabel('Z (m)', fontsize=10)
        self.ax_trajectory.set_title('Both Trajectories - Generated Gesture', fontsize=12, fontweight='bold')
        self.ax_trajectory.legend(fontsize=8, loc='upper right')
        
        # Setup time series plots
        time_max = self.bvh.num_frames * self.bvh.frame_time
        
        # RIGHT WRIST plots
        self.ax_r_x.set_title('Right Wrist - Generated Gesture - X vs Time', fontsize=11, fontweight='bold', color='darkred')
        self.ax_r_x.set_xlim(0, time_max); self.ax_r_x.set_ylim(self.xlim_static)
        self.ax_r_x.set_ylabel('X Position (m)', fontsize=9); self.ax_r_x.grid(True, alpha=0.3)
        self.ax_r_x.legend(fontsize=8, loc='upper right')
        
        self.ax_r_y.set_title('Right Wrist - Generated Gesture - Y vs Time', fontsize=11, fontweight='bold', color='darkred')
        self.ax_r_y.set_xlim(0, time_max); self.ax_r_y.set_ylim(self.ylim_static)
        self.ax_r_y.set_xlabel('Time (s)', fontsize=9); self.ax_r_y.set_ylabel('Y Position (m)', fontsize=9); self.ax_r_y.grid(True, alpha=0.3)
        self.ax_r_y.legend(fontsize=8, loc='upper right')
        
        self.ax_r_z.set_title('Right Wrist - Generated Gesture - Z vs Time', fontsize=11, fontweight='bold', color='darkred')
        self.ax_r_z.set_xlim(0, time_max); self.ax_r_z.set_ylim(self.zlim_static)
        self.ax_r_z.set_ylabel('Z Position (m)', fontsize=9); self.ax_r_z.grid(True, alpha=0.3)
        self.ax_r_z.legend(fontsize=8, loc='upper right')
        
        # LEFT WRIST plots
        self.ax_l_x.set_title('Left Wrist - Generated Gesture - X vs Time', fontsize=11, fontweight='bold', color='darkgreen')
        self.ax_l_x.set_xlim(0, time_max); self.ax_l_x.set_ylim(self.xlim_static)
        self.ax_l_x.set_ylabel('X Position (m)', fontsize=9); self.ax_l_x.grid(True, alpha=0.3)
        self.ax_l_x.legend(fontsize=8, loc='upper right')
        
        self.ax_l_y.set_title('Left Wrist - Generated Gesture - Y vs Time', fontsize=11, fontweight='bold', color='darkgreen')
        self.ax_l_y.set_xlim(0, time_max); self.ax_l_y.set_ylim(self.ylim_static)
        self.ax_l_y.set_xlabel('Time (s)', fontsize=9); self.ax_l_y.set_ylabel('Y Position (m)', fontsize=9); self.ax_l_y.grid(True, alpha=0.3)
        self.ax_l_y.legend(fontsize=8, loc='upper right')
        
        self.ax_l_z.set_title('Left Wrist - Generated Gesture - Z vs Time', fontsize=11, fontweight='bold', color='darkgreen')
        self.ax_l_z.set_xlim(0, time_max); self.ax_l_z.set_ylim(self.zlim_static)
        self.ax_l_z.set_ylabel('Z Position (m)', fontsize=9); self.ax_l_z.grid(True, alpha=0.3)
        self.ax_l_z.legend(fontsize=8, loc='upper right')
        
        # Setup top-down views
        self.ax_top_r.set_title('Right Wrist - Generated Gesture - Top View', fontsize=11, fontweight='bold', color='darkred')
        self.ax_top_r.set_xlim(self.xlim_static); self.ax_top_r.set_ylim(self.ylim_static)
        self.ax_top_r.set_xlabel('X (m)', fontsize=9); self.ax_top_r.set_ylabel('Y (m)', fontsize=9)
        self.ax_top_r.grid(True, alpha=0.3); self.ax_top_r.set_aspect('equal')
        self.ax_top_r.legend(fontsize=8, loc='upper right')
        
        self.ax_top_l.set_title('Left Wrist - Generated Gesture - Top View', fontsize=11, fontweight='bold', color='darkgreen')
        self.ax_top_l.set_xlim(self.xlim_static); self.ax_top_l.set_ylim(self.ylim_static)
        self.ax_top_l.set_xlabel('X (m)', fontsize=9); self.ax_top_l.set_ylabel('Y (m)', fontsize=9)
        self.ax_top_l.grid(True, alpha=0.3); self.ax_top_l.set_aspect('equal')
        self.ax_top_l.legend(fontsize=8, loc='upper right')
        
        # Setup 3D individual trajectories
        self.ax_3d_r.set_title('Right 3D Trajectory - Generated Gesture', fontsize=11, fontweight='bold', color='darkred')
        self.ax_3d_r.set_xlim(self.xlim_static); self.ax_3d_r.set_ylim(self.ylim_static); self.ax_3d_r.set_zlim(self.zlim_static)
        self.ax_3d_r.set_xlabel('X', fontsize=8); self.ax_3d_r.set_ylabel('Y', fontsize=8); self.ax_3d_r.set_zlabel('Z', fontsize=8)
        self.ax_3d_r.legend(fontsize=8, loc='upper right')
        
        self.ax_3d_l.set_title('Left 3D Trajectory - Generated Gesture', fontsize=11, fontweight='bold', color='darkgreen')
        self.ax_3d_l.set_xlim(self.xlim_static); self.ax_3d_l.set_ylim(self.ylim_static); self.ax_3d_l.set_zlim(self.zlim_static)
        self.ax_3d_l.set_xlabel('X', fontsize=8); self.ax_3d_l.set_ylabel('Y', fontsize=8); self.ax_3d_l.set_zlabel('Z', fontsize=8)
        self.ax_3d_l.legend(fontsize=8, loc='upper right')
        
        self.fig.tight_layout(pad=2.0)
        
        artists = (self.skeleton_lines + [
            self.r_wrist_trail_line, self.l_wrist_trail_line, self.frame_text,
            self.full_traj_line_r, self.full_traj_line_l,
            self.r_x_line, self.r_y_line, self.r_z_line,
            self.l_x_line, self.l_y_line, self.l_z_line,
            self.top_r_line, self.top_r_marker, self.top_l_line, self.top_l_marker,
            self.traj_3d_r_line, self.traj_3d_l_line
        ])
        return artists

    def _update_frame(self, frame_idx):
        skeleton_data = self.all_frame_positions[frame_idx]
        
        # Update skeleton
        for i, (p_name, c_name) in enumerate(self.skeleton_connections):
            p1, p2 = skeleton_data.get(p_name), skeleton_data.get(c_name)
            if p1 is not None and p2 is not None:
                self.skeleton_lines[i].set_data(np.array([[p1[0], p2[0]], [p1[1], p2[1]]]))
                self.skeleton_lines[i].set_3d_properties(np.array([p1[2], p2[2]]))

        # Update trails
        trail_start = max(0, frame_idx - self.trail_length)
        r_trail_data = self.r_wrist_full_history[trail_start:frame_idx+1]
        l_trail_data = self.l_wrist_full_history[trail_start:frame_idx+1]
        
        self.r_wrist_trail_line.set_data(r_trail_data[:,0], r_trail_data[:,1])
        self.r_wrist_trail_line.set_3d_properties(r_trail_data[:,2])
        self.l_wrist_trail_line.set_data(l_trail_data[:,0], l_trail_data[:,1])
        self.l_wrist_trail_line.set_3d_properties(l_trail_data[:,2])

        self.frame_text.set_text('Frame {}/{}'.format(frame_idx + 1, self.bvh.num_frames))

        # Update complete trajectories
        sub_r_history = self.r_wrist_full_history[:frame_idx+1]
        sub_l_history = self.l_wrist_full_history[:frame_idx+1]
        
        self.full_traj_line_r.set_data(sub_r_history[:,0], sub_r_history[:,1])
        self.full_traj_line_r.set_3d_properties(sub_r_history[:,2])
        self.full_traj_line_l.set_data(sub_l_history[:,0], sub_l_history[:,1])
        self.full_traj_line_l.set_3d_properties(sub_l_history[:,2])

        # Update time series plots
        times = np.arange(frame_idx + 1) * self.bvh.frame_time
        
        # RIGHT WRIST time series
        self.r_x_line.set_data(times, sub_r_history[:,0])
        self.r_y_line.set_data(times, sub_r_history[:,1])
        self.r_z_line.set_data(times, sub_r_history[:,2])
        
        # LEFT WRIST time series
        self.l_x_line.set_data(times, sub_l_history[:,0])
        self.l_y_line.set_data(times, sub_l_history[:,1])
        self.l_z_line.set_data(times, sub_l_history[:,2])

        # Update top-down views
        self.top_r_line.set_data(sub_r_history[:,0], sub_r_history[:,1])
        self.top_r_marker.set_data(sub_r_history[-1,0], sub_r_history[-1,1])
        self.top_l_line.set_data(sub_l_history[:,0], sub_l_history[:,1])
        self.top_l_marker.set_data(sub_l_history[-1,0], sub_l_history[-1,1])
        
        # Update 3D individual trajectories
        self.traj_3d_r_line.set_data(sub_r_history[:,0], sub_r_history[:,1])
        self.traj_3d_r_line.set_3d_properties(sub_r_history[:,2])
        self.traj_3d_l_line.set_data(sub_l_history[:,0], sub_l_history[:,1])
        self.traj_3d_l_line.set_3d_properties(sub_l_history[:,2])

        # Dynamic camera tracking
        if self.track_wrist and r_trail_data.size > 0:
            padding = 0.4
            xlim = [r_trail_data[:, 0].min() - padding, r_trail_data[:, 0].max() + padding]
            ylim = [r_trail_data[:, 1].min() - padding, r_trail_data[:, 1].max() + padding]
            zlim = [r_trail_data[:, 2].min() - padding, r_trail_data[:, 2].max() + padding]
            self.ax_skeleton.set_xlim(xlim); self.ax_skeleton.set_ylim(ylim); self.ax_skeleton.set_zlim(zlim)

        artists = (self.skeleton_lines + [
            self.r_wrist_trail_line, self.l_wrist_trail_line, self.frame_text,
            self.full_traj_line_r, self.full_traj_line_l,
            self.r_x_line, self.r_y_line, self.r_z_line,
            self.l_x_line, self.l_y_line, self.l_z_line,
            self.top_r_line, self.top_r_marker, self.top_l_line, self.top_l_marker,
            self.traj_3d_r_line, self.traj_3d_l_line
        ])
        return artists

    def export_time_series_plots(self):
        """Export high-resolution individual plots for each wrist separately"""
        print("Exporting separated wrist time series plots for analysis...")
        
        times = np.arange(len(self.r_wrist_full_history)) * self.bvh.frame_time
        
        # Export RIGHT WRIST plots
        print("Exporting Right Wrist plots...")
        
        # Right X vs Time
        plt.figure(figsize=(12, 8))
        plt.plot(times, self.r_wrist_full_history[:,0], 'red', lw=3, alpha=0.8, label='Right Wrist X Position')
        plt.title('Generated Gesture Right Wrist - X Position vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('X Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'right_wrist_x_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Right Y vs Time
        plt.figure(figsize=(12, 8))
        plt.plot(times, self.r_wrist_full_history[:,1], 'red', lw=3, alpha=0.8, label='Right Wrist Y Position')
        plt.title('Generated Gesture Right Wrist - Y Position vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Y Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'right_wrist_y_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Right Z vs Time
        plt.figure(figsize=(12, 8))
        plt.plot(times, self.r_wrist_full_history[:,2], 'red', lw=3, alpha=0.8, label='Right Wrist Z Position')
        plt.title('Generated Gesture Right Wrist - Z Position vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Z Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'right_wrist_z_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Export LEFT WRIST plots
        print("Exporting Left Wrist plots...")
        
        # Left X vs Time
        plt.figure(figsize=(12, 8))
        plt.plot(times, self.l_wrist_full_history[:,0], 'red', lw=3, alpha=0.8, label='Left Wrist X Position')
        plt.title('Generated Gesture Left Wrist - X Position vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('X Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'left_wrist_x_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Left Y vs Time
        plt.figure(figsize=(12, 8))
        plt.plot(times, self.l_wrist_full_history[:,1], 'red', lw=3, alpha=0.8, label='Left Wrist Y Position')
        plt.title('Generated Gesture Left Wrist - Y Position vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Y Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'left_wrist_y_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Left Z vs Time
        plt.figure(figsize=(12, 8))
        plt.plot(times, self.l_wrist_full_history[:,2], 'red', lw=3, alpha=0.8, label='Left Wrist Z Position')
        plt.title('Generated Gesture Left Wrist - Z Position vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Z Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'left_wrist_z_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Export COMBINED plots for comparison
        print("Exporting comparison plots...")
        
        # Combined X comparison
        plt.figure(figsize=(15, 8))
        plt.plot(times, self.r_wrist_full_history[:,0], 'red', lw=3, alpha=0.8, label='Right Wrist - Generated Gesture')
        plt.plot(times, self.l_wrist_full_history[:,0], 'green', lw=3, alpha=0.8, label='Left Wrist - Generated Gesture')
        plt.title('X Position Comparison - Generated Gesture - Both Wrists vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('X Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'comparison_x_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Combined Y comparison
        plt.figure(figsize=(15, 8))
        plt.plot(times, self.r_wrist_full_history[:,1], 'red', lw=3, alpha=0.8, label='Right Wrist - Generated Gesture')
        plt.plot(times, self.l_wrist_full_history[:,1], 'green', lw=3, alpha=0.8, label='Left Wrist - Generated Gesture')
        plt.title('Y Position Comparison - Generated Gesture - Both Wrists vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Y Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'comparison_y_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Combined Z comparison
        plt.figure(figsize=(15, 8))
        plt.plot(times, self.r_wrist_full_history[:,2], 'red', lw=3, alpha=0.8, label='Right Wrist - Generated Gesture')
        plt.plot(times, self.l_wrist_full_history[:,2], 'green', lw=3, alpha=0.8, label='Left Wrist - Generated Gesture')
        plt.title('Z Position Comparison - Generated Gesture - Both Wrists vs Time', fontsize=18, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Z Position (m)', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12, loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'comparison_z_vs_time.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Export RIGHT WRIST combined XYZ
        plt.figure(figsize=(15, 12))
        
        plt.subplot(3, 1, 1)
        plt.plot(times, self.r_wrist_full_history[:,0], 'red', lw=3, alpha=0.8, label='Generated Gesture')
        plt.title('Right Wrist - Generated Gesture - X Position vs Time', fontsize=14, fontweight='bold')
        plt.ylabel('X Position (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10, loc='upper right')
        
        plt.subplot(3, 1, 2)
        plt.plot(times, self.r_wrist_full_history[:,1], 'red', lw=3, alpha=0.8, label='Generated Gesture')
        plt.title('Right Wrist - Generated Gesture - Y Position vs Time', fontsize=14, fontweight='bold')
        plt.ylabel('Y Position (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10, loc='upper right')
        
        plt.subplot(3, 1, 3)
        plt.plot(times, self.r_wrist_full_history[:,2], 'red', lw=3, alpha=0.8, label='Generated Gesture')
        plt.title('Right Wrist - Generated Gesture - Z Position vs Time', fontsize=14, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Z Position (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10, loc='upper right')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'right_wrist_combined_xyz.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Export LEFT WRIST combined XYZ
        plt.figure(figsize=(15, 12))
        
        plt.subplot(3, 1, 1)
        plt.plot(times, self.l_wrist_full_history[:,0], 'green', lw=3, alpha=0.8, label='Generated Gesture')
        plt.title('Left Wrist - Generated Gesture - X Position vs Time', fontsize=14, fontweight='bold')
        plt.ylabel('X Position (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10, loc='upper right')
        
        plt.subplot(3, 1, 2)
        plt.plot(times, self.l_wrist_full_history[:,1], 'green', lw=3, alpha=0.8, label='Generated Gesture')
        plt.title('Left Wrist - Generated Gesture - Y Position vs Time', fontsize=14, fontweight='bold')
        plt.ylabel('Y Position (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10, loc='upper right')
        
        plt.subplot(3, 1, 3)
        plt.plot(times, self.l_wrist_full_history[:,2], 'green', lw=3, alpha=0.8, label='Generated Gesture')
        plt.title('Left Wrist - Generated Gesture - Z Position vs Time', fontsize=14, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Z Position (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=10, loc='upper right')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'left_wrist_combined_xyz.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Export 3D trajectories
        print("Exporting 3D trajectory plots...")
        
        # Right wrist 3D trajectory
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.r_wrist_full_history[:,0], self.r_wrist_full_history[:,1], self.r_wrist_full_history[:,2], 
                'red', lw=3, alpha=0.8, label='Generated Gesture')
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_zlabel('Z Position (m)', fontsize=12)
        ax.set_title('Right Wrist - Generated Gesture - Complete 3D Trajectory', fontsize=16, fontweight='bold')
        ax.legend(fontsize=12, loc='upper right')
        plt.savefig(os.path.join(self.output_dir, 'right_wrist_3d_trajectory.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Left wrist 3D trajectory
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.l_wrist_full_history[:,0], self.l_wrist_full_history[:,1], self.l_wrist_full_history[:,2], 
                'green', lw=3, alpha=0.8, label='Generated Gesture')
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_zlabel('Z Position (m)', fontsize=12)
        ax.set_title('Left Wrist - Generated Gesture - Complete 3D Trajectory', fontsize=16, fontweight='bold')
        ax.legend(fontsize=12, loc='upper right')
        plt.savefig(os.path.join(self.output_dir, 'left_wrist_3d_trajectory.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # Both trajectories together
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.r_wrist_full_history[:,0], self.r_wrist_full_history[:,1], self.r_wrist_full_history[:,2], 
                'red', lw=3, alpha=0.8, label='Right Wrist - Generated Gesture')
        ax.plot(self.l_wrist_full_history[:,0], self.l_wrist_full_history[:,1], self.l_wrist_full_history[:,2], 
                'green', lw=3, alpha=0.8, label='Left Wrist - Generated Gesture')
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_zlabel('Z Position (m)', fontsize=12)
        ax.set_title('Both Wrists - Generated Gesture - Complete 3D Trajectories', fontsize=16, fontweight='bold')
        ax.legend(fontsize=12, loc='upper right')
        plt.savefig(os.path.join(self.output_dir, 'both_wrists_3d_trajectory.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        print("All plots exported successfully!")
        print("Exported files:")
        print("- Individual wrist plots: right_wrist_[x|y|z]_vs_time.png, left_wrist_[x|y|z]_vs_time.png")
        print("- Comparison plots: comparison_[x|y|z]_vs_time.png")
        print("- Combined plots: right_wrist_combined_xyz.png, left_wrist_combined_xyz.png")
        print("- 3D trajectories: right_wrist_3d_trajectory.png, left_wrist_3d_trajectory.png, both_wrists_3d_trajectory.png")
        print("Output directory: {}".format(self.output_dir))

    def export_raw_data(self):
        """Export raw wrist position data for comparison analysis"""
        print("Exporting raw wrist data for comparison analysis...")
        
        # Prepare data
        times = np.arange(len(self.r_wrist_full_history)) * self.bvh.frame_time
        r_wrist_array = self.r_wrist_full_history
        l_wrist_array = self.l_wrist_full_history
        
        # Export right wrist data
        r_wrist_data = np.column_stack([
            times,
            r_wrist_array[:, 0],  # X
            r_wrist_array[:, 1],  # Y  
            r_wrist_array[:, 2]   # Z
        ])
        r_wrist_path = os.path.join(self.output_dir, 'bvh_right_wrist_data.csv')
        np.savetxt(r_wrist_path, r_wrist_data, delimiter=',', 
                   header='time,x,y,z', comments='', fmt='%.6f')
        
        # Export left wrist data
        l_wrist_data = np.column_stack([
            times,
            l_wrist_array[:, 0],  # X
            l_wrist_array[:, 1],  # Y
            l_wrist_array[:, 2]   # Z
        ])
        l_wrist_path = os.path.join(self.output_dir, 'bvh_left_wrist_data.csv')
        np.savetxt(l_wrist_path, l_wrist_data, delimiter=',',
                   header='time,x,y,z', comments='', fmt='%.6f')
        
        # Export metadata
        metadata = {
            'total_frames': len(self.r_wrist_full_history),
            'frame_time': self.bvh.frame_time,
            'total_duration': times[-1] if len(times) > 0 else 0,
            'right_wrist_joint': self.r_wrist_name,
            'left_wrist_joint': self.l_wrist_name,
            'source_file': self.bvh.filename
        }
        
        metadata_path = os.path.join(self.output_dir, 'bvh_metadata.txt')
        with open(metadata_path, 'w') as f:
            for key, value in metadata.items():
                f.write('{}={}\n'.format(key, value))
        
        print("Raw data exported:")
        print("- Right wrist: {}".format(r_wrist_path))
        print("- Left wrist: {}".format(l_wrist_path))
        print("- Metadata: {}".format(metadata_path))

    def run(self, export_only=False):
        # Always export the plots
        self.export_time_series_plots()
        
        # Always export raw data for comparison
        self.export_raw_data()
        
        if export_only:
            print("Export-only mode: plots and data saved, skipping animation.")
            return
            
        print("Starting animation with separated wrist analysis...")
        interval = self.bvh.frame_time * 1000
        self.ani = animation.FuncAnimation(self.fig, self._update_frame, frames=self.bvh.num_frames,
                                           init_func=self._init_animation, blit=True, interval=interval,
                                           repeat=self.loop)
        plt.show()
    
    # Helper methods (same as before)
    def _find_joint_name(self, names):
        for name in names:
            if name in [n.name for n in self.bvh.ordered_nodes]: return name
        return None

    def _get_skeleton_connections(self):
        return [(n.parent.name, n.name) for n in self.bvh.ordered_nodes if n.parent]

    def _apply_root_transform(self):
        roll, yaw = 1.57, 1.57
        Rx = np.array([[1,0,0], [0,math.cos(roll),-math.sin(roll)], [0,math.sin(roll),math.cos(roll)]])
        Rz = np.array([[math.cos(yaw),-math.sin(yaw),0], [math.sin(yaw),math.cos(yaw),0], [0,0,1]])
        tm = np.dot(Rz, Rx)
        for frame in self.all_frame_positions:
            for name in frame: frame[name] = np.dot(tm, frame[name])

    def _build_rot_mat(self, vals, order):
        mat = np.identity(4)
        for ch, val in zip(order, vals):
            rad = math.radians(val); c, s = math.cos(rad), math.sin(rad)
            if ch.upper() == "XROTATION": rot = np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])
            elif ch.upper() == "YROTATION": rot = np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])
            elif ch.upper() == "ZROTATION": rot = np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
            else: continue
            mat = np.dot(mat, rot)
        return mat

    def _calculate_all_frame_positions(self):
        all_pos, scale = [], 1.0/100.0
        for frame_data in self.bvh.frames:
            positions, transforms, ch_idx = {}, {}, 0
            for node in self.bvh.ordered_nodes:
                n_ch = len(node.channels); node_vals = frame_data[ch_idx:ch_idx+n_ch]; ch_idx += n_ch
                parent_t = transforms.get(node.parent.name, np.identity(4)) if node.parent else np.identity(4)
                local_t = np.identity(4); local_t[0:3,3] = node.offset
                rot_ch = [c for c in node.channels if "rot" in c.lower()]; pos_ch = [c for c in node.channels if "pos" in c.lower()]
                
                if pos_ch:
                    pos_vals = [v for c,v in zip(node.channels, node_vals) if "pos" in c.lower()]
                    if "Xposition" in pos_ch: local_t[0,3]+=pos_vals[pos_ch.index("Xposition")]
                    if "Yposition" in pos_ch: local_t[1,3]+=pos_vals[pos_ch.index("Yposition")]
                    if "Zposition" in pos_ch: local_t[2,3]+=pos_vals[pos_ch.index("Zposition")]
                
                if rot_ch:
                    rot_vals = [v for c,v in zip(node.channels, node_vals) if "rot" in c.lower()]
                    rot_m = self._build_rot_mat(rot_vals, rot_ch); local_t = np.dot(local_t, rot_m)
                
                global_t = np.dot(parent_t, local_t); transforms[node.name] = global_t
                positions[node.name] = global_t[0:3,3] * scale
            all_pos.append(positions)
        return all_pos

def main():
    parser = argparse.ArgumentParser(description="Enhanced BVH analyzer with separated wrist time series analysis and image export.")
    parser.add_argument('bvh_file', help="Path to BVH file.")
    parser.add_argument('--track-wrist', action='store_true', help="Enables dynamic wrist tracking.")
    parser.add_argument('-l', '--loop', action='store_true', help="Makes the animation repeat in an infinite loop.")
    parser.add_argument('--export-only', action='store_true', help="Only export plots without showing animation.")
    parser.add_argument('--output-dir', default='./bvh_analysis', help="Directory to save exported plots (default: ./bvh_analysis).")
    args = parser.parse_args()

    try:
        reader = BVHReader(args.bvh_file)
        reader.read()
        
        analyzer = BVHAnalyzer(reader, trail_length=150, track_wrist=args.track_wrist, 
                              loop=args.loop, output_dir=args.output_dir)
        analyzer.run(export_only=args.export_only)
        
    except Exception as e:
        print("\nAn error occurred: {}".format(e)); traceback.print_exc()

if __name__ == '__main__':
    main()