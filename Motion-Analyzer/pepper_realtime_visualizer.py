#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from collections import deque
import threading
import time
from std_msgs.msg import String ### NEW ### Import String message type
plt.rcParams['font.family'] = 'DejaVu Sans'

class PepperRealtimeVisualizer:
    def __init__(self, trail_length=200, stillness_threshold=0.005, stillness_duration=3.0):
        self.listener = tf.TransformListener()
        self.reference_frame = 'map'
        
        self.skeleton_frames = {
            'base': 'base_footprint', 'torso': 'torso', 'head': 'Head',
            'neck': 'Neck', 'hip': 'Hip', 'pelvis': 'Pelvis',
            'l_shoulder': 'LShoulder', 'l_bicep': 'LBicep', 'l_elbow': 'LElbow',
            'l_forearm': 'LForeArm', 'l_wrist': 'l_wrist', 'r_shoulder': 'RShoulder',
            'r_bicep': 'RBicep', 'r_elbow': 'RElbow', 'r_forearm': 'RForeArm',
            'r_wrist': 'r_wrist',
        }
        
        self.skeleton_connections = [
            ('base', 'torso'), ('torso', 'neck'), ('neck', 'head'),
            ('torso', 'hip'), ('hip', 'pelvis'), ('torso', 'l_shoulder'),
            ('l_shoulder', 'l_bicep'), ('l_bicep', 'l_elbow'), ('l_elbow', 'l_forearm'),
            ('l_forearm', 'l_wrist'), ('torso', 'r_shoulder'), ('r_shoulder', 'r_bicep'),
            ('r_bicep', 'r_elbow'), ('r_elbow', 'r_forearm'), ('r_forearm', 'r_wrist'),
        ]
        
        self.r_wrist_full_history = []
        self.l_wrist_full_history = []
        
        self.running = True
        self.data_lock = threading.Lock()
        
        self.stillness_threshold_sq = stillness_threshold**2
        self.stillness_duration = rospy.Duration(stillness_duration)
        self.movement_stopped = False
        self.last_r_wrist_pos = None
        self.still_time_start = None
        
        ### NEW ### State logic to wait for start signal
        self.state = "WAITING_FOR_START"  # Possible states: WAITING_FOR_START, RECORDING
        self.status_sub = rospy.Subscriber(
            '/animation/status', String, self._status_callback, queue_size=1
        )
        
        plt.ion()
        rospy.loginfo("Real-time visualizer started.")
        self.wait_for_transforms()
    
    ### NEW ### Callback to handle /animation/status messages
    def _status_callback(self, msg):
        with self.data_lock:
            if msg.data == "START" and self.state == "WAITING_FOR_START":
                rospy.loginfo(">>> START signal received. Starting trajectory recording. <<<")
                # Reset history to record only the new animation
                self.r_wrist_full_history = []
                self.l_wrist_full_history = []
                self.state = "RECORDING"
            elif msg.data == "STOP":
                rospy.loginfo(">>> STOP signal received. Ending visualization. <<<")
                self.movement_stopped = True
                self.running = False
    
    def wait_for_transforms(self):
        rospy.loginfo("Waiting for transforms (TF) to be available...")
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(
                    self.reference_frame, 'r_wrist', rospy.Time(0), rospy.Duration(1.0)
                )
                rospy.loginfo("TF ready!")
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle(1.0, "Still waiting for TF...")
                rospy.sleep(0.5)
    
    def capture_skeleton(self):
        skeleton_positions = {}
        for joint_name, frame_name in self.skeleton_frames.items():
            try:
                (trans, _) = self.listener.lookupTransform(self.reference_frame, frame_name, rospy.Time(0))
                skeleton_positions[joint_name] = np.array(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                skeleton_positions[joint_name] = None
        return skeleton_positions
    
    def data_thread(self):
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            try:
                skeleton = self.capture_skeleton()
                with self.data_lock:
                    self.current_skeleton = skeleton
                    
                    ### MODIFICATION ###
                    # Only record and check for inactivity if we're in 'RECORDING' state
                    if self.state == "RECORDING":
                        if skeleton['r_wrist'] is not None:
                            current_pos = skeleton['r_wrist']
                            self.r_wrist_full_history.append(current_pos)
                            if self.last_r_wrist_pos is not None:
                                dist_sq = np.sum((current_pos - self.last_r_wrist_pos)**2)
                                if dist_sq < self.stillness_threshold_sq:
                                    if self.still_time_start is None:
                                        self.still_time_start = rospy.Time.now()
                                    elif rospy.Time.now() - self.still_time_start > self.stillness_duration:
                                        rospy.loginfo("Inactivity detected. Ending data capture.")
                                        self.movement_stopped = True
                                        self.running = False
                                else:
                                    self.still_time_start = None
                            self.last_r_wrist_pos = current_pos
                        if skeleton['l_wrist'] is not None:
                            self.l_wrist_full_history.append(skeleton['l_wrist'])
            except Exception as e:
                rospy.logwarn_throttle(1.0, "Error in data capture: %s", str(e))
            rate.sleep()
    
    def run_visualizer(self):
        data_thread = threading.Thread(target=self.data_thread)
        data_thread.daemon = True
        data_thread.start()
        
        fig = plt.figure(figsize=(16, 12))
        axes = (fig.add_subplot(221, projection='3d'),
                fig.add_subplot(222, projection='3d'),
                fig.add_subplot(223),
                fig.add_subplot(224))
        
        ### MODIFICATION ###
        rospy.loginfo("Visualizer ready. Waiting for 'START' signal on /animation/status topic...")
        rospy.loginfo("Launch your movement script to begin.")
        
        try:
            while not self.movement_stopped and not rospy.is_shutdown():
                self.update_plots(axes)
                plt.pause(0.05)
        except KeyboardInterrupt:
            rospy.loginfo("User interruption (Ctrl+C).")
        
        finally:
            self.running = False
            data_thread.join(timeout=1.0)
            
            if not self.r_wrist_full_history:
                rospy.logwarn("No movement data captured. Exiting.")
                return
            rospy.loginfo("Movement has ended. Showing final result.")
            rospy.loginfo("Close the plot window to exit the script.")
            
            self.update_plots(axes, final_plot=True)
            plt.ioff()
            plt.show()
    
    def update_plots(self, axes, final_plot=False):
        ax1, ax2, ax3, ax4 = axes
        
        with self.data_lock:
            current_skeleton = getattr(self, 'current_skeleton', {})
            r_trail = list(self.r_wrist_full_history)
            l_trail = list(self.l_wrist_full_history)
        
        for ax in axes: ax.clear()
        
        # Dynamic titles based on state
        if self.state == "WAITING_FOR_START" and not final_plot:
            ax1.set_title('Pepper - Waiting for Start...')
        else:
            ax1.set_title('Pepper - Final Position' if final_plot else 'Pepper - Real Time')
        
        self.draw_skeleton(ax1, current_skeleton, 'blue')
        if r_trail: ax1.plot(np.array(r_trail)[:,0], np.array(r_trail)[:,1], np.array(r_trail)[:,2], 'r', lw=2, alpha=0.7, label='R_Wrist')
        if l_trail: ax1.plot(np.array(l_trail)[:,0], np.array(l_trail)[:,1], np.array(l_trail)[:,2], 'g', lw=2, alpha=0.7, label='L_Wrist')
        if self.state != "WAITING_FOR_START" or final_plot: ax1.legend()
        
        ax2.set_title('Complete Right Wrist Trajectory')
        ax3.set_title('Top View (XY)')
        ax4.set_title('Z Height vs Time')
        
        if r_trail:
            trail_array = np.array(r_trail)
            colors = plt.cm.viridis(np.linspace(0, 1, len(trail_array)))
            ax2.scatter(trail_array[:,0], trail_array[:,1], trail_array[:,2], c=colors, s=20)
            ax2.plot(trail_array[:,0], trail_array[:,1], trail_array[:,2], 'r', lw=1, alpha=0.5)
            ax3.plot(trail_array[:,0], trail_array[:,1], 'r', lw=2)
            ax3.scatter(trail_array[-1,0], trail_array[-1,1], c='r', s=100, marker='o', label='Current')
            if final_plot:
                ax3.scatter(trail_array[0,0], trail_array[0,1], c='b', s=100, marker='x', label='Start')
                ax3.legend()
            z_history = trail_array[:, 2]
            time_history = np.arange(len(z_history)) / 30.0
            ax4.plot(time_history, z_history, 'b', lw=2)
            
            xlim = [trail_array[:,0].min()-0.15, trail_array[:,0].max()+0.15]
            ylim = [trail_array[:,1].min()-0.15, trail_array[:,1].max()+0.15]
            zlim = [trail_array[:,2].min()-0.15, trail_array[:,2].max()+0.15]
            ax1.set_xlim(xlim); ax1.set_ylim(ylim); ax1.set_zlim(zlim)
            ax2.set_xlim(xlim); ax2.set_ylim(ylim); ax2.set_zlim(zlim)
            ax3.set_xlim(xlim); ax3.set_ylim(ylim)
        
        ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
        ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)'); ax2.set_zlabel('Z (m)')
        ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)'); ax3.grid(True); ax3.set_aspect('equal')
        ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Z (m)'); ax4.grid(True)
        plt.tight_layout()
    
    def draw_skeleton(self, ax, skeleton_data, color):
        if not skeleton_data: return
        for joint1, joint2 in self.skeleton_connections:
            pos1, pos2 = skeleton_data.get(joint1), skeleton_data.get(joint2)
            if pos1 is not None and pos2 is not None:
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]], color=color, lw=3, alpha=0.8)
        positions = np.array([pos for pos in skeleton_data.values() if pos is not None])
        if positions.size > 0:
            ax.scatter(positions[:,0], positions[:,1], positions[:,2], c='k', s=50, alpha=0.6)

def main():
    rospy.init_node('pepper_realtime_visualizer_node')
    try:
        visualizer = PepperRealtimeVisualizer(stillness_threshold=0.005, stillness_duration=3.0)
        visualizer.run_visualizer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", str(e))

if __name__ == '__main__':
    main()