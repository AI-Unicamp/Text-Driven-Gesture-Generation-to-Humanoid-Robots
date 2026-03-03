#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pepper Data Collector - Simple data capture without real-time visualization
Only captures TF data and exports CSV + images at the end.

Usage:
    python pepper_data_collector.py --output-dir ./pepper_analysis/
"""
import rospy
import tf
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
import os
from std_msgs.msg import String
import argparse
plt.rcParams['font.family'] = 'DejaVu Sans'

class PepperDataCollector:
    def __init__(self, output_dir='./pepper_analysis'):
        self.listener = tf.TransformListener()
        self.reference_frame = 'map'
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print("Created output directory: {}".format(self.output_dir))
        
        self.r_wrist_full_history = []
        self.l_wrist_full_history = []
        self.timestamps = []
        
        self.running = True
        self.data_lock = threading.Lock()
        
        # State logic for waiting for start signal
        self.state = "WAITING_FOR_START"
        self.recording_start_time = None
        self.status_sub = rospy.Subscriber('/animation/status', String, self._status_callback, queue_size=1)
        
        print("Pepper data collector initialized.")
        self.wait_for_transforms()

    def _status_callback(self, msg):
        with self.data_lock:
            if msg.data == "START" and self.state == "WAITING_FOR_START":
                print(">>> START signal received. Beginning data recording. <<<")
                self.r_wrist_full_history = []
                self.l_wrist_full_history = []
                self.timestamps = []
                self.recording_start_time = rospy.Time.now()
                self.state = "RECORDING"
            elif msg.data == "STOP":
                print(">>> STOP signal received. Stopping data recording and exporting. <<<")
                self.running = False

    def wait_for_transforms(self):
        print("Waiting for transforms (TF) to be available...")
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(self.reference_frame, 'r_wrist', rospy.Time(0), rospy.Duration(1.0))
                print("TF ready!")
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Still waiting for TF...")
                rospy.sleep(0.5)

    def capture_wrist_positions(self):
        """Capture both wrist positions"""
        try:
            # Right wrist
            (r_trans, _) = self.listener.lookupTransform(self.reference_frame, 'r_wrist', rospy.Time(0))
            # Left wrist  
            (l_trans, _) = self.listener.lookupTransform(self.reference_frame, 'l_wrist', rospy.Time(0))
            return np.array(r_trans), np.array(l_trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    def data_collection_thread(self):
        """Main data collection loop"""
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            try:
                if self.state == "RECORDING":
                    current_time = rospy.Time.now()
                    time_since_start = (current_time - self.recording_start_time).to_sec()
                    
                    r_pos, l_pos = self.capture_wrist_positions()
                    
                    if r_pos is not None and l_pos is not None:
                        with self.data_lock:
                            self.r_wrist_full_history.append(r_pos)
                            self.l_wrist_full_history.append(l_pos)
                            self.timestamps.append(time_since_start)
                            
                            # Print progress occasionally
                            if len(self.timestamps) % 30 == 0:  # Every second at 30fps
                                print("Recording... {} frames captured ({:.1f}s)".format(len(self.timestamps), time_since_start))
                                
            except Exception as e:
                print("Error in data capture: {}".format(str(e)))
            rate.sleep()

    def export_raw_data(self):
        """Export raw wrist position data for comparison analysis"""
        if not self.r_wrist_full_history or not self.l_wrist_full_history:
            print("No data to export for comparison analysis.")
            return
            
        print("Exporting raw wrist data for comparison analysis...")
        
        r_wrist_array = np.array(self.r_wrist_full_history)
        l_wrist_array = np.array(self.l_wrist_full_history)
        times = np.array(self.timestamps)
        
        # Export right wrist data
        r_wrist_data = np.column_stack([times, r_wrist_array[:, 0], r_wrist_array[:, 1], r_wrist_array[:, 2]])
        r_wrist_path = os.path.join(self.output_dir, 'pepper_right_wrist_data.csv')
        np.savetxt(r_wrist_path, r_wrist_data, delimiter=',', header='time,x,y,z', comments='', fmt='%.6f')
        
        # Export left wrist data  
        l_wrist_data = np.column_stack([times, l_wrist_array[:, 0], l_wrist_array[:, 1], l_wrist_array[:, 2]])
        l_wrist_path = os.path.join(self.output_dir, 'pepper_left_wrist_data.csv')
        np.savetxt(l_wrist_path, l_wrist_data, delimiter=',', header='time,x,y,z', comments='', fmt='%.6f')
        
        # Export metadata
        metadata = {
            'total_frames': len(times),
            'recording_duration': times[-1] if len(times) > 0 else 0,
            'average_fps': len(times) / times[-1] if len(times) > 1 and times[-1] > 0 else 0,
            'right_wrist_joint': 'r_wrist',
            'left_wrist_joint': 'l_wrist',
            'source_system': 'Pepper_ROS_Gazebo'
        }
        
        metadata_path = os.path.join(self.output_dir, 'pepper_metadata.txt')
        with open(metadata_path, 'w') as f:
            for key, value in metadata.items():
                f.write('{}={}\n'.format(key, value))
        
        print("Raw data exported:")
        print("- Right wrist: {}".format(r_wrist_path))
        print("- Left wrist: {}".format(l_wrist_path))
        print("- Metadata: {}".format(metadata_path))

    def export_images(self):
        """Export analysis images without real-time visualization"""
        if not self.r_wrist_full_history or not self.l_wrist_full_history:
            print("No data to export images.")
            return
            
        print("Generating analysis images...")
        
        r_wrist_array = np.array(self.r_wrist_full_history)
        l_wrist_array = np.array(self.l_wrist_full_history)
        times = np.array(self.timestamps)
        
        # Right wrist individual plots
        for i, axis_name in enumerate(['x', 'y', 'z']):
            plt.figure(figsize=(12, 8), dpi=100)
            plt.plot(times, r_wrist_array[:,i], 'blue', lw=3, alpha=0.8, 
                    label='Right Wrist {} Position'.format(axis_name.upper()), antialiased=True)
            plt.title('Pepper Right Wrist - {} Position vs Time'.format(axis_name.upper()), fontsize=14, fontweight='bold')
            plt.xlabel('Time (s)', fontsize=12)
            plt.ylabel('{} Position (m)'.format(axis_name.upper()), fontsize=12)
            plt.grid(True, alpha=0.3)
            legend = plt.legend(fontsize=11, loc='upper right', frameon=True, fancybox=True, shadow=True)
            legend.get_frame().set_facecolor('white')
            legend.get_frame().set_alpha(0.9)
            plt.tight_layout()
            plt.savefig(os.path.join(self.output_dir, 'pepper_right_wrist_{}_vs_time.png'.format(axis_name)), 
                       dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
            plt.close()
            
        # Left wrist individual plots
        for i, axis_name in enumerate(['x', 'y', 'z']):
            plt.figure(figsize=(12, 8), dpi=100)
            plt.plot(times, l_wrist_array[:,i], 'blue', lw=3, alpha=0.8, 
                    label='Left Wrist {} Position'.format(axis_name.upper()), antialiased=True)
            plt.title('Pepper Left Wrist - {} Position vs Time'.format(axis_name.upper()), fontsize=14, fontweight='bold')
            plt.xlabel('Time (s)', fontsize=12)
            plt.ylabel('{} Position (m)'.format(axis_name.upper()), fontsize=12)
            plt.grid(True, alpha=0.3)
            legend = plt.legend(fontsize=11, loc='upper right', frameon=True, fancybox=True, shadow=True)
            legend.get_frame().set_facecolor('white')
            legend.get_frame().set_alpha(0.9)
            plt.tight_layout()
            plt.savefig(os.path.join(self.output_dir, 'pepper_left_wrist_{}_vs_time.png'.format(axis_name)), 
                       dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
            plt.close()
        
        # 3D trajectory plots
        from mpl_toolkits.mplot3d import Axes3D
        
        # Right wrist 3D
        fig = plt.figure(figsize=(12, 10), dpi=100)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(r_wrist_array[:,0], r_wrist_array[:,1], r_wrist_array[:,2], 
                'blue', lw=3, alpha=0.8, label='Right Wrist Trajectory')
        ax.set_xlabel('X Position (m)', fontsize=11)
        ax.set_ylabel('Y Position (m)', fontsize=11)
        ax.set_zlabel('Z Position (m)', fontsize=11)
        ax.set_title('Pepper Right Wrist - Complete 3D Trajectory', fontsize=12, fontweight='bold')
        legend = ax.legend(fontsize=10, loc='upper right', frameon=True, fancybox=True, shadow=True)
        legend.get_frame().set_facecolor('white')
        legend.get_frame().set_alpha(0.9)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'pepper_right_wrist_3d_trajectory.png'), 
                   dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
        plt.close()
        
        # Left wrist 3D
        fig = plt.figure(figsize=(12, 10), dpi=100)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(l_wrist_array[:,0], l_wrist_array[:,1], l_wrist_array[:,2], 
                'blue', lw=3, alpha=0.8, label='Left Wrist Trajectory')
        ax.set_xlabel('X Position (m)', fontsize=11)
        ax.set_ylabel('Y Position (m)', fontsize=11)
        ax.set_zlabel('Z Position (m)', fontsize=11)
        ax.set_title('Pepper Left Wrist - Complete 3D Trajectory', fontsize=12, fontweight='bold')
        legend = ax.legend(fontsize=10, loc='upper right', frameon=True, fancybox=True, shadow=True)
        legend.get_frame().set_facecolor('white')
        legend.get_frame().set_alpha(0.9)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'pepper_left_wrist_3d_trajectory.png'), 
                   dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
        plt.close()
        
        print("Images exported successfully!")

    def run(self):
        """Main execution function"""
        print("Pepper data collector ready.")
        print("Waiting for 'START' signal on /animation/status topic...")
        print("Launch your movement script to begin data recording.")
        
        # Start data collection thread
        data_thread = threading.Thread(target=self.data_collection_thread)
        data_thread.daemon = True
        data_thread.start()
        
        try:
            # Wait for data collection to complete
            while self.running and not rospy.is_shutdown():
                rospy.sleep(0.1)
        except KeyboardInterrupt:
            print("Interrupted by user (Ctrl+C).")
            self.running = False
        
        finally:
            data_thread.join(timeout=1.0)
            
            if not self.r_wrist_full_history:
                print("No movement data captured. Exiting.")
                return
            
            print("Data collection completed!")
            print("Captured {} frames over {:.2f} seconds".format(len(self.r_wrist_full_history), self.timestamps[-1]))
            
            # Export everything
            self.export_raw_data()
            self.export_images()
            
            print("All data and images exported to: {}".format(self.output_dir))

def main():
    parser = argparse.ArgumentParser(description="Pepper data collector - CSV export without real-time visualization")
    parser.add_argument('--output-dir', default='./pepper_analysis', 
                       help="Directory to save data and images (default: ./pepper_analysis)")
    
    args = parser.parse_args()
    
    rospy.init_node('pepper_data_collector_node')
    
    try:
        collector = PepperDataCollector(output_dir=args.output_dir)
        collector.run()
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        print("An error occurred: {}".format(str(e)))

if __name__ == '__main__':
    main()