#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse
from scipy.stats import pearsonr
from scipy.spatial.distance import euclidean
from scipy.interpolate import interp1d
import warnings
warnings.filterwarnings('ignore')

def dtw_distance(seq1, seq2):
    n, m = len(seq1), len(seq2)
    dtw_matrix = np.full((n + 1, m + 1), np.inf)
    dtw_matrix[0, 0] = 0
    
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = euclidean(seq1[i-1], seq2[j-1])
            dtw_matrix[i, j] = cost + min(dtw_matrix[i-1, j],
                                         dtw_matrix[i, j-1],
                                         dtw_matrix[i-1, j-1])
    
    return dtw_matrix[n, m] / (n + m)

def normalize_by_range(data):
    normalized = np.zeros_like(data)
    for i in range(data.shape[1]):
        axis_data = data[:, i]
        min_val, max_val = axis_data.min(), axis_data.max()
        if max_val != min_val:
            normalized[:, i] = (axis_data - min_val) / (max_val - min_val)
        else:
            normalized[:, i] = 0.5
    return normalized

def remove_initial_delay(pepper_data, pepper_times, delay_seconds):
    """Remove initial delay from Pepper data"""
    # Find the index where time >= delay_seconds
    start_idx = np.where(pepper_times >= delay_seconds)[0]
    
    if len(start_idx) == 0:
        print("Warning: delay_seconds ({}) is longer than total recording time".format(delay_seconds))
        return pepper_data, pepper_times
    
    start_idx = start_idx[0]
    
    # Remove initial frames
    pepper_data_trimmed = pepper_data[start_idx:]
    pepper_times_trimmed = pepper_times[start_idx:] - pepper_times[start_idx]  # Reset time to start at 0
    
    print("Removed initial {:.2f}s delay: {} frames removed, {} frames remaining".format(
        delay_seconds, start_idx, len(pepper_data_trimmed)))
    
    return pepper_data_trimmed, pepper_times_trimmed

def interpolate_to_same_length(data1, data2):
    """Interpolate both datasets to the same length (minimum of both)"""
    min_len = min(len(data1), len(data2))
    
    if len(data1) == len(data2):
        return data1, data2
    
    # Create time arrays
    time1 = np.linspace(0, 1, len(data1))
    time2 = np.linspace(0, 1, len(data2))
    common_time = np.linspace(0, 1, min_len)
    
    # Interpolate each axis
    data1_interp = np.zeros((min_len, data1.shape[1]))
    data2_interp = np.zeros((min_len, data2.shape[1]))
    
    for axis in range(data1.shape[1]):
        f1 = interp1d(time1, data1[:, axis], kind='linear')
        f2 = interp1d(time2, data2[:, axis], kind='linear')
        data1_interp[:, axis] = f1(common_time)
        data2_interp[:, axis] = f2(common_time)
    
    return data1_interp, data2_interp

class MotionComparison:
    def __init__(self, bvh_dir, pepper_dir, output_dir='./comparison_results', delay_seconds=0.8):
        self.bvh_dir = bvh_dir
        self.pepper_dir = pepper_dir
        self.output_dir = output_dir
        self.delay_seconds = delay_seconds  # Initial delay to remove from Pepper
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print("Created output directory: {}".format(self.output_dir))
    
    def load_data(self):
        print("Loading motion data...")
        
        bvh_r_path = os.path.join(self.bvh_dir, 'bvh_right_wrist_data.csv')
        bvh_l_path = os.path.join(self.bvh_dir, 'bvh_left_wrist_data.csv')
        pepper_r_path = os.path.join(self.pepper_dir, 'pepper_right_wrist_data.csv')
        pepper_l_path = os.path.join(self.pepper_dir, 'pepper_left_wrist_data.csv')
        
        for path, name in [(bvh_r_path, 'BVH right'), (bvh_l_path, 'BVH left'), 
                          (pepper_r_path, 'Pepper right'), (pepper_l_path, 'Pepper left')]:
            if not os.path.exists(path):
                raise FileNotFoundError("{} wrist data not found: {}".format(name, path))
        
        # Load BVH data
        bvh_r_df = pd.read_csv(bvh_r_path)
        bvh_l_df = pd.read_csv(bvh_l_path)
        self.bvh_r = bvh_r_df.values[:, 1:4]  # X,Y,Z
        self.bvh_l = bvh_l_df.values[:, 1:4]
        
        # Load Pepper data with time info
        pepper_r_df = pd.read_csv(pepper_r_path)
        pepper_l_df = pd.read_csv(pepper_l_path)
        
        pepper_r_times = pepper_r_df.values[:, 0]  # Time column
        pepper_l_times = pepper_l_df.values[:, 0]
        pepper_r_original = pepper_r_df.values[:, 1:4]  # X,Y,Z
        pepper_l_original = pepper_l_df.values[:, 1:4]
        
        print("Original data loaded:")
        print("  BVH Right: {} frames".format(self.bvh_r.shape[0]))
        print("  BVH Left: {} frames".format(self.bvh_l.shape[0]))
        print("  Pepper Right: {} frames".format(pepper_r_original.shape[0]))
        print("  Pepper Left: {} frames".format(pepper_l_original.shape[0]))
        
        # Remove initial delay from Pepper data
        if self.delay_seconds > 0:
            print("\nRemoving initial delay of {:.2f} seconds from Pepper data...".format(self.delay_seconds))
            self.pepper_r, _ = remove_initial_delay(pepper_r_original, pepper_r_times, self.delay_seconds)
            self.pepper_l, _ = remove_initial_delay(pepper_l_original, pepper_l_times, self.delay_seconds)
            
            print("After delay removal:")
            print("  Pepper Right: {} frames".format(self.pepper_r.shape[0]))
            print("  Pepper Left: {} frames".format(self.pepper_l.shape[0]))
        else:
            self.pepper_r = pepper_r_original
            self.pepper_l = pepper_l_original
        
        return True
    
    def run_analysis(self):
        print("Motion retargeting analysis with temporal alignment...")
        print("="*60)
        
        self.load_data()
        
        # Calculate raw DTW
        dtw_r_raw = dtw_distance(self.bvh_r, self.pepper_r)
        dtw_l_raw = dtw_distance(self.bvh_l, self.pepper_l)
        
        # Calculate normalized DTW
        bvh_r_norm = normalize_by_range(self.bvh_r)
        pepper_r_norm = normalize_by_range(self.pepper_r)
        bvh_l_norm = normalize_by_range(self.bvh_l)
        pepper_l_norm = normalize_by_range(self.pepper_l)
        
        dtw_r_norm = dtw_distance(bvh_r_norm, pepper_r_norm)
        dtw_l_norm = dtw_distance(bvh_l_norm, pepper_l_norm)
        
        print("\nDTW Results (after temporal alignment):")
        print("  RAW DATA:")
        print("    Right Wrist: {:.4f}".format(dtw_r_raw))
        print("    Left Wrist: {:.4f}".format(dtw_l_raw))
        print("    Average: {:.4f}".format((dtw_r_raw + dtw_l_raw) / 2))
        
        print("  NORMALIZED DATA (scale + time aligned):")
        print("    Right Wrist: {:.4f}".format(dtw_r_norm))
        print("    Left Wrist: {:.4f}".format(dtw_l_norm))
        print("    Average: {:.4f}".format((dtw_r_norm + dtw_l_norm) / 2))
        
        # Interpolate data for correlation and MAE analysis
        print("\nInterpolating temporally aligned data for analysis...")
        bvh_r_interp, pepper_r_interp = interpolate_to_same_length(self.bvh_r, self.pepper_r)
        bvh_l_interp, pepper_l_interp = interpolate_to_same_length(self.bvh_l, self.pepper_l)
        
        bvh_r_norm_interp, pepper_r_norm_interp = interpolate_to_same_length(bvh_r_norm, pepper_r_norm)
        bvh_l_norm_interp, pepper_l_norm_interp = interpolate_to_same_length(bvh_l_norm, pepper_l_norm)
        
        # Calculate MAE for each axis
        print("MAE Analysis (temporally aligned):")
        axes = ['X', 'Y', 'Z']
        
        print("  RAW DATA:")
        mae_raw_values = []
        for i, axis in enumerate(axes):
            mae_r = np.mean(np.abs(bvh_r_interp[:, i] - pepper_r_interp[:, i]))
            mae_l = np.mean(np.abs(bvh_l_interp[:, i] - pepper_l_interp[:, i]))
            mae_raw_values.extend([mae_r, mae_l])
            print("    {} - Right: {:.4f}m, Left: {:.4f}m".format(axis, mae_r, mae_l))
        
        print("  NORMALIZED DATA:")
        mae_norm_values = []
        for i, axis in enumerate(axes):
            mae_r_norm = np.mean(np.abs(bvh_r_norm_interp[:, i] - pepper_r_norm_interp[:, i]))
            mae_l_norm = np.mean(np.abs(bvh_l_norm_interp[:, i] - pepper_l_norm_interp[:, i]))
            mae_norm_values.extend([mae_r_norm, mae_l_norm])
            print("    {} - Right: {:.4f}, Left: {:.4f}".format(axis, mae_r_norm, mae_l_norm))
        
        # Calculate correlations
        print("Correlation Analysis (temporally aligned):")
        
        print("  RAW DATA:")
        r_corrs_raw = []
        l_corrs_raw = []
        for i, axis in enumerate(axes):
            r_corr, _ = pearsonr(bvh_r_interp[:, i], pepper_r_interp[:, i])
            l_corr, _ = pearsonr(bvh_l_interp[:, i], pepper_l_interp[:, i])
            r_corrs_raw.append(r_corr)
            l_corrs_raw.append(l_corr)
            print("    {} - Right: {:.3f}, Left: {:.3f}".format(axis, r_corr, l_corr))
        
        print("  NORMALIZED DATA:")
        r_corrs_norm = []
        l_corrs_norm = []
        for i, axis in enumerate(axes):
            r_corr, _ = pearsonr(bvh_r_norm_interp[:, i], pepper_r_norm_interp[:, i])
            l_corr, _ = pearsonr(bvh_l_norm_interp[:, i], pepper_l_norm_interp[:, i])
            r_corrs_norm.append(r_corr)
            l_corrs_norm.append(l_corr)
            print("    {} - Right: {:.3f}, Left: {:.3f}".format(axis, r_corr, l_corr))
        
        # Summary
        avg_dtw_norm = (dtw_r_norm + dtw_l_norm) / 2
        avg_corr_norm = (sum(r_corrs_norm) + sum(l_corrs_norm)) / 6
        avg_mae_norm = np.mean(mae_norm_values)
        avg_mae_raw = np.mean(mae_raw_values)
        
        print("\n" + "="*60)
        print("FINAL ASSESSMENT (temporally and scale aligned):")
        print("  Average Raw MAE:          {:.4f}m".format(avg_mae_raw))
        print("  Average Normalized MAE:   {:.4f}".format(avg_mae_norm))
        print("  Average Normalized DTW:   {:.4f}".format(avg_dtw_norm))
        print("  Average Normalized Corr:  {:.3f}".format(avg_corr_norm))
        
        if avg_dtw_norm < 0.15 and avg_corr_norm > 0.7 and avg_mae_norm < 0.2:
            assessment = "EXCELLENT - Retargeting preserves movement patterns very well"
        elif avg_dtw_norm < 0.25 and avg_corr_norm > 0.5 and avg_mae_norm < 0.3:
            assessment = "GOOD - Acceptable retargeting quality"
        elif avg_dtw_norm < 0.4 and avg_corr_norm > 0.3 and avg_mae_norm < 0.5:
            assessment = "FAIR - Moderate retargeting quality"
        else:
            assessment = "POOR - Significant differences in movement patterns"
            
        print("  Assessment: {}".format(assessment))
        print("\nMETRIC INTERPRETATION:")
        print("  - MAE measures position accuracy (lower = better)")
        print("  - Correlation measures pattern similarity (higher = better)")
        print("  - DTW measures overall sequence similarity (lower = better)")
        print("\nTEMPORAL ALIGNMENT IMPACT:")
        print("By removing the initial {:.2f}s delay from Pepper, the metrics now".format(self.delay_seconds))
        print("reflect the true movement similarity after proper temporal alignment.")
        print("="*60)

def main():
    parser = argparse.ArgumentParser(description="Motion comparison with temporal alignment and MAE analysis")
    parser.add_argument('--bvh-data', required=True, help="BVH data directory")
    parser.add_argument('--pepper-data', required=True, help="Pepper data directory")
    parser.add_argument('--output-dir', default='./comparison_results', help="Output directory")
    parser.add_argument('--delay', type=float, default=0.8, 
                       help="Initial delay in seconds to remove from Pepper data (default: 0.8)")
    
    args = parser.parse_args()
    
    try:
        comparator = MotionComparison(args.bvh_data, args.pepper_data, args.output_dir, args.delay)
        comparator.run_analysis()
    except Exception as e:
        print("\nError: {}".format(e))
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()