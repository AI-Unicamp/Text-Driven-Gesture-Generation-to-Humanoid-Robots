kevao@kevao-System-Product-Name:~/pepper_sim_ws/src/scripts-funcionais/Motion-Analyzer$ python motion_comparison.py --bvh-data ./bvh_analysis/ --pepper-data ./pepper_analysis/
Motion retargeting analysis with temporal alignment...
============================================================
Loading motion data...
Original data loaded:
  BVH Right: 173 frames
  BVH Left: 173 frames
  Pepper Right: 187 frames
  Pepper Left: 187 frames

Removing initial delay of 0.80 seconds from Pepper data...
Removed initial 0.80s delay: 24 frames removed, 163 frames remaining
Removed initial 0.80s delay: 24 frames removed, 163 frames remaining
After delay removal:
  Pepper Right: 163 frames
  Pepper Left: 163 frames

DTW Results (after temporal alignment):
  RAW DATA:
    Right Wrist: 0.9282
    Left Wrist: 0.8353
    Average: 0.8817
  NORMALIZED DATA (scale + time aligned):
    Right Wrist: 0.2568
    Left Wrist: 0.1687
    Average: 0.2127

Interpolating temporally aligned data for analysis...
MAE Analysis (temporally aligned):
  RAW DATA:
    X - Right: 0.6843m, Left: 0.7692m
    Y - Right: 0.2829m, Left: 0.2798m
    Z - Right: 1.6313m, Left: 1.3849m
  NORMALIZED DATA:
    X - Right: 0.3187, Left: 0.2793
    Y - Right: 0.3344, Left: 0.2508
    Z - Right: 0.1576, Left: 0.1370
Correlation Analysis (temporally aligned):
  RAW DATA:
    X - Right: -0.065, Left: -0.045
    Y - Right: 0.022, Left: 0.240
    Z - Right: 0.714, Left: 0.870
  NORMALIZED DATA:
    X - Right: -0.065, Left: -0.045
    Y - Right: 0.022, Left: 0.240
    Z - Right: 0.714, Left: 0.870

============================================================
FINAL ASSESSMENT (temporally and scale aligned):
  Average Raw MAE:          0.8387m
  Average Normalized MAE:   0.2463
  Average Normalized DTW:   0.2127
  Average Normalized Corr:  0.289
  Assessment: POOR - Significant differences in movement patterns

METRIC INTERPRETATION:
  - MAE measures position accuracy (lower = better)
  - Correlation measures pattern similarity (higher = better)
  - DTW measures overall sequence similarity (lower = better)

TEMPORAL ALIGNMENT IMPACT:
By removing the initial 0.80s delay from Pepper, the metrics now
reflect the true movement similarity after proper temporal alignment.
============================================================
