#!/usr/bin/env python3
"""
Depth Calibration Helper Script

This script helps you calculate the correct depth scale factor for your camera.
Based on your measurements:
- Measured distance: 4.6 meters
- Reported distance: 8.0 meters
- Calculated scale factor: 4.6 / 8.0 = 0.575
"""

def calculate_depth_scale_factor(actual_distance, measured_distance):
    """Calculate the depth scale factor needed to correct depth measurements"""
    if measured_distance == 0:
        print("Error: Measured distance cannot be zero")
        return None
    
    scale_factor = actual_distance / measured_distance
    print(f"Actual distance: {actual_distance:.2f}m")
    print(f"Measured distance: {measured_distance:.2f}m")
    print(f"Required scale factor: {scale_factor:.6f}")
    return scale_factor

def main():
    print("=== Depth Calibration Helper ===")
    print()
    
    # Your current measurements
    actual = 4.6  # meters
    measured = 8.0  # meters
    
    scale_factor = calculate_depth_scale_factor(actual, measured)
    
    if scale_factor:
        print()
        print("=== ROS2 Parameter Commands ===")
        print(f"ros2 param set /pose_detection_node depth_scale_factor {scale_factor:.6f}")
        print()
        print("=== Launch File Parameter ===")
        print(f"<param name='depth_scale_factor' value='{scale_factor:.6f}' />")
        print()
        print("=== Test Different Distances ===")
        print(f"If person moves to 2m, system should show: {2.0 / scale_factor:.2f}m")
        print(f"If person moves to 6m, system should show: {6.0 / scale_factor:.2f}m")
        print()
        print("=== Alternative: Individual Calibration ===")
        print("You can also set the scale factor interactively:")
        
        try:
            print("\nEnter your own measurements:")
            actual_input = float(input("Actual distance (meters): "))
            measured_input = float(input("Measured distance from system (meters): "))
            
            custom_scale = calculate_depth_scale_factor(actual_input, measured_input)
            if custom_scale:
                print(f"\nCustom scale factor: {custom_scale:.6f}")
                print(f"ros2 param set /pose_detection_node depth_scale_factor {custom_scale:.6f}")
        
        except (ValueError, KeyboardInterrupt):
            print("\nUsing default values.")
    
    print("\n=== Quick Test Values ===")
    print("Common scale factors to try:")
    print("- 0.575 (based on your 4.6m/8m measurement)")
    print("- 0.5 (if depth is consistently 2x too large)")
    print("- 1.0 (no scaling)")
    print("- 2.0 (if depth is 2x too small)")

if __name__ == "__main__":
    main()
