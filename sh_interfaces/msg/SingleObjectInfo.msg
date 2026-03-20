# Represents information about a single detected object
# 3D position
float64 x
float64 y
float64 z
float64[3] grasp_orientation_euler
float64[4] grasp_orientation_quaternion

# Detection results
float64 confidence  # Confidence score of the detection
uint8 class_id      # Class identifier of the detected object
float64 distance    # Distance from the robot to the object in meters