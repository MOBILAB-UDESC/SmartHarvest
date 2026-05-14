# Represents information about a single detected object
# 3D position
geometry_msgs/Pose pose

# Detection results
float64 confidence  # Confidence score of the detection
uint8 class_id      # Class identifier of the detected object
float64 distance    # Distance from the robot to the object in meters