# Represents information about a single detected object
# Bounding box points
int32 x1
int32 y1
int32 x2
int32 y2

# Detection results
float64 confidence  # Confidence score of the detection
int32 class_id      # Class identifier of the detected object
string class_name      # Class identifier of the detected object