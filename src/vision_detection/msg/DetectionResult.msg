std_msgs/Header header

# Detection status
bool detected
float32 confidence

# Pixel coordinates (bounding box)
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height

# Camera frame coordinates (meters)
geometry_msgs/Point camera_coords

# NED world coordinates (meters)
geometry_msgs/Point ned_coords
bool ned_valid
float32 depth
