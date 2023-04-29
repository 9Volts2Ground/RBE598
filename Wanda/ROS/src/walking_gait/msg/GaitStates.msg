Header header

# Define gait types
uint8 RIPPLE=0
uint8 TRIPOD=1
uint8 WAVE=2

uint8 gait_type # Assign one of the enumerated gait types above
float64 max_stride_length
float64 min_stride_length
float64 stride_length
geometry_msgs/Point[] foot_center
