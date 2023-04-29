Header header

# Define target tracking states
uint8 SEARCH=0
uint8 TRACK=1
uint8 LOST=2
uint8 ACQUIRED=3

uint8 tracking_state
uint8 tracking_state_prev

geometry_msgs/Point tgt_pos_in_frame
