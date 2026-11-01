# Progress of the harvest mission, published when the state changes and at 2 Hz.
std_msgs/Header header

uint8 IDLE = 0                # waiting for a Harvest goal
uint8 STARTING = 1            # waiting for the robot's inputs, moving the arm to its survey pose
uint8 DRIVING = 2             # following the row, scanning for fruit
uint8 CONFIRMING = 3          # stopped; collecting detections to confirm a fruit
uint8 APPROACHING = 4         # arm moving to the pre-cut pose
uint8 ENGAGING = 5            # arm moving the cutter onto the pedicel
uint8 CUTTING = 6             # blade closing
uint8 CARRYING = 7            # arm carrying the fruit to the crate
uint8 DROPPING = 8            # trap door open over the crate
uint8 DONE = 9                # goal finished normally
uint8 FAILED = 10             # goal aborted; see `detail`
uint8 CANCELED = 11           # goal canceled by the client

uint8 state
string detail                 # one line, human readable
uint32 attempts               # picks attempted so far in this goal
uint32 verified               # picks verified in the crate so far
bool focus_valid
geometry_msgs/Point focus     # point of interest in the world frame (the fruit while picking, else the rover)
