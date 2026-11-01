# What the row follower sees in one front-camera image.
#
# `near` and `far` are where the furrow's centre line crosses the bottom and the
# top of the image band the follower uses, in half-widths of the image from its
# centre: 0 is dead centre, +1 is the right edge, -1 the left edge. `near`
# mostly measures sideways offset, `far` mostly heading.
std_msgs/Header header        # stamp and frame of the image
float32 near                  # [-1 .. 1]; 0 when tracking is false
float32 far                   # [-1 .. 1]; 0 when tracking is false
float32 confidence            # [0 .. 1] how well a line fits the furrow; 0 = no furrow found
bool tracking                 # confidence is at or above the follower's min_confidence
