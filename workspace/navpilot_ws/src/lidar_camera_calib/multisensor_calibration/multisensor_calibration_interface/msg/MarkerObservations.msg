########################################################################
#
#   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
#   System Technologies and Image Exploitation IOSB
#
########################################################################

# IDs of markers for which to add the reference points
uint32[] marker_ids

# Position of the top-left corner. In case of 2D observations (i.e. image points) only 
# x and y coordinates are used.
geometry_msgs/Point[] marker_top_left_point