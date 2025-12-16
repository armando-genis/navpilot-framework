########################################################################
#
#   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
#   System Technologies and Image Exploitation IOSB
#
########################################################################

# Flag indicating if calibration was successful
bool is_successful

# Frame id of source sensor.
string src_frame_id

# Frame id of reference sensor.
string ref_frame_id

# Base frame id. If empty, the pose is given between the source and the ref frame id
string base_frame_id

# Calibrated sensor extrinsics, given as LOCAL_2_REF. This means that the given translation and 
# rotation of the source frame ID is given with respect to the reference frame ID.
geometry_msgs/Transform sensor_extrinsics