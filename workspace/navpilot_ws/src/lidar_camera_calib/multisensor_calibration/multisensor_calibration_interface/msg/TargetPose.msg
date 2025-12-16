########################################################################
#
#   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
#   System Technologies and Image Exploitation IOSB
#
########################################################################

std_msgs/Header header

# Flag indicating if pose is an actual detection or just a preview
bool is_detection

# Pose of the calibration target in REF_2_LOCAL format. Meaning a point in the reference coordinate
# system is transformed with this pose into the local coordinate system of the calibration target.
geometry_msgs/Pose target_pose