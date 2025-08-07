# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.
geometry_msgs/PoseStamped[] pose_stamp 

# (optional) Joint Angle Seed(s) for IK solver.
# * specify a JointState seed for each pose_stamp, using name[] and position[]
# * empty arrays or a non-default seed_mode will cause user seed to not be used
sensor_msgs/JointState[] seed_angles 

# Seed Type Mode
# * default (SEED_AUTO) mode: iterate through seed types until first valid
#                             solution is found
# * setting any other mode:   try only that seed type
uint8 SEED_AUTO = 0
uint8 SEED_USER = 1
uint8 SEED_CURRENT = 2
uint8 SEED_NS_MAP = 3

uint8 seed_mode 
