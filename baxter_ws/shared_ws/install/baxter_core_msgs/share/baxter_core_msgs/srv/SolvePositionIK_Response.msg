
# joints[i]      == joint angle solution for each pose_state[i]
sensor_msgs/JointState[] joints 

# NOTE: isValid will be deprecated by result_type in future versions
bool[] is_valid 

# result_type[i] == seed type used to find valid solution, joints[i];
# otherwise,     == RESULT_INVALID (no valid solution found).
uint8 RESULT_INVALID = 0
uint8[] result_type 