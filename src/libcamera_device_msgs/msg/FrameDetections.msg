# All detections for a particular frame.

std_msgs/Header header
Detection[] detections

# Appearance features for the frame (serialized numpy array)
# Increase maximum size if needed, but this is sufficient for all the models
# that we're currently using.
uint8[<=3500000] appearance_features
# Original appearance feature shape
uint16[4] appearance_feature_shape
