# The pose structure contains position and orientation information of a given tag,
# defined with respect to the camera (a.k.a. static / world) frame.
# More concretely, this pose information together defines a transformation
# matrix T, such that:
#
# homogeneous_point_in_camera_frame = T * homogeneous_point_in_tag_frame
#
# Both the tag frame and camera frame adhere to right-handed conventions.
#
# +x in camera's frame: moving tag towards right of camera, a.k.a. towards left in image
# +y in camera's frame: moving tag towards bottom of camera, a.k.a. towards bottom in image
# +z in camera's frame: moving tag away from camera, a.k.a. decreasing size in image
#
# +x in tag's frame: towards right of tag image
# +y in tag's frame: towards bottom of tag image
# +z in tag's frame: point into tag image
geometry_msgs/Pose pose

float64 width
