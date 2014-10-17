hand_detector
===========

Implement node for the hand detection with primesense sensor

The node considers all points within a fixed 3D box in front of it and returns the coordinates of the closest point in a geometry_msgs::Point.msg message (in camera coordinates).

If no point can be detected the message contains coordinates (0.0, 0.0, 0.0)
