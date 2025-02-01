#!/usr/bin/env python3
import rospy
import random
from motion_test_pkg.srv import RandomPose, RandomPoseResponse

# Workspace boundaries
X_MIN, X_MAX = -1.0, 1.0
Y_MIN, Y_MAX = -1.0, 1.0
Z_HEIGHT = 0.7  # Fixed Z height of the table

def handle_random_pose_request(req):
    """Generate a random pose and return it as a service response."""
    x = random.uniform(X_MIN, X_MAX)
    y = random.uniform(Y_MIN, Y_MAX)
    z = Z_HEIGHT
    rospy.loginfo(f"Generated random pose: x={x}, y={y}, z={z}")
    return RandomPoseResponse(x=x, y=y, z=z)

def main():
    rospy.init_node("random_pose_generator_node")
    service = rospy.Service("/random_pose_generator", RandomPose, handle_random_pose_request)
    rospy.loginfo("Random Pose Generator Service is ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
