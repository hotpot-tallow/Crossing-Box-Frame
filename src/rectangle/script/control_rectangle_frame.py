#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

def pose_publisher():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    poses_msg = ModelState()
    poses_msg.model_name = 'my_model_rectangle'
    poses_msg.twist = Pose()
    poses_msg.twist = Twist()
    poses_msg.pose.position.x = 10
    poses_msg.pose.position.y = 3
    poses_msg.pose.position.z = 4.5
    poses_msg.pose.orientation.w = 0.7071
    poses_msg.pose.orientation.x = 0
    poses_msg.pose.orientation.y = 0
    poses_msg.pose.orientation.z = 0.7071
    f = 20
    v = 0.5
    direction = 1.0
    rate = rospy.Rate(f)
    while not rospy.is_shutdown():
        #for i in range(2):
        if poses_msg.pose.position.y < -3 :
            direction = 1.0
        if poses_msg.pose.position.y > 3 :
            direction = -1.0
        poses_msg.pose.position.y = poses_msg.pose.position.y + direction * v / f
        pub.publish(poses_msg)
        rate.sleep()
if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher()
      except rospy.ROSInterruptException:
          pass
