#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import tf

rospy.init_node('odom_publisher')

odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)


listener = tf.TransformListener()

rate = rospy.Rate(10.0)  # 10Hz

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    current_time = rospy.Time.now()

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"


    odom.pose.pose = Pose(Point(trans[0], trans[1], 0.), Quaternion(*rot))

    odom.twist.twist = Twist()

    odom_pub.publish(odom)

    rate.sleep()

