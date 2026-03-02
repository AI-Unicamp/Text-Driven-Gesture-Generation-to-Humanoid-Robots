import rospy
import tf

def publish_transform():
    # Initialize the ROS node
    rospy.init_node('frame_a_to_frame_b_broadcaster')

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # Define the publishing rate (70 Hz in this case)
    rate = rospy.Rate(70)

    while not rospy.is_shutdown():
        # Define the transformation from A (parent) to B (child)
        # This can be any translation and rotation value you desire
        translation = (1.0, 0.0, 0.0) # Example: 1m translation on the X axis from A to B
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0) # No rotation in this example

        # Publish the transformation from frame A to frame B
        br.sendTransform(translation,
                        rotation,
                        rospy.Time.now(),
                        "odom", # Child (child_frame)
                        "map") # Parent (parent_frame)

        # Wait until the next iteration
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass