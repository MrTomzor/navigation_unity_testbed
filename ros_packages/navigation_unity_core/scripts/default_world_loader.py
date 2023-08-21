import time
import serial
import rospy
import sys
import rospkg
# import keyboard
from pynput import keyboard

from std_msgs.msg import Float32
from std_msgs.msg import String

from navigation_unity_msgs.srv import ResetWorld, ResetWorldRequest, ResetWorldResponse


class DefaultWorldLoader(object):

    def __init__(self):

        rospy.loginfo('Default world loader initialized.')
        # self.keyboard_reading_rate = rospy.Rate(self.keyboard_reading_freq)

        self.mainloop()

    def mainloop(self):
        # Get the file path from the ROS package
        # file_path = rospy.get_param("unity_world_config_file", "default_world.yaml")

        rospy.loginfo("waiting for service")
        rospy.wait_for_service('reset_world')

        rospack = rospkg.RosPack()
        file_path = "default_world.yaml"
        # file_path = rospy.get_package_path(rospy.get_name()) + file_path
        file_path = rospack.get_path('navigation_unity_core') + "/unity_world_config/" + file_path

        # TODO - function for blockingly trying to reset environment - in for loop sends config and waits for ACK
        #TODO - service call for loading world! simple as! 

        with open(file_path, 'r') as file:
            file_content = file.read()


        rospy.loginfo("calling reset service")
        reset_world = rospy.ServiceProxy('reset_world', ResetWorld)
        reset_world(file_content)


        # while True:
        #     try:
        #         # Read the content of the file

        #         # Publish the content of the file as a ROS message
        #         pub = rospy.Publisher('world_config', String, queue_size=10)
        #         rospy.loginfo("Publishing file content as a string ROS message...")
        #         pub.publish(file_content)

        #     except IOError as e:
        #         rospy.logerr("Error reading the file: %s", str(e))

        #     # Wait for a short duration to allow the message to be published
        #     rospy.sleep(1)
        rospy.loginfo("exiting ")


if __name__ == '__main__':
    rospy.init_node('default_world_loader', log_level=rospy.INFO)
    node = DefaultWorldLoader()


