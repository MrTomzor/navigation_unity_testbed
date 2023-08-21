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


class SessionCycler(object):

    def __init__(self):

        rospy.loginfo('Default world loader initialized.')
        # self.keyboard_reading_rate = rospy.Rate(self.keyboard_reading_freq)

        self.mainloop()

    def mainloop(self):
        # Get the file path from the ROS package
        # file_path = rospy.get_param("unity_world_config_file", "default_world.yaml")
        change_period = 3

        sessions = ["default_world.yaml", "second_world.yaml", "rainy_day.yaml"]
        session_index = 0

        rospy.loginfo("waiting for service")
        rospy.wait_for_service('reset_world')

        rospack = rospkg.RosPack()

        while True:
            file_path = sessions[session_index]
            file_path = rospack.get_path('navigation_unity_core') + "/unity_world_config/" + file_path

            session_index = (session_index +1) % len(sessions)

            with open(file_path, 'r') as file:
                file_content = file.read()

            rospy.loginfo("calling reset service")
            reset_world = rospy.ServiceProxy('reset_world', ResetWorld)
            reset_world(file_content)
            rospy.sleep(change_period)

        rospy.loginfo("exiting ")


if __name__ == '__main__':
    rospy.init_node('session_cycler', log_level=rospy.INFO)
    node = SessionCycler()


