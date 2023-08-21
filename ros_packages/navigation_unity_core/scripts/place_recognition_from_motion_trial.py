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

import random
import yaml

class DefaultWorldLoader(object):

    def __init__(self):

        rospy.loginfo('Default world loader initialized.')
        # self.keyboard_reading_rate = rospy.Rate(self.keyboard_reading_freq)

        self.mainloop()

    def mainloop(self):
        # Get the file path from the ROS package
        # file_path = rospy.get_param("unity_world_config_file", "default_world.yaml")

        randomize_spawnpoints = True
        max_spawnpoints_per_zone = 4
        # zones = ["home", "highmount", "longbeach", "shortbeach", "corner1", "corner2", "step", "valley", "bridge2", "highlake"]
        zones = ["home", "corner1", "step", "corner2", "shortbeach"]
        original_zones = [a for a in zones]
        exploration_time = 20
        answer_time = 10


        rospack = rospkg.RosPack()
        file_path = "default_world.yaml"
        # file_path = rospy.get_package_path(rospy.get_name()) + file_path
        file_path = rospack.get_path('navigation_unity_core') + "/unity_world_config/" + file_path

        # TODO - function for blockingly trying to reset environment - in for loop sends config and waits for ACK
        #TODO - service call for loading world! simple as! 


        # READ YAML DATA OF DEFAULT WORLD SETTINGS
        with open(file_path, 'r') as file:
            file_content = file.read()
        yamldata = yaml.safe_load(file_content)

        # THE TRIAL LOOP
        for i in range(len(zones)):
            rospy.loginfo("waiting for service to reset world")
            rospy.wait_for_service('reset_world')
            rospy.loginfo("service available")

            # Modify the "world_name" parameter
            yamldata['spawn_area'] = zones[i]
            yamldata['spawn_point_number'] = random.randint(0, max_spawnpoints_per_zone-1)

            # Convert the modified dictionary back to a YAML string
            updated_yaml_string = yaml.dump(yamldata)

            # reset world
            print("spawning at: " + zones[i] + " sp: " + str(yamldata['spawn_point_number']))
            reset_world = rospy.ServiceProxy('reset_world', ResetWorld)
            reset_world(updated_yaml_string)

            # wait
            rospy.sleep(exploration_time)

        # SHUFFLE LIST OF ZONES
        random.shuffle(zones)

        # START TRIAL
        for i in range(len(zones)):
            rospy.loginfo("waiting for service to reset world")
            rospy.wait_for_service('reset_world')
            rospy.loginfo("service available")

            # Modify the "world_name" parameter
            yamldata['spawn_area'] = zones[i]
            yamldata['daynight_cycle_start'] = 18
            yamldata['spawn_point_number'] = random.randint(0, max_spawnpoints_per_zone-1)

            # Convert the modified dictionary back to a YAML string
            updated_yaml_string = yaml.dump(yamldata)

            # reset world
            print("spawning at: " + zones[i] + " sp: " + str(yamldata['spawn_point_number']))
            reset_world = rospy.ServiceProxy('reset_world', ResetWorld)
            reset_world(updated_yaml_string)

            # wait
            rospy.sleep(answer_time)

        # PRINT THE TRUE LABELS
        print("ORIGINAL LABELS: ")
        print(original_zones)
        print("SHUFFLED LABELS: ")
        print(zones)


        rospy.loginfo("exiting ")


if __name__ == '__main__':
    rospy.init_node('default_world_loader', log_level=rospy.INFO)
    node = DefaultWorldLoader()


