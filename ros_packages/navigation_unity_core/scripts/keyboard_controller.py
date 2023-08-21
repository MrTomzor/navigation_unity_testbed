import time
import serial
import rospy
import sys
# import keyboard
from pynput import keyboard

# from snek_core.common_utils import say_it_works
# from snek_core.common_utils import *
# from snek_msgs.srv import *
# from snek_msgs.msg import *
# from std_srvs.srv import Trigger
from std_msgs.msg import Float32

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

class KeyboardController(object):

# # #{ INIT
    def __init__(self):
        # self.control_freq = rospy.get_param('~control_freq', 10.0) # control loop frequency (Hz)
        # self.body_config = rospy.get_param('~body_config', [[0,0,0], [1,0,0]])
        self.body_config = []
        self.vel_r = 0
        self.vel_l = 0

        rospy.loginfo('Keyboard controller initialized.')
        self.keyboard_reading_freq = 100
        self.keyboard_reading_rate = rospy.Rate(self.keyboard_reading_freq)
        self.should_stop = False
        self.is_pressed_w = False
        self.key_press_cache = {}

        self.internal_setpoints = []

        # WAIT FOR CORE AND GET BODY CONFIG

        self.bangbang_bendies_angle = 90.0
        self.bangbang_wheels_speed = 0

        self.selected_bendy = 0

        self.mode_button_pressed = False
        self.mode = "BB"

        # INIT DEFAULT SETPOINT ARRAY MSG AND SEND IT
        # setpoints_to_send  = self.generate_setpoints_for_all_modules()

        # # SEND DEFAULT CONFIG
        # self.publish_setpoints(setpoints_to_send)

        self.mainloop()
# # #}

#     def handle_BodyConfigMsg(self, msg):
#         self.body_config = []
#         n_modules = len(msg.data) // 3
#         print("BC:", msg.data, " GOT ",n_modules, " MODULES" )
#         num_bendies = 0
#         self.bendy_angles = []
#         for i in range(n_modules):
#             self.body_config.append([msg.data[i*3], msg.data[i*3+1], msg.data[i*3+2]])
#             if msg.data[i*3] == 0:
#                 num_bendies += 1
#                 self.bendy_angles.append(90.0)
#         print("BA:")
#         print(self.bendy_angles)

#         rospy.loginfo('BODY CONFIG MSG RECEIVED:')
#         print(self.body_config)

# # #{ KEY PRESS HANDLING
    def is_pressed(self, c):
        if c in self.key_press_cache:
            return self.key_press_cache[c]
        return False
    
    def on_press(self, key):
        try:
            # print('alphanumeric key {0} pressed'.format(
                # key.char))
            self.key_press_cache [key.char] = True
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
    
    def on_release(self, key):
        try:
            self.key_press_cache [key.char] = False
            if key.char == "q":
                # Stop listener
                print("setting stop true")
                self.should_stop = True
                return False
        except AttributeError:
            print('special key {0} released'.format(
                key))

# # #}

    def publish_setpoints(self):
        self.cmd_pub1 = rospy.Publisher('wheel_left_vel_reference', Float32, queue_size=20) # Publisher of velocity commands
        self.cmd_pub2 = rospy.Publisher('wheel_right_vel_reference', Float32, queue_size=20) # Publisher of velocity commands

        msg1 = Float32()
        msg1.data = self.vel_l

        msg2 = Float32()
        msg2.data = self.vel_r

        self.cmd_pub1.publish(msg1)
        self.cmd_pub2.publish(msg2)
    
    def update_internal_values_based_on_input(self):
        main_speed = 600
        if self.mode == "BB":
            if self.is_pressed("w") and not self.is_pressed("d") and not self.is_pressed("a"):
                self.vel_r = main_speed
                self.vel_l = -main_speed
            elif self.is_pressed("s") and not self.is_pressed("d") and not self.is_pressed("a"):
                self.vel_r = -main_speed
                self.vel_l = main_speed
            elif self.is_pressed("a") and not self.is_pressed("d") and not self.is_pressed("w") and not self.is_pressed("s"):
                self.vel_r = main_speed
                self.vel_l = main_speed
            elif self.is_pressed("d") and not self.is_pressed("a") and not self.is_pressed("w") and not self.is_pressed("s"):
                self.vel_r = -main_speed
                self.vel_l = -main_speed
            elif self.is_pressed("w") and self.is_pressed("d"):
                self.vel_r = main_speed * 0.5
                self.vel_l = -main_speed 
            elif self.is_pressed("w") and self.is_pressed("a"):
                self.vel_r = main_speed 
                self.vel_l = -main_speed * 0.5
            elif not self.is_pressed("w") and not self.is_pressed("a") and not self.is_pressed("s") and not self.is_pressed("d"):
                self.vel_r = 0
                self.vel_l = 0
            # if self.is_pressed("a") and not self.is_pressed("d") and not self.is_pressed("w"):
            #     self.vel_r = main_speed
            #     self.vel_l = main_speed
            # elif self.is_pressed("d") and not self.is_pressed("a") and not self.is_pressed("w"):
            #     self.vel_r = -main_speed
            #     self.vel_l = -main_speed
            # # elif self.is_pressed("w") and not 
            # else:
            #     pass
            #     # self.bangbang_bendies_angle = 90.0

    def generate_setpoints_for_all_modules(self):
        setpoints = []
        gen_time = rospy.Time()
        bend_index = 0
        for i in range(len(self.body_config)):
            # rospy.loginfo("MODULE %d", i)
            msg = ModuleSetpoint()
            msg.header.stamp = gen_time
            msg.module_address = i + 1

            if self.mode == "BB":
                if self.body_config[i][0] == 0:
                    rospy.loginfo_once("BENDY MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 0
                    msg.setpoint = self.bangbang_bendies_angle
                elif self.body_config[i][0] == 1:
                    rospy.loginfo_once("WHEEL MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 1
                    msg.setpoint = self.bangbang_wheels_speed
                else:
                    rospy.logwarn_once("UNRECOGNIZED MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 0
                setpoints.append(msg)

            elif self.mode == "C":
                if self.body_config[i][0] == 0:
                    rospy.loginfo_once("BENDY MODULE AT ADDR %d, BI:%d", msg.module_address, bend_index)
                    msg.control_type = 0
                    msg.setpoint = self.bendy_angles[bend_index]
                    rospy.loginfo("SETTING %f DEG TO BENDY n %d", msg.setpoint, bend_index)
                    bend_index += 1
                elif self.body_config[i][0] == 1:
                    rospy.loginfo_once("WHEEL MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 1
                    msg.setpoint = self.bangbang_wheels_speed
                else:
                    rospy.logwarn_once("UNRECOGNIZED MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 0
                setpoints.append(msg)

            elif self.mode == "T":
                if self.body_config[i][0] == 0:
                    rospy.loginfo_once("BENDY MODULE AT ADDR %d, BI:%d", msg.module_address, bend_index)
                    msg.control_type = 2
                    msg.setpoint = self.bendy_angles[bend_index]
                    rospy.loginfo("SETTING %f TORQUE TO BENDY n %d", self.bendy_angles[bend_index], bend_index)
                    bend_index += 1
                elif self.body_config[i][0] == 1:
                    rospy.loginfo_once("WHEEL MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 1
                    msg.setpoint = self.bangbang_wheels_speed
                else:
                    rospy.logwarn_once("UNRECOGNIZED MODULE AT ADDR %d", msg.module_address)
                    msg.control_type = 0
                setpoints.append(msg)
        return setpoints

    def get_nonredundant_setpoint_msgs(self, current_setpoints, last_sent_setpoints):
        setpoints_to_send = []
        blocked_msgs = []
        for msg in current_setpoints:
            is_redudnant = False
            for msg2 in last_sent_setpoints:
                if msg2.module_address != msg.module_address:
                    continue
                if msg2.control_type != msg.control_type:
                    continue
                if abs(msg2.setpoint - msg.setpoint) > 0.00001:
                    continue
                is_redudnant = True
                blocked_msgs.append(msg2)
                break
            if not is_redudnant:
                setpoints_to_send.append(msg)
        return setpoints_to_send, blocked_msgs

    def mainloop(self):
        # NONBLOCKING LISTENER
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

        # LOOP
        while not self.should_stop:
            # print(self.is_pressed("w"), self.is_pressed("a"))
            self.update_internal_values_based_on_input()
            # orig_msgs = self.generate_setpoints_for_all_modules()
            # reduced_msgs, blocked_msgs = self.get_nonredundant_setpoint_msgs(orig_msgs, self.last_sent_setpoints)
            self.publish_setpoints()
            # if len(reduced_msgs) > 0:
            #     rospy.loginfo("SENDING %d MSGS, REDUCED FROM %d COMMANDS", len(reduced_msgs), len(orig_msgs))

            self.keyboard_reading_rate.sleep()
        rospy.loginfo("exiting keyboard control")


if __name__ == '__main__':
    rospy.init_node('keyboard_controller', log_level=rospy.INFO)
    node = KeyboardController()


