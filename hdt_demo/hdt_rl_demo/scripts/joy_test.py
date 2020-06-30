import copy
import ros
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

# TODO: add effort feedback to the gripper control


class HDTAnglerControl():
    def __init__(self):
        self.pub = rospy.Publisher('joy', Joy, queue_size=1)
        rospy.init_node('hdt_joy_control', anonymous=True)
        self.rate = rospy.Rate(20)

        axes = [0, 0, 1.0, 0, 0, 1.0, 0, 0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joy_zeros = Joy()
        self.joy_zeros.axes = copy.deepcopy(axes)
        self.joy_zeros.buttons = copy.deepcopy(buttons)
        self.joy_home = copy.deepcopy(self.joy_zeros)
        self.joy_home.buttons[0] = 1
        self.joy_joint_mode = copy.deepcopy(self.joy_zeros)
        self.joy_joint_mode.buttons[1] = 1
        self.joy_end_effector_mode = copy.deepcopy(self.joy_zeros)
        self.joy_end_effector_mode.buttons[2] = 1

    def go_home(self):
        self.send_joy_cmd(self.joy_home, 20)
        # self.send_joy_cmd(self.joy_zeros,1)

    def switch_joint(self):
        self.send_joy_cmd(self.joy_joint_mode, 20)

    def switch_end_effector_mode(self):
        self.send_joy_cmd(self.joy_end_effector_mode, 20)

    def joint_control(self):
        axes = [0, 0, 1.0, 0, 0, 1.0, 0, 0]
        buttons = [0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0]
        joy_cmd = Joy()
        joy_cmd.axes = axes
        joy_cmd.buttons = buttons
        self.send_joy_cmd(joy_cmd, 20)

    def end_effector_control(self):
        axes = [0, 0, 1.0, 0, 0, 1.0, 0, 0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        joy_cmd = Joy()
        joy_cmd.axes = axes
        joy_cmd.buttons = buttons
        self.send_joy_cmd(joy_cmd, 60)

    def gripper_open(self):
        axes = [0, 0, 1.0, 0, 0, 0.5, 0, 0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        joy_cmd = Joy()
        joy_cmd.axes = axes
        joy_cmd.buttons = buttons
        self.send_joy_cmd(joy_cmd, 60)

    def gripper_close(self):
        axes = [0, 0, 0.5, 0, 0, 1.0, 0, 0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        joy_cmd = Joy()
        joy_cmd.axes = axes
        joy_cmd.buttons = buttons
        self.send_joy_cmd(joy_cmd, 60)

    def send_joy_cmd(self, joy_cmd, num_cmd=20):
        num = 0
        while not rospy.is_shutdown():
            self.pub.publish(joy_cmd)
            self.rate.sleep()
            num += 1
            if num > num_cmd:
                self.pub.publish(self.joy_zeros)
                break

    def test_gripper(self):
        self.gripper_open()
        rospy.sleep(2)
        self.gripper_close()

    def test_arm(self):
        self.joint_control()

    def test_all(self):
        # self.test_arm()
        self.go_home()
        rospy.sleep(2)
        # self.switch_joint()
        # rospy.sleep(2)
        self.switch_end_effector_mode()
        # rospy.sleep(2)
        # self.joint_control()
        self.end_effector_control()


def main():
    hdt_control = HDTAnglerControl()
    hdt_control.test_all()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
