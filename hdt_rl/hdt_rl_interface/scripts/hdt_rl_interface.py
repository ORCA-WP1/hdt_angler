"""
Get the action from gym and transfer it to hdt controller

gym action: 
    1) joint control:
    (j1,j2,j3,j4,j5,j6,gripper)

    2) end-effector control:
    (x,y,z,gripper)

joy control:

"""

class HDTAnglerRLInterface():
    def __init__(self):

    def callback_gym_cmd(self):
        