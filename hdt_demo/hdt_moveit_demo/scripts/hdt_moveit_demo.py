import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal        A list of floats, a Pose or a PoseStamped
    @param: actual      A list of floats, a Pose or a PoseStamped
    @param: tolerance   A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """ MoveGroupPythonInterfaceTutorial """
    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
        
        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object. This object is an interface to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object. This object is an interface
        # to one group of joints. In this case the group is the joints in the HDT
        # arm so we set ``group_name = arm``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group.
        # This interface can be used to plan and execute motions on the HDT:
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)

        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference 
        planning_frame = group.get_planning_frame()
        print("=========== Reference frame: %s", planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("=========== End effector: %s", eef_link)

        # We can get a list of all the groups in  the robot:
        group_names = robot.get_group_names()
        print("=========== Robot Groups: ", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("=========== Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        group = self.group

        # Planning to a Joint Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the joint values from the group and adjust some of the values
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0.16
        joint_goal[2] = 0.5
        joint_goal[3] = 0
        joint_goal[4] = -0.65
        joint_goal[5] = 0
        
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)
        
        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # For testing
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorial more clear.
        # In practice, you should use the class variables directly unless you have a good reason not to.
        group = self.group
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)

        # Planning to a Pose Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # We can plan a motion for this group to a desired pose for the 
        # end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.07
        pose_goal.position.z = 0.3
        group.set_pose_target(pose_goal)
#         pose: 
#   position: 
#     x: 0.529997398101
#     y: 0.0715500000049
#     z: 0.26722710624
#   orientation: 
#     x: -3.00570905542e-13
#     y: 0.00469794225441
#     z: 5.77802303695e-12
#     w: 0.999988964608 ]

        # Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that is no residual movement
        group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        group = self.group

        # Cartesian Paths
        # ^^^^^^^^^^^^^^^
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1 # First move up (z)
        # wpose.position.y += scale * 0.2 # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1 # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1 # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation. We will disable the jump threshold by setting it to 0.0 disabling
        (plan, fraction) = group.compute_cartesian_path(
                                            waypoints,      # waypoints to follow
                                            0.01,           # eef_step
                                            0.0)            # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # Displaying a Trajectory
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again)

        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        group = self.group

        # Executing a Plan
        # ^^^^^^^^^^^^^^^^
        # Use execute if you would like the robot to follow
        # the plan that has already been computed
        group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the 
        # first waypoints in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        pass


def main():
    try:
        print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit...")
        raw_input()
        tutorial = MoveGroupPythonInterfaceTutorial()

        print("============ Press `Enter` to execute a movement using a joint state goal ...")
        raw_input()
        tutorial.go_to_joint_state()

        print("============ Press `Enter` to execute a movement using a pose goal ...")
        raw_input()
        tutorial.go_to_pose_goal()

        print("============ Press `Enter` to plan and display a Cartesian path ...")
        raw_input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0.5)

        print("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path) ...")
        raw_input()
        tutorial.display_trajectory(cartesian_plan)

        print("============ Press `Enter` to execute a saved path ...")
        raw_input()
        tutorial.execute_plan(cartesian_plan)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()