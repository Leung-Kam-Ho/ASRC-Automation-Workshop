from robot_arm import RobotArm
from robot_arm.position import Cartesian, JPosition


def initialize_arm(ra : RobotArm):
    ra.move_joint(ra.Init_pose)


if __name__ == "__main__":
    ra = RobotArm('192.168.1.47', base_offset=240)
    initialize_arm(ra)
    