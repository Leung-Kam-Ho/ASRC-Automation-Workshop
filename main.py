from robot_arm.robot_arm import RobotArm
from robot_arm.position import Cartesian




def main():
    # ra = RobotArm(host="192.168.1.64", base_offset=-90)
    ra = RobotArm(host="192.168.1.48", base_offset=60)


    # go to init pose
    ra.move_joint(ra.Init_pose)

    # wait until reach
    while ra.is_moving():
        pass

    # read current cartesian position
    ra.move_joint(jPos=ra.Init_pose)

    # wait until reach
    cartesian = ra.read_cartesian_pos()

    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x + 100,
        y=cartesian.y,
        z=cartesian.z,
        rx=cartesian.rx,
        ry=cartesian.ry,
        rz=cartesian.rz
    )

    # move to new position
    ra.move_linear(cartesian=new_cartesian)


    # wait until reach
    while ra.is_moving():
        pass

    # go back to init pose
    ra.move_joint(ra.Init_pose)

if __name__ == "__main__":
    main()
