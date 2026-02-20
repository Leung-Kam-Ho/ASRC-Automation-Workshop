from robot_arm.robot_arm import RobotArm
from robot_arm.position import Cartesian




def main():
    # ra = RobotArm(host="192.168.1.64", base_offset=0)
    ra = RobotArm(host="192.168.1.49", base_offset=-30)

    ra.set_speed(1.0)  # set speed to 100mm/s
    # go to init pose
    ra.move_joint(ra.Init_pose)

    # wait until reach
    while ra.is_moving():
        pass

    # read current cartesian position
    # ra.move_joint(jPos=ra.Init_pose)

    # wait until reach
    cartesian = ra.read_cartesian_pos()
    
    # ra.set_kinematic_coordinate(cartesian)  # set current position as kinematic coordinate origin
    # ra.set_tool_coordinate(1)  # set current position as tool coordinate origin

    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x,
        y=cartesian.y,
        z=cartesian.z - 50,
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

    # wait until reach
    while ra.is_moving():
        pass
    
    
    
    cartesian = ra.read_cartesian_pos()
    
    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x + 50,
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


    # wait until reach
    while ra.is_moving():
        pass
    
        cartesian = ra.read_cartesian_pos()
    
    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x,
        y=cartesian.y + 50,
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


    # wait until reach
    while ra.is_moving():
        pass
    
        cartesian = ra.read_cartesian_pos()
    
    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x,
        y=cartesian.y,
        z=cartesian.z,
        rx=cartesian.rx + 50,
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

    # wait until reach
    while ra.is_moving():
        pass
    
        cartesian = ra.read_cartesian_pos()
    
    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x,
        y=cartesian.y,
        z=cartesian.z,
        rx=cartesian.rx,
        ry=cartesian.ry - 50,
        rz=cartesian.rz
    )

    # move to new position
    ra.move_linear(cartesian=new_cartesian)
    
    # wait until reach
    while ra.is_moving():
        pass
    

    # go back to init pose
    ra.move_joint(ra.Init_pose)


    # wait until reach
    while ra.is_moving():
        pass
    
        cartesian = ra.read_cartesian_pos()
    
    # move 100mm in x direction
    new_cartesian = Cartesian(
        x=cartesian.x,
        y=cartesian.y,
        z=cartesian.z,
        rx=cartesian.rx,
        ry=cartesian.ry,
        rz=cartesian.rz + 50
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
