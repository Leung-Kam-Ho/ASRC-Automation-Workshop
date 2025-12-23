from robot_arm.robot_arm import RobotArm
from robot_arm.position import Cartesian




def main():
    ra = RobotArm(host="192.168.1.64", base_offset=-90)
    
    ra.move_joint(ra.Init_pose)
    
    while ra.is_moving():
        pass
    
    ra.move_joint(jPos=ra.Init_pose)
    
    cartesian = ra.read_cartesian_pos()
    
    new_cartesian = Cartesian(
        x=cartesian.x + 100,
        y=cartesian.y,
        z=cartesian.z,
        rx=cartesian.rx,
        ry=cartesian.ry,
        rz=cartesian.rz
    )
    
    ra.move_linear(cartesian=new_cartesian)

if __name__ == "__main__":
    main()
