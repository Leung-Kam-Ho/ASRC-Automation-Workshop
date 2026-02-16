# Test move_linear method with py_trees (relative moves)
from pathlib import Path
from robot_arm.position import Cartesian, JPosition
from robot_arm.robot_arm import RobotArm
import py_trees
import time
from example.robot_action_node import MoveJoint, MoveLinear, MoveGripper

class CheckerboardSize:
    WIDTH = 410
    HEIGHT = 410
    Robot_X = 125  # distance from the robot to the center of the checkerboard in x direction

def create_behavior_tree(ra_1: RobotArm, ra_2: RobotArm):
    # Assume you are at the bottom center of the checkerboard
    

    # define the relative moves that trace the bottom rectangle, move to top plane, then trace top rectangle, and return
    topLeft = Cartesian((CheckerboardSize.WIDTH + CheckerboardSize.Robot_X), -CheckerboardSize.HEIGHT / 2, 0, 0, 0, 0) # top left corner of checkerboard
    bottomLeft = Cartesian(CheckerboardSize.Robot_X, -CheckerboardSize.HEIGHT / 2, 0, 0, 0, 0) # bottom left corner of checkerboard (back to start)
    bottomRight = Cartesian(CheckerboardSize.Robot_X, CheckerboardSize.HEIGHT / 2, 0, 0, 0, 0) # bottom right corner of checkerboard
    topRight = Cartesian((CheckerboardSize.WIDTH + CheckerboardSize.Robot_X), CheckerboardSize.HEIGHT / 2, 0, 0, 0, 0) # top right corner of checkerboard
    deltas = [
        # bottom plane rectangle (relative to current)
        topLeft,
        bottomLeft,
        bottomRight,
        topRight,
        bottomRight,
        bottomLeft,
        topLeft,
    ]

    # Create sequence for rectangle drawing using relative moves
    rectangle_sequence = py_trees.composites.Sequence("DrawCheckerboardRelative", memory=True)
    for i, delta in enumerate(deltas):
        parallel_move = py_trees.composites.Parallel(f"RelMove{i+1}", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
        parallel_move.add_children([
            MoveLinear(f"RelMove{i+1}_Arm1", ra_1, delta, relative=False),
            MoveLinear(f"RelMove{i+1}_Arm2", ra_2, delta, relative=False)
        ])
        rectangle_sequence.add_child(parallel_move)

    # Gripper sequence (unchanged)
    ra_1_gripper_open_close_sequence = py_trees.composites.Sequence("GripperSequence", memory=True)
    ra_1_gripper_open_close_sequence.add_children([
        MoveGripper(ra_1, 90, "OpenGripper"),
        MoveGripper(ra_1, 0, "CloseGripper")
    ])
    ra_2_gripper_open_close_sequence = py_trees.composites.Sequence("GripperSequence2", memory=True)
    ra_2_gripper_open_close_sequence.add_children([
        MoveGripper(ra_2, 90, "OpenGripper"),
        MoveGripper(ra_2, 0, "CloseGripper")
    ])

    # Repeat indefinitely both the cube path and gripper cycle
    repeat_rectangle = py_trees.decorators.Repeat("RepeatRectangle", child=rectangle_sequence, num_success=None)
    repeat_open_close = py_trees.decorators.Repeat("RepeatGripper", child=ra_1_gripper_open_close_sequence, num_success=None)
    repeat_open_close_2 = py_trees.decorators.Repeat("RepeatGripper2", child=ra_2_gripper_open_close_sequence, num_success=None)

    parallel = py_trees.composites.Parallel("ParallelActions", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_children([repeat_rectangle, ])

    # Main sequence: initial gripper operations, then go into repeating parallel actions
    root = py_trees.composites.Sequence("Main", memory=True)
    root.add_children([
        MoveJoint("Init Position", ra_1, ra_1.Init_pose, relative=False),
        MoveJoint("Init Position", ra_2, ra_2.Init_pose, relative=False),
        
        parallel
    ])

    return py_trees.trees.BehaviourTree(root)


if __name__ == "__main__":
    ra_1 = RobotArm('192.168.1.64')  # Arm 1
    ra_2 = RobotArm('192.168.1.49', base_offset=-30)  # Arm 2
    ra_1.set_speed(1)
    ra_2.set_speed(1)

    tree = create_behavior_tree(ra_1, ra_2)

    py_trees.display.render_dot_tree(tree.root, target_directory= Path() / "Tree", name="Checkerboard_Twin_tree")
    tree.setup(timeout=15)

    while True:
        tree.tick()
        print(py_trees.display.unicode_tree(tree.root, show_status=True))
        time.sleep(0.1)