# Test move_linear method with py_trees (relative moves)
from pathlib import Path
from robot_arm.position import Cartesian, JPosition
from robot_arm.robot_arm import RobotArm
import py_trees
import time
from src.example.robot_action_node import MoveJoint, MoveLinear, MoveGripper


def create_behavior_tree(ra: RobotArm):
    # We'll perform moves as deltas relative to the current position at each MoveLinear start.
    width_range = (0,370)
    depth_range = (0, 200)

    # define the relative moves that trace the bottom rectangle, move to top plane, then trace top rectangle, and return
    deltas = [
        # bottom plane rectangle (relative to current)
        Cartesian(0, 0, +depth_range[1], 0, 0, 0),   # up in z
        Cartesian(-width_range[1], 0, 0, 0, 0, 0),  # -x
        Cartesian(0, 0, -depth_range[1], 0, 0, 0),  # -z
        Cartesian(+width_range[1], 0, 0, 0, 0, 0),  # +x (back to start)
    ]

    # Create sequence for rectangle drawing using relative moves
    rectangle_sequence = py_trees.composites.Sequence("DrawCubeRelative", memory=True)
    for i, delta in enumerate(deltas):
        rectangle_sequence.add_child(MoveLinear(f"RelMove{i+1}", ra, delta, relative=True))

    # Gripper sequence (unchanged)
    gripper_open_close_sequence = py_trees.composites.Sequence("GripperSequence", memory=True)
    gripper_open_close_sequence.add_children([
        MoveGripper(ra, 130, "OpenGripper"),
        MoveGripper(ra, 0, "CloseGripper")
    ])

    # Repeat indefinitely both the cube path and gripper cycle
    repeat_rectangle = py_trees.decorators.Repeat("RepeatRectangle", child=rectangle_sequence, num_success=None)
    repeat_open_close = py_trees.decorators.Repeat("RepeatGripper", child=gripper_open_close_sequence, num_success=None)

    parallel = py_trees.composites.Parallel("ParallelActions", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_children([repeat_rectangle, ])

    # Main sequence: initial gripper operations, then go into repeating parallel actions
    root = py_trees.composites.Sequence("Main", memory=True)
    root.add_children([
        MoveLinear("Init Position", ra, Cartesian(0, 0, 0, 0, 0, 0), relative=False),
        # MoveGripper(ra, 130, "OpenGripper"),
        # MoveGripper(ra, 0, "CloseGripper"),
        # MoveGripper(ra, 130, "OpenGripper"),
        # MoveJoint("Look at table", ra, JPosition(0, 0, 0, 0, -25, 0), relative=True),
        # repeat_rectangle,  # start repeating the relative cube path
        # parallel
    ])

    return py_trees.trees.BehaviourTree(root)


if __name__ == "__main__":
    ra = RobotArm('192.168.10.228')
    ra.set_speed(.4)

    tree = create_behavior_tree(ra)

    py_trees.display.render_dot_tree(tree.root, target_directory= Path() / "Tree", name="moveInCube_relative_tree")
    tree.setup(timeout=15)

    while True:
        tree.tick()
        print(py_trees.display.unicode_tree(tree.root, show_status=True))
        time.sleep(0.1)