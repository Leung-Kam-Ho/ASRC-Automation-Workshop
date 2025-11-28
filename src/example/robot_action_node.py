from pathlib import Path
from robot_arm.position import Cartesian, JPosition
from robot_arm.robot_arm import RobotArm
import py_trees
import time

# Robot arm action behaviors

class MoveJoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_arm: RobotArm, target_joints: JPosition, relative: bool = True):
        super().__init__(name)
        self.robot_arm: RobotArm = robot_arm
        self.relative = relative

        # Helpers to normalize between sequences and JPosition objects
        def _jvals(j):
            try:
                return tuple(j)  # if JPosition is iterable or a sequence
            except TypeError:
                # fallback to common attribute names if not iterable
                return tuple(getattr(j, n) for n in ('j1', 'j2', 'j3', 'j4', 'j5', 'j6'))

        def _to_jpos(x):
            if isinstance(x, JPosition):
                return x
            vals = _jvals(x)
            return JPosition(*vals)

        # store target as a JPosition
        self.target_joints: JPosition = _to_jpos(target_joints)
        self._jvals = _jvals  # keep helper for update()

    def update(self):
        self.feedback_message = f"Starting joint move ({'relative' if self.relative else 'absolute'}) to {self.target_joints}"
        if not hasattr(self, 'started'):
            # read current joint positions and compute absolute target if relative
            current_raw = self.robot_arm.read_joint_pos()
            current_vals = self._jvals(current_raw)
            target_vals = self._jvals(self.target_joints)

            if self.relative:
                target = [c + d for c, d in zip(current_vals, target_vals)]
            else:
                target = list(target_vals)

            # RobotArm.move_joint expects a JPosition with attributes j1..j6; convert the list to JPosition
            target_jpos = JPosition(*target)
            self.robot_arm.move_joint(target_jpos)
            self.started = True
            self.feedback_message = f"Moving joints -> target {target_jpos.__dict__}"
            return py_trees.common.Status.RUNNING

        # wait until the robot arm finished moving
        if self.robot_arm.is_moving():
            return py_trees.common.Status.RUNNING

        delattr(self, 'started')
        self.feedback_message = "Reached joint target"
        return py_trees.common.Status.SUCCESS


class MoveLinear(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_arm: RobotArm, target_delta: Cartesian, relative: bool = True):
        super().__init__(name)
        self.robot_arm: RobotArm = robot_arm
        # target_delta is interpreted as a delta when relative=True, otherwise as absolute target
        self.target_delta: Cartesian = target_delta
        self.relative = relative

    def update(self):
        self.feedback_message = f"Starting move ({'relative' if self.relative else 'absolute'}) to {self.target_delta}"
        if not hasattr(self, 'started'):
            if self.relative:
                # read current position at the moment of starting and compute absolute target
                current = self.robot_arm.read_cartesian_pos()
                target = Cartesian(
                    current.x + self.target_delta.x,
                    current.y + self.target_delta.y,
                    current.z + self.target_delta.z,
                    current.rx + self.target_delta.rx,
                    current.ry + self.target_delta.ry,
                    current.rz + self.target_delta.rz,
                )
            else:
                target = self.target_delta
            self.robot_arm.move_linear(target)
            self.started = True
            self.feedback_message = f"Moving to {self.name} -> target {target.__dict__}"
            return py_trees.common.Status.RUNNING

        # Wait until the robot arm is not moving
        if self.robot_arm.is_moving():
            return py_trees.common.Status.RUNNING

        delattr(self, 'started')
        self.feedback_message = f"Reached position"
        return py_trees.common.Status.SUCCESS


class MoveGripper(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm: RobotArm, position, name):
        super().__init__(name=name)
        self.ra: RobotArm = robot_arm
        self.position = position

    def update(self):
        if not hasattr(self, 'started'):
            self.ra.move_gripper(self.position)
            self.started = True
            self.feedback_message = f"Gripper moving to {self.position}..."
            return py_trees.common.Status.RUNNING

        if self.ra.is_gripper_moving():
            self.feedback_message = f"Gripper still moving to {self.position}..."
            return py_trees.common.Status.RUNNING
        else:
            delattr(self, 'started')
            self.feedback_message = f"Gripper reached {self.position}"
            return py_trees.common.Status.SUCCESS