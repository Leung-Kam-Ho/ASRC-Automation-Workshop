from .tcp import Tcp
from .ra_error import *
from .position import Cartesian

class RobotArm:
    def __init__(self, host):
        self.tcp = Tcp(host)

    def _validate(self, reply):
        values = reply.split(',')
        if len(values) < 2:
            raise RAError(f"Received invalid response: {reply}")
        if values[1] != 'OK':
            raise RAError(f"Received error: {reply}")

    # ---------------- Core System Control ----------------
    def power_on(self):
        reply = self.tcp.send("Electrify,;")
        self._validate(reply)

    def power_off(self):
        reply = self.tcp.send("BlackOut,;")
        self._validate(reply)

    def start_master(self):
        reply = self.tcp.send("StartMaster,;")
        self._validate(reply)

    def close_master(self):
        reply = self.tcp.send("CloseMaster,;")
        self._validate(reply)

    def servo_on(self):
        reply = self.tcp.send("GrpPowerOn,0,;")
        self._validate(reply)

    def servo_off(self):
        reply = self.tcp.send("GrpPowerOff,0,;")
        self._validate(reply)

    def stop(self):
        reply = self.tcp.send("GrpStop,0,;")
        self._validate(reply)

    def reset(self):
        reply = self.tcp.send("GrpReset,0,;")
        self._validate(reply)

    # ---------------- Motion Control ----------------
    def set_kinematic_coordinate(self, cartesian: Cartesian):
        msg = f"SetKinematicCoordinate,0,{cartesian.x},{cartesian.y},{cartesian.z},{cartesian.rx},{cartesian.ry},{cartesian.rz},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def move_joint(self, jPos):
        msg = f"MoveJ,0,{jPos.j1},{jPos.j2},{jPos.j3},{jPos.j4},{jPos.j5},{jPos.j6},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def move_linear(self, cartesian: Cartesian):
        msg = f"MoveL,0,{cartesian.x},{cartesian.y},{cartesian.z},{cartesian.rx},{cartesian.ry},{cartesian.rz},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def move_home(self):
        reply = self.tcp.send("MoveHoming,0,;")
        self._validate(reply)

    def move_relative_joint(self, direction_id, direction, distance):
        msg = f"MoveRelJ,0,{direction_id},{direction},{distance},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def move_relative_linear(self, direction_id, direction, distance):
        msg = f"MoveRelL,0,{direction_id},{direction},{distance},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def is_moving(self):
        reply = self.tcp.send("ReadMoveState,0,;")
        self._validate(reply)
        if "1009" in reply:
            return True
        if "0" in reply:
            return False
        raise RAError(f"Unexpected motion state: {reply}")

    # ---------------- Speed Control ----------------
    def set_speed(self, speed: float):
        msg = f"SetOverride,0,{speed},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def get_speed(self) -> float:
        reply = self.tcp.send("ReadOverride,0,;")
        self._validate(reply)
        values = reply.split(',')
        if len(values) < 3:
            raise RAError("Invalid ReadOverride response")
        return float(values[2])

    # ---------------- Position Reading ----------------
    def read_joint_pos(self):
        reply = self.tcp.send("ReadAcsActualPos,0,;")
        self._validate(reply)
        values = reply.split(',')
        return [float(v) for v in values[2:8]]

    def read_cartesian_pos(self):
        reply = self.tcp.send("ReadPcsActualPos,0,;")
        self._validate(reply)
        values = reply.split(',')
        return Cartesian(*[float(v) for v in values[2:8]])

    def read_state(self):
        reply = self.tcp.send("ReadRobotState,0,;")
        self._validate(reply)
        return reply.split(',')

    # ---------------- IO Control ----------------
    def set_output_io(self, io_index, state):
        msg = f"SetOutIOState,0,{io_index},{state},;"
        reply = self.tcp.send(msg)
        self._validate(reply)

    def read_input_io(self, io_index):
        msg = f"ReadInIOState,0,{io_index},;"
        reply = self.tcp.send(msg)
        self._validate(reply)
        return int(reply.split(',')[2])

    def read_output_io(self, io_index):
        msg = f"ReadOutIOState,0,{io_index},;"
        reply = self.tcp.send(msg)
        self._validate(reply)
        return int(reply.split(',')[2])

    # ---------------- Gripper ----------------
    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def move_gripper(self, position, speed=250, force=10):
        position = self.clamp(position, 0, 140)
        speed = self.clamp(speed, 30, 250)
        force = self.clamp(force, 10, 125)
        msg = f"SetRobotiq,{position},{speed},{force},;"
        reply = self.tcp.send(msg)
        self._validate(reply)
    
    def read_gripper_pos(self):
        reply = self.tcp.send("ReadRobotiqPosition,;")
        self._validate(reply)
        values = reply.split(',')
        if len(values) < 3:
            raise RAError("Invalid ReadRobotiqPosition response")
        return int(values[2])

    def reset_gripper(self):
        reply = self.tcp.send("RobotIQReset,;")
        self._validate(reply)

    def is_gripper_moving(self):
        reply = self.tcp.send("RobotiqStatus,;")
        self._validate(reply)
        if "0,3,1,1" in reply:
            return True
        elif "3,3,1,1" in reply or "2,3,1,1" in reply:
            return False
        raise RAError(f"Unexpected gripper status: {reply}")