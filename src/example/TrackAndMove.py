import lmstudio as lms
from pydantic import BaseModel, Field
import cv2
import py_trees
from py_trees.common import Status
from robot_arm.robot_arm import RobotArm
from robot_arm.position import Cartesian
import time
from math import inf

import threading

# Configurable tracking item
TRACKING_ITEM = "water bottle"


class PositionInfo(BaseModel):
    # return the move direction based on tracking item position
    position: str = Field(..., pattern="^(|left|right|touching|none)$")


# Blackboard keys
class BB:
    FRAME = "frame"
    FRAME_COUNT = "frame_count"
    POSITION = "position"
    CAP = "cap"
    MODEL = "model"
    ROBOT_ARM = "robot_arm"
    CLIENT = "client"
    CENTER_COUNT = "center_count"


class InitializeCamera(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Camera"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.CAP, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            BB.FRAME_COUNT, access=py_trees.common.Access.WRITE
        )

    def update(self):
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            self.blackboard.set(BB.CAP, cap)
            self.blackboard.set(BB.FRAME_COUNT, 0)
            self.feedback_message = "Camera initialized successfully"
            return Status.SUCCESS
        self.feedback_message = "Failed to initialize camera"
        return Status.FAILURE


class InitializeModel(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Model"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.MODEL, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(BB.CLIENT, access=py_trees.common.Access.WRITE)

    def update(self):
        client = lms.Client("lablab.local:1234")
        self.blackboard.set(BB.CLIENT, client)
        model = client.llm.model("internvl3_5-30b-a3b")
        # model = client.llm.model('internvl3_5-2b')
        self.blackboard.set(BB.MODEL, model)
        self.feedback_message = "Model initialized successfully"
        return Status.SUCCESS


class InitializeRobotArm(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Robot Arm"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.ROBOT_ARM, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(BB.CENTER_COUNT, access=py_trees.common.Access.WRITE)
        self.blackboard.set(BB.CENTER_COUNT, 0)

    def update(self):
        ra = RobotArm("169.254.190.254")
        ra.set_speed(1)
        self.blackboard.set(BB.ROBOT_ARM, ra)
        self.feedback_message = "Robot arm initialized successfully"
        return Status.SUCCESS


class InitialMove(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initial Move"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.ROBOT_ARM, access=py_trees.common.Access.READ)

    def update(self):
        ra = self.blackboard.get(BB.ROBOT_ARM)
        if ra is None:
            self.feedback_message = "Robot arm not available"
            return Status.FAILURE
        ra.move_linear(Cartesian(0, -210, 0, 0, 0, 90))
        while ra.is_moving():
            time.sleep(0.1)
        self.feedback_message = "Initial move completed"
        return Status.SUCCESS


class CaptureFrame(py_trees.behaviour.Behaviour):
    def __init__(self, name="Capture Frame"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.CAP, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            BB.FRAME_COUNT, access=py_trees.common.Access.WRITE
        )

    def update(self):
        cap = self.blackboard.get(BB.CAP)
        if cap is None or not cap.isOpened():
            self.feedback_message = "Camera not available or not opened"
            return Status.FAILURE
        ret, frame = cap.read()

        if not ret:
            self.feedback_message = "Failed to capture frame"
            return Status.FAILURE

        # frame = cv2.resize(frame, (640, 480))
        # draw two lines to divide the frame into three vertical sections
        height, width, _ = frame.shape
        frame_size = (frame.shape[1], frame.shape[0])
        # narrow = width // 6  # 1/6th of the width for narrow sections
        # for i in range(1, 3):
        #     cv2.line(
        #         frame,
        #         (i * frame_size[0] // 3 , 0),
        #         (i * frame_size[0] // 3, frame_size[1]),
        #         (0, 255, 0),
        #         2,
        #     )
        # draw a circle at the center of the frame, radius 5 % of width
        cv2.circle(frame, (width // 2, height // 2), int(width * 0.01), (0, 0, 255), -1)
        frame_count = self.blackboard.get(BB.FRAME_COUNT) + 1
        self.blackboard.set(BB.FRAME, frame)
        self.blackboard.set(BB.FRAME_COUNT, frame_count)
        self.feedback_message = f"Captured frame {frame_count}"
        return Status.SUCCESS


class DetectBottlePosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="Detect Bottle Position"):
        super().__init__(name)
        self.thread = None
        self.result = None

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.MODEL, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.POSITION, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(BB.CLIENT, access=py_trees.common.Access.READ)

    def update(self):
        if self.thread is None:

            def worker():
                frame = self.blackboard.get(BB.FRAME)
                model = self.blackboard.get(BB.MODEL)
                client = self.blackboard.get(BB.CLIENT)

                cv2.imwrite("input.jpg", frame.copy())

                chat = lms.Chat()
                prompt = f"""
                You are a robotics arm vision system. The camera view is provided.
                
                CRITICAL: The {TRACKING_ITEM} must touch/overlap the RED DOT at the center of the image to be pickable. This is the PRIMARY requirement.
                
                Position definitions (from the robot gripper's perspective):
                - "touching": The {TRACKING_ITEM} is fully visible, touching/overlapping the red dot at the center, AND positioned between the two green lines
                - "left": The {TRACKING_ITEM} is visible but positioned to the left of the center (left of the red dot)
                - "right": The {TRACKING_ITEM} is visible but positioned to the right of the center (right of the red dot)
                - "none": The {TRACKING_ITEM} is not visible in the image
                
                IMPORTANT: Do NOT return "touching" unless the {TRACKING_ITEM} is physically touching or overlapping the red dot. Being between the green lines is NOT sufficient - the red dot contact is mandatory.
                
                Return only one of: "left", "touching", "right", or "none"
                """
                
                chat.add_system_prompt(prompt)

                image_handle = client.prepare_image("input.jpg")
                chat.add_user_message(prompt, images=[image_handle])

                result = model.respond(chat, response_format=PositionInfo)
                self.result = result.parsed

            self.thread = threading.Thread(target=worker)
            self.thread.start()
            self.feedback_message = f"Detecting {TRACKING_ITEM} position"
            return Status.RUNNING
        elif self.thread.is_alive():
            self.feedback_message = "Detection in progress"
            return Status.RUNNING
        else:
            self.thread.join()
            position = ""
            if self.result:
                position = self.result["position"]
                self.blackboard.set(BB.POSITION, position)
            else:
                self.blackboard.set(BB.POSITION, "")
            self.feedback_message = f"Detected position: {position}"
            self.thread = None
            self.result = None
            return Status.SUCCESS


class MoveBasedOnPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move Based on Position", steps=50):
        super().__init__(name)
        self.steps = steps

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.POSITION, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.ROBOT_ARM, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.CENTER_COUNT, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(BB.CENTER_COUNT, access=py_trees.common.Access.READ)
        

    def update(self):
        position = self.blackboard.get(BB.POSITION)
        ra = self.blackboard.get(BB.ROBOT_ARM)

        if not hasattr(self, "started"):
            if position != "center" or self.blackboard.get(BB.CENTER_COUNT) is None:
                self.blackboard.set(BB.CENTER_COUNT, 0)
            else:
                center_count = self.blackboard.get(BB.CENTER_COUNT) + 1
                self.blackboard.set(BB.CENTER_COUNT, center_count)
                self.feedback_message = f"{TRACKING_ITEM.capitalize()} in center count {center_count}"
        
            if position == "left":
                # Move right (positive x)
                delta = Cartesian(self.steps, 0, 0, 0, 0, 0)
                current_pos = ra.read_cartesian_pos()
                target = Cartesian(
                    current_pos.x + delta.x,
                    current_pos.y + delta.y,
                    current_pos.z + delta.z,
                    current_pos.rx + delta.rx,
                    current_pos.ry + delta.ry,
                    current_pos.rz + delta.rz,
                )
                ra.move_linear(target)
                self.feedback_message = f"Moving right due to {TRACKING_ITEM} on left"
            elif position == "right":
                # Move left (negative x)
                delta = Cartesian(-self.steps, 0, 0, 0, 0, 0)
                current_pos = ra.read_cartesian_pos()
                target = Cartesian(
                    current_pos.x + delta.x,
                    current_pos.y + delta.y,
                    current_pos.z + delta.z,
                    current_pos.rx + delta.rx,
                    current_pos.ry + delta.ry,
                    current_pos.rz + delta.rz,
                )
                ra.move_linear(target)
                self.feedback_message = f"Moving left due to {TRACKING_ITEM} on right"
            elif position == "center":
                self.feedback_message = (
                    f"{TRACKING_ITEM.capitalize()} in center, no movement count {self.blackboard.get(BB.CENTER_COUNT)}"
                )
                return Status.SUCCESS
            elif position == "none":
                self.feedback_message = (
                    f"{TRACKING_ITEM.capitalize()} not visible, no movement"
                )
                return Status.SUCCESS
            else:
                self.feedback_message = (
                    f"{TRACKING_ITEM.capitalize()} not visible, no movement"
                )
                return Status.SUCCESS

            self.started = True
            return py_trees.common.Status.RUNNING

        # Wait until the robot arm is not moving
        if ra.is_moving():
            return py_trees.common.Status.RUNNING

        delattr(self, "started")
        self.feedback_message = "Reached position"
        return py_trees.common.Status.SUCCESS


# adapt MoveLinear to be a behaviour


class DisplayFrame(py_trees.behaviour.Behaviour):
    def __init__(self, name="Display Frame"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.READ)

    def update(self):
        frame = self.blackboard.get(BB.FRAME)
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.feedback_message = "Quit key pressed"
            return Status.FAILURE
        self.feedback_message = "Displaying frame"
        return Status.SUCCESS


def create_tree():
    # Initialize nodes
    init_camera = InitializeCamera()
    init_model = InitializeModel()
    init_robot = InitializeRobotArm()
    initial_move = InitialMove()

    # Main loop sequence
    capture = py_trees.decorators.Repeat(name="Repeat Capture", child=CaptureFrame(), num_success=inf)
    detect = DetectBottlePosition()
    move = MoveBasedOnPosition()
    display = py_trees.decorators.Repeat(name="Repeat Display", child=DisplayFrame(), num_success=inf)

    # Main processing (runs every frame)
    processing = py_trees.composites.Sequence(
        name="Processing", memory=True, children=[detect,move]
    )
    processing = py_trees.decorators.Repeat(
        name="Repeat Processing", child=processing, num_success=inf
    )

    parallel_move = py_trees.composites.Parallel(
        name="Parallel Move",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[
            capture,
            display,
            processing],
    )

    # Initialization
    initialization = py_trees.composites.Parallel(
        name="Initialization",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[init_camera, init_model, init_robot],
    )

    # Root sequence
    root = py_trees.composites.Sequence(
        name="Track and Move System",
        memory=True,
        children=[
            initialization,
            initial_move,
            DetectBottlePosition(),  # Do one first detection to avoid empty position
            CaptureFrame(),  # Do one first capture to avoid empty frame
            py_trees.decorators.Repeat(
                name="Main Loop",
                child=parallel_move,
                num_success=inf,
            ),
        ],
    )
    return root


if __name__ == "__main__":
    tree = create_tree()
    tree.setup_with_descendants()

    try:
        while True:
            print(py_trees.display.unicode_tree(tree, show_status=True))
            tree.tick_once()

            if tree.status == Status.FAILURE:
                break
            time.sleep(1 / 30)  # Fixed interval of 0.1 seconds
    finally:
        cv2.destroyAllWindows()
