from robot_arm.position import Cartesian, JPosition
from robot_arm.robot_arm import RobotArm
# Install minimal dependencies (`torch`, `transformers`, `timm`, `tokenizers`, ...)
# > pip install -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch
from utils.get_image_client import get_image
from src.example.robot_action_node import MoveJoint, MoveLinear, MoveGripper
import py_trees
import numpy as np
import cv2

DEVICE = "mps"
CAMERA_IP = "sparks-beta.local"  # adjust as needed
# robot.act(action, ...)

def initialize_arm(ra : RobotArm):
    tree = py_trees.trees.BehaviourTree(
        MoveLinear("Init Position", ra, Cartesian(0, 0, 0, 0, 0, 0), relative=False)
    )
    # Execute...
    tree.tick()
    print(py_trees.display.unicode_tree(tree.root, show_status=True))

if __name__ == "__main__":
    ra = RobotArm('192.168.10.228')
    ra.set_speed(0.7)
    # get current joint position
    
    # initialize_arm(ra)
    
    
    # Load Processor & VLA
    processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
    vla = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b", 
        # attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
        torch_dtype=torch.float16, 
        low_cpu_mem_usage=True, 
        trust_remote_code=True
    ).to(DEVICE)

    # Grab image input & format prompt
    while True:
        current_pos = ra.read_cartesian_pos()
        print("Current Position:", current_pos.__dict__)
        image_data = get_image(ip=CAMERA_IP, port=8000)
        if image_data:
            # Convert byte data to numpy array
            nparr = np.frombuffer(image_data, np.uint8)
            img_np = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            # resize for display
            # img_np = cv2.resize(img_np, (224, 224))
        image = Image.fromarray(img_np)
        INSTRUCTIONS = "grab the black handle on the table" #"move a little bit forward"
        prompt = f"""
        In:  What action should the robot take to {INSTRUCTIONS}?
        Out:
        """

        # Predict Action (7-DoF; un-normalize for BridgeData V2)
        inputs = processor(prompt, image).to(DEVICE, dtype=torch.float16)
        action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)
        
        # the last token is the gripper, remove it
        gripper = action[-1]
        action = action[:-1]
        
        action *= 10
        
        # convert to numpy and scale to real-world joint ranges
        action = action 
        # action = action * (100 - (-100)) + (-100)  # un-normalize from [0, 1] to [-100, 100]
        """
        action 0 = move x
        action 1 = move y
        action 2 = move z
        action 3 = move rx
        action 4 = move ry
        action 5 = move rz
        
        y = +forward/-back
        x = +left/-right
        z = +up/-down
        
        however, in the robot coordinate system:
        x = +forward/-back
        y = +left/-right
        z = +up/-down
        """
        
        
        # x = action[1]
        # y = action[0]
        # z = action[2]
        # rx = action[4]
        # ry = action[3]
        # rz = action[5]

        print("Predicted Action:", action)
        print("Predicted Gripper:", gripper)
        # _ = input("Press Enter to continue...")
        # # create behavior tree to execute the action
        tree = py_trees.trees.BehaviourTree(
            MoveLinear("Execute Predicted Action", ra, Cartesian(*action), relative=True)
        )
        
        # if gripper > 0:
            # gripper_position =   # open
        while True:
            # Execute...
            tree.tick()
            # print(py_trees.display.unicode_tree(tree.root, show_status=True))
            if tree.root.status == py_trees.common.Status.SUCCESS:
                break
        
        