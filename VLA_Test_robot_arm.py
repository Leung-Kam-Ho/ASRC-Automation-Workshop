from robot_arm.position import Cartesian, JPosition
from robot_arm.robot_arm import RobotArm
# Install minimal dependencies (`torch`, `transformers`, `timm`, `tokenizers`, ...)
# > pip install -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch

DEVICE = "mps"
# robot.act(action, ...)

if __name__ == "__main__":
    ra = RobotArm('192.168.10.228')
    ra.set_speed(.2)
    # get current joint position
    
    
    
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

    current_jpos = ra.read_joint_pos()

    image: Image.Image = Image.open("current.jpg").convert("RGB")
    prompt = f"In: Current joint position: {current_jpos} What action should the robot take to go to init pose?\nOut:"

    # Predict Action (7-DoF; un-normalize for BridgeData V2)
    inputs = processor(prompt, image).to(DEVICE, dtype=torch.float16)
    action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)
    


    print("Predicted Action:", action)
    # Execute...
    
    