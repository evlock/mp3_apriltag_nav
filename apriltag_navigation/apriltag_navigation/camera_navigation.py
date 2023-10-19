import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3
from enum import Enum, auto
import os
from gtts import gTTS
import pygame
from typing import Tuple
from pathlib import Path
from time import sleep


class NavState(Enum):
    START = auto()
    SET_LZ = auto()
    UP = auto()
    X_DIR = auto()
    Y_DIR = auto()
    Z_DIR = auto()
    LANDED = auto()

class CameraNavigationNode(Node):
    tag_pos = {
        "ID0": (-0.05715, 0.0508),
        "ID1": (-0.05715, -0.0508),
        "ID2": (0.05715, -0.0508),
        "ID3": (0.05715, 0.0508),
        "ID4": (0, 0),
    }
    letter_pos = {
        "A": (-0.05715, 0),
        "B": (0, 0.0508),
        "C": (0.05715, 0),
        "D": (0, -0.0508),
    }

    landing_zone: list = ["A", "D"]

    def __init__(self):
        super().__init__("camera_navigation_node")
        self.create_subscription(TFMessage, "tf", self.get_tf, 10)
        self.timer = self.create_timer(2, self.run_loop)

        self.state: Enum = NavState.START
        self.landing_count: int = 0
        self.x_dir_count: int = 0
        
        pygame.init()
        pygame.mixer.init()
        
        self.desired_lz: str = None
        self.curr_tf: TransformStamped = None
        self.curr_pose: Tuple[float, float] = None
        self.desired_pose: Tuple[float, float] = None
       

    def get_tf(self, msg: TFMessage):
        if msg.transforms:
            self.curr_tf = msg.transforms[0]
        else:
            self.curr_tf = None        

    def pose_update(self):
        x = self.curr_tf.transform.translation.x
        y = self.curr_tf.transform.translation.y
        x_offset, y_offset = self.tag_pos[self.curr_tf.child_frame_id]
        self.curr_pose = [-x + x_offset, y + y_offset]

    def text_to_speech(self, audio_line):
        tts = gTTS(text=audio_line, lang="en", slow=False)
        audio_file = str(Path(__file__).parent / "temp_audio.mp3")
        tts.save(audio_file)
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()
        pygame.time.wait(1000)
        os.remove(audio_file)

    def run_loop(self):
        if self.state == NavState.START:
            if self.curr_tf:
                self.pose_update()
                self.state = NavState.SET_LZ
            else:
                self.text_to_speech("No tags")
        elif self.state == NavState.SET_LZ:
            if self.landing_count < len(self.landing_zone):
                self.desired_lz = self.landing_zone[self.landing_count]
                self.desired_pose = self.letter_pos.get(self.desired_lz, (0, 0))
                self.state = NavState.UP
            else:
                self.state = NavState.LANDED
        elif self.state == NavState.UP:
            if self.curr_tf and self.curr_tf.transform.translation.z > 0.1:
                self.state = NavState.X_DIR
            else:
                self.text_to_speech("Move up")
        elif self.state in [NavState.X_DIR, NavState.Y_DIR]:
            self.pose_update()
            if self.state == NavState.X_DIR:
                if self.x_dir_count == 1:
                    self.x_dir_count = 0
                    self.text_to_speech("Move down")
                    sleep(3)
                    self.text_to_speech("Landed")
                    self.landing_count += 1
                    self.state = NavState.SET_LZ
                    sleep(3)
                move_direction = "right" if self.curr_pose[0] < self.desired_pose[0] else "left"
                self.text_to_speech(f"Move {move_direction}")
                if abs(self.curr_pose[0] - self.desired_pose[0]) < 0.0125:
                    self.state = NavState.Y_DIR
            elif self.state == NavState.Y_DIR:
                move_direction = "forwards" if self.curr_pose[1] < self.desired_pose[1] else "backwards"
                self.text_to_speech(f"Move {move_direction}")
                if abs(self.curr_pose[1] - self.desired_pose[1]) < 0.0125:
                    self.state = NavState.Z_DIR
        elif self.state == NavState.Z_DIR:
            self.text_to_speech("Move down")
            sleep(3)
            self.text_to_speech("Stop")
            self.x_dir_count += 1
            self.state = NavState.X_DIR    
        elif self.state == NavState.LANDED:
            self.text_to_speech("Finished")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()
