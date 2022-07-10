import gym
import time
import pixelate_arena
import pybullet as p
import pybullet_data
import cv2
import os

if __name__ == "__main__":
    
    env = gym.make("pixelate_arena-v0")
    time.sleep(0.5)
    img = env.camera_feed()
    cv2.imshow("img", img)
    cv2.imwrite("sample_arena_img.png",img)
    cv2.waitKey(0)
 