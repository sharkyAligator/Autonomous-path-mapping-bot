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
    while True:
        p.stepSimulation()
        env.move_husky(0.2, 0.2, 0.2, 0.2)
        