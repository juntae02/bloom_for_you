import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import queue, time, math

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL = config.VEL 
ACC = config.ACC
DRAWING_PAHT = config.DRAWING_PATH
SAMPLE_THRESHOLD = config.SAMPLE_THRESHOLD
SAMPLE_RATIO = config.SAMPLE_RATIO

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DR_common2 import posx

ON, OFF = 1, 0

class Forcer:
    def set_dependencies(
            self,

            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,

            DR_FC_MOD_REL
        ):

        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.release_force = release_force
        self.release_compliance_ctrl = release_compliance_ctrl
        self.check_force_condition = check_force_condition
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_FC_MOD_REL = DR_FC_MOD_REL

    def force_on(self, fd=[0, 0, 20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0]):
        """로봇 펜/지우개을 보드에 누르는 동작을 수행합니다."""
        self.task_compliance_ctrl()
        time.sleep(0.1)
        self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)

    def force_off(self):
        """로봇의 힘/컴플라이언스(유연제어) 제어를 모두 해제합니다."""
        self.release_force()
        self.release_compliance_ctrl()

    def check_touch(self, mode, fd = [0, 0, 2, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0]):
        """Z축 힘(접촉력)이 5~21 사이가 될 때까지 대기"""
        if mode == 2: fd[2] = 5 # erase mode
        while self.check_force_condition(self.DR_AXIS_Z, min=5, max=21): pass
        self.release_force()
        self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)