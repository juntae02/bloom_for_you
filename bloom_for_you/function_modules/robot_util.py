import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import queue, time, math

from bloom_for_you.function_modules import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL = config.VEL 
ACC = config.ACC

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DR_common2 import posx, posj

ON, OFF = 1, 0

class MoverUtil:
    home = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    
    def set_dependencies(
            self,
            movel,
            movej,
            get_current_posx,
        ):
        self.movel = movel
        self.movej = movej
        self.get_current_posx = get_current_posx

    # 포즈 이동
    def move(self, pos):
        self.movel(posx(pos), VEL, ACC)

    # 홈 위치 이동
    def move_home(self):
        self.movej(self.home, VEL, ACC)
    
    # 툴 좌표계 기준 이동
    def move_by_tool(self, pos):
        self.movel(posx(pos), VEL, ACC, ref = 1) 

    def _get_cur_posx(self):
        """
        현재 로봇의 작업 좌표계(Task Space Position, posx)를 읽어오는 함수.

        주요 동작:
            - 최대 5초 동안 반복해서 get_current_posx()를 호출하여 posx 정보를 시도한다.
            - IndexError가 발생하면 0.1초 후 재시도한다.
            - 5초 이내에 posx를 정상적으로 얻으면 해당 값을 반환한다.
            - 5초가 지나도 값을 얻지 못하면 오류 로그를 남기고, 모든 값이 0인 posx([0,0,0,0,0,0])를 반환한다.

        Returns:
            posx: 6차원 작업좌표(posx) 리스트 객체 또는 실패시 [0,0,0,0,0,0]
        """
        start = time.time()
        while time.time() - start < 5:
            try:
                cur_posx = self.get_current_posx()
                return cur_posx
            except IndexError as e:
                time.sleep(0.1)
                continue
        return posx([0,0,0,0,0,0])

class GripperUtil:
    def set_dependencies(
            self,
            
            set_digital_output,
            get_digital_input,
            wait,
        ):

        self.get_digital_input = get_digital_input
        self.wait = wait
        self.set_digital_output = set_digital_output

    def open_grip(self):
        self.set_digital_output(2, ON)
        self.set_digital_output(1, OFF)
        self.wait(0.5)

    def close_grip(self):
        self.open_grip()
        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)
        self.wait(0.5)

class ForcerUtil:
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
    
    # 힘과 방향 입력해서 force 제어
    def force_on(self, fd=[0, 0, 20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0]):
        self.task_compliance_ctrl()
        time.sleep(0.1)
        self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)
    
    # z축 force 만 입력해서 force 제어
    def force_on_z(self, force_n):
        fd=[0, 0, force_n, 0, 0, 0]
        dir=[0, 0, 1, 0, 0, 0]
        self.task_compliance_ctrl()
        time.sleep(0.1)
        self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)

    def force_off(self):
        """로봇의 힘/컴플라이언스(유연제어) 제어를 모두 해제합니다."""
        self.release_force()
        self.release_compliance_ctrl()

    def check_touch(self, min=5, max=21):
        """Z축 힘(접촉력)이 min과 Max 사이를 벗어나면 힘 제어 종료"""

        while self.check_force_condition(self.DR_AXIS_Z,min,max): 
            pass

        self.release_force()
        self.release_compliance_ctrl()

