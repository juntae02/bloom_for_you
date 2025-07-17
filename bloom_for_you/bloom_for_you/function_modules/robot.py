from bloom_for_you.function_modules import config
from rclpy.node import Node
import time

ON, OFF = 1, 0

from DR_common2 import posx, posj

class Robot:
    def __init__(self):

        self.node = config.g_node
        
        from DSR_ROBOT2 import (
            get_tcp, get_tool,
            
            # MoverUtil 함수
            movel,
            movej,
            get_current_posx,
            
            # GripperUtil 함수
            set_digital_output,
            get_digital_input,
            wait,
            
            # ForcerUtil 함수
            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,
            DR_FC_MOD_REL
        )

        self.set_dependencies(
            get_tcp, 
            get_tool,
            
            # MoverUtil 함수
            movel,
            movej,
            get_current_posx,
            
            # GripperUtil 함수
            set_digital_output,
            get_digital_input,
            wait,
            
            # ForcerUtil 함수
            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,
            DR_FC_MOD_REL
        )
        if not self.validate_tcp_and_tool():
            print('로봇 초기화 실패')
            return

    def set_dependencies(
            self, 
            get_tcp, 
            get_tool,
            
            # Move 함수
            movel,
            movej,
            get_current_posx,
            
            # Gripper 함수
            set_digital_output,
            get_digital_input,
            wait,
            
            # ForcerUtil 함수
            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,
            DR_FC_MOD_REL
        ):
            # Tcp 관련 함수
            self.get_tcp=get_tcp
            self.get_tool=get_tool
            
            # Move 함수
            self.movel = movel
            self.movej = movej
            self.get_current_posx = get_current_posx
            
            # Gripper 함수
            self.get_digital_input = get_digital_input
            self.wait = wait
            self.set_digital_output = set_digital_output
            
            # Force 관련 함수
            self.task_compliance_ctrl = task_compliance_ctrl
            self.set_desired_force = set_desired_force
            self.release_force = release_force
            self.release_compliance_ctrl = release_compliance_ctrl
            self.check_force_condition = check_force_condition
            self.DR_AXIS_Z = DR_AXIS_Z
            self.DR_FC_MOD_REL = DR_FC_MOD_REL

    def validate_tcp_and_tool(self):
        tcp = self.get_tcp()
        tool = self.get_tool()

        if tcp is None or tool is None:
            self.node.get_logger().error('[ERROR] TCP 또는 TOOL 정보를 가져오지 못했습니다.')
            raise RuntimeError('로봇 초기화 실패: TCP/TOOL 없음')
        else:
            return True

    # ========== move 관련 함수 ==========
    # 포즈 이동
    def move(self, pos):
        self.movel(posx(pos), config.VEL, config.ACC)
    
    # 상대 포즈 이동
    def move_relative(self, pos):
        self.movel(posx(pos), config.VEL, config.ACC, mod=config.DR_MV_MOD_REL)

    # 홈 위치 이동
    def move_home(self):
        home = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        self.movej(home, config.VEL, config.ACC)
    
    # 툴 좌표계 기준 이동
    def move_by_tool(self, pos):
        self.movel(posx(pos), config.VEL, config.ACC, ref = 1) 

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

    # ========== gripper 관련 함수 ==========
    # 그리퍼 열기
    def open_grip(self):
        self.set_digital_output(2, ON)
        self.set_digital_output(1, OFF)
        self.wait(0.5)
    
    # 그리퍼 닫기
    def close_grip(self):
        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)
        self.wait(0.5)
        
    # ========== force 관련 함수 ==========
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