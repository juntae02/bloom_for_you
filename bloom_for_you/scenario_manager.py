import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import queue, time, math


from bloom_for_you.function_modules import robot_util
from bloom_for_you.function_modules import config

import DR_init
DR_init.__dsr__id = config.ROBOT_ID
DR_init.__dsr__model = config.ROBOT_MODEL

from DR_common2 import posx,posj
ON, OFF = 1, 0

class ScenarioManager:
    def __init__(self, node: Node):
        # super().__init__('drawer_and_eraser', namespace=ROBOT_ID)

        self.node = node

        self.mover = robot_util.MoverUtil()
        self.gripper = robot_util.GripperUtil()
        self.forcer = robot_util.ForcerUtil()
        self.i = 0

    def set_dependencies(
            self, 
            
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
        ):
        self.mover.set_dependencies(
            movel,
            movej,
            get_current_posx,
        )

        self.gripper.set_dependencies(
            set_digital_output,
            get_digital_input,
            wait,
        )
        
        self.forcer.set_dependencies(
            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,

            DR_FC_MOD_REL
        )

    def run_scenario(self):
        print("hello", self.i)
        self.i +=1
        
def main():
    rclpy.init()
    node = rclpy.create_node('scenario_manager', namespace=robot_util.ROBOT_ID)
    scenario_manager = ScenarioManager(node)
    DR_init.__dsr__node = node

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
    scenario_manager.set_dependencies(
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
    
    tcp, tool = get_tcp(), get_tool()
    print(f'tcp: {tcp}, tool: {tool}')
    if tcp == None or tool == None:
        scenario_manager.node.destroy_node()
        return

    try:
        print('Spin start')
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 노드 테스트
            # scenario_manager.run_scenario()
            
            # move 테스트 1
            # pos_test1 = [300, 0, 300, 0, 180, 0]
            # pos_test2 = [300, 0, 250, 0, 180, 0]
            
            # move 테스트 2
            # scenario_manager.mover.move(pos_test1)
            # scenario_manager.mover.move(pos_test2)
            # movel(posx(pos_test1), 60, 60)
            # movel(posx(pos_test2), 60, 60)
            
            # 그리퍼 테스트
            # print("close")
            # scenario_manager.gripper.close_grip()
            # print("open")
            # scenario_manager.gripper.open_grip()
            
            # forcer 테스트
            # time.sleep(5.0)
            # scenario_manager.forcer.force_on_z(20)
            # time.sleep(5.0)
            # scenario_manager.forcer.force_off()
            # time.sleep(5.0)
            # scenario_manager.mover.move_home()

    except KeyboardInterrupt:
        scenario_manager.forcer.force_off()
        print('Shutting down...')
        pass

    scenario_manager.node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
