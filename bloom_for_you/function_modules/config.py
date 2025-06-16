ROBOT_ID = 'dsr01'
ROBOT_MODEL = "m0609"
ROBOT_TOOL = 'Tool Weight'
ROBOT_TCP = 'GripperDA_v1'
VEL = 60
ACC = 60

# ======================== for robot module ========================
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

import rclpy
import DR_init

rclpy.init()
g_node = rclpy.create_node('global_node', namespace=ROBOT_ID)
DR_init.__dsr__node = g_node

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL