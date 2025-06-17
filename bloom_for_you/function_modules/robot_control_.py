import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from bloom_for_you_interfaces.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("bloom_for_you")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0


from bloom_for_you.function_modules import robot

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### Robot Controller ############


class RobotController(Node):
    def __init__(self):
        print("aaaa")
        super().__init__("pick_and_place")
        print("bbbb")
        self.robot_instance= robot.Robot()
        print("cccc")

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        print("dddd")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        print("eeee")
        self.get_position_request = SrvDepthPosition.Request()
        

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self, grip_target, z_move):
        self.get_logger().info("robot control start")
        
        target_pos = self.get_target_pos(grip_target)
        # target_pos = [300, 0, 250, 0, 180, 0]
        self.get_logger().info("==============")
        # self.get_logger().info(target_pos)
        self.get_logger().info("hhhh")
        self.robot_instance.open_grip()
        self.robot_instance.move_home()
        
        if target_pos is None:
            self.get_logger().info("iiii")
            self.get_logger().warn("No target position")
        else:
            self.get_logger().info("jjjj")
            target_pos[2] = target_pos[2] + z_move
            self.get_logger().info(f"target position: {target_pos}")
            self.robot_instance.move(target_pos)
            self.robot_instance.close_grip()
            time.sleep(2.0)
            self.robot_instance.move_home()
            self.robot_instance.open_grip()
            
            # time.sleep(2.0)
            # self.robot_instance.close_grip()

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            self.get_logger().info(f"=========== gripper cam path ===========: {gripper2cam_path}")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)
            self.get_logger().info("ffff")

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0
            self.get_logger().info("gggg")
            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos

def main(args=None):
    node = RobotController()
    
    while rclpy.ok():
        x = input("input target")
        node.robot_control(x, -5)
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
