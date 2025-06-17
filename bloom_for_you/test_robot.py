from bloom_for_you.function_modules import robot

def main(args=None):
    robot_instance = robot.Robot()
    
    # # move 테스트
    pos_test1 = [300, 0, 200, 0, 180, 0]
    pos_test2 = [300, 0, 250, 0, 180, 0]
    pos_test3 = [100, 0, 0, 0, 0, 0]
    # robot_instance.move(pos_test1)
    robot_instance.move(pos_test2)
    robot_instance.move_home()
    robot_instance.move_relative(pos_test3)
    robot_instance.move_home()
    
    # 그리퍼 테스트
    robot_instance.close_grip()
    robot_instance.open_grip()
    
    # force 테스트
    robot_instance.force_on_z(20)
    # time.sleep(5.0)
    robot_instance.force_off()

if __name__ == '__main__':
    main()