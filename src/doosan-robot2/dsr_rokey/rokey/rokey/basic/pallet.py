# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    node = rclpy.create_node("grip_simple", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            DR_MV_MOD_REL,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass
    
    # 그리퍼 놓기
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)
    
    # 그리퍼 잡기
    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
        
    # 각 지점 선언
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx(502.04, -150.91, 200, 154.5, 179.02, 154.04)
    pos2 = posx(399.55, -150.91, 200, 154.67, 178.33, 154.51)
    pos3 = posx(500.91, -48.43, 200, 160.45, 178.19, 160.1)
    pos4 = posx(399.1, -49.84, 200, 163.29, 177.86, 163.24)
    p1 = posx(501.04, 0, 200, 154.5, 179.02, 154.04)
    p2 = posx(398.55, 0, 200, 154.67, 178.33, 154.51)
    p3 = posx(499.91, 100, 200, 160.45, 178.19, 160.1)
    p4 = posx(398.1, 100, 200, 163.29, 177.86, 163.24)
    down = posx(0,0,-163,0,0,0)
    up = posx(0,0,163,0,0,0)
    
    # 반복문 사용을 위한 지점 리스트 생성
    position = [pos1,pos2,pos3,pos4]
    des = [p1,p2,p3,p4]
    
    # 실행
    while rclpy.ok():
        set_tool("Tool Weight_2FG")
        set_tcp("2FG_TCP")
        release()
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        for i in range(len(position)):
            movel(position[i], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            grip()
            movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            movel(des[i], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            release()
            movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        for i in range(len(position)):
            movel(des[i], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            grip()
            movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            movel(position[i], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            release()
            movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movej(JReady, vel=VELOCITY, acc=ACC)
        break
if __name__ == "__main__":
    main()