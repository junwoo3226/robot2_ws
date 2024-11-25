import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
        )
        from DSR_ROBOT2 import (
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_FC_MOD_REL,
            DR_TOOL,
            DR_QSTOP
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")



    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def cgrip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)



    def go_grip_cup(grip_place_z):
        # 그립할 위치 선언
        grip_place = posx(482.50, 290.94, grip_place_z, 143.11, 179.25,128.22)
    
        down = posx(0, 0, -44, 0, 0, 0)
        up = posx(0, 0, 30, 0, 0, 0)
        movel(grip_place, vel=200, acc=200)
        cgrip()

        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=9, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                movel(up, vel=200, acc=200,mod=DR_FC_MOD_REL)
                wait(1)

                release()
                wait(1)

                movel(down, vel=200, acc=200,mod=DR_FC_MOD_REL)
                wait(1)

                cgrip()
                wait(1)

                movel(posx(0, 0, 100, 0, 0, 0), vel=200, acc=200,mod=DR_FC_MOD_REL)  

                # place위치에서 z값을 올린 값
                go_point_z_plus = posx(287.13,0.32,430,102.49,-179.28,102.72)
                movel(go_point_z_plus ,vel=200, acc=200) 
                break
    

    def go_place(x_go,y_go,down_z):
        # 컵놓을 위치 선언
        place = posx(279.6,-44.87,400,144.87,179.75,144.2)
        move_go = posx(x_go, y_go,0,0,0,0)
        down = posx(0, 0, down_z, 0, 0, 0)
        up = posx(0, 0, 100, 0, 0, 0)
        wait(1)
        movel(place, vel=200, acc=200)
        wait(1)
        movel(move_go, vel=100, acc=100, mod=DR_MV_MOD_REL)
        wait(1)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
#########################################힘제어##################################
        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=9, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                break
            
        release()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
#########################################해제###################################



  
    # 시작위치에서 그리퍼 열기
    movej([0, 0, 66.8, 0, 110, 0], vel=VELOCITY, acc=ACC)
    release()

    # 공정 필요한 변수 선언
    count = 0
    grip_place_z = 257.90 # 247.90
    x_go = -80
    y_go = 0
    # 공정
    while rclpy.ok():
        count += 1
        wait(0.3)
        grip_place_z -= 10
        go_grip_cup(grip_place_z)
        # 1층 - 1
        if count < 4:
            x_go += 80
            go_place(x_go,y_go, -300)
        # 1층 - 2
        elif count == 4:
            x_go = 40
            y_go += 70
            go_place(x_go,y_go, -300)
        elif count == 5:
            x_go += 80
            go_place(x_go,y_go, -300)
        # 1층 - 3
        elif count == 6:
            x_go = 80
            y_go += 70
            go_place(x_go,y_go, -300)
        # 2층 - 1
        elif count == 7:
            x_go = 40
            y_go = 35
            go_place(x_go,y_go, -180)
        elif count == 8:
            x_go += 80
            go_place(x_go,y_go, -180)
        # 2층 - 2
        elif count == 9:
            x_go = 80
            y_go += 70
            go_place(x_go,y_go, -180)
        elif count == 10:
            y_go = 70   
            go_place(x_go,y_go, -80)
            break 

    
    # 시작위치 복귀
    # movej([0, 3, 66.8, 0, 110, 0], vel=VELOCITY, acc=ACC)
    release()


if __name__ == "__main__":
    main()