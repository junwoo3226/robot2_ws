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
            DR_AXIS_X,
            DR_AXIS_Y,
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
        grip_place = posx(268.89, -79.70, grip_place_z, 65.46, -179.67, 66.99)
    
        down = posx(0, 0, -44, 0, 0, 0)
        up = posx(0, 0, 30, 0, 0, 0)
        movel(grip_place, vel=VELOCITY, acc=ACC)
        cgrip()

        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=9, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                movel(up, vel=VELOCITY, acc=ACC,mod=DR_FC_MOD_REL)
                wait(0.3)

                release()
                wait(0.3)

                movel(down, vel=VELOCITY, acc=ACC,mod=DR_FC_MOD_REL)
                wait(0.3)

                cgrip()
                wait(0.3)

                movel(posx(0, 0, 100, 0, 0, 0), vel=VELOCITY, acc=ACC,mod=DR_FC_MOD_REL)  

                # place위치에서 z값을 올린 값
                go_point_z_plus = posx(287.13,0.32,330,102.49,-179.28,102.72)
                movel(go_point_z_plus ,vel=VELOCITY, acc=ACC) 
                break
    

    # 마지막 잡을때 좌표(263.21, -56.39, 74.21, 91.09, 90, -90)
    def go_grip_cup_last(grip_place_z):
        print("라스트 함수 실행")
        # 그립할 위치 선언
        grip_place = posx(268.89, -79.70, grip_place_z, 65.46, -179.67, 66.99)
    
        grip_ = posx(263.21, -56.39, 119.21, 91.09, 90, -90)
        down = posx(0, 0, -45, 0, 0, 0)
        up = posx(0, 0, 40, 0, 0, 0)
        movel(grip_place, vel=VELOCITY, acc=ACC)
        cgrip()

        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=9, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                movel(up, vel=VELOCITY, acc=ACC,mod=DR_FC_MOD_REL)
                wait(0.3)

                release()
                wait(0.3)

                movel(grip_, vel=100, acc=100)
                wait(0.3)
                movel(down, vel=100, acc=100,mod=DR_FC_MOD_REL)
                wait(0.3)

                cgrip()
                wait(0.3)
                movel(posx(0, 0, 50, 0, 0, 0), vel=100, acc=100,mod=DR_FC_MOD_REL)
                movej(posj(0,0,0,0,0,-180), vel=50, acc=50,mod=DR_FC_MOD_REL)
                wait(0.3)
                #movel(posx(0, 0, 100, 0, 0, 0), vel=100, acc=100,mod=DR_FC_MOD_REL) 
                

                # place위치에서 z값을 올린 값
                '''go_point_z_plus = posx(263.21, -56.39, 119.21, 91.09, 90, -90)
                movel(go_point_z_plus ,vel=20, acc=20) '''
                break
    



    def go_place(x_go,y_go,place_z,down_z):
        # 컵놓을 위치 선언
        place = posx(404.71,7.84,place_z,47.34,180,46.39)
        move_go = posx(x_go, y_go,0,0,0,0)
        down = posx(0, 0, down_z, 0, 0, 0)
        up = posx(0, 0, 100, 0, 0, 0)
        wait(0.3)
        movel(place, vel=VELOCITY, acc=ACC)
        wait(0.3)
        movel(move_go, vel=100, acc=100, mod=DR_MV_MOD_REL)
        wait(0.3)
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

    # (455.4, -173.9, 330.76, 51.5, 90, 90)
    # (34.4, 0, 0, 0, 0, 0)
    def go_place_last():
        # 컵놓을 위치 선언
        place = posx(454.41, -174.0, 330.69, 51.5, 90, 90)
        move_j1 = posj(33.5, 0, 0, 0, 0, 0)
        wait(0.3)
        movel(place, vel=VELOCITY, acc=ACC)
        wait(0.3)
        movej(move_j1, vel=100, acc=100, mod=DR_MV_MOD_REL)
        wait(0.3)
        movel(posx(468.6 ,106,330.69,85.04, 90, 90),vel=100, acc=100)
#########################################힘제어##################################
        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Y, min=7, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                break

        release()
        wait(0.3)
        move_j1 = posj(-35, 0, 0, 0, 0, 0)
        movej(move_j1, vel=100, acc=100, mod=DR_MV_MOD_REL)
        wait(0.3)
#########################################해제###################################

  
    # 시작위치에서 그리퍼 열기
    movej([0, 0, 66.8, 0, 110, 0], vel=VELOCITY, acc=ACC)
    release()

    # 공정 필요한 변수 선언
    count = 0
    grip_place_z = 257.90 # 247.90
    x_go = 0
    y_go = -80
    # 공정
    while rclpy.ok():
        count += 1
        wait(0.3)
        grip_place_z -= 10
        go_grip_cup(grip_place_z)
        # 1층 - 1
        if count < 4:
            y_go += 80
            go_place(x_go,y_go,280, -180)
        # 1층 - 2
        elif count == 4:
            y_go = 40
            x_go += 70
            go_place(x_go,y_go,280, -180)
        elif count == 5:
            y_go += 80
            go_place(x_go,y_go,280, -180)
        # 1층 - 3
        elif count == 6:
            y_go = 80
            x_go += 70
            go_place(x_go,y_go,280, -180)
        # 2층 - 1
        elif count == 7:
            y_go = 40
            x_go = 30
            go_place(x_go,y_go,310, -100)
        elif count == 8:
            y_go += 80
            go_place(x_go,y_go,310, -100)
        # 2층 - 2
        elif count == 9:
            y_go = 80
            x_go += 70
            go_place(x_go,y_go,310, -100)
        # 3층
        elif count == 10:
            x_go = 60
            go_place(x_go,y_go,350, -50)
            break
    

    # 4층
    wait(0.3)
    grip_place_z -= 10
    #grip_place_z -= 110
    go_grip_cup_last(grip_place_z)
    go_place_last()
    movej([0, 0, 0, 0, 0, 0], vel=100, acc=100)


if __name__ == "__main__":
    main()