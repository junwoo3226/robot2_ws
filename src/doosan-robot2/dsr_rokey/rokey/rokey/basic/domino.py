import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
## 특정 위치에서 줄때까지 대기
## 받으면
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
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
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            amove_periodic,
        )
        from DSR_ROBOT2 import (
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
            DR_TOOL,
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

    def place(point):
        """Performs pick-and-place from src to dest."""
        movel(point, vel=VELOCITY, acc=ACC)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
#########################################힘제어##################################
        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=9, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                movel(up, vel=200, acc=200)
                break
        release()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
##########################################해제###################################

    def wait_block():
        while True:
            movel(initpos, vel=200, acc=200)
            release()
            wait(1)
            cgrip()
            wait(1)
            if(get_digital_input(1)):
                wait(0.5)
                print("성공")
                break

    initpos = posx(425, 256, 200, 54, 136, 41)
    down = posx(0, 0, -112, 0, 0, 0)
    up = posx(0, 0, 163, 0, 0, 0)

    ######################################
    x = 270
    y = 0
    z = 200
    count = 0 
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    release()
    while rclpy.ok():
        count += 1
        wait(0.3)
        wait_block()
        point = posx(x, y, z, 179, 179, 80)
        place(point)
        x += 50
        y += 10
        if count == 3:
            x = 240
            y = 0
            z = 80
            point = posx(x, y, z, 179, 179, 80)
            movel(point, vel=200, acc=200)
            cgrip()
            wait(0.5)
            #  춤추기
            amove_periodic(amp=[0, 0, 0, 0, 0, 30], period=1.0, atime=0.02, repeat=3, ref=DR_TOOL)
            # 춤추는 시간 고려
            wait(3)
            # x += 30
            join_point = posj(0,0,0,0,10,0)
            movej(join_point, vel=100, acc=100, mod=DR_MV_MOD_REL)
            join_point = posj(0,0,0,0,-25,0)
            movej(join_point, vel=200, acc=200, mod=DR_MV_MOD_REL)
            break
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    release()       
    


if __name__ == "__main__":
    main()